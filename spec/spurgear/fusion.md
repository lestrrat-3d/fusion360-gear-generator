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
  centre.) Concretely: `if angle != 0: spineAngularDimension.parameter.value = angle`. The angular
  dimension itself exists for **every** angle including 0, because it is what says which way the
  spine points (`[SPUR-F-SPINE]`); at `angle = 0` it is created at 0 and there is simply nothing to
  set afterwards.

## Per-step constraint recipes (the over-constraint-sensitive ones)

These are the spur-specific constraint constructions whose **exact set and order** matter — a
different set or order throws `VCS_SKETCH_OVER_CONSTRAINTS` or `VCS_SKETCH_SOLVING_FAILED`. (They
build on the shared `[PB-FULL-CONSTRAINT]`, `[PB-SHARE-XOR-COINCIDENT]`, `[PB-NO-OVERCONSTRAIN]`,
`[PB-DRIVING-DIM]` rules; here is the spur application.)

- **[SPUR-F-TOOTHTOP-ARC] Tooth-top arc — centred on the local origin (step 6).** The arc caps the
  tooth at the tip circle, so it *is* part of that circle and must bulge outward. Say that by
  **sharing the local origin as the arc's centre**, and add nothing else.
  1. Materialize a **tooth-top point**: a `SketchPoint` at the tip, **rotated by `angle`** to match
     the rotated flanks — `(Tip Circle Radius · cos(angle), Tip Circle Radius · sin(angle))` —
     constrained **coincident to the tip circle**.
  2. Create the arc with `sketchArcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint,
     leftFlankEndPoint)` — pass the local origin and the two flank splines' **end `SketchPoint`s
     directly**, so the arc shares all three and needs no separate coincidences.
  3. Add **no diameter dimension**. The shared centre and the two shared ends already determine the
     arc, and its radius follows from the flank ends being on the tip circle.

  ⚠️ A **free centre plus a diameter dimension** determines the arc's size but not which way it
  curves: an arc of the same radius through the same two ends can bulge inward, back through the
  tooth. The sketch then reaches DOF 0 with two valid answers and the solver picks by where the
  centre was seeded. Sharing the origin removes the choice.

  ⚠️ Sharing the centre makes the **last rib's perpendicular redundant** — see `[SPUR-F-RIBS]`,
  which omits it. Keeping both is what throws `VCS_SKETCH_OVER_CONSTRAINTS`.

- **[SPUR-F-SPINE] Spine + +X reference + angular pin (step 7).** Draw the spine as a construction
  line `addByTwoPoints(localOrigin, toothTopPoint)` — pass **both** existing `SketchPoint`s
  directly (share them). Do **not** create it from `.geometry`, do **not** add a separate
  start-coincident to the origin (sharing already ties it; an extra coincident makes the solver
  fail), and do **not** constrain the spine's end onto the arc (the tooth-top point already lies on
  the tip circle).

  Build the **+X reference construction line** for **every** `angle`, including 0:
  1. Add a far endpoint at `(Tip Circle Radius, 0)` and pin it with **two axis dimensions from
     the local origin** — `addDistanceDimension(..., HorizontalDimensionOrientation, Tip Circle
     Radius)` and the vertical one at `0`; both values are non-negative magnitudes and the
     endpoint is seeded on the +X side, per `[PB-DIM-VALUE-SEMANTICS]`. Pin it this way rather
     than with `addCoincident(end, tipCircle)`: a point on a circle has two answers, and pinning
     its x at the tip radius instead touches the circle at its extreme, where the numbers go
     unstable.
  2. Draw the reference line from the origin to that endpoint and mark it construction.
  3. Add an angular dimension **from the reference to the spine, in that argument order**
     (`addAngularDimension(reference, spine, …)`); place its text on the **bisector of the intended
     angle** (`(R·cos(angle/2), R·sin(angle/2))` for small R) so Fusion selects `angle`, not its
     supplement. Set its value to `angle` as the very last action (see `[SPUR-F-ROTATE-CONFIRM]`).

  ⚠️ Do **not** use a plain `addHorizontal` on the spine for the `angle = 0` case. Horizontal fixes
  the line's direction but says nothing about which way it points, so the tooth top can settle at
  either end of the tip circle and the whole tooth comes out 180° around. The angular dimension
  against a reference that is pinned to +X is what says which way, and using it for every `angle`
  keeps spur, helical, herringbone and the bevel virtual tooth on one path.

- **[SPUR-F-RIBS] Ribs — exact construction order (step 8).** A rib construction line runs between
  each pair of matching left/right flank points — **one per fit-point index `i` for all N indices,
  endpoints included** (the base-circle pair `i=0` and the tip pair `i=N-1` both get a rib, even
  though the tip ends are also joined by the tooth-top arc; the fit-points have no other constraint,
  so an omitted endpoint rib leaves the sketch under-constrained). Each needs a materialized
  **midpoint sketch point** on the spine. Build each rib in **this exact order** — a different order
  over-constrains the sketch (`VCS_SKETCH_OVER_CONSTRAINTS`):
  1. Add the rib with `addByTwoPoints(leftSpline.fitPoints[i], rightSpline.fitPoints[i])` — pass the
     two fit-point `SketchPoint`s **directly** so the rib shares them; mark it construction.
  2. Dimension the rib with an **axis** dimension (horizontal/vertical), not an aligned one: for
     `angle = 0` use `addDistanceDimension(left, right, VerticalDimensionOrientation, …)`, created
     with the fit points already at their seeded positions and its value left at the measured
     magnitude — the direction is captured from the seed at creation, per
     `[PB-DIM-VALUE-SEMANTICS]`. For a rotated tooth, the rib takes the axis **across** the spine
     and the midpoint chain takes the one **along** it: use vertical for the rib and horizontal
     for the chain when `|cos(angle)| >= |sin(angle)|`, and swap both otherwise. That reduces to
     the pair above at `angle = 0`, and a tooth at 90° fails without it. An **aligned** dimension
     gives only the length, which the left and right flanks satisfy equally well swapped over, so
     the tooth can come out mirrored; the axis dimension's captured direction is what forbids the
     swap.
  3. Add a fresh `SketchPoint` for the midpoint, created **already on the spine**. The spine is the
     line at `angle` through the local origin, so seed the midpoint at the **foot of the left fit
     point on that line**: with `t = fitX·cos(angle) + fitY·sin(angle)`, the seed is
     `(t·cos(angle), t·sin(angle))`. (For `angle = 0` this reduces to `(fitX, 0)`.) Do **not** seed
     it at the rib's true 2-D midpoint, and do **not** seed it at `(fitX, 0)` for a rotated tooth.
  4. `addCoincident(midpoint, spine)` — pin the point onto the spine **first**.
  5. `addMidPoint(midpoint, rib)` — then make it the rib's midpoint.
  6. `addPerpendicular(spine, rib)` — then make the rib perpendicular to the spine. **Skip this for
     the last rib.** That rib joins the two flank tips, which the tooth-top arc already holds at
     equal radius either side of the spine (`[SPUR-F-TOOTHTOP-ARC]`), so its perpendicular says
     nothing new and Fusion rejects it with `VCS_SKETCH_OVER_CONSTRAINTS`.

  Then dimension the distance from each rib's midpoint to the previous rib's midpoint with an
  **axis** dimension along the spine direction (horizontal for `angle = 0`) — and **for the first
  rib, dimension it from the local origin to its midpoint** (start the chain with
  `previous = local origin`). The axis dimension's direction, captured from the seeded midpoints
  at creation (`[PB-DIM-VALUE-SEMANTICS]`), makes the chain run outward; an aligned dimension is
  equally happy running the other way, which is one of the ways the tooth ends up reversed. Without that origin-to-first dim the whole rib chain has
  one residual DOF (it slides along the spine as a unit) and the sketch never fully constrains. Per
  rib this is exactly determined; any further constraint, wrong order, or off-spine midpoint seed
  over-constrains it.

- **[SPUR-F-FLANK-ROOT] Flank-to-root lines — exactly two constraints, and embedded-case detection
  (step 9).** If the flank's first point (on the base circle) lies **outside** the root circle, draw
  a short radial line from the root circle up to that start point on each side. Build each as
  `addByTwoPoints(rootEndGeometry, flankStartFitPoint)` — pass the flank spline's **start
  `SketchPoint` directly** as the far endpoint (share it; no separate coincident). Then place the
  root end with **exactly these two axis dimensions from the local origin**, no others:
  - (a) `addDistanceDimension(localOrigin, rootEnd, HorizontalDimensionOrientation, …)`;
  - (b) the same with `VerticalDimensionOrientation`.

  The root end is seeded at its exact computed position **before** the dimensions are created, so
  each dimension captures its direction from that seed and its created value is already the exact
  magnitude — set values only to `abs(Δx)` / `abs(Δy)`, and **never to the axis-signed deltas: a
  negative `parameter.value` flips the point to the other side of the origin**
  (`[PB-DIM-VALUE-SEMANTICS]`; this exact flip mirrored the right-hand root end and left the
  tooth loop open, found in-Fusion 2026-08-24). Together the two dimensions exactly constrain the
  root end (2 DOF → 0), and their captured directions say **which side of the gear centre** it
  sits on.

  ⚠️ Do **not** place it instead with "root end on the root circle" plus "local origin on the
  line". Those two are satisfied by **two** points, because the line through the flank start and
  the centre carries on and meets the root circle again on the far side. The stub then stops being
  a stub and becomes a long line straight across the gear, and the sketch reaches DOF 0 with both
  answers available. The dimensions' captured directions are what rule the far one out. This common case yields a tooth loop
  of **6 curves** (2 splines + 2 flank-to-root lines + 2 arcs). If instead the flank starts
  **inside** the root circle (**high** tooth counts drop the base circle below the root: it happens
  above `2.5 / (1 - cos(PressureAngle))` teeth, which is 41.5 at 20°, 78.5 at 14.5° and 26.7 at
  25°, so a larger pressure angle brings it on sooner), no flank-to-root line is drawn and the loop has **4 curves** (2 splines + 2
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
