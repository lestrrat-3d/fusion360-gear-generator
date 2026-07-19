# Bevel Gear — Fusion realization notes

This is the **Fusion-specific** sidecar to `instructions.md`. `instructions.md` carries the design
and geometry *intent*; this file carries the bevel-specific Fusion-API realization (the §2 lattice
constraint style, the sketch-local apex positioning, the full-constraint gate and its exemptions,
the cleanup recipe) that is not worth promoting to the shared `PLAYBOOK.md`. The build steps cite
these rules by anchor (`[BEVEL-F…]`). The cross-gear conventions they build on stay cited as
`[PB-…]`.

**Anchor convention** (mirrors the playbook's `[PB-…]`): rules carry stable IDs like
`**[BEVEL-F-COINCIDENT-STYLE]**`; `instructions.md` cites the anchor, the text HERE is
authoritative, and IDs are permanent. Where a rule is the bevel delta of a cross-gear convention it
cites the `[PB-…]` anchor and states only the bevel-specific difference. These rules are
**binding** — every ⚠️ encodes a known failure mode.

Bevel's sketch work differs from the spur family — these are the deltas (the general Fusion gotchas
in `PLAYBOOK.md` still apply).

## Full-constraint gate (and its exemptions)

- **[BEVEL-F-FULL-CONSTRAINT] Every *permanent* sketch this generator authors MUST end FULLY
  CONSTRAINED.** Gate every permanent sketch with the `[PB-FULL-CONSTRAINT]` check (raise, naming
  the sketch), called at the END of each sketch-building step. This applies to **the Anchor Sketch,
  the Gear Profiles (§2) sketch, both per-gear Profile sketches, and the Bore sketch**. A free DOF
  is a **generation defect, not a warning** — raise, don't warn. **Do NOT** reach full constraint by
  dimensioning the *driven* §2 lines (Apex→A/B, the module-length extensions) — those are determined
  by the perpendicular/collinear/closing constraints (`[PB-NO-OVERCONSTRAIN]`). Once the Anchor Line
  direction is fixed, the §2 lattice is fully determined by its existing net; the per-gear Profile
  sketches are made fully constrained by recreating their six vertices as **fixed points** per the
  `[PB-PROJECT-NOT-FIXED]` recreate-share-fix recipe, not by projecting §2 points.
  - **Exemption — the two tooth-profile sketches.** They are drawn by the borrowed spur generator,
    which leaves the **embedded** (low-tooth-count) tooth under-constrained: when the involute flank
    starts *inside* the root circle the generator omits the flank-to-root lines that radially pin the
    flanks (see `spec/spurgear/fusion.md`, `[SPUR-F-FLANK-ROOT]`), so the flanks keep a free radial
    DOF. This is **benign here** — the spur generator places every involute point at its computed
    position and the tooth profile is consumed immediately by the apex→profile loft, so the residual
    DOF never moves anything. After `draw()` returns, do **NOT** raise on the tooth sketch — at most
    `futil.log` it if `not toothSketch.isFullyConstrained`. Forcing full constraint would require
    pinning the embedded tooth inside the shared spur generator, risking the whole
    spur/helical/herringbone family for no benefit. (Lower-tooth-count gears — e.g. module 2 /
    driving 19 / pinion 13 — fail the gate otherwise.)
  - **Exemption — the spiral build's auxiliary sketches.** When ψ > 0 the spiral tooth build (§3a)
    authors transient construction sketches — the `{gear} 2D Tooth Trace` (cutter arc) and the
    `{gear} Cone Element` line / `{gear} Trace Plane` it sits on. (There is **no** 3-D projection,
    root-cone, or twist-angle sketch — the spiral twist is computed analytically in §3a step G.) The
    cutter arc is a genuine arc (radius dimension + center-coincidence) **deliberately left with
    free DOF** (its toe/heel endpoints are pinned by 3-point construction, not dimensioned). These
    sketches are consumed by the build and hidden in cleanup, so do **NOT** gate them. The gate
    applies **only to the bevel's own permanent sketches** (Anchor, Gear Profiles, the two per-gear
    Profile sketches, Bore) — for both straight and spiral builds.

## §2 lattice constraint construction

- **[BEVEL-F-COINCIDENT-STYLE] §2 lattice lines use the COINCIDENT style, not sharing** — a stricter
  delta to `[PB-SHARE-XOR-COINCIDENT]` (which allows either style; §2 allows only one). When a §2
  line must *start at* / connect to an already-existing point (Apex, A, B, C, **the projected
  center**, …), **create the line from raw `Point3D` coordinates and pin the connecting endpoint
  with exactly one `addCoincident(line.endpoint, <existingPoint>)`** — never pass the existing
  `SketchPoint` into `addByTwoPoints` to *share* it. Load-bearing both ways: (a) sharing *without* a
  coincident leaves the Gear Profiles sketch **under**-constrained (the gate fails on "Gear
  Profiles"); (b) sharing **and also** coinciding is redundant and the §2 solve **fails outright**
  with `RuntimeError … VCS_SKETCH_SOLVING_FAILED - failed to create offset`.
  - ⚠️ **This covers the short "reference"/connector lines too — the ones whose BOTH endpoints
    already exist: `C→K`, `D→L` (and `C→K′`/`D→L′`), `M→C`, `N→A`, `O→D`, `P→B`, `B→I`, `A→G`.** Do
    **NOT** draw these by sharing both existing `SketchPoint`s: sharing even these already-pinned
    points tips the Gear Profiles sketch to **under-constrained** and the gate fails (observed: a
    regen that shared only these reference lines came out ~14 coincidents short). Build **every** §2
    line — lattice or reference — from raw `Point3D` coordinates, then `addCoincident` **each**
    endpoint to its existing point (one per end). No §2 line is exempt.
- **[BEVEL-F-LINE-ONCE] Each named §2 line is created ONCE; later references REUSE that line
  object — never redraw it.** The module-length extensions (A→E, B→F, E→G, F→I) and the dedendum /
  closing lines (C→H, D→J, G→H, I→J) are *named* construction lines. When a later step says "from
  point E collinear to **line A->E**" or "**lines A->E and C->E** should be perpendicular," it means
  the very line you drew earlier — so the helper that creates a module-extension must **RETURN the
  line** and you keep that reference. Do **NOT** draw a *second* line between the same two points
  just to obtain a reference: that duplicate carries its own constraints over the same segment,
  **over-determines** the coupled §2 net, and the solve fails with `RuntimeError …
  VCS_SKETCH_OVER_CONSTRAINTS - failed to create offset`. One segment ⇒ one line ⇒ its constraints
  live on that one line.
  - **Scope of the failure consequence:** the `VCS_SKETCH_OVER_CONSTRAINTS` crash is what happens
    when the duplicate carries its own geometric constraints (perpendicular/collinear/dimension)
    over the same segment. A duplicate line whose endpoints carry **only per-end coincidents** has
    been observed to solve and even pass the full-constraint gate. The rule stands unchanged — one
    segment ⇒ one line — but an over-constraint failure is NOT a guaranteed tripwire, so do not
    rely on the solver or the gate to catch a duplicate for you.
- **[BEVEL-F-DRIVEN-DIMS] The §2 driven lengths are NOT dimensioned.** The along-shaft lengths
  (Apex→A, Apex→B) and the module-length extensions are DRIVEN by the closing/collinear constraints —
  do not dimension them (`[PB-NO-OVERCONSTRAIN]`). The "do NOT add a dimensional constraint" notes
  in §2 are as load-bearing as the dimensions that ARE added.

## Orientation (keeping the figure off world XY)

- **[BEVEL-F-APEX-LOCAL] Place the Apex (and the ENTIRE §2 figure) POSITION in the gear-profiles
  sketch's own 2-D coordinates — NEVER compute a §2 POSITION from a world round-trip.** This is the
  single biggest source of orientation bugs (gear collapsing onto world XY). Why it works: the
  gear-profiles plane is built perpendicular to the target plane and contains the anchor line, so
  *inside its sketch* the direction perpendicular to the projected anchor line **is** the
  target-plane normal, and "up toward the Apex" is simply that in-plane perpendicular. So:
  - Project the center point and the anchor line into the gear-profiles sketch. Let `c` be the
    projected center and `d` the projected anchor line's 2-D unit direction.
  - In-plane perpendicular `perp = (-d.y, d.x)`. **Pick the sign by the target-plane normal, NOT by
    the sketch's local +Y** (see `[BEVEL-F-GROW-SIDE]`).
  - The Apex is the 2-D point `c + perp·DPD`. Build it as the free **end** of a construction line
    from the projected center, seeded at that 2-D point but **left undimensioned** (pinned later via
    "Constrain Point I with center"). Construct it by the COINCIDENT style
    (`[BEVEL-F-COINCIDENT-STYLE]`): pass **raw `Point3D` coordinates for BOTH endpoints** to
    `addByTwoPoints`, then pin the start with exactly one `addCoincident(centerToApex.startSketchPoint,
    projectedCenter)`. Do NOT share the projected-center `SketchPoint` and also coincident it (the
    "share **and** coincident" redundancy fails with `VCS_SKETCH_SOLVING_FAILED`).
  The single permitted world use in §2 is reading the target normal as a *direction* to choose
  `perp`'s sign — a one-bit comparison, not a position round-trip — so the figure cannot collapse to
  world XY.
- **[BEVEL-F-GROW-SIDE] Grow side (the apex/perp sign) — decide by the target NORMAL, not by the
  sketch's local +Y.** A sketch-local rule like `perp.y >= 0` is deterministic but NOT tied to a
  physical side: the sketch's local +Y maps to different world sides depending on how the
  gear-profiles plane was oriented, so the gear grows inconsistently. Pick `perp`'s sign so it points
  **toward the target-plane normal** (one-bit direction only — the apex *position* stays sketch-local
  `c + perp·DPD`, so this does NOT reintroduce the XY collapse). Consistent across all target planes;
  flip the single comparison to grow on the opposite side.

## Component activation & cleanup

- **[BEVEL-F-NEVER-ACTIVATE] Never activate any occurrence (`[PB-NEVER-ACTIVATE]`).** Bevel-specific
  reason: the Anchor Sketch is created on the user's **EXTERNAL** (root-owned) target plane — an
  activated occurrence resolves that external plane in its own local frame, so the build collapses
  onto XY regardless of the real plane tilt. All features run in the single Design component, so no
  cross-sibling reference is ever needed (`[PB-NO-CROSS-SIBLING]`). (Sole exception: the spiral
  crown's `scaleFeatures` step activates the Design occurrence and restores root — see §3a step H.)
- **[BEVEL-F-CLEANUP] Cleanup hides by entity kind.** Recursively walk the Bevel Gear component tree
  (dedupe by `entityToken`) and set `isLightBulbOn = False` on every sketch, construction plane, and
  construction axis (construction planes/axes are **not** hidden by `isVisible` — see
  `[PB-HIDE-AFTER-USE]`). There is no sketch-only mode and no per-mode guard — bevel always builds
  solids. (Realized by the framework's `solids.hide_construction_geometry(bevelComponent)` — call
  it, don't re-implement the walk.)
