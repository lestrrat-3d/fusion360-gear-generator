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
  - **Exemption — the two tooth-profile sketches, and ONLY because they are labelled.** They are
    drawn by the borrowed spur generator, whose `drawCircles` labels each of the four circles with
    along-path sketch text (`[PB-SKETCH-TEXT]`). `sketch.isFullyConstrained` counts sketch text, and
    text placed with `setAsAlongPath` carries its own position along the curve, which nothing in
    these recipes pins — so a tooth sketch whose geometry is completely determined still reads
    `False` purely because it is labelled. Measured in Fusion 2026-09-02: `False` with the four
    circle labels, `True` for the same sketch with the labels deleted and no other change. Bevel's
    own four sketches carry no text, which is why they gate normally. (This behaviour holds for any
    gear that labels a sketch, so it belongs in `PLAYBOOK.md`; it is recorded here for now because
    promoting it restamps every other gear's step list.) After `draw()`
    returns, do **NOT** raise on the tooth sketch — at most `futil.log` it if
    `not toothSketch.isFullyConstrained`.

    ⚠️ **This exemption covers the labels and nothing else. Never read it as licence for loose
    geometry.** Its earlier wording claimed the embedded tooth kept a free radial DOF because the
    flank-to-root stubs are omitted, that the residual DOF was benign since the profile is consumed
    immediately by the loft, and that fixing it would mean pinning the embedded tooth inside the
    shared generator at risk to the whole spur family. All three were wrong. Each stub is
    DOF-neutral (it adds a free root end and the two dimensions that pin it), and
    `spec/spurgear/sketch/main.go` proves the embedded scheme reaches DOF 0 without them. The
    residual DOF was the tooth-top arc's **centre**, which `addByCenterStartEnd` copies rather than
    shares, and step 5's anchor coincidence moves it a great deal: everything else is dragged onto
    K′/L′ while the stranded centre stays put. And the fix was one coincident constraint in the
    shared generator, which all eleven bench cases pass.

    That wrong reasoning let a deformed tooth ship. Measured in Fusion 2026-09-02 on a default
    31/31 pair, the pinion's tooth-top arc came out at 0.5743 mm and the driving gear's at
    17.0204 mm where both should have been the 22.5 mm tip radius, giving the pinion a 27% larger
    cross-section and a visible crown, from two sketches with byte-identical constraint counts and
    dimension values. With the fix both read 22.5 mm exactly, a centre gap of 0.000000 mm, and an
    identical 3.5475 mm² profile. Before renewing this exemption, delete the four labels and
    re-check: the geometry must read `True` on its own (it does, on both gears), and if it ever
    does not, that is a defect to fix rather than to excuse.
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
- **[BEVEL-F-COLLINEAR-CHAIN] A §2 `addCollinear` names the line the new line's start point
  actually sits ON, never a farther line up the same chain.** The module-length extensions form a
  chain along each shaft: A→E is collinear with the pinion axis Apex→A, and E→G is collinear with
  **A→E** — not with Apex→A. The same on the driving side: B→F against Apex→B, then F→I against
  **B→F**. C→H names the Pinion Dedendum Apex2→C, and D→J the Driving Dedendum Apex2→D.

  Both readings describe the same infinite line, so this looks like a free choice. It is not.
  Fusion's `addCollinear` carries **two** point-on-line rows. When the new line's start point is
  already pinned to the reference line's own endpoint — E is A→E's endpoint — one of those rows is
  already satisfied and Fusion absorbs the other. When the start point reaches the named line only
  *through an earlier collinear* — E reaches Apex→A only because A→E is collinear with it — the two
  chains assert the same fact independently and the sketch over-determines.

  Measured in Fusion, on the default pair, from `addCollinear(E→G, Apex→A)`:
  `RuntimeError: 3 : failed to create offset: VCS_SKETCH_OVER_CONSTRAINTS`, raised at the second
  such call while the first was absorbed. Writing the same constraint as `addCollinear(E→G, A→E)`
  builds. **This is not a bench-versus-Fusion divergence**: the sketch engine counts collinear the
  same two rows, which is why the proof substitutes the single point-on-line row that is not already
  implied. That substitution is also why **the proof cannot catch this** — it never models Fusion's
  collinear, so it cannot tell the two readings apart, and only a Fusion session distinguishes them.

  This is the general form of the rule `[BEVEL-F-LINE-ONCE]` already states for K and L, where §2
  uses two point-on-line `addCoincident` calls precisely because "G and C are already fixed, so an
  `addCollinear` here over-constrains". K and L were the special case; this is the rule.

  It belongs in `PLAYBOOK.md` as a cross-gear Fusion rule and is stated here instead, so that
  adopting it does not restamp every gear's step-list provenance and force a recompile per gear.
  Move it when the batched playbook PR runs.

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
