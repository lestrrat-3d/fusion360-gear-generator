# Bevel Gear Creation Instructions

## Component Setup

Bevel gears are inherently created in pairs. As such, we shall have a single component containing three components

The first two components are for the driving gear and pinion gear. Because the designs are shared between the two components, the third one will be the design component.

## Variables

User inputs are listed below in the order they appear in the command dialog. Target Plane comes first so it receives keyboard/pick focus when the dialog opens (Fusion auto-focuses the first selection input — see PLAYBOOK); Center Point follows so the user flows naturally from plane to point; Parent Component comes third since it is already pre-selected to the root component. Calculated values are listed after the inputs they depend on.

Target Plane: user-specified plane. This is where the bottom of the driving gear will sit flush against.

Center Point: user-specified point. This is where the driving bevel gear will be centered on. Does not need to be co-planar with target plane.

Parent Component: user-specified component. Defaults to the root component (pre-selected).

Module: user-supplied number. Specifies the module of gears.

Shaft Angle: User-supplied angle in degrees between 30° and 150°. Default 90° (perpendicular shafts — the classic bevel pair). The input is a `deg` Fusion expression (e.g. `60 deg`); the read-back comes back in radians, so convert to degrees before the 30–150 range check (see PLAYBOOK).

Driving Gear Teeth Number: user-specified number of teeth on the driving gear. Default is 31.

Pinion Gear Teeth Number: user-specified number of teeth on the pinion gear. Default is 31.

Driving Gear Pitch Diameter: calculated number. Module * Driving Gear Teeth Number.

Pinion Gear Pitch Diameter: calculated number. Module * Pinion Gear Teeth Number.

Driving Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified).

Pinion Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified).

Enable Bore: user-specified boolean, default `true`. Applies to both gears. When unchecked, no bore is cut on either gear and the per-gear bore diameter inputs below are ignored.

Driving Gear Bore Diameter: user-specified positive number, default 0mm. Only consulted when Enable Bore is checked. A value of 0 means "auto-calculate" — use `Driving Gear Pitch Diameter / 4` as the bore diameter.

Pinion Gear Bore Diameter: user-specified positive number, default 0mm. Only consulted when Enable Bore is checked. A value of 0 means "auto-calculate" — use `Pinion Gear Pitch Diameter / 4` as the bore diameter.

Face Width: User-specified positive number. If unspecified, default to (Cone Distance / 6). In **every** case (default or user-specified) the Face Width is bounded by the Maximum Face Width (defined below):
- If unspecified, use `min(Cone Distance / 6, Maximum Face Width)`.
- If the user specifies a value greater than the Maximum Face Width, this is an error: reject it with a message stating the maximum, rather than proceeding (the gear-body revolve in "Create the Pinion Gear" would otherwise fail — see the Maximum Face Width rationale).

Cone Distance: calculated number. `sqrt((Module * Driving Gear Teeth Number)**2 + (Module * Pinion Gear Teeth Number)**2)`.

Maximum Face Width: a geometric upper bound that cannot be evaluated until the Gear Profiles sketch points A, B, C, D, H, J exist (see §2 — apply the bound there). It is `0.95 *` the smaller of:
- the perpendicular distance from point A to the line through C and H (the Pinion Gear Dedendum line, i.e. Apex2->C extended), and
- the perpendicular distance from point B to the line through D and J (the Driving Gear Dedendum line, i.e. Apex2->D extended).

**Compute both distances from the points' SOLVED sketch geometry — `pointA.geometry`, `pointB.geometry`, `pointC.geometry`, `pointD.geometry`, `pointH.geometry`, `pointJ.geometry` (their actual positions after the §2 constraint network has solved) — NOT from the pre-solve seed coordinates you used to place them.** By the time §2 reaches this step the constraints have located every one of these points, so `.geometry` is exact. The seed values can diverge substantially from the solved positions for asymmetric tooth counts (e.g. Driving 17 / Pinion 31) or non-90° shaft angles; a seed-based bound is then too loose on the binding side and the toe still crosses the axis, defeating the whole cap. Read the solved `.geometry`.

Rationale (do not drop this when regenerating): the toe line M->N is C->H offset *toward the Apex* by Face Width, with N pinned to line A->Apex2; its mirror O->P is D->J offset toward the Apex by Face Width, with P pinned to line B->Apex2. When the offset reaches the perpendicular distance from A to line C->H, point N lands exactly on A; any larger value drives N **past** A, across the gear's own shaft axis (Apex->A). The frustum profile (hexagon A, G, H, C, M, N, built here in §2 and revolved later in "Create the Pinion Gear") is revolved about that shaft axis, so a profile that has crossed the axis self-intersects the axis of revolution and Fusion aborts the revolve (`RuntimeError ... ASM_WIRE_X_AXIS ... the profile crosses the axis of revolution`). The pinion side is normally the binding one (its smaller pitch radius gives the smaller distance), but compute both and take the minimum so the bound holds for any Shaft Angle. The `0.95` factor keeps N clearly off A, since a near-coincident N≈A degenerates the toe edge even before it strictly crosses. At Shaft Angle 90° this limit equals `Pinion Gear Pitch Diameter**2 / (2 * Cone Distance)`, so the naive `Cone Distance / 6` default exceeds it — and the gear fails to generate — for any gear ratio above roughly √2 (e.g. Driving 31 / Pinion 17).

Tooth Spacing: user-specified non-negative number in mm, default 0mm. A single value applied to **both** gears. It is a clearance offset that shifts each virtual spur tooth profile's **center** radially outward along the dedendum line — *away from the lower corner* (the rim corner opposite the Apex: point C for the pinion, point D for the driving gear), i.e. in the C->K direction beyond K (and D->L beyond L) — by this distance, **while the tooth itself is still drawn at the original virtual pitch radius** (virtual tooth number × Module / 2; the virtual tooth number is unchanged). At 0 (the default) the tooth center sits exactly at K / L and the generated geometry is byte-for-byte the prior behavior; a positive value moves the center farther from the rim, loosening the mesh so 3D-printed teeth have more clearance. Applied in §3; see "Gear Tooth Profiles".

Mean Spiral Angle (ψ): user-specified angle in degrees, default 35°, valid range **[0, 60)**. The angle between the tooth trace and the cone element, measured at the mean cone distance (see `spiral-tooth-trace.md`). **ψ = 0 means a STRAIGHT bevel gear** — the tooth-body build takes the original straight path unchanged (apex-point loft + the two conical trims) and every spiral input below is ignored; any value **> 0 builds a curved (spiral) tooth**. The driving gear uses this hand; the meshing pinion is built with the **opposite** hand (mirror) so the pair meshes. Input is a `deg` Fusion expression; read-back is radians, so convert to degrees before the [0, 60) range check (see PLAYBOOK).

Hand of Spiral: user-specified dropdown — `Right` (default) or `Left`. The **driving** gear's hand of spiral; the pinion is built with the opposite hand. Only consulted when ψ > 0. The two list-item strings `Right`/`Left` are reproduced surface (module constants `_HAND_RIGHT = 'Right'`, `_HAND_LEFT = 'Left'`).

Cutter Radius: user-specified non-negative number in mm, default 0mm. The face-mill cutter radius `r_c` that sets the radius of the tooth-trace arc (see `spiral-tooth-trace.md`). **0 means auto** — use the mean cone distance `R_mean` as `r_c`. Only consulted when ψ > 0. Reject negative values.
### Exact input ids and parameter-name strings

These literal strings are part of the reproduced surface. Use them verbatim. The dialog **display
order** (the order `configure()` adds inputs) is fixed as the rows below — Target Plane first so it
wins Fusion's auto-focus, then Center Point, then the pre-selected Parent Component, then the
numeric/bool fields. Module-level constants name the input ids (`INPUT_ID_PLANE = 'targetPlane'`, …).

| # | Dialog input | input id | input type | unit | default | selection filters |
|---|---|---|---|---|---|---|
| 1 | Target Plane | `targetPlane` | `addSelectionInput` | — | — | `ConstructionPlanes`, `PlanarFaces`; limit 1 |
| 2 | Center Point | `centerPoint` | `addSelectionInput` | — | — | `ConstructionPoints`, `SketchPoints`; limit 1 |
| 3 | Parent Component | `parentComponent` | `addSelectionInput` | — | root component pre-selected | `Occurrences`, `RootComponents`; limit 1 |
| 4 | Module | `module` | `addValueInput` | `''` | `createByReal(1)` | — |
| 5 | Shaft Angle | `shaftAngle` | `addValueInput` | `deg` | `createByString('90 deg')` | — |
| 6 | Driving Gear Teeth | `drivingTeeth` | `addValueInput` | `''` | `createByReal(31)` | — |
| 7 | Pinion Gear Teeth | `pinionTeeth` | `addValueInput` | `''` | `createByReal(31)` | — |
| 8 | Driving Gear Base Height | `drivingBaseHeight` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 9 | Pinion Gear Base Height | `pinionBaseHeight` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 10 | Enable Bore | `boreEnable` | `addBoolValueInput` (checkbox) | — | `True` | — |
| 11 | Driving Gear Bore Diameter | `drivingBore` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 12 | Pinion Gear Bore Diameter | `pinionBore` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 13 | Face Width | `faceWidth` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 14 | Tooth Spacing | `toothSpacing` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 15 | Mean Spiral Angle | `spiralAngle` | `addValueInput` | `deg` | `createByString('35 deg')` | — |
| 16 | Hand of Spiral | `spiralHand` | `addDropDownCommandInput` (text-list) | — | items `Right` (selected), `Left` | — |
| 17 | Cutter Radius | `cutterRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
There are now **17** dialog inputs and **17 `INPUT_ID_*`** module constants (the 14 above plus
`INPUT_ID_SPIRAL_ANGLE='spiralAngle'`, `INPUT_ID_HAND='spiralHand'`,
`INPUT_ID_CUTTER_RADIUS='cutterRadius'`). Inputs 15–17 are
appended **after** Tooth Spacing in display order. The Hand dropdown is a
`DropDownStyles.TextListDropDownStyle` with `Right` added selected and `Left` added unselected; read
its `selectedItem.name` (default `Right` if none). `spiralAngle` reads back in radians (range-check
[0, 60)° after converting to degrees); `cutterRadius` reads back in internal cm via `'mm'`. Still **no live
Fusion user parameters** — the spiral values are precomputed in Python like everything else.

Selection filters and limit-1 are set per PLAYBOOK; the Parent selection pre-selects
`get_design().rootComponent`. The numeric `mm`/`deg` defaults are passed in internal units
(`to_cm(...)` for lengths; `createByString('90 deg')` for the angle so the expression engine parses
it). Shaft Angle is read back in radians, so convert to degrees before range-checking 30–150° (see
PLAYBOOK).

**No live Fusion user parameters.** Unlike the spur family, bevel registers **no** user parameters
under a prefix. Every value (pitch diameters, cone distance, base heights, bore diameters, virtual
tooth counts, face width) is **precomputed in Python in internal cm** and written into geometry
numerically — sketch dimensions via `dimension.parameter.value = <number>` and feature inputs via
`ValueInput.createByReal(<number>)`. There are therefore no `PARAM_*` name strings to reproduce;
the only module-level constants are the 14 `INPUT_ID_*` strings. (See PLAYBOOK, "all-Python-
precomputed mode".)

**Reading the raw numbers.** Read each numeric/angle input by evaluating its expression with units
`''`/`'mm'`/`'deg'` as appropriate; the values come back in Fusion internal units (cm / radians)
regardless of the unit string (see PLAYBOOK).

**Units — critical (Module is unitless mm; everything else is already internal):**
- The `'mm'` inputs (Driving/Pinion Base Height, Driving/Pinion Bore Diameter, Face Width, Tooth
  Spacing) and the
  `'deg'` input (Shaft Angle) come back from `evaluateExpression(expr, 'mm'|'deg')` **already in
  internal units** (cm / radians). Use them as-is; do **not** `to_cm` them again.
- **`Module` is read with unit `''`, so it comes back as a raw number that means *millimetres*** (a
  module of `1` is 1 mm). Therefore **every length derived from Module must be `to_cm`-converted
  before it touches geometry**: Pitch Diameter = `to_cm(Module * teeth)`, Cone Distance = `to_cm(...)`,
  dedendum = `to_cm(1.25 * Module)`, the module-length construction extensions (E, F, G, H, I, J),
  and the default Face Width (`Cone Distance / 6`). The `_VirtualSpurProxy` likewise receives Module
  in mm and applies the standard spur formulas, then `to_cm`'s the resulting circle radii/diameters
  it serves (they must be in cm, matching what the spur tooth generator expects). Mixing a raw-mm
  Module-derived length with an already-cm `'mm'` input (e.g. comparing Face Width against the
  Module-derived Maximum Face Width) without this conversion makes the gear come out ~10× off and
  the Face-Width bound meaningless. The boolean Enable-Bore input is read with
`get_boolean` (or `input.value`). Read all inputs up front in a single `_readInputs` pass that
validates ranges (teeth ≥ 3, shaft angle 30–150°, non-negative heights/bores/width/tooth-spacing) and returns the
primary values, stashing the rest on `self`.

## Architecture

Bevel uses a **standalone generator** — it does **not** subclass `base.Generator` and uses **no
`GenerationContext`**. **One** generator handles **both** straight and spiral bevels: the same class
builds a straight bevel when Mean Spiral Angle ψ = 0 and a curved (spiral) bevel when ψ > 0. There is
**no separate spiral subclass or command** — the spiral is a branch inside the tooth-body step
(`_transformToothBody`, see Method contract), gated on ψ. Three plain classes (names are the
reproduced surface; the entry point binds the first and second by name):

1. **`BevelGearCommandInputsConfigurator`** — `@classmethod def configure(cls, cmd)` that adds the
   **17** dialog inputs above in display order.
2. **`BevelGearGenerator`** — `__init__(self, design)` (stores `self.design`, `self.bevelOccurrence
   = None`); `generate(inputs)`; `deleteComponent()`. Creates the occurrence tree directly with
   `parent.occurrences.addNewComponent(...)` — it does **not** use `getOccurrence` / `addParameter`
   / `parameterName` / `createSketchObject` from `base.Generator`, and registers no user parameters.
3. **`_VirtualSpurProxy`** (with a tiny `_Val` value-wrapper) — a fake spur `parent` so the borrowed
   spur tooth generator can run without registering Fusion user parameters. See Dependencies.

`base.py` is star-imported but its `Generator`/`ParamNamePrefix`/`ComponentCleaner` machinery is
unused.

## Generation Context — none

Bevel carries **no `GenerationContext` object.** State is threaded three ways: (a) `_readInputs`
returns a 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
shaftAngle_deg)` and stashes the rest as instance attributes (`self._drivingBaseHeight_cm`,
`self._pinionBaseHeight_cm`, `self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`,
`self._faceWidth_cm`, `self._toothSpacing_cm`); (b) the ~30 geometric anchors (Apex, Apex2, points A–P, K, L, the two shaft
axes, the cone-cutter faces, the two tooth sketches, the per-gear bodies) are passed as locals /
return values between `_buildGearProfiles` and the per-gear helpers; (c) `self.bevelOccurrence`
holds the top occurrence for cleanup. A regenerated bevel must not introduce a `ctx` object.

## Method contract — call graph

No gear subclasses bevel, so there are **no override boundaries to preserve** (unlike spur). The
only hard external bindings are `commands/bevelgear/entry.py` → `BevelGearCommandInputsConfigurator.
configure(args.command)` and `BevelGearGenerator(design).generate(inputs)` + `deleteComponent()` on
failure. The internal decomposition below is the intended structure; private helper names may vary,
but keep the step boundaries (they map to the Instructions sections):

```
generate(inputs)
  → _readInputs(inputs)                      # read+validate all inputs; returns 7-tuple, stashes the rest on self
  → resolve pitch diameters & bores (Python, cm)
  → build component tree                     # Bevel Gear → Design (Pinion/Driving components made in _createGearBody)
  → _buildAnchorSketch(design, plane, center)        # §1 → anchorLine
  → _buildGearProfiles(...)                  # §2 + §3 + per-gear body creation; internally:
        → _buildVirtualSpurProfile(...)  ×2  # §3 pinion then driving: tooth plane + spur tooth + axis
        → _createGearBody(...)           ×2  # revolve → loft (uncut tooth) → _transformToothBody → pattern → combine → bore → mesh-rotate → moveToComponent
              → _transformToothBody(...)     # the tooth-body step. ψ=0 → _cutConicalEnds (2 conical trims, straight bevel). ψ>0 → spiral build (see §3 "Spiral tooth body")
              # mesh-rotate: driving by 180°/drivingTeeth; pinion by _pinionMeshPhase() (0 unless a spiral pair needs a nudge)
  → _hideConstructionGeometry(bevelComponent)        # Cleanup
deleteComponent()                            # error rollback (entry point calls on exception)
```

`_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld, apexSketchPoint,
toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane, gearLabel, teethNumber)` is the single
tooth-body hook `_createGearBody` calls after lofting the uncut apex→heel tooth. Its first line is the
gate `if self._spiralAngle_rad <= 0: return self._cutConicalEnds(...)` — straight bevels are byte-for-
byte the prior behavior. `_pinionMeshPhase()` returns the pinion's extra mesh rotation about its own
shaft axis (0 for straight; for spiral it is `_PINION_MESH_PHASE_TEETH` tooth-fractions, default 0).

Helpers used by the above (names may vary; behaviour is pinned by the Instructions): `_readInputs`,
`_pointWorldGeometry` (world Point3D for a SketchPoint via `.worldGeometry` / ConstructionPoint via
`.geometry`), `_findSpurToothProfile` (match the tooth cross-section loop by curve-**type** mix, not count alone, or an unrelated loop can match: **2 NURBS flanks + 2 arcs (tip/root) + 2 lines** for a non-embedded tooth, or **2 NURBS + 2 arcs + 0 lines** when embedded),
`_applyConicalCut` (sequential split by cone faces; keep the largest non-apex piece),
`_findConeFaceForCutLine` (the `ConeSurfaceType` face whose surface contains both world endpoints of
a cut line), `_surfaceDistance`, `_cutBore`, `_cutConicalEnds` (the toe-then-heel two-cone trim that
trims a tooth body to a flush band; used both for the straight tooth and to flush the spiral tooth's
ends). **Spiral-only helpers** (used only when ψ > 0): `_combine` (world point = base + a·e1 (+ b·e2),
lengths in cm), `_planeByAngle` (a construction plane at N° to a reference plane through a sketch
line, via `ConstructionPlaneInput.setByAngle`), `_circleIntersectNearest` (the two-circle intersection
nearest a reference point — solves the cutter arc's toe/heel ends), `_bottomEdgeMid`, `_perpToAxis`,
`_distDim`, and `_pinionMeshPhase`. Pinion is built first; driving second (mesh offset
`180° / drivingTeeth`); the pinion additionally gets `_pinionMeshPhase()` (0 unless a spiral pair
needs it).

## Sketch Discipline (bevel-specific)

Bevel's sketch work differs from the spur family — note these deltas (the general Fusion gotchas in
PLAYBOOK still apply):

- **Every sketch this generator authors MUST end FULLY CONSTRAINED.** Gate every sketch this generator authors with the PLAYBOOK full-constraint check (raise, naming the sketch), called at the END of each sketch-building step. This applies to **the Anchor Sketch, the Gear Profiles (§2) sketch, both per-gear Profile sketches, both tooth-profile sketches, and the Bore sketch** — i.e. every sketch the bevel build produces. A free DOF is a **generation defect, not a warning**, so raise rather than warn. **Do NOT** reach full constraint by dimensioning the *driven* §2 lines (Apex→A/B, the module-length extensions) — those are determined by the perpendicular/collinear/closing constraints (see PLAYBOOK on over-constraining). Once the Anchor Line direction is fixed, the §2 lattice is fully determined by its existing net; the per-gear Profile sketches are made fully constrained by recreating their six vertices as **fixed points** per the playbook's recreate-share-fix recipe, not by projecting §2 points. (The tooth-profile sketches are drawn by the borrowed spur generator, but they **too** must pass the gate — call it on each tooth sketch right after the spur generator's `draw()` returns. The spur generator fully constrains the `angle != 0` tooth, including pinning the horizontal reference line's far end to the tip circle — see `spurgear.md`. If a tooth sketch reports under-constrained, fix the cause in `spurgear.md` and regenerate spur — not here.)
- **The spiral build's auxiliary sketches are EXEMPT from the full-constraint gate.** When ψ > 0 the spiral tooth build (§3 "Spiral tooth body") authors several **transient construction sketches** — the `{gear} 2D Tooth Trace` (the cutter arc) and the `{gear} Cone Element` line / `{gear} Trace Plane` it sits on. (There is **no** 3-D projection, root-cone, or twist-angle sketch — the spiral twist is computed analytically in §3a step G, not measured off a projected curve.) The cutter arc is a genuine arc constrained by a radius dimension plus a center-coincidence but is **deliberately left with free DOF** (its toe/heel endpoints are pinned by 3-point construction, not dimensioned). These sketches are consumed by the build and hidden in cleanup, so do **NOT** call the full-constraint gate on them. The gate above applies **only to the bevel's own permanent sketches** (Anchor, Gear Profiles, the two Profile sketches, the two tooth-profile sketches, Bore) — those must still be fully constrained for both straight and spiral builds.
- **Constraint-network construction — §2 lattice lines use the COINCIDENT style, not sharing.** §2
  builds the A–P / K / L / Apex / Apex2 lattice with coincident / perpendicular / collinear /
  angular / dimensional constraints. When a line must *start at* (or connect to) an already-existing
  point (Apex, A, B, C, **the projected center**, …), **create the line from raw `Point3D` coordinates
  for the connecting endpoint and pin it with exactly one `addCoincident(line.endpoint,
  <existingPoint>)`** — do **NOT** instead pass the existing `SketchPoint` object into `addByTwoPoints`
  to *share* it. This is the playbook's "share OR coincident, **never both**" rule, and for the §2
  lattice it is **load-bearing in two directions**: (a) sharing the point *without* a coincident
  leaves the Gear Profiles sketch **under**-constrained (free DOF → the full-constraint gate fails on
  "Gear Profiles"); (b) sharing the point **and also** adding the coincident **over**-constrains it —
  the duplicate fix is a redundant constraint that makes the §2 solve **fail outright** with
  `RuntimeError … VCS_SKETCH_SOLVING_FAILED - failed to create offset`. The correct construction is
  always raw-`Point3D` endpoints + a **single** `addCoincident` per existing-point connection. (This
  is the one place the generic playbook rule is too permissive — §2 must use coincident-style.)
- **Each named §2 line is created ONCE and later references REUSE that same line object — never redraw it.** The module-length extensions (A→E, B→F, E→G, F→I) and the dedendum / closing lines (C→H, D→J, G→H, I→J) are *named* construction lines. When a later step says, e.g., "from point E collinear to **line A->E**" (point G) or "**lines A->E and C->E** should be perpendicular", it means **the very line you drew in the point-E step** — so the helper that creates a module-extension must **RETURN the line** (not just its endpoint) and you must keep that reference. Do **NOT** draw a *second* line between the same two points (e.g. a fresh `addByTwoPoints(A, E)` / `_lineBetween(A, E)`) just to obtain a reference for a perpendicular/collinear: that duplicate line carries its own constraints over the same segment, **over-determines** the coupled §2 net, and the solve fails with `RuntimeError … VCS_SKETCH_OVER_CONSTRAINTS - failed to create offset` (often several lines later, when the redundancy finally exceeds the DOF). One segment ⇒ one line ⇒ its constraints live on that one line.
- **Place the Apex (and the ENTIRE §2 figure) POSITION in the gear-profiles sketch's own 2-D coordinates — NEVER compute a §2 POSITION from a world round-trip.** This is the single biggest source of orientation bugs (gear collapsing onto world XY). The reasoning, which is exactly why this works: the gear-profiles plane is built perpendicular to the target plane and contains the anchor line; therefore, *inside its sketch*, the direction perpendicular to the (projected) anchor line **is** the target-plane normal, and "up toward the Apex" is simply that in-plane perpendicular. So:
  - Project the center point and the anchor line into the gear-profiles sketch. Let `c` be the projected center and `d` the projected anchor line's 2-D unit direction.
  - In-plane perpendicular: `perp = (-d.y, d.x)` (one of the two senses points to each side of the target plane). **Pick the sign by the target-plane normal, NOT by the sketch's local +Y** (see the grow-side note below).
  - The Apex is the 2-D point `c + perp·DPD`. Build it as the free **end** of a construction line from the projected center, seeded at that 2-D point but **left undimensioned** (pinned later via "Constrain Point I with center"). Do not create the Apex as a standalone point. **Construct this line by the COINCIDENT style (it connects to the already-existing projected center):** pass **raw `Point3D` coordinates for BOTH endpoints** to `addByTwoPoints` (the projected center's `.geometry` for the start, the seed apex 2-D point for the end), then pin the start with **exactly one** `addCoincident(centerToApex.startSketchPoint, projectedCenter)`. **Do NOT pass the projected-center `SketchPoint` itself into `addByTwoPoints`** (which *shares* it) and *also* add a coincident — that "share **and** coincident" on the same point is a redundant constraint that makes the §2 solve **fail outright** with `RuntimeError … VCS_SKETCH_SOLVING_FAILED - failed to create offset`. The projected center is subject to the coincident-style rule exactly like A/B/C — see the §2 lattice bullet below.
  The single permitted world use in §2 is reading the target normal as a *direction* to choose `perp`'s sign — a one-bit direction comparison, not a position round-trip, so it cannot collapse the figure. Because every §2 coordinate is otherwise sketch-local on an already-correctly-oriented plane, the figure **cannot** collapse to world XY.
- **The dimensional constraints in §2 are load-bearing, and the "do NOT add a dimensional
  constraint" notes equally so:** the along-shaft lengths (Apex→A, Apex→B) and the module-length
  extensions are DRIVEN by the closing/collinear constraints — do not dimension them (see PLAYBOOK).
- **Grow side (the apex/perp sign) — decide by the target NORMAL, not by the sketch's local +Y.**
  Choose which side the gear grows by the SIGN of the target-plane normal, used as one bit (grow
  toward the normal), **not** by the sketch's local +Y. A sketch-local rule like `perp.y >= 0` is
  deterministic as code but NOT tied to a physical side: the sketch's local +Y maps to different
  world sides depending on how the gear-profiles plane was oriented for the chosen target plane, so
  the gear grows on an inconsistent side. Pick `perp`'s sign so it points **toward the target-plane
  normal**, used only as a one-bit direction — the apex *position* stays sketch-local (`c + perp·DPD`),
  so this does NOT reintroduce the XY-collapse (that came from a world *position* round-trip, not
  from reading a direction). This convention is consistent across all target planes; flip the single
  comparison if a user ever needs the opposite side.
- **Never activate any occurrence (see PLAYBOOK).** All sketches/features run in the Design
  component via that component's own collections without activating it. The bevel-relevant reason:
  the Anchor Sketch is created on the user's **EXTERNAL** (root-owned) target plane, which is exactly
  why activating would collapse the build onto XY — an activated occurrence resolves that external
  plane in its own local frame, so the anchor line and the Gear Profiles plane come out XY-aligned
  regardless of the real plane tilt. All features run in the single Design component, so no
  cross-sibling reference is ever needed during construction.
- **Cleanup hides by entity kind.** Recursively walk the Bevel Gear component tree (dedupe by
  `entityToken`) and set `isLightBulbOn = False` on every sketch, construction plane, and
  construction axis (construction planes/axes are **not** hidden by `isVisible`). There is no
  sketch-only mode and no per-mode guard — bevel always builds solids.

## Generation Order

`generate()` reads **all** inputs first (`_readInputs`), then creates occurrences. Because bevel
registers no user parameters, nothing creates an occurrence until after every selection is already
read, so the selection-context-shift hazard the spur spec warns about does not bite here — but keep
the order (read inputs → build tree → build geometry) so it stays that way. The geometry steps run
in the order of the Instructions below (§1 → §2 → §3 → Pinion → Driving → Cleanup), pinion before
driving. **When ψ > 0**, the per-gear tooth-body step (inside `_createGearBody`, via
`_transformToothBody`) builds the spiral tooth — trace → slice → rotate → loft → crown → flush trim —
*in place of* the straight tooth's two conical trims, before pattern/combine/bore (see §3 "Spiral
tooth body"). The straight (ψ = 0) order and behavior are unchanged.

## Dependencies

Bevel **borrows the spur tooth generator** — `from .spurgear import
SpurGearInvoluteToothDesignGenerator`. It is used only inside `_buildVirtualSpurProfile`, once per
gear:

```python
proxy  = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))   # the 180° tooth rotation IS the draw() angle
```

The borrowed generator's surface (from `spurgear.md`): constructor `(sketch, parent, angle=0)`;
`draw(anchorPoint, angle=0)` runs `drawCircles()` → `drawTooth(angle)` → anchor-projection; it reads
parameters via `parent.getParameter(name).value`. So bevel must supply a `parent` exposing
`getParameter(name)` → an object with a `.value`. That is **`_VirtualSpurProxy`**: it precomputes,
in internal cm, exactly the keys the spur drawer reads and returns each wrapped in `_Val`:
`Module`, `ToothNumber`, `PressureAngle` (radians; bevel hardcodes 20° — pressure angle is not a
bevel dialog input), `PitchCircleDiameter`, `PitchCircleRadius`, `BaseCircleDiameter`,
`BaseCircleRadius`, `RootCircleDiameter`, `RootCircleRadius`, `TipCircleDiameter`, `TipCircleRadius`,
`InvoluteSteps` (15). Standard spur formulas: pitch = teeth·module, base = pitch·cos α,
root = pitch − 2.5·module, tip = pitch + 2·module.

The **180° rotation is delivered through the `draw()` angle argument** (the spur generator rotates
the whole tooth by `angle`, per `spurgear.md`), *not* a post-hoc Move/sketch rotation — this relies
on spur's radial flank-to-root pinning so the connecting lines rotate with the tooth. (The
construction axis through point K / L, built normal to the tooth plane via `setByTwoPlanes`, is
still created as described in §3.)

## Instructions

### Create the Parent Component

Create the Bevel Gear component as a child of the Parent Component.

## Creating the Design Component

The Design Component shall contain the necessary sketches and construction planes / axis / etc that will be used by the components creating the actual gears later.

Create the Design component as a child of the Bevel Gear component.

### 1: Anchor Sketch

Start the Anchor Sketch **directly on the user-selected target plane**, whether the selection is a `ConstructionPlane` or a `PlanarFace`; don't re-derive or offset it (see PLAYBOOK — re-deriving it inside the Design sub-component loses the plane's world orientation and collapses the gear onto XY). Mark the center point by projecting the user-specified center point onto the sketch.

Create a line through the projected center point. Apply **BOTH** `addCoincident(projectedCenter, anchorLine)` (the "intersection" — pins the center onto the line) **and** `addMidPoint(projectedCenter, anchorLine)` (center bisects the line). Use both, not midpoint alone. Give it a ~10mm length dimension (the value is arbitrary; it's only a reference line). **Then pin its direction so the sketch ends FULLY CONSTRAINED:** add `addHorizontal(anchorLine)` (sketch-local — fixes the line to the Anchor Sketch's own X axis). Use the sketch-local Horizontal constraint, **not** a world-axis lock — Horizontal is defined in the sketch's own frame, so it works on any tilted target plane, whereas pinning to a world axis would mis-orient on a tilt. The anchor line's absolute direction is arbitrary (nothing downstream depends on it — §2 derives all directions *relative* to the projected anchor line), but it must not be a free degree of freedom: with midpoint + length + Horizontal the line has zero DOF. **Stash the projected-center SketchPoint** (e.g. on `self`) so §2 re-projects *this* anchor-sketch point — not the raw user-selected center — into the Gear Profiles sketch. This line is the Anchor Line. After all constraints, the Anchor Sketch must report `isFullyConstrained` (see the full-constraint gate in Sketch Discipline).

### 2: Gear Profiles

Using setByAngle, create a plane that includes the Anchor Line, set at 90° (by default it would lie flush to the anchor line's plane, but we want it perpendicular). **Build it off the original `targetPlane`** as the reference — don't re-derive/offset it (see PLAYBOOK) — so the Gear Profiles plane inherits the true target-plane orientation; this is the other place the target-plane orientation reaches the bodies, and substituting a different plane here also collapses the gear onto XY. Create a sketch on this plane.

In the sketch, project **the Anchor Sketch's center SketchPoint** (the one you stashed in §1) — NOT the raw user-selected center point. Both happen to be coincident, but projecting the anchor-sketch point keeps the chain within the Design component and faithful to the anchor geometry; projecting the raw external point is a cross-component reference and can resolve inconsistently.

From the projected center point, draw a construction line **perpendicular to the (projected) anchor line, in the gear-profiles sketch's own 2-D frame** (apply a Perpendicular constraint to the anchor line). Its far end is the **Apex**, placed in sketch-local coordinates at `c + perp·(Driving Gear Pitch Diameter)` where `c` is the projected center and `perp` is the in-plane unit vector perpendicular to the projected anchor line (`(-d.y, d.x)` for anchor-line direction `d`). The apex **position** is sketch-local — do **NOT** compute it from a world-coordinate round-trip; that is what caused the XY-collapse. The **sign of `perp`** (which side the gear grows) is chosen by the target-plane normal as a one-bit direction (see Sketch Discipline "Grow side" — toward the normal), NOT by the sketch's local +Y. Do NOT add a length constraint on this line.

Create a construction line from the apex representing the Driving Gear Shaft Axis, pointing from the apex back toward the anchor line (seed its far end at `c - perp·(some length)`, i.e. the `-perp` direction). It must run **parallel to the center→apex construction line** — apply `addParallel(drivingShaftAxis, centerToApex)`. **Do NOT use `addVertical`**: `addVertical` forces the line to the sketch's world-vertical, which is wrong on a tilted target plane (the gear-profiles sketch is not world-aligned) and over-constrains/mis-orients the figure. The shaft axis must be parallel to the *in-plane* apex direction (`perp`), expressed via `addParallel` to the center→apex line — never an absolute Horizontal/Vertical constraint. Beginning of this line uses a coincidence constraint with the apex. The end of this line shall be called point B. Do **not** dimension the line's length — it is determined by the closing constraint at Apex 2 below.

Create a construction line from the apex representing the Pinion Gear Shaft Axis. Apply an angular dimension between this line and the Driving Gear Shaft Axis equal to Shaft Angle (this is the traditional "angle between the two shaft axes" — Shaft Angle = 90° gives the classic perpendicular bevel pair). **Place the angular dimension so it measures Σ and not its supplement 180−Σ** (see PLAYBOOK). The pinion shaft is the Driving Gear Shaft Axis direction rotated about the apex by the Shaft Angle. Rotating by the Shaft Angle has two senses (one to each side of the driving shaft), and they place point A on opposite sides — choosing wrong mirrors the whole gear onto the wrong side of the target plane. **Select the sense this way: form both candidate point-A positions (the driving-shaft direction rotated about the apex by +Shaft Angle and by −Shaft Angle) and keep the candidate whose endpoint has the greater X coordinate in the Gear Profiles sketch.** Compare the two candidates' X and take the larger — do **not** rotate one fixed sense and only flip it when its X comes out negative; when *both* candidates have a positive X that shortcut keeps the wrong one (this is exactly the side-flip to avoid). The +X-most endpoint is the side away from the anchor sketch's leading direction and is consistent across all target planes. Beginning of this line should use coincidence constraint with apex. The end of this line shall be called point A. Do **not** dimension the line's length — it is determined by the closing constraint at Apex 2 below.

From A, create a construction line perpendicular to the Pinion Gear Shaft Axis, drawn toward the side where Apex 2 will lie (between the two shaft axes, in the direction of the anchor line). Apply a perpendicular constraint against the Pinion Gear Shaft Axis. Apply a dimensional constraint with length = Pinion Gear Pitch Diameter / 2 (this equals the pinion's pitch radius at the heel, which is the perpendicular distance from Apex 2 to the Pinion Gear Shaft Axis for any Shaft Angle). Beginning of this line should use coincidence constraint with A. **Naming convention used throughout this spec: this perpendicular drop line (A → its far end, which becomes Apex 2) is what "A->Apex2" / "line A->Apex2" always refers to — it is NOT the Apex->A shaft axis. The two share point A but are different lines (one is the PPD/2 perpendicular drop, the other the shaft axis). Whenever a later step says to pin something to or dimension against "A->Apex2", it means this drop line. The same holds for "B->Apex2" (the DPD/2 drop) vs the Apex->B shaft axis.**

From B, create a construction line perpendicular to the Driving Gear Shaft Axis, drawn toward the side where Apex 2 will lie. Apply a perpendicular constraint against the Driving Gear Shaft Axis. Apply a dimensional constraint with length = Driving Gear Pitch Diameter / 2 (the driving pitch radius at the heel, perpendicular distance from Apex 2 to the Driving Gear Shaft Axis for any Shaft Angle). Beginning of this line should use coincidence constraint with B.

Constrain the end points of the two perpendicular lines from the previous two paragraphs with a coincident constraint. Let this point be called Apex 2. (At Shaft Angle = 90° the four points Apex, A, Apex 2, B form a rectangle. For other shaft angles the figure is a non-rectangular parallelogram-like quadrilateral; the lengths of Apex→A and Apex→B adjust so the perpendicular drops of length PPD/2 and DPD/2 coincide at Apex 2.)

Note that this quadrilateral deliberately lies well above the anchor line. The Apex's upward offset from the anchor line (= Driving Gear Pitch Diameter) is chosen to keep the whole figure above the anchor line for Shaft Angle in the supported range 30°–150°.

**Seed the along-shaft lengths (Apex→A, Apex→B) with the closed-form cone geometry** so the solver converges on the right branch for any Shaft Angle Σ (these are seed coordinates only — the lengths stay undimensioned, fixed by the Apex2 closing constraint): `tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`, `γ_g = Σ − γ_p`, cone distance `R = (PPD/2) / sin γ_p`; seed `|Apex→A| = R · cos γ_p` and `|Apex→B| = R · cos γ_g`. (γ_p, γ_g are also reused in §3 for the virtual tooth radii.) Seeding A/B merely by a pitch diameter is wrong for Σ≠90° and can send the solver to the wrong branch.

Draw a construction line from Apex to Apex 2. This line shall be called the Pitch Line. Each end of the Pitch Line should be constrained to the respective points using coincidence constraint.

From the Apex 2, create a construction line in either side whose length is constrained Module * 1.25, and are perpendicular to the Pitch Line. Constrain them against the Pitch Line with the Perpendicular constraint. Let the line drawn towards the anchor line be Driving Gear Dedendum, whose end point shall be point D. The one drawn away from the anchor line be Pinion Gear Dedendum; whose end point shall be point C.

Draw two construction lines, from the Apex to the point D and point C, respectively. Apply coincidence constraints on beginning and end of these lines. These lines shall be the Root Axis for driving and pinion gear, respectively.

From point A, create construction line collinear with the line from Apex to point A, extending for length equals to module (but do NOT add dimensional constraint). The line should receive a collinear constraint, and the end of Apex->A and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point E.

Draw a construction line from point C to point E. constrain each end to respective points from pre-existing lines. The lines A->E and C->E should be constrained with perpendicular constraint.

From point B, create a construction line collinear with Apex->B, extending for length equals to module (but do NOT add dimensional constraint). The line should receive a collinear constraint, and the end of Apex->B and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point F.

Draw a construction line from point D to point F. constrain each end to respective points from pre-existing lines. The lines B->F and D->F should be constrained with perpendicular constraint.

Draw a construction line from point E collinear to line A->E, with length equal to module (but do NOT add dimensional constraint). Constrain point E and the beginning of this line. Let the end be known as point G.

From point C, draw a line with length equal to module (but do NOT add dimensional constraint). Constrain point C and the beginning of this line. Let the end of the new line be point H. Line C->H should be collinear with line Apex2->C.

Connect point G and H with a line. Constrain end points of line accordingly with coincidence constraints. Constrain line E->G and H->G with a perpendicular constraint.


Draw a construction line from point F collinear to line B->F, with length equal to module (but do NOT add dimensional constraint). Constrain point F and the beginning of this line. Let the end be known as point I.

From point D, draw a line with length equal to module (but do NOT add dimensional constraint). Constrain point D and the beginning of this line. Let the end of the new line be point J. Line D->J should be collinear with line Apex2->D.

Connect point I and J with a line. Constrain end points of line accordingly with coincidence constraints. Constrain line F->I and J->I with a perpendicular constraint.

Create an **offset dimension between the B->Apex2 perpendicular drop line and J->I**. **"B->Apex2" here is the perpendicular construction line of length DPD/2 you drew earlier "From B, perpendicular to the Driving Gear Shaft Axis" — NOT the Apex->B shaft axis.** This matters because J->I is **already parallel** to that perpendicular drop by construction (J->I ⊥ F->I, which runs along the driving shaft, so J->I is perpendicular-to-shaft = parallel to the B->Apex2 drop), whereas the Apex->B shaft axis is not — the offset dimension needs the drop, and being already-parallel it takes no extra parallel constraint (see PLAYBOOK). Set the value equal to Driving Gear Base Height _if_ specified (non-0); otherwise `module * Driving Gear Teeth Number / 8`.

Create an **offset dimension between the A->Apex2 perpendicular drop line and G->H** — again "A->Apex2" is the perpendicular construction line of length PPD/2 (drawn "From A, perpendicular to the Pinion Gear Shaft Axis"), **not** the Apex->A shaft axis; G->H is **already parallel** to it by construction (G->H ⊥ E->G which runs along the pinion shaft), the shaft axis is not. As with the driving side, use the perpendicular drop and add no parallel constraint (already parallel; see PLAYBOOK). The value should be equal to Pinion Gear Base Height _if_ specified (non-0); otherwise `Driving Gear Base Height * (Pinion Gear Teeth Number / Driving Gear Teeth Number)`.

Draw a line from A to G. Constrain endpoints appropriately.

Constrain Point I with center point.


Draw a construction line away from Apex, starting from point G, extending along Apex->A, and call its end point K. Then **pin K with two point-on-line coincident constraints** — `addCoincident(K, line Apex->A)` and `addCoincident(K, the Pinion Dedendum line Apex2->C extended)` — rather than `addCollinear` on the connecting lines. By the time K is added, G and C are already fixed, so an `addCollinear` here over-constrains the sketch and Fusion errors; the two point-on-line coincidents locate K exactly (intersection of the two lines) without over-constraining. Draw a construction line from point C to K for reference.

**Tooth-center point K′ (Tooth Spacing offset).** The §3 spur tooth is centered not at K but at a tooth-center point **K′**, obtained by shifting K outward along the dedendum line by **Tooth Spacing**, *away from the lower corner C*. **When Tooth Spacing is 0 (the default), do NOT build anything here — set K′ ≡ K and reuse the C->K reference line**, so the output stays byte-identical to the no-spacing build (a zero-length dimensioned line would be degenerate). When Tooth Spacing > 0: draw a construction line starting at K with its far end seeded on the *far side of K from C* along the dedendum direction; pin its far end **the same way K is pinned to its line** — `addCoincident(start, K)` and `addCoincident(K′, the Pinion Dedendum line Apex2->C extended)` to keep K′ on the dedendum line — then add a **length dimension on this line = Tooth Spacing** (do **not** use `addCollinear`, for the same over-constraint reason as K). The far end is K′. Build it **here, inside the Gear Profiles sketch, before that sketch's end-of-step full-constraint gate**, so the gate covers it. Finally draw the **tooth-center reference line C->K′** (from the lower corner C to K′) for §3 to use in place of C->K. Only the tooth's center moves; the virtual tooth number and drawn tooth size are unchanged (see §3).

At this point all of A, B, C, D, H, J exist **and are solved**, so resolve the **Maximum Face Width** (see the Parameters section) from their solved `.geometry` (NOT the seed coordinates — see that section) and apply it before using Face Width below: cap the auto default to it, and reject a user value that exceeds it. Skipping this — or computing it from seeds — makes the M->N / O->P line push N/P across the shaft axis for asymmetric tooth counts (either gear can be the smaller, binding side), which fails the gear-body revolve with `ASM_WIRE_X_AXIS`.

Create line M->N. **Seed it near its solved position** (the solver is seed-sensitive — see PLAYBOOK): seed M at roughly the **midpoint of Apex->C**, and seed N by sliding from that M-seed **along the C->H direction far enough to roughly reach line A->Apex2** (e.g. by the distance from the M-seed to A). Do NOT seed M/N just `Face Width` away from C/H — that starts N near H, far from its constraint target (line A->Apex2), and the solve fails to converge. Then apply **exactly these constraints** — all four are required, and the two coincident pins are what make the Maximum-Face-Width guarantee real (N reaches A exactly at the cap):
- `addCoincident(M, Pinion Root Axis)` — M lies on the Apex->C root axis;
- `addCoincident(N, line A->Apex2)` — N lies on the A->Apex2 line (this is the pin; do not merely place N numerically). **"A->Apex2" is the perpendicular DROP line of length PPD/2 you drew "From A, perpendicular to the Pinion Gear Shaft Axis" — NOT the Apex->A shaft axis.** This distinction is load-bearing: pinning N to the shaft axis puts N *on the axis of revolution* (radius 0), so the revolved M->N edge becomes a cone whose vertex sits on the axis; the later conical split then fails with `ASM_API_FAILED` for asymmetric tooth counts (narrow cone half-angle 90°−γ_p), even though it happens to survive the symmetric 45° case. Pin N to the **A->Apex2 perpendicular drop**, never the shaft axis;
- `addParallel(M->N, C->H)` — the toe line is parallel to C->H;
- `addOffsetDimension(C->H, M->N, textPoint).parameter.value = Face Width` — the parallel offset between C->H and M->N equals Face Width.

(Because N is pinned to A->Apex2, the Maximum Face Width above is exactly the value at which N reaches A; the capped Face Width keeps N between Apex2 and A.)

Let the beginning of this new line be point M, the end be point N. Draw a line from M to C. Draw a line from N to A.

Draw a construction line away from Apex, starting from point I, extending along Apex->B, and call its end point L. **Pin L the same way as K** — `addCoincident(L, line Apex->B)` and `addCoincident(L, the Driving Dedendum line Apex2->D extended)`; do not use `addCollinear`. Draw a construction line from point D to L for reference.

**Tooth-center point L′ (Tooth Spacing offset).** Build the driving-side tooth center **L′** exactly as K′ on the pinion side: when Tooth Spacing is 0, set L′ ≡ L and reuse the D->L reference line; when Tooth Spacing > 0, draw a construction line from L seeded on the far side of L from D along the driving dedendum direction, pin its far end with `addCoincident(start, L)` and `addCoincident(L′, the Driving Dedendum line Apex2->D extended)`, add a length dimension on it = Tooth Spacing (no `addCollinear`), and draw the tooth-center reference line **D->L′** for §3. Same single Tooth Spacing value as the pinion; same full-constraint and "byte-identical at 0" guarantees.

Create line O->P, the mirror of M->N on the driving side. Seed it the same way (O near the midpoint of Apex->D, P slid along D->J toward line B->Apex2), then apply the same four constraints:
- `addCoincident(O, Driving Root Axis)` — O on the Apex->D root axis;
- `addCoincident(P, line B->Apex2)` — P on the B->Apex2 line, i.e. the perpendicular DROP line of length DPD/2 drawn "From B, perpendicular to the Driving Gear Shaft Axis" — **NOT the Apex->B shaft axis** (same trap as N above; pinning P to the shaft axis puts it on the axis of revolution and breaks the cut);
- `addParallel(O->P, D->J)`;
- `addOffsetDimension(D->J, O->P, textPoint).parameter.value = Face Width`.

Let the beginning of this new line be point O, the end be point P. Draw a line from O to D. Draw a line from P to B. Draw line from B to I.

### 3: Gear Tooth Profiles

**API note for this whole section:** pass the relevant **sketch line directly** to `setByAngle` and
`setByDistanceOnPath`; never wrap it in `Path.create` first (see PLAYBOOK).

**Throughout this section the tooth center is the §2 tooth-center point K′ (pinion) / L′ (driving) and the center reference line is C->K′ / D->L′ — which equal K / L and C->K / D->L exactly when Tooth Spacing is 0.** The virtual tooth number below is computed from the pitch diameter and is **independent of Tooth Spacing**; the spacing offset moves only the center, not the tooth size.

Compute the pinion's virtual (back-cone / Tredgold) tooth number from the closed form, **not** by measuring Apex2->K′ (the tooth-center point K′ is used as the tooth's center, below): virtual pitch radius = `(Pinion Gear Pitch Diameter / 2) / cos(γ_p)`, where `γ_p` is the pinion pitch-cone half-angle from §2 (`tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`). Virtual tooth number = `floor(2 · virtualPitchRadius / Module)`, as an int. (Module here is the raw mm value; the radius is a length — keep the unit handling consistent, see the Units note.)

Create a new plane that includes line C->K′. Use setByAngle to make this plane perpendicular to the Gear Profiles sketch plane.

Using the new plane and point K′ as the center point, create a spur gear tooth profile with module and the virtual tooth number obtained from the previous step. Draw it **already rotated 180°** by passing `angle=math.radians(180)` to the spur tooth generator's `draw(anchorPoint, angle=…)` (see Dependencies) — the generator rotates the whole tooth by that angle; do not draw it flat and rotate the sketch afterward. **After `draw()` returns, assert the tooth sketch is fully constrained: `_assertFullyConstrained(toothSketch)`** — like every other sketch, the tooth sketch must reach zero DOF (the spur generator pins the `angle != 0` reference line; if this raises, the gap is in `spurgear.md`).

Create a construction axis through point K′, normal to the plane the tooth profile was drawn on, via `setByTwoPlanes` (`setByPerpendicularAtPoint` would need a `BRepFace`). The two planes are: the **Gear Profiles plane** and a **helper plane built `setByDistanceOnPath(C->K′ line, 1.0)`** (perpendicular to C->K′ at its far end, K′); their intersection is the line through K′ normal to the tooth plane.

Compute the driving gear's virtual tooth number the same way (the tooth-center point L′ is used as the tooth center): virtual pitch radius = `(Driving Gear Pitch Diameter / 2) / cos(γ_g)`, where `γ_g = Σ − γ_p` is the driving pitch-cone half-angle from §2. Virtual tooth number = `floor(2 · virtualPitchRadius / Module)`, as an int.

Create a new plane that includes line D->L′. Use setByAngle to make this plane perpendicular to the Gear Profiles sketch plane.

Using the new plane and point L′ as the center point, create a spur gear tooth profile with module and the virtual tooth number obtained from the previous step. Draw it **already rotated 180°** by passing `angle=math.radians(180)` to the spur tooth generator's `draw(anchorPoint, angle=…)` (see Dependencies), exactly as for the pinion. **After `draw()` returns, assert the tooth sketch is fully constrained: `_assertFullyConstrained(toothSketch)`** (same as the pinion).

Create a construction axis through point L′, normal to the tooth plane, via `setByTwoPlanes`: the **Gear Profiles plane** intersected with a **helper plane `setByDistanceOnPath(D->L′ line, 1.0)`**.

### 3a: Spiral tooth body (ψ > 0)

This is the ψ > 0 branch of the tooth-body hook `_transformToothBody` (Method contract) — it **replaces** the straight tooth's two conical trims with a curved tooth. When **ψ = 0 the hook returns immediately** with `_cutConicalEnds` (the straight tooth, trimmed to a flush band — byte-for-byte the prior behavior); everything below runs **only when ψ > 0**. It is invoked once per gear (pinion then driving), inside `_createGearBody`, on the freshly lofted uncut apex→heel `toothBody`, before pattern/combine/bore. The arc math it realizes is derived in `spiral-tooth-trace.md`; this section states **how** that construction is realized as Fusion sketches/features and the order it runs in. Use the existing `{gear}` sketch-naming (`gearLabel` is `Pinion` or `Driving`).

**A. Gate & frame.** Build a world frame from the geometry already constructed for this gear:

- `axisDir` = the **shaft axis** direction, from the two **world** endpoints of `shaftAxisEdge` (the in-sketch profile edge A→G / B→I), normalized.
- `coneVec` = the **dedendum (root) cone element** Apex→D (driving) / Apex→C (pinion), realized as `normalize(heelConeWorld − apex)` where `apex` = `apexWorld`. (`heelConeWorld` is the heel end of that dedendum element; `toeConeWorld` is its toe end.)
- `v` = `axisDir × coneVec`, normalized — the **circumferential** direction (the sideways sense the tooth is displaced from the radial element).
- `tpNormal` = `coneVec × v`, normalized — the **tangent-plane normal** (the direction the flat trace is projected onto the cone, step D).
- `distAlong(p)` = `(p − apex) · coneVec` — the **cone distance** of a point (its distance from the apex measured along the cone element).

⚠️ **The heel MUST be the OUTER end (farther from the apex) so `coneVec` points outward and `span > 0`.** Before building `coneVec`, check the passed midpoints and **fix swapped toe/heel**: if `apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid ↔ heelMid` **and** `toeConeWorld ↔ heelConeWorld`, then build `coneVec = apex → heelConeWorld`. A negative `span` (toe farther than heel) **silently inverts the entire spiral frame** — it flips the cutter-arc direction, the slice direction (the first cut misses; see step E), and the per-segment twist — and the gear comes out completely wrong with no error. (The inversion can also originate upstream in §2/§3 mislabeling the toe vs heel edges; this guard catches it at the frame.)

From `toeMid`/`heelMid` (the toe/heel root-edge midpoints, **after** the swap guard above) get `R_toe = distAlong(toeMid)`, `R_heel = distAlong(heelMid)`, `R_mean = ½(R_toe + R_heel)`, and `span = R_heel − R_toe` (the face width, now **positive**). These are the only quantities the rest of the build needs.

**B. Cutter-arc geometry.** Work in the tangent-plane 2-D frame with origin at the apex, **x = coneVec** (so a point's x is its cone distance) and **y = v** (circumferential). The cutter radius is `r_c = Cutter Radius` if non-zero, **else `R_mean`** (the auto default). The hand sign is `handSign = +1` for `Right` else `−1`, then **negated for the pinion** (the pair meshes with opposite hands). The cutter-circle centre is

```
Cx = R_mean − r_c · sin ψ
Cy = handSign · r_c · cos ψ
```

⚠️ **Gotcha — the hand sign goes on the `cos`/`Cy` term, NOT the `sin`/`Cx` term** (this was a real bug). Opposite hand mirrors the cutter centre **across the cone element (y = 0)**, which flips `Cy`. Putting `handSign` on `Cx` mirrors about `x = R_mean` instead — a *different* curve that gives the two gears **unequal twist**; for equal teeth the driving and pinion traces must come out as exact mirror images.

The trace's toe/heel arc endpoints are circle∩circle intersections taken a hair **past** the face so the kept arc reaches cleanly past the end-trims: `toe2d = _circleIntersectNearest(R_lo, …)` and `heel2d = _circleIntersectNearest(R_hi, …)` with `R_lo = R_toe − 0.06·span` and `R_hi = R_heel + 0.06·span`. `_circleIntersectNearest` intersects the apex circle of radius R with the cutter circle (centre `(Cx,Cy)`, radius `r_c`) and keeps the solution nearest `(R_mean, 0)` — the branch the mean point sits on. (See `spiral-tooth-trace.md` §6 for why the near branch, and §5 for the centre derivation.)

**C. 2-D trace sketch (the genuine cutter arc).** Build the tangent plane with `_planeByAngle`: first draw a **cone-element construction line** Apex→(Apex + R_heel·coneVec) in a sketch on the **axial / Gear Profiles plane** (name it `{gear} Cone Element`), then make the tangent plane = that axial plane rotated **90°** about the cone-element line (`_planeByAngle(comp, coneElementLine, axialPlane, 90)`). Add a sketch on it named **`{gear} 2D Tooth Trace`**. In it draw, with `tanW(px,py) = _combine(apex, px, coneVec, py, v)` mapping 2-D coords to world:

- the **cutter circle** — centre at `tanW(Cx, Cy)`, radius `r_c` — `isConstruction`, with its centre constrained (coincident) and a diameter dimension = `2·r_c`;
- the **trace arc** — a 3-point arc through `tanW(toe2d)`, `tanW(R_mean, 0)` (the mean point on the cone element), `tanW(heel2d)`, with its **centre coincident to the cutter circle's centre** and a **radius dimension = `r_c`**, so it is the genuine cutter circle and not a look-alike spline.

This sketch is **deliberately left with free DOF** — the arc's endpoints are pinned by the 3-point construction, not by endpoint dimensions (dimensioning them over-constrains the solve against the cone-element plane). It is therefore **exempt from the full-constraint gate**, as already declared in Sketch Discipline ("The spiral build's auxiliary sketches are EXEMPT") — do not gate it.

**D. (No 3-D projection.)** The 2-D cutter-arc sketch from step C is the only trace geometry needed — the spiral twist is computed **analytically** from it in step G, so there is **no `projectToSurface`, no root-cone-face search, and no 3-D trace sketch.** (Earlier versions projected the 2-D arc onto the root cone along `tpNormal` and measured the trace azimuth there. That projection is *fragile*: for unequal-ratio pairs the arc wraps around the cone and `projectToSurface` returns it as **multiple disjoint fragments**, so the measured azimuth collapses to a fraction of the true sweep — the pinion comes out grossly under-twisted and the pair interferes. The analytic crown-gear law in step G is exact, deterministic, and cannot wrap.)

**E. Slice the straight tooth.** Split the uncut apex→heel `toothBody` into cross-section slabs by planes **perpendicular to the cone element**, spanning a touch past toe and heel, via a **fixed** slice scheme (≈8 planes — the count is not user-configurable). The first cut plane is the **parent transverse tooth plane** (`parentToothPlane`, the virtual-spur tooth-profile plane `{label} Plane` from §3, passed into the hook) offset toward the apex by `span/6`; the offset **sign is chosen per gear** so it moves toward the apex (the parent plane's normal points opposite ways for the two gears — pick `sign` so `sign·normal` points apex-ward, i.e. test `(apex − planeOrigin)·normal`). Then a sequence of ~8 planes stepped further toward the apex in `span/6` increments (`sign·(k+1)·span/6` for k = 0…7, k = 0 being the first cut plane). Split the body **piece-by-piece**: maintain a list of pieces, split each with each plane in turn, and **skip any plane that misses a piece** (the split throws — catch it and keep the piece whole). ⚠️ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in **one piece** (no plane cut it), the offset sign was wrong or `parentToothPlane` sits outside the tooth's span — **retry the whole cut once with the opposite sign**. If it is *still* one piece, **`raise` a clear self-diagnosing error** naming the gear, the final piece count, `span`, and the sign tried. Do **NOT** return an unsliced (single-piece) result: step F then drops that one piece as the apex scrap, leaving `segments` **empty**, and the crown later crashes with `ValueError: max() iterable argument is empty` far from the cause. The result is the set of cross-section segments.

**F. Order & drop scrap.** Sort the segments by `distAlong` of their centroid (`physicalProperties.centerOfMass`). The first (apex-most) is the long **apex-side scrap** below the toe — **remove it**; keep the rest as the working `segments`. (Drop the scrap by re-slicing the list, *then* delete it — `segments = segments[1:]` before `removeFeatures.add(scrap)`.) After dropping the scrap, **`segments` must be non-empty** (≥1 cross-section); if it is empty the slice failed in step E — `raise` a clear error rather than proceeding into the twist (G) and crown (H), which assume ≥1 segment.

**G. Twist (the spiral).** Rotate each segment about the **shaft axis** (`axisDir` through `apex`) so the tooth follows the trace, **centred on R_mean so the mid-face section stays unrotated** — that section then meshes exactly like the straight tooth (critical; the pinion's zero mesh nudge depends on it). The total toe→heel shaft-axis twist comes from the **conjugate crown-gear generation law** (the standard Gleason/Litvin model — see `spiral-tooth-trace.md` and the NASA references): a spiral bevel is generated by an imaginary flat *crown gear*, and the work gear’s shaft rotation relates to the developed crown-plane azimuth by the **roll ratio `1/sin γ`** (the generating crown gear has `N/sin γ` teeth; γ = this gear’s **pitch cone angle**). Compute it **analytically — no projection, no curve sampling:**

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])   # developed azimuth of the cutter arc at the apex
total     = abs(phi_crown) / math.sin(gamma)                          # shaft-axis twist magnitude
```

`phi_crown` is the angle the cutter arc’s **toe and heel endpoints subtend at the apex** in the flat 2-D crown frame (apex at the origin, x = cone distance along `coneVec`, y = circumferential along `v` — exactly the `toe2d`/`heel2d` pairs from step B). `gamma` is this gear’s **pitch cone angle**: `self._gamma_p` (Pinion) / `self._gamma_g` (Driving), already computed in §2. `handSign` sets the direction; `total` is the magnitude. ⚠️ **Use the PITCH cone angle γ from §2 — NOT `acos(coneVec·axisDir)`** (that is the *root/dedendum* cone angle, e.g. ~14° vs the pitch ~29° for a 17-tooth pinion, and yields a twist ~1.6× too large). ⚠️ The two members of a meshing pair **legitimately get different twists**: same cutter, same spiral angle ψ, but γ differs, so `1/sin γ` differs (≈2.08× for a 17-tooth pinion vs ≈1.14× for a 31-tooth gear — a ratio ~1.83). This is *why* equal-teeth pairs (31/31, equal γ) always meshed while ratio pairs failed under any method that gets `1/sin γ` wrong. **Do NOT** measure the twist off a projected 3-D cone trace (the old approach): `projectToSurface` wraps the arc around the cone for ratio pairs and the measurement collapses. The analytic law here is exact and deterministic. Each segment's rotation angle is a **linear share** keyed to the **cone distance of its HEEL FACE** (the segment's farthest-along-the-element face — the exact section the later loft samples). **Define a slab's heel face precisely: the face whose centroid has the GREATEST `distAlong(face.centroid)`, searched across ALL of the slab's faces with NO surface-type filter** (its toe/apex-side face is the LEAST-centroid one). ⚠️ Do **NOT** restrict this search to `PlaneSurfaceType` (or any surface type) — a sliced slab is bounded by a mix of the two planar cut faces and ruled side faces, and a type filter can pick the wrong face or miss the cut face, which makes the step-I loft fail with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this **same all-faces-by-centroid** rule (max → heel, min → toe) everywhere a slab end face is needed: the twist key here (G), the crown base (H), and the loft sections (I). The rotation:

```
ang = −handSign · total · (R_mean − R_heelFace(seg)) / span
```

⚠️ **Gotcha — key the twist on the segment's HEEL-FACE cone distance, NOT its centroid.** The loft (step I) samples each segment's heel face, so that face is what must land at the right azimuth. Centroid-keying leaves the loft's mid-face section rotated by half a segment → mid-face overlap. Apply the rotation with a free-move by a `Matrix3D.setToRotation(ang, axisDir, apex)`.

**H. Lengthwise crown (relief).** For each segment **except the outermost (heel) one**, scale it about the **centre of its heel face** by `1 − _CROWN_PER_RAD·|ang|` (`ang` = that segment's twist angle), so toe and heel thin and contact localizes at mid-face; mid-face (twist ≈ 0) is untouched. **`_CROWN_PER_RAD` is a tunable class constant, default `0.5`** (0 disables the crown) — set it to 0.5, do not leave it unset/0. Two gotchas:

1. **The scale base must be a sketch point** (a point added in a sketch on the heel face, or a BRep vertex) — `constructionPoints.setByPoint(Point3D)` raises *"Environment is not supported"* in this nested Design component. (`scaleFeatures` likewise needs the Design occurrence to be the **active** edit target: call **`designOccurrence.activate()`** (a method on the `Occurrence`) before the crown scales, and restore afterward — in a `finally` — with **`design.activateRootComponent()`** (a method on `Design`). ⚠️ Do **NOT** write `design.rootComponent.activate()` or `someComponent.activate()` — a `Component` has **no** `.activate()` method and it raises `AttributeError: 'Component' object has no attribute 'activate'`. Only `Occurrence` has `.activate()`; the root is re-activated via `Design.activateRootComponent()`.)
2. **Skip the outermost (heel) segment.** Its heel face is the loft's heel end and must stay full so the heel cone (step J) trims it flush with the gear base.

**I. Loft → curved tooth.** ⚠️ **Re-sort the segments by their heel-face cone distance HERE, AFTER the twist (G) and crown (H) — do NOT reuse the pre-twist slice/centroid order from step F.** The twist rotates each slab about the shaft axis, and for high-twist *unequal-ratio* pairs that rotation changes the slabs' along-cone (`distAlong`) order enough to **reorder adjacent slabs**; lofting in the stale pre-twist order then assembles the cross-sections out of sequence and the crowned tooth comes out distorted → the two gears interfere. (For equal/low-twist pairs the two orders coincide, which is why equal-teeth gears mesh even with the stale order but unequal ratios distort — this is the single thing that makes a ratio pair like 31/17 fail while 31/31 looks fine.) So: compute `order = sorted(segment indices, key = distAlong(slabHeelFace(seg).centroid))` **now**, and loft a NewBody through, in that order: first the **toe-most segment's apex-side (toe-facing) face** — the toe segment is `order[0]`, its toe face added first to push the loft past the toe cone so the toe trim bites — then the **heel-facing face of every segment, iterated in `order`** (each segment's farthest-along-the-element face by post-twist centroid; the last reaches past the heel cone). Name the resulting body **`{gear} Spiral Tooth`**. Then remove the segment scaffolding (the loft has captured their faces).

**J. Flush trim + mesh phase.** Return `_cutConicalEnds(designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)` — the same toe-then-heel two-cone trim used for the straight tooth — so the curved tooth's ends sit **flush** on the gear base. The toe/heel **mesh phasing** is handled outside this hook by `_createGearBody`'s mesh-rotate step (driving by `180°/drivingTeeth`; the **pinion additionally** by `_pinionMeshPhase()`, which is 0 by default because the mid-face section is unrotated and already meshes — see Method contract).

## Create the Pinion Gear

Create a new component as a child of the **Bevel Gear** component (the same component that owns Design — *not* the user's Parent Component; this intentionally overrides the looser "child of Parent Component" phrasing so the pair nests cleanly inside Bevel Gear). The resulting bodies for this gear end up in this component. (Implementation note: Fusion's API rejects cross-sibling sketch and project calls even when the target is activated or the entities are wrapped in `createForAssemblyContext` proxies, so the actual feature operations run in the Design component and the finished bodies are `moveToComponent`'d here at the end. The visible end state is identical.)

Open a **fresh sketch on the axial (Gear Profiles) plane** named e.g. `Pinion Profile` — **one profile sketch per gear**, so `sketch.profiles` holds exactly this one hexagon loop (do not draw both gears' hexagons in the shared Gear Profiles sketch — that would leave two identically-shaped loops to disambiguate). Recreate the six vertices A, G, H, C, M, N in this sketch as **fixed points** (see the next paragraph), then draw the closed hexagon A -> G -> H -> C -> M -> N -> A as six `SketchLine`s sharing those fixed points.

**Getting the revolve profile:** this sketch holds exactly one hexagon loop, so take its single profile (see PLAYBOOK).

**Build the hexagon on fixed vertices per the playbook's recreate-share-fix recipe:** recreate the six §2 vertices A, G, H, C, M, N as new points at their exact (world-mapped) positions, draw the closed hexagon **sharing** those points, then fix the lines and their endpoints **after** the lines exist (not before). The §2 vertices' world positions are valid because §2 is fully constrained by now. The hexagon's **first edge (A->G) is the gear's shaft axis** for the revolve, pattern, bore plane AND the meshing-rotation axis, so it must be fixed well enough to carry a trustworthy world position: fixed endpoints give that edge a well-defined `worldGeometry`, whereas a free edge would resolve against a default/world-XY frame and silently move the driving body onto world XY (the pinion looks fine because it never reads the edge's `worldGeometry`).

**The shaft axis used by every body operation below is this profile sketch's first edge (the A->G line you just drew), NOT the §2 `Apex->A` construction line.** A→G is collinear with the shaft axis but lives in the *same* sketch as the profile, which is what Fusion's revolve/pattern/path accept; reusing the §2 construction line (a different sketch) fails or misbuilds. Use that A->G edge as the axis for: the revolve, the circular pattern, the bore-plane `setByDistanceOnPath`, and the meshing-rotation axis.

Revolve the profile around that A->G edge; let the result be Pinion Gear Body. Because M->N is one edge of the revolved profile, the body already carries the conical face produced by sweeping M->N around the axis — that face is reused as the cutting tool below.

Loft the **§2 Apex sketch point** (the `centerToApex.endSketchPoint` from the Gear Profiles sketch — the degenerate point-section) to the Pinion Gear Tooth profile; let the result be the Pinion Gear Tooth Body. Use the §2 Apex SKETCH point directly — do NOT create a construction point for it (see PLAYBOOK: construction geometry needs an active component, which the Design component never is).

Cut the Pinion Gear Tooth Body twice, in order — **selecting the keeper piece between the two cuts so each cut splits exactly one body (exactly two split features; see "Select the keeper BETWEEN the two cuts" below).** **Two distinct bodies are involved — do not conflate them:** the **cutting TOOL** each time is a `ConeSurfaceType` face of the **Pinion Gear Body (the revolved-hexagon frustum)** — the lofted Tooth Body has no cone faces, so searching *it* for the cone face finds none (`distances: []`). The **TARGET being split** is the **Pinion Gear Tooth Body (the loft)**. So: find the cone face on the **frustum (Gear Body)**, then split the **Tooth Body** (and, for cut 2, the pieces from cut 1) with it.

1. First, cut using the conical face **on the Pinion Gear Body (frustum)** generated from the M->N edge — locate the existing face on the frustum (don't build a fresh revolved surface from the M->N sketch line; see PLAYBOOK). **Cone-face selection — identify the cone by the cut edge's MIDPOINT, not its endpoints.** The cut tool for cut#1 is the *specific* cone the M->N toe edge revolved into (for cut#2, the C->H heel edge's cone); picking the wrong cone here makes cut#1 split the tooth in the wrong place and then cut#2's correct cone has nothing left to intersect (observed: "cut#2 heel: no cone face split any piece" even though the heel cone is found). Identify it robustly:
- Compute the **world midpoint of the cut edge** (`(M_world + N_world)/2` for cut#1; `(C_world + H_world)/2` for cut#2). Use the **midpoint, not the endpoints** — the toe endpoint N sits near the cone's apex/singularity where `surface.evaluator.getParameterAtPoint` returns `None`, which is what made endpoint-distance unusable; the midpoint is well clear of the apex, so its surface distance is reliable, and it **uniquely** distinguishes the toe cone from the heel cone (and from neighbour cones) because each contains a *different* edge midpoint.
- Among the frustum's `ConeSurfaceType` faces, order them by ascending distance from their surface to that edge midpoint (`_surfaceDistance(face.geometry, edgeMidWorld)`; treat an unevaluable `None` as +inf, tried last). Then **try each, best-first, as the `splitBodyFeatures` tool (`isSplittingToolExtended=True`) on the Tooth Body, and keep the first that actually splits it (>1 piece)** — best-by-midpoint puts the correct cone first, and the split confirms it. Only if *every* cone face fails to split is this cut non-intersecting (raise the self-diagnosing error). Do not use endpoint distance, a hard tolerance gate, or `body.faces` order.
2. Then, cut **only the keeper piece** (the single largest non-apex piece selected after cut 1 — see "Select the keeper BETWEEN the two cuts" below; do NOT cut the apex tip) using the conical face **on the Gear Body (frustum)** generated from the C->H edge. Locate it the **same way — by the C->H edge's world MIDPOINT** (`(C_world + H_world)/2`): order the frustum's `ConeSurfaceType` faces by surface-distance to that midpoint and try each best-first as the split tool until it splits the keeper (`isSplittingToolExtended=True`).

**Select the keeper BETWEEN the two cuts so each cut splits exactly one body — this is what makes it exactly two split features for every gear ratio.** Do NOT feed both cut-1 pieces into cut 2. Instead:
- **After the toe cut**, immediately **drop the apex tip**: remove every piece that contains the apex (by point-containment; see PLAYBOOK) and keep the single **largest** remaining (non-apex) piece — the *keeper*. (The toe cut must produce ≥2 pieces — the apex tip plus the keeper; if it produced only one, that is the self-diagnosing cut failure below.)
- **Then run the heel cut on the keeper ALONE.** Removing the apex tip *before* the heel cut is essential: the heel cone is an **extended/infinite** surface, so if the apex tip were still present the heel cut would slice it too — a **geometry-dependent spurious third cut** (this is exactly why v33 showed **3** split features on the pinion but only **2** on the driving: the extended heel cone reaches the pinion's apex tip but not the driving's). Cutting only the keeper makes it deterministically **two** cuts for both gears. The heel cone may not intersect the keeper at all (the tooth profile sits on the heel/back cone, so there may be no overshoot) — wrap the heel split in `try/except RuntimeError` and, on `SPLIT_TARGET_TOOL_NOT_INTERSECT` (or localized `交差`), keep the keeper whole (it is already the tooth).

**Make every cut failure self-diagnosing** (these errors are how the build is debugged when run headless, so they must carry the measured numbers, not a bare count):
- **The cut never silently no-ops.** If, after trying **every** `ConeSurfaceType` face of the frustum as the split tool, none actually splits the Tooth Body, `raise` an error naming the cut (#1/#2, pinion/driving) and listing, for every frustum cone face, its two endpoint distances `(dA, dB)` (which may be `None` when the evaluator can't project the point) **and whether the split was attempted/failed** — so a genuinely-absent toe cone is distinguishable from an evaluator/selection problem. Do **not** return the bodies unchanged on a miss.
- **Log each cut's outcome** with `futil.log(..., force_console=True)`: how many bodies went in, which face was selected (with its `(dA,dB)`), and how many pieces came out (or "tool did not intersect, kept intact").
- **Errors must report the per-cut history**, e.g. `"<gear>: toe cut produced N pieces (faces found=…, selected dist=…); keeper selection found M non-apex pieces; heel cut …"`, so the message alone reveals which cut under- or over-split and whether it was a tolerance/face-selection problem. (The toe cut must yield ≥2 pieces — apex tip + keeper; the keeper selection must find ≥1 non-apex piece. Do not assert a fixed total piece count — with interleaved selection each cut acts on one body, so the counts are toe→2, heel→1-or-2.)

**The keeper/tooth selection rule — apply it AFTER EACH cut.** Both times the rule is identical: remove every piece that contains the Apex (by point-containment), then of the remaining pieces keep the single **largest** by volume and remove the rest (see PLAYBOOK for point-containment / piece-removal APIs). **After the toe cut** this drops the apex tip and yields the keeper (one piece) to feed into the heel cut. **After the heel cut** it yields the tooth band (any thin heel stub is the removed smaller piece; no apex piece remains, since it was removed after the toe cut). Raise only if there is no non-apex piece at all. Because each cut acts on a single body, you get exactly **two** split features regardless of gear ratio.

Circular-pattern the remaining tooth piece around the **A->G edge** (the shaft axis — the same in-sketch profile edge used for the revolve, not the §2 `Apex->A` construction line). The number of copies equals the Pinion Gear Teeth Number. (Although the pitch diameter shrinks from heel toward apex, the *angular* spacing around the shaft axis stays constant at `360° / N` for the entire face width — the radial taper is already produced by the loft from Apex to the heel-end tooth profile, so the pattern just rotates that single tapered tooth into N evenly spaced copies.)

Join all patterned tooth pieces with Pinion Gear Body in a single Combine-Join (Pinion Gear Body as the target, the patterned tooth bodies as the tools).

If Enable Bore is checked, cut a cylindrical through bore through Pinion Gear Body along the shaft axis. The bore diameter is the Pinion Gear Bore Diameter if specified (non-zero); otherwise use `Pinion Gear Pitch Diameter / 4`. Build the bore plane normal to the shaft at its start (`setByDistanceOnPath(A->G edge, 0.0)`; pass the in-sketch A->G edge, not the §2 construction line). Sketch the bore circle centered at the sketch origin (the plane is rooted at the shaft start, so the origin is on the axis): **fix the bore circle's center and add a diameter dimension** set to the bore diameter (see PLAYBOOK circle-at-origin rule). Extrude-cut it as a symmetric through-cut restricted to `[Pinion Gear Body]`. Skip this step entirely if Enable Bore is unchecked.



## Create the Driving Gear

Create a new component as a child of the **Bevel Gear** component (not the user's Parent Component — same override as the Pinion Gear section). The resulting bodies end up in this component. (Same Fusion API caveat as the Pinion Gear section above.)

Open a **fresh sketch on the axial plane** named e.g. `Driving Profile`. Recreate B, I, J, D, O, P exactly as for the pinion via the playbook's recreate-share-fix recipe — draw the closed hexagon B -> I -> J -> D -> O -> P -> B sharing the recreated vertices, then fix the lines/endpoints after the lines exist. One profile per gear. As for the pinion, **the shaft axis for all body operations is this sketch's first edge (the B->I line), not the §2 `Apex->B` construction line** — used for the revolve, pattern, bore-plane path, and mesh axis; fixed endpoints give it the defined `worldGeometry` the mesh rotation needs (a free edge lands the driving body on XY). Revolve it around that B->I edge; let the result be Driving Gear Body. As with the pinion, the O->P edge revolves into a conical face on the body that is reused as the cutting tool below.

Loft the **§2 Apex sketch point** (`centerToApex.endSketchPoint`) to the Driving Gear Tooth profile, same as the pinion; let the result be the Driving Gear Tooth Body. Use the §2 Apex sketch point directly — do **NOT** create a construction point (see PLAYBOOK: construction geometry needs an active component).

Cut the Driving Gear Tooth Body twice, in order:

1. First, cut using the conical face on Driving Gear Body generated from the O->P edge — same selection rule as the pinion, **by the O->P edge's world MIDPOINT** (`(O_world + P_world)/2`): order the frustum's `ConeSurfaceType` faces by surface-distance to that midpoint and try each best-first as the split tool until one splits the Tooth Body (`isSplittingToolExtended=True`). Do not use endpoint distance.
2. **After the toe (O->P) cut, drop the apex tip and keep the single largest non-apex piece (the keeper) — exactly as for the pinion — then cut the keeper ALONE** using the cone face generated from the D->J edge (the analogue of C->H), located the same way by the **D->J edge midpoint** (`(D_world + J_world)/2`).

Interleave the keeper selection between the two cuts exactly as for the pinion (this is what makes it **two** split features, not the geometry-dependent 3): toe cut → drop apex tip, keep largest non-apex (keeper) → heel cut on the keeper alone, catching `SPLIT_TARGET_TOOL_NOT_INTERSECT` / `交差` and keeping the keeper whole if the heel cone doesn't reach it.

Select the tooth exactly as for the pinion (same rule applied after each cut): remove **every** Apex-containing piece (by point-containment), then of all remaining non-apex pieces keep the single **largest** by volume and remove the rest. Raise only if no non-apex piece remains.

Circular-pattern the remaining tooth piece around the **B->I edge** (the shaft axis — the in-sketch profile edge used for the revolve, not the §2 `Apex->B` construction line). The number of copies equals the Driving Gear Teeth Number, for the same reason as the pinion: the angular spacing around the shaft axis is constant at `360° / N`, regardless of the radial taper from heel toward apex.

Join all patterned tooth pieces with Driving Gear Body in a single Combine-Join (Driving Gear Body as the target, the patterned tooth bodies as the tools).

If Enable Bore is checked, cut a cylindrical through bore through Driving Gear Body along the shaft axis, exactly as for the pinion (bore plane normal to the shaft at its start via the B->I edge, circle centered/fixed at the sketch origin with a diameter dimension, symmetric through-cut restricted to `[Driving Gear Body]`). The bore diameter is the Driving Gear Bore Diameter if specified (non-zero); otherwise use `Driving Gear Pitch Diameter / 4`. Skip if Enable Bore is unchecked.

**Meshing rotation (driving gear only) — do this here, in the Design component, before the body is moved out.** Rotate the driving body by `180° / Driving Gear Teeth Number` (half a tooth pitch) about its shaft axis — the B->I profile edge, whose **world** endpoints give the rotation axis/origin (the same shaft axis used for the revolve/pattern; see PLAYBOOK for the move API) — so a driving valley sits where the pinion tooth crosses the axial plane, giving the interlocked meshing look. This runs in Design before `moveToComponent` because a construction axis can't be added in the moved-out gear component (construction geometry needs an active component; see PLAYBOOK), so the rotation must use the edge's world geometry while still in Design.

## Cleanup

Recursively walk the Bevel Gear component tree (dedupe by `entityToken`) and hide every sketch, construction plane, and construction axis with `isLightBulbOn = False` (construction planes/axes are **not** hidden by `isVisible`; sketches use `isVisible = False` or the same light-bulb). Leave only the two finished gear bodies visible.

(The driving gear's half-tooth-pitch **meshing rotation** is performed earlier, at the end of "Create the Driving Gear", in the Design component before the body is moved out — see that section. It is not a cleanup step, and the rationale is: both gears are patterned from a starting tooth in the axial plane, so without the offset a driving tooth and a pinion tooth would both sit at the axial-plane crossing and visually collide; rotating the driving gear by half its own tooth pitch interlocks them.)