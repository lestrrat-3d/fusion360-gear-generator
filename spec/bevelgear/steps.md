# Bevel Gear — compiled step list

The proof for these steps is `proof/bevelgear/geometry_test.go`, `proof/bevelgear/sketches_test.go`,
`proof/bevelgear/solids_test.go` and the generated registration file
`proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `a478dac1f0da80e89237896d4e07984c3630c86d` |
| `spec/bevelgear/fusion.md` | `5b3350a781c8c6da63d3768c224db5e939c365fd` |
| `spec/bevelgear/spiral-tooth-trace.md` | `c7ae12c0486248c2a02705081b41e9061fb244b2` |
| `spec/spurgear/fusion.md` | `5dccd871606c3709ecfa07c05f58c126369f2927` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9ee2dcbaed7b5480aa69e9295e8b61acaea081f3` |

## S1 `[PROSE]` Add the 20 command dialog inputs

`BevelGearCommandInputsConfigurator.configure(cls, cmd)` adds every input below to
`cmd.commandInputs`, **in this row order**. The order is the contract: Target Plane is first so it
wins Fusion's auto-focus (`[PB-AUTOFOCUS-FIRST]`), Center Point follows so the user flows from
plane to point, and the pre-selected Parent Component comes third.

Reproduce this table verbatim. Every id, label, unit string, default and tooltip below is part of
the surface; none of it may be re-spelled.

| # | Dialog input | input id | input type | unit | default | selection filters / tooltip |
|---|---|---|---|---|---|---|
| 1 | Target Plane | `targetPlane` | `addSelectionInput` | — | — | `ConstructionPlanes`, `PlanarFaces`; limit 1; tooltip `Plane the bottom of the driving gear sits flush against` |
| 2 | Center Point | `centerPoint` | `addSelectionInput` | — | — | `ConstructionPoints`, `SketchPoints`; limit 1; tooltip `Point the driving bevel gear is centered on` |
| 3 | Parent Component | `parentComponent` | `addSelectionInput` | — | root component pre-selected | `Occurrences`, `RootComponents`; limit 1; tooltip `Component the gear pair is created under` |
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
| 18 | Toe Extension (%) | `toeExtension` | `addValueInput` | `''` | `createByReal(0)` | — |
| 19 | Driving Gear Toe Radius | `drivingToeRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 20 | Pinion Gear Toe Radius | `pinionToeRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |

There are **20** dialog inputs and **20 `INPUT_ID_*`** module constants, named exactly, holding the
table's id strings in row order:

`INPUT_ID_PLANE = 'targetPlane'`, `INPUT_ID_CENTER_POINT = 'centerPoint'`,
`INPUT_ID_PARENT = 'parentComponent'`, `INPUT_ID_MODULE = 'module'`,
`INPUT_ID_SHAFT_ANGLE = 'shaftAngle'`, `INPUT_ID_DRIVING_TEETH = 'drivingTeeth'`,
`INPUT_ID_PINION_TEETH = 'pinionTeeth'`, `INPUT_ID_DRIVING_BASE_HEIGHT = 'drivingBaseHeight'`,
`INPUT_ID_PINION_BASE_HEIGHT = 'pinionBaseHeight'`, `INPUT_ID_BORE_ENABLE = 'boreEnable'`,
`INPUT_ID_DRIVING_BORE = 'drivingBore'`, `INPUT_ID_PINION_BORE = 'pinionBore'`,
`INPUT_ID_FACE_WIDTH = 'faceWidth'`, `INPUT_ID_TOOTH_SPACING = 'toothSpacing'`,
`INPUT_ID_SPIRAL_ANGLE = 'spiralAngle'`, `INPUT_ID_HAND = 'spiralHand'`,
`INPUT_ID_CUTTER_RADIUS = 'cutterRadius'`, `INPUT_ID_TOE_EXTENSION = 'toeExtension'`,
`INPUT_ID_DRIVING_TOE_RADIUS = 'drivingToeRadius'`,
`INPUT_ID_PINION_TOE_RADIUS = 'pinionToeRadius'`.

Two further module-level constants and no others: `_HAND_RIGHT = 'Right'` and `_HAND_LEFT = 'Left'`.
There are **no** `PARAM_*` strings — bevel registers no live Fusion user parameters
(`[PB-PRECOMPUTED-MODE]`).

Mechanics:

- Selection inputs: `inputs.addSelectionInput(id, label, tooltip)`, then
  `addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)` and the other filters the
  table lists, then `setSelectionLimits(1, 1)`. Write every filter as the named constant, never as a
  quoted literal (`[PB-SELECTION-FILTER-ENUM]`, `[PB-SELECTION-DECL]`). The Parent input pre-selects
  `get_design().rootComponent`.
- Value inputs: `inputs.addValueInput(id, label, unit, adsk.core.ValueInput.createByReal(default))`.
  The `mm` defaults are passed in **internal units** — `to_cm(0)` — because a `createByReal` default
  is always internal regardless of the unit string (`[PB-DIALOG-DEFAULT-UNITS]`). The Shaft Angle and
  Mean Spiral Angle defaults use `adsk.core.ValueInput.createByString` so the expression engine
  parses them.
- The checkbox: `inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)`.
- The dropdown:
  `inputs.addDropDownCommandInput(INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)`,
  then `listItems.add(_HAND_RIGHT, True)` and `listItems.add(_HAND_LEFT, False)`.

**Conditional visibility — the spiral-only inputs show only when the Mean Spiral Angle is above 0.**
Hand of Spiral and Cutter Radius are relevant only for curved bevels; Mean Spiral Angle is the
controller and is always visible. There is no declarative show-if in the API, so this is realized
with `commandInput.isVisible`:

- A `@classmethod _updateSpiralInputVisibility(cls, inputs)` helper evaluates the `spiralAngle`
  input's **`.expression`** through `unitsManager.evaluateExpression(spiral.expression, 'rad')` —
  internal **radians**, and NOT the input's `.value` — and sets
  `inputs.itemById(INPUT_ID_HAND).isVisible` and `inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible`
  to `(value > 0)`. Guard it: if any of the three inputs is `None`, return early; wrap the expression
  evaluation in `try/except`, because a half-typed expression can raise mid-edit, and on failure
  leave both inputs **shown**.
- `configure()` calls `cls._updateSpiralInputVisibility(inputs)` as its **last** step, so the initial
  state is right (default 35 degrees, both shown).
- A second classmethod, `handle_input_changed`, taking `(cls, args)`, calls
  `cls._updateSpiralInputVisibility(args.inputs)` and nothing else — recompute on every input change,
  which is cheap and needs no branch on which input changed.

`isVisible` only hides the dialog row. The input still exists, `_readInputs` reads it normally, and a
Mean Spiral Angle of 0 ignores Hand and Cutter Radius anyway, so hiding is cosmetic and cannot affect
generation.

`configure` and `handle_input_changed` are methods this module DEFINES for
`commands/bevelgear/entry.py` to bind by name; the module does not call them.

<!-- check-step-calls: ignore configure handle_input_changed _updateSpiralInputVisibility -->

**From:** `spec/bevelgear/instructions.md` L35-44, L222-332; `.claude/skills/generate-gear/PLAYBOOK.md` L42-74, L128-143, L326-358, L557-568, L850-861

## S2 `[PROSE]` Read and validate every input

`BevelGearGenerator._readInputs(inputs)` runs first, before anything creates an occurrence, and
returns the 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
shaftAngle_deg)`, stashing the rest on `self`: `self._drivingBaseHeight_cm`,
`self._pinionBaseHeight_cm`, `self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`,
`self._faceWidth_cm`, `self._toothSpacing_cm`, `self._spiralAngle_rad`, `self._hand`,
`self._cutterRadius_cm`, `self._toeExtension_pct`, `self._drivingToeRadius_cm`,
`self._pinionToeRadius_cm`.

**Reading.** Selections come from `get_selection(inputs, id)`; the checkbox from
`get_boolean(inputs, id)` — never `get_value`, which reads `.expression` and raises `AttributeError`
on a `BoolValueCommandInput` (`[PB-INPUT-READ]`). The dropdown is read as
`inputs.itemById(INPUT_ID_HAND).selectedItem` and its `.name` taken, defaulting to `_HAND_RIGHT` when
none is selected. Every numeric and angle input is read by evaluating its expression through
`design.unitsManager.evaluateExpression(expr, units)` with `''` / `'mm'` / `'deg'`, which always
returns Fusion internal units — cm for length, radians for angle — whatever the unit string says
(`[PB-EVAL-EXPRESSION]`).

**Units, and this is where a gear comes out ten times off.** The `mm` inputs (both base heights, both
bore diameters, Face Width, Tooth Spacing, both toe radii) and the two `deg` inputs come back
**already internal**; use them as-is and do not `to_cm` them again. **`Module` is read with unit
`''`, so it comes back as a raw number meaning millimetres** — a module of 1 is 1 mm. Every length
derived from Module must therefore be `to_cm`-converted before it touches geometry: the pitch
diameters, the Cone Distance, the dedendum `1.25 * Module`, every construction seed length in S6,
and the default Face Width. `toeExtension` is a plain unitless percentage and needs no conversion.

Both teeth inputs are coerced with `int(round(...))` before validation.

**Range checks, in this order.**

1. `module > 0`; `teeth >= 3` on both gears; non-negative base heights, bore diameters, Face Width,
   Tooth Spacing, Cutter Radius and toe radii; Toe Extension in `[0, 100]`; Mean Spiral Angle in
   `[0, 60)` degrees.
2. **Shaft Angle**: at least 30 degrees and at most the Maximum Shaft Angle. Convert to degrees
   before the check. The Maximum Shaft Angle is
   `min(150, degrees(acos(-min(PPD, DPD) / max(PPD, DPD))))` with `PPD = Module * Pinion Gear Teeth
   Number` and `DPD = Module * Driving Gear Teeth Number`; the cone-angle half is **exclusive** (a
   pitch cone angle reaching 90 degrees turns that gear's cone inside out, `R * cos gamma` passes
   through zero and changes sign, and `acos` is a hard singularity) while the 150 degree half is
   inclusive. It depends on both tooth counts, so check it after both are read and coerced, and name
   the computed limit in the rejection message. A 31/17 pair gives `acos(-17/31) = 123.26` degrees;
   equal tooth counts give `acos(-1) = 180`, which is no constraint at all.
3. Compute the two pitch cone angles from the closed form:
   `tan gamma_p = sin(Sigma) * PPD / (DPD + PPD * cos(Sigma))`, `gamma_g = Sigma - gamma_p`, and the
   Pitch Cone Distance `R = (PPD / 2) / sin(gamma_p)`.
4. **Minimum Teeth**, per gear with that gear's own `gamma`: `teeth >= 5.27 * cos(gamma)`, on top of
   the blanket `teeth >= 3`. The constant is `2 * (1.05 * 1.25 / 0.95 + 1.25) = 5.2632` rounded **UP**
   to 5.27 so the published floor stays at or above the exact crossing; do not round it down. Name the
   computed floor in the message. This check is exactly the statement that the base-height window
   below is non-empty, which is why it runs before it.
5. **The two base heights**, driving first. For one gear with pitch radius `r` and its own `gamma`:
   - `Minimum Base Height = 1.05 * 1.25 * Module * sin(gamma)`
   - `Maximum Base Height = 0.95 * (r - 1.25 * Module * cos(gamma)) * tan(gamma)`

   The driving fallback when the input is 0 is `Module * Driving Gear Teeth Number / 8`; the pinion's
   is the **resolved** driving height times `Pinion Gear Teeth Number / Driving Gear Teeth Number`.
   Apply the bounds in **both** directions and per gear: raise a fallback below the minimum, cap one
   above the maximum, and reject a user value outside either end naming the bound it broke. The pinion
   gets its **own** bounds after the scaling — the two gears have different cone angles whenever the
   tooth counts differ, so the driving cap does not imply the pinion's.

   The Maximum Base Height is measured from **Apex 2's plane**, not from the dedendum point: walking
   out along the dedendum line from Apex 2 the perpendicular distance to the shaft axis falls at
   `cos(gamma)` per unit while the along-shaft coordinate rises at `sin(gamma)`, so the true crossing
   is at `r * tan(gamma)` and this bound sits `1.25 * Module * sin(gamma)` below it. It is
   deliberately conservative, not exact. Past the true crossing the hexagonal frustum profile has
   crossed its own axis of revolution and the revolve fails with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`).

**The bore bound is NOT part of this pass.** Validate the two bore diameters here only as
non-negative numbers. The Maximum Bore Diameter resolves in S6, because its toe term needs the Root
Length, which needs the resolved Face Width, which needs solved S6 geometry.

`get_value` is named here only to forbid it on the checkbox; the module does not call it there.

<!-- check-step-calls: ignore get_value -->

**From:** `spec/bevelgear/instructions.md` L45-188, L333-379; `.claude/skills/generate-gear/PLAYBOOK.md` L99-126, L435-438, L657-658, L863-867

## S3 `[PROSE]` Create the component tree

Create the occurrence tree directly with
`parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` — bevel does not subclass
`base.Generator` and does not use `getOccurrence` (`[PB-OCCURRENCE-TREE]`):

1. `Bevel Gear` as a child of the user's Parent Component. Its occurrence is `self.bevelOccurrence`,
   which `deleteComponent()` uses for error rollback; the component is `self.bevelComponent`.
2. `Design` as a child of **Bevel Gear**, named `Design`. It owns every sketch, construction plane,
   construction axis and feature. `self.designOccurrence` / `self.designComponent`.
3. One `{gearLabel} Gear` component per gear — `Pinion Gear`, `Driving Gear` — also a child of **Bevel
   Gear** and not of the user's Parent Component. These are created in S16, where the bodies for that
   gear are about to be built.

**Never activate any occurrence** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`). The Anchor
sketch is created on the user's **external**, root-owned target plane; an activated occurrence
resolves that external plane in its own local frame and the whole build collapses onto world XY
regardless of the real plane tilt. The only exception is the spiral crown's scale feature in S20,
which states its own activate-and-restore.

All feature operations run in the one Design component and the finished bodies are moved out at the
end, because Fusion rejects cross-sibling sketch and project references even when the target is
activated or the entities are wrapped in assembly-context proxies (`[PB-NO-CROSS-SIBLING]`).

The per-gear anchors travel in plain per-gear dicts `pinionCtx` / `drivingCtx`, built in S6 and
passed on. **These 18 key strings are reproduced surface — never rename one, split the dict, wrap it
in a class, or carry a value under a different shape.** Both gears carry the same 18 and no others:

| key | value it carries | type / unit | written | read by |
|---|---|---|---|---|
| `label` | `'Pinion'` or `'Driving'` | `str` | S6 | every `{gearLabel}` name and the `gearLabel` argument of the tooth-body hook and the conical cuts |
| `teeth` | this gear's Teeth Number | `int` | S6 | the pattern quantity; the meshing rotation angle; the tooth-body hook's `teethNumber` |
| `gamma` | this gear's pitch cone angle | `float`, radians | S6 | S11 step 1; the tooth-body hook's `gamma` |
| `pitchDiameter_cm` | this gear's Pitch Diameter | `float`, internal cm | S6 | S11 step 1 |
| `toothCenterPoint` | the tooth-center point K' / L' | `SketchPoint` | S6 | S13, as the spur drawer's `draw(anchorPoint, ...)` anchor |
| `toothCenterRefLine` | the tooth-center reference line C->K' / D->L' | `SketchLine` | S6 | S12; S14's helper plane |
| `hexVertices` | the six profile vertices in draw order — A', G, H, C, M, N / B', I, J, D, O, P | `list[SketchPoint]`, length 6 | S6 | S16 |
| `toeEdgePoints` | the toe edge's two endpoints — M and N / O and P, in that order | `tuple[SketchPoint, SketchPoint]` | S6 | the `toeMid` midpoint; its FIRST element is `toeConeWorld` |
| `heelEdgePoints` | the heel edge's two endpoints — C and H / D and J, in that order | `tuple[SketchPoint, SketchPoint]` | S6 | the `heelMid` midpoint; its FIRST element is `heelConeWorld` — the dedendum corner C / D, **NEVER** H / J |
| `boreDiameter_cm` | this gear's Bore Diameter, already resolved AND already bounded | `float`, internal cm | S6 | S24 |
| `toothPlane` | the `{gearLabel} Plane` construction plane | `ConstructionPlane` | S12 | the tooth-body hook's `parentToothPlane` |
| `toothSketch` | the `{gearLabel} Tooth` sketch | `Sketch` | S13 | the tooth-profile selection |
| `toothEmbedded` | the spur drawer's `_lastToothEmbedded`, read back off the proxy | `bool` | S13 | the tooth-profile selection, as `wantLines = 0 if toothEmbedded else 2` |
| `toothAxis` | the `{gearLabel} Tooth Axis` construction axis | `ConstructionAxis` | S14 | nothing — the one entry with no reader, listed so a regen that stashes the axis is not read as having invented a key |
| `gearOccurrence` | the `{gearLabel} Gear` occurrence | `Occurrence` | S16 | `moveToComponent`'s destination |
| `profileSketch` | the `{gearLabel} Profile` sketch | `Sketch` | S16 | the revolve's single profile |
| `shaftAxisEdge` | that sketch's first edge — A'->G / B'->I | `SketchLine` | S16 | the revolve axis; the pattern axis; the bore plane's path; the meshing rotation; the tooth-body hook's `shaftAxisEdge` |
| `gearBody` | the revolved Gear Body | `BRepBody` | S17 | the bore extrude's `participantBodies` |

Six values are used where they are made and **never** enter the dict, so a regen that stashes one has
invented an entry: the Root Axis (consumed inside S6 itself), the toe and heel cone points (the first
element of the two edge tuples), the shaft-edge point pair (the first two `hexVertices`), the virtual
tooth number and the root sink (computed in S11 and consumed in S13), and the meshing rotation angle
(computed in S25 from `teeth`).

`getOccurrence`, `addParameter` and `parameterName` are named only to say that bevel does not use
them. `deleteComponent` is a method this module DEFINES for the framework to call — the entry point
calls it on the generator when `generate(inputs)` raises — so the module never calls it itself; it is
named here because this step creates the occurrence that method deletes.

<!-- check-step-calls: ignore getOccurrence addParameter parameterName deleteComponent -->

**From:** `spec/bevelgear/instructions.md` L29-34, L380-488, L588-599, L636-647; `spec/bevelgear/fusion.md` L208-215;
`.claude/skills/generate-gear/PLAYBOOK.md` L17-36, L75-101, L811-837

## S4 `[GO]` Anchor sketch

Start the Anchor sketch, named `Anchor`, **directly on the user-selected target plane**, whether the
selection is a `ConstructionPlane` or a `PlanarFace`: `designComponent.sketches.add(targetPlane)`.
Do not re-derive or offset it — a coplanar construction plane created inside the sub-component
resolves in that component's own frame and collapses the whole build onto world XY
(`[PB-USE-SELECTED-PLANE]`).

Mark the center by projecting the user-specified center point into the sketch with
`sketch.project(centerPoint)`.

Draw the Anchor Line through the projected center:

- Create it with `sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)` from raw `Point3D`
  coordinates, seeding its two endpoints at **exactly ±0.5 cm** from the projected centre along the
  sketch-local X, so the seeded length is 10 mm.
- Apply **BOTH** `addCoincident(projectedCenter, anchorLine)` — the intersection, which pins the
  centre onto the line — **and** `addMidPoint(projectedCenter, anchorLine)`, which makes the centre
  bisect it. Use both, not the midpoint alone.
- Add an aligned distance dimension with `addDistanceDimension(start, end,
  adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)` and **do not assign
  `.parameter.value`**: the dimension simply locks the length at the seeded 10 mm. The value is
  arbitrary; nothing downstream reads it.
- Pin the direction with `addHorizontal(anchorLine)` — sketch-local, per `[PB-REFLINE-DIRECTION]`,
  which works on any tilted target plane where a world-axis lock would mis-orient it.

The line's absolute direction is arbitrary — S6 derives every direction relative to the projected
anchor line — but it must not be a free degree of freedom: midpoint plus length plus Horizontal
leaves zero.

**Stash the projected-centre `SketchPoint` on `self`** (`self._anchorCenterPoint`) so S6 re-projects
*this* point rather than the raw user selection.

End by gating the sketch: raise if `sketch.isFullyConstrained` is false, naming the sketch
(`[PB-FULL-CONSTRAINT]`, `[BEVEL-F-FULL-CONSTRAINT]`).

`sketch.project` is the call to write. The compiled API reference declares `project2(entities,
isLinked)` and no `project`, so every gate in this repo reports `project` as unverified; that report
is expected and is not a defect to fix here. The two are not interchangeable in any case —
`project2` takes a list and returns a list — so do not substitute it.

<!-- check-step-calls: ignore project2 -->

Proved by `stepAnchorSketch`. The proof writes the midpoint alone: the engine's midpoint is a
two-row constraint that already places the point on the line, so writing Fusion's point-on-line row
beside it is redundant there. It also seeds the user's Center Point off the sketch origin, which
nothing in the dialog forbids, and measures the same line with its direction constraint removed to
show the free rotation `[PB-REFLINE-DIRECTION]` exists to take out.

<!-- proof-run: proofkit.RunParallel(anchorCases, stepAnchorSketch) -->

**From:** `spec/bevelgear/instructions.md` L561-587, L648-653; `spec/bevelgear/fusion.md` L19-30; `.claude/skills/generate-gear/PLAYBOOK.md` L441-457, L501-516, L838-848

## S5 `[PROSE]` Gear Profiles plane

Create the construction plane the whole S6 lattice is drawn on:
`designComponent.constructionPlanes.createInput()`, then
`setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)`, then
`constructionPlanes.add(planeInput)`. Name it `Gear Profiles Plane`.

**Build it off the ORIGINAL `targetPlane`** as the reference. This is the second place the
target-plane orientation reaches the bodies; substituting a re-derived or offset plane here also
collapses the gear onto world XY (`[PB-USE-SELECTED-PLANE]`). Pass the `SketchLine` **directly** to
`setByAngle` — never wrap it in `Path.create` first, which raises an internal validation error
whenever the curve's owner sketch is not trivially resolvable in a multi-component context
(`[PB-CONSTRUCTION-PLANES]`).

Stash the plane as `self._gearProfilesPlane`.

`Path.create` is named only to forbid it.

<!-- check-step-calls: ignore Path.create -->

**From:** `spec/bevelgear/instructions.md` L654-657, L809-810; `.claude/skills/generate-gear/PLAYBOOK.md` L775-786, L838-848

## S6 `[GO]` Gear Profiles sketch — the S2 lattice

One sketch, named `Gear Profiles`, on the Gear Profiles Plane, holding the whole shared figure for
both gears. This is the largest step in the build and every rule in it is load-bearing.

**Three rules govern every line in this sketch.**

- **Every line here is a construction line** — `line.isConstruction = True` — the lattice lines, the
  toe lines M->N / O->P, the front faces N->A' / P->B', and the short reference and connector lines
  M->C, O->D, A'->G, B'->I, C->K', D->L' alike. The solid features later consume only the per-gear
  Profile sketches, never a S6 curve directly.
- **Every line uses the COINCIDENT style, never sharing** (`[BEVEL-F-COINCIDENT-STYLE]`): create the
  line from raw `Point3D` coordinates with `addByTwoPoints`, then pin each endpoint that already
  exists with exactly one `addCoincident(line.startSketchPoint, existingPoint)`. Never pass an
  existing `SketchPoint` into `addByTwoPoints` to share it. Sharing without a coincident leaves the
  sketch **under**-constrained; sharing *and* coinciding is redundant and the solve fails outright
  with `VCS_SKETCH_SOLVING_FAILED`. This covers the short reference lines too — a regen that shared
  only those came out about fourteen coincidents short.
- **Each named line is created ONCE and the reference is kept** (`[BEVEL-F-LINE-ONCE]`). When a later
  paragraph says "collinear to line A->E", it means the very line drawn earlier. Drawing a second
  line over the same segment over-determines the coupled net and the solve fails with
  `VCS_SKETCH_OVER_CONSTRAINTS`.

**Every length dimension in this sketch is `AlignedDimensionOrientation`.**
`addDistanceDimension(pointOne, pointTwo, orientation, textPoint)` takes an
`adsk.fusion.DimensionOrientations` value, and this figure has no axis-aligned line in it: the shaft
axes sit at the Shaft Angle to each other and the whole lattice tilts with the target plane. A
horizontal or vertical orientation would dimension the line's *projection* onto a sketch axis instead
of its length. Wherever a paragraph below says "a dimensional constraint with length = X" it means an
aligned distance dimension of that value. The offset dimensions are a different call,
`addOffsetDimension`, which takes no orientation.

**Every seed below is load-bearing geometry, not a convergence hint.** Fifteen constraint sites in
this lattice admit a mirrored solution that satisfies every constraint, and no Fusion constraint pins
any of them: a flipped seed solves cleanly and the wrong gear gets built rather than refused
(`[BEVEL-F-MIRROR-FIGURE]`). Never add a Fusion constraint to pin a side — `addSymmetry` on C and D
against the Pitch Line rules out the collapse but not the swap and replaces a dedendum's
perpendicular plus length, and `SketchPoint.isFixed` over-constrains and turns the parametric lattice
into placed geometry. Both are refused, and the end-of-step gate is what catches a flip instead.

### The construction, in order

Let `c` be the projected centre and `d` the projected anchor line's 2-D unit direction, with
`perp = (-d.y, d.x)`. **Pick `perp`'s sign by the target-plane normal, read as
`targetPlane.geometry.normal` for BOTH selection kinds** — a `BRepFace`'s `geometry` and a
`ConstructionPlane`'s `geometry` are each a `core.Plane` carrying `.normal` — and never by the
sketch's local +Y, which maps to different world sides depending on how the plane was oriented
(`[BEVEL-F-GROW-SIDE]`). That one-bit comparison is the only permitted world use in this step; every
**position** below is sketch-local (`[BEVEL-F-APEX-LOCAL]`).

Write the closed forms once and reuse them: `PPD = Module * Pinion Gear Teeth Number`,
`DPD = Module * Driving Gear Teeth Number`,
`tan gamma_p = sin(Sigma) * PPD / (DPD + PPD * cos(Sigma))`, `gamma_g = Sigma - gamma_p`,
`R = (PPD / 2) / sin(gamma_p)`.

1. **Project the Anchor sketch's centre `SketchPoint`** — the one stashed in S4, not the raw
   user-selected point. Both are coincident, but projecting the anchor-sketch point keeps the chain
   inside the Design component; projecting the raw external point is a cross-component reference and
   can resolve inconsistently. `sketch.project(self._anchorCenterPoint)`, and project the Anchor Line
   as well so there is something to take the perpendicular against.
2. **centre -> Apex.** A construction line from the projected centre, perpendicular to the projected
   anchor line (`addPerpendicular`). Seed its far end at
   `c + perp * (R * cos(gamma_g) + <resolved Driving Gear Base Height>)`. **Not `c + perp * DPD`**,
   which earlier revisions said: the net closes this line at `R * cos(gamma_g)` above point I plus
   that base height, so on the default 31/31 pair at Shaft Angle 90 the old seed sat 11.6 mm past
   where the solve puts it, and a seed that disagrees with its own closure by that margin is a seed
   waiting to pick the wrong branch (`[PB-SEED-NEAR]`). **Do not add a length constraint on this
   line.** Pin its start with exactly one `addCoincident` to the projected centre.
3. **Driving Gear Shaft Axis**, from the Apex back toward the anchor line, i.e. in the `-perp`
   direction. Seed its far end at `apex - perp * (R * cos(gamma_g))`, which is
   `c + perp * <resolved Driving Gear Base Height>` — measure from the Apex, not from `c`. Apply
   `addParallel(drivingShaftAxis, centerToApex)`. **Do NOT use `addVertical`**, which forces the line
   to the sketch's world-vertical and is wrong on a tilted target plane. Coincident at the Apex. The
   far end is **point B**. Do not dimension the length.
4. **Pinion Gear Shaft Axis**, the driving direction rotated about the Apex by the Shaft Angle.
   Rotating has two senses and they place point A on opposite sides. **Form BOTH candidate A
   positions — the driving direction rotated by +Shaft Angle and by -Shaft Angle — and keep the one
   whose endpoint has the greater X in this sketch.** Compare the two and take the larger; do not
   rotate one fixed sense and flip only when its X comes out negative, because when both candidates
   have a positive X that shortcut keeps the wrong one. Call the chosen unit direction `pinionDir`.
   Coincident at the Apex; the far end is **point A**; no length dimension.
5. **The Shaft Angle dimension.** `addAngularDimension(drivingShaftAxis, pinionShaftAxis, textPoint)`
   set to the Shaft Angle. **Place the text point inside the wedge so it measures Sigma and not its
   supplement** (`[PB-ANGULAR-DIM]`): the interior bisector,
   `apex + normalize(pinionDir + drivingDir) * (PPD / 4)`, where `drivingDir` is the unit Apex->B
   direction. The angular dimension fixes the magnitude only; the pinion's side is held by the seed
   above together with the Apex 2 closure below.
6. **The A->Apex 2 drop.** From A, a construction line perpendicular to the Pinion Gear Shaft Axis,
   **drawn toward the OTHER shaft axis / point B** — Apex 2 sits in the interior wedge *between* the
   two axes. Pick the perpendicular sense by the sign of its dot product with the A->B direction, not
   against a "toward the anchor line" reference. `addPerpendicular` against the pinion shaft axis, and
   an aligned distance dimension of length `Pinion Gear Pitch Diameter / 2`. Coincident at A.
   **Naming convention used throughout: "A->Apex2" always means THIS drop line and never the Apex->A
   shaft axis.** The two share point A and are different lines. The same holds for "B->Apex2" against
   Apex->B.
7. **The B->Apex 2 drop.** From B, perpendicular to the Driving Gear Shaft Axis, **toward point A** —
   pick the sense by the sign of its dot with the B->A direction. **Do NOT choose this sense by a
   "toward the anchor line" reference**: the Driving Gear Shaft Axis is itself parallel to that grow
   direction, so the perpendicular's dot with it is about 0, a degenerate test that silently selects
   an arbitrary side. Both drops must aim at the *same* interior-wedge point; if one seeds Apex 2 on
   the wrong side, the coincidence below makes the solver flip the whole figure to the mirror
   solution and A, C, D, G, H, K, M, N and A' all land at negative X. `addPerpendicular` against the
   driving shaft axis, aligned distance dimension of `Driving Gear Pitch Diameter / 2`, coincident at
   B.
8. **Close them.** `addCoincident` the two drops' far endpoints. That point is **Apex 2**. At Shaft
   Angle 90 the four points Apex, A, Apex 2, B form a rectangle; at other angles a non-rectangular
   quadrilateral whose along-shaft lengths adjust so the two drops coincide.
9. **Seed the along-shaft lengths** with the closed-form cone geometry so the solver converges on the
   right branch for any Sigma. These are seed coordinates only — the lengths stay undimensioned:
   `|Apex->A| = R * cos(gamma_p)` and `|Apex->B| = R * cos(gamma_g)`. Both cosines are positive for
   every Shaft Angle the range check admits, which is what the Maximum Shaft Angle guarantees.
   Seeding A and B merely by a pitch diameter is wrong for any Sigma other than 90 and can send the
   solver to the wrong branch.
10. **The Pitch Line**, Apex to Apex 2, each end coincident to its point.
11. **The two dedendum lines.** From Apex 2, two construction lines perpendicular to the Pitch Line,
    each with an aligned distance dimension of `Module * 1.25`. The one whose direction `u` satisfies
    `u . <unit Apex->A> > 0` is the **Pinion Gear Dedendum**, ending at **point C**; its negation,
    which satisfies `(-u) . <unit Apex->B> > 0`, is the **Driving Gear Dedendum**, ending at **point
    D**. Those two dot products are exactly `sin(gamma_p)` and `sin(gamma_g)`, strictly positive for
    every admitted configuration — unlike the anchor-line test step 7 warns about, which reads about
    0 by construction. Seed `C = Apex2 + 1.25 * Module * <pinion dedendum direction>` and
    `D = Apex2 + 1.25 * Module * <driving dedendum direction>`.

    **These two sites are where "C collapses onto D" lives, and each is held by its seed alone.** The
    perpendicular fixes the direction and the dimension the magnitude; neither picks a side. Flip the
    pinion seed and C solves exactly onto D; flip the driving seed and D solves onto C. The collapsed
    figure inverts that gear — the toe ends up *outside* the heel, the revolved frustum is degenerate,
    and the conical end cut finds no cone face at the toe midpoint.
12. **The two Root Axes**, Apex->C and Apex->D, coincident at both ends. They are consumed inside
    this step by the toe-line constraints and never enter the per-gear dict.
13. **Point E.** From A, a construction line collinear with Apex->A. Seed its far end at
    `E = A + <unit Apex->A> * (1.25 * Module * sin(gamma_p))`, so
    `|Apex->E| = R * cos(gamma_p) + 1.25 * Module * sin(gamma_p)`. **Do not dimension it.**
    `addCollinear` against Apex->A, and coincident at A. E is the foot of the perpendicular dropped
    from C onto the pinion shaft axis, which is what closes it.
14. **Line C->E**, coincident at both ends, with `addPerpendicular(A->E, C->E)`.
15. **Point F**, the driving twin: from B, collinear with Apex->B, seeded at
    `F = B + <unit Apex->B> * (1.25 * Module * sin(gamma_g))`, undimensioned; then **line D->F** with
    `addPerpendicular(B->F, D->F)`.
16. **Point G.** From E, a construction line collinear with **line A->E** — the collinear names A->E
    and **never the Apex->A shaft axis further up the chain**, even though both describe the same
    infinite line (`[BEVEL-F-COLLINEAR-CHAIN]`, `[PB-COLLINEAR-CHAIN]`; naming the axis raises
    `VCS_SKETCH_OVER_CONSTRAINTS`). Seed at `G = A + <unit Apex->A> * <resolved Pinion Gear Base
    Height>`, undimensioned. Coincident at E.
17. **Point H.** From C, a line seeded at
    `H = Apex2 + <unit Apex2->C> * (<resolved Pinion Gear Base Height> / sin(gamma_p))`,
    undimensioned, coincident at C, collinear with **line Apex2->C** — the Pinion Dedendum line C is
    the endpoint of.

    **These two seeds pick the side of the pinion base-height offset below, which is unsigned.** Flip
    them and G sits one base height on the Apex side of A instead of beyond it, H follows, and the
    pinion's heel end folds back inside the figure.
18. **Line G->H**, coincident at both ends, **with `addPerpendicular(E->G, H->G)`**. That
    perpendicular is required in Fusion and is not optional: `addOffsetDimension` is a distance
    dimension whose documentation requires the second entity to be a line parallel to the first, and
    it controls only the perpendicular distance, so the parallelism has to exist before the offset can
    be applied at all. E->G runs along the pinion shaft, so making H->G perpendicular to it makes
    H->G parallel to the A->Apex2 drop. Perpendicular plus offset is two equations for two freedoms
    and nothing is redundant.
19. **Point I** and **point J**, the driving twins of G and H: I from F collinear with **line B->F**,
    seeded at `I = B + <unit Apex->B> * <resolved Driving Gear Base Height>`; J from D, seeded at
    `J = Apex2 + <unit Apex2->D> * (<resolved Driving Gear Base Height> / sin(gamma_g))`, collinear
    with **line Apex2->D**. Then **line I->J** with `addPerpendicular(F->I, J->I)`.

    **The driving pair's seeds carry the whole figure**, because step 22 hangs everything off I. Flip
    the driving base-height offset's side and the entire lattice drops by twice the resolved Driving
    Gear Base Height, gear and pinion together, with every relative length still correct — which is
    why nothing downstream refuses it.
20. **The driving base-height offset.** `addOffsetDimension(<the B->Apex2 drop line>, <line J->I>,
    textPoint)` with `.parameter.value` set to the **resolved** Driving Gear Base Height. J->I is
    already parallel to the drop by construction, so add **no** extra parallel constraint
    (`[PB-OFFSET-DIM]`). The value is the one after the driving Maximum Base Height has been applied.
    `addOffsetDimension` is unsigned and does not pick which side J->I lands on — the I and J seeds
    are the only thing that does.
21. **The pinion base-height offset.** `addOffsetDimension(<the A->Apex2 drop line>, <line G->H>,
    textPoint)` set to the **resolved** Pinion Gear Base Height. Already parallel by construction, so
    again no parallel constraint. Unsigned in the same way; the G and H seeds pick its side.
22. **Line A'->G**, the hexagon's shaft-axis edge. It starts at the front face's foot **A'**, not at
    A; the two coincide at Toe Extension 0. **This line is what CREATES A'** — nothing above it does —
    so draw it with its start seeded at `A' = Apex + <unit Apex->A> * <the along-shaft coordinate of
    N>`, the foot of the perpendicular from N onto the pinion shaft axis. Coincident at G. The front
    face in step 29 is what pins A' to the axis; until then A' is a free endpoint sitting at its seed.
    Draw it **here** rather than after the front face, so the hexagon's edges are created in the walk
    order A' -> G -> H -> C -> M -> N that the Profile sketch's first-edge rule depends on.
23. **Constrain point I with the projected centre** — `addCoincident(I, projectedCenter)`. This is
    what closes the one remaining freedom, the Apex's distance along `perp`.
24. **Point K.** A construction line from G extending along Apex->A, ending at K. **Pin K with two
    point-on-line coincidents** — `addCoincident(K, <line Apex->A>)` and `addCoincident(K, <the
    Pinion Dedendum line Apex2->C>)` — and **not** `addCollinear`: by the time K is added G and C are
    already fixed, so a collinear over-constrains and Fusion errors. Draw the reference line C->K.
25. **Tooth-centre point K'.** The tooth in S13 is centred not at K but at K', K shifted outward
    along the dedendum line by **Tooth Spacing**, away from the lower corner C.
    - **At Tooth Spacing 0 — the default — build nothing here.** Set K' identical to K and reuse the
      existing C->K line. A zero-length dimensioned line is degenerate, and one segment gets one line.
    - Above 0: draw a construction line starting at K, seeded at
      `K' = Apex2 + <unit Apex2->C> * (<the pinion's virtual pitch radius> + Tooth Spacing)`.
      **"Virtual pitch radius" here is the exact back-cone radius
      `(Pinion Gear Pitch Diameter / 2) / cos(gamma_p)` that S11 step 1 defines, and never a radius
      rebuilt from a tooth count**: reading the term as a rounded count times half a Module puts the
      seed 0.4203 mm short on the shipped default geometry, which is 420 times the gate tolerance
      below. Pin it the same way K is pinned — `addCoincident(start, K)` and
      `addCoincident(K', <the Pinion Dedendum line>)` — then add an aligned **length dimension of
      Tooth Spacing** on this line. No `addCollinear`, for the same over-constraint reason as K.

      **That length dimension is unsigned**, so the point-on-line pin plus the length admit K' one
      Tooth Spacing on the C side of K just as readily — the two candidates sit `2 x Tooth Spacing`
      apart — and this seed is the only thing that rules the wrong one out. A flipped K' tightens the
      mesh by the clearance the input asked to add and builds a gear that looks right.

      Finally draw the tooth-centre reference line **C->K'** for S12 and S14 to use in place of C->K.

    Build it here, inside this sketch, before the end-of-step gates, so they cover it. Only the
    tooth's centre moves; the virtual tooth number and the drawn tooth size are unchanged.
26. **Resolve the Maximum Face Width and apply it.** A, B, C, D, H and J now exist and are solved, so
    read their **solved** `pointA.geometry`, `pointB.geometry`, `pointC.geometry`, `pointD.geometry`,
    `pointH.geometry`, `pointJ.geometry` — **not** the pre-solve seed coordinates
    (`[PB-SOLVED-GEOMETRY]`). The bound is `0.95 *` the smaller of the perpendicular distance from A
    to the line through C and H, and from B to the line through D and J. Compute **both** and take the
    minimum: the pinion is only *usually* the smaller, binding side, and written with the pinion's
    diameter by name the bound is wrong whenever the driving gear carries the smaller tooth count. On
    a Driving 17 / Pinion 31 pair at Module 1 the real bound is 3.883 mm against the pinion form's
    13.591, so the naive `Cone Distance / 6` default exceeds it and the gear fails to generate for any
    ratio above roughly the square root of 2.

    Then resolve the Face Width: `min(Cone Distance / 6, Maximum Face Width)` when the input is 0, and
    reject a user value above the maximum with a message stating it. `Cone Distance` here is
    `sqrt(PPD**2 + DPD**2)`, the diagonal of the two pitch diameters — **not** the Pitch Cone Distance
    `R`. The two coincide as `Cone Distance = 2 * R` exactly when the Shaft Angle is 90 and diverge
    everywhere else. Stash the result as `self._faceWidthResolved_cm`.
27. **Resolve the Root Length and each gear's Maximum Bore Diameter.** With the Face Width resolved:
    - `|Apex->Ded| = sqrt(R**2 + (1.25 * Module)**2)`; the root cone angle
      `gamma_root = gamma - atan(1.25 * Module / R)`.
    - At Toe Extension 0 the Root Length is `Face Width * |Apex->Ded| / R`, the Face Width re-measured
      along the root element rather than perpendicular to the pitch line.
    - Each gear's **Toe Radius**: a user value, or when the input is 0 the auto value
      `<this gear's Pitch Radius> - Face Width / sin(gamma)`, which is the inner toe corner radius at
      Toe Extension 0 and the one value that reproduces today's profile exactly. A user value must be
      **strictly below** that gear's **Toe Radius Ceiling**
      `(<Pitch Radius> - 1.25 * Module * cos(gamma)) * (1 - Face Width / R)`; reject it naming the
      ceiling. Stash the two as `self._drivingToeRadiusResolved_cm` and
      `self._pinionToeRadiusResolved_cm`.
    - Each gear's **Toe Limit** `|Ded->X| = sqrt(R**2 + (1.25 * Module)**2) - Toe Radius /
      sin(gamma_root)`, where X is the point on the root element at that gear's Toe Radius.
    - At a positive Toe Extension the Root Length is the Toe Extension 0 value plus that percentage of
      **0.99** of the way to the **smaller** of the two gears' Toe Limits. The smaller wins because the
      pair shares one root length. **Do not drop the 0.99**: at the limit exactly the toe face has zero
      length, the revolved body carries no cone at its toe end, and the conical end cut in S18 — whose
      toe cut must split or the build fails — has no cone face to find. The last percent is worth well
      under a tenth of a millimetre of root length on every case in the proof's table.
    - **A defaulted Toe Radius can leave no room at all, and that is a real configuration rather than
      a defect.** On a driving gear with a large pitch cone angle the inner toe corner already sits at
      a larger radius than the outer one, so X falls behind the toe corner and the Toe Limit comes out
      below the Toe Extension 0 root length. **Reject a Toe Extension above 0 on such a pair**, naming
      the gear and the Toe Radius Ceiling it needs to come below. Toe Extension 0 still resolves, so
      the gear stays buildable exactly as before. Do **not** silently substitute a smaller Toe Radius.
    - **Maximum Bore Diameter**, per gear and only when Enable Bore is checked:
      `r_heel = <Pitch Radius> - <resolved Base Height> / tan(gamma)`,
      `r_toe = (|Apex->Ded| - Root Length) * sin(gamma_root)`, and the bound is
      `2 * 0.95 * min(r_heel, r_toe)`. Cap an auto-calculated bore — `min(<Pitch Diameter> / 4, <the
      bound>)` — and reject a user value above it naming the maximum. This is the only step at which
      the whole bound can resolve, which is why S2 deliberately leaves the bore diameters unbounded.
      Write the result into each gear's dict as `boreDiameter_cm`; no later reader re-derives it.

      The flat **front** face is deliberately not protected: its radius is the Toe Radius, which is
      not on the body's outer envelope, so a bore wider than it only exits through the toe cone
      instead of through that face and the frustum stays whole. A bore past `r_heel` or `r_toe` does
      not — it cuts a corner off the profile being revolved.
28. **Line M->N, the pinion toe line. Seed BOTH ends at their closed-form solved positions, not near
    them** (`[PB-SEED-NEAR]`). Seed M on Apex->C at the fraction `1 - <Root Length> / |Apex->C|` from
    the Apex. Then seed N by sliding from that M seed along the **C->H** direction by exactly

        (<M seed's perpendicular distance from the Pinion Gear Shaft Axis> - <Pinion Gear Toe Radius>)
        / cos(gamma_p)

    **The slide runs in the C->H sense — from C toward H, the same outward sense as Apex2->C continued
    — and along it the perpendicular distance from the shaft axis FALLS at `cos(gamma_p)` per unit**
    while the along-shaft coordinate rises at `sin(gamma_p)`. That is why the quantity is divided by
    `cos(gamma_p)`: the slide gives back exactly `<M seed's perpendicular distance> - <Toe Radius>` of
    perpendicular distance, so N lands at the Toe Radius. **Read as a rise it is the wrong sign**, and
    that is the natural misreading — C->H runs outward from the figure, so "slide outward" reads as
    "move away from the axis", while the dedendum line leans back toward the axis as it goes. The
    compile round that first wrote this rule reported 20 of its 21 lattice cases failing to converge
    until it corrected the sign. The pinion's C->H unit direction is
    `sin(gamma_p) * <unit Apex->A> - cos(gamma_p) * <the A->Apex2 drop direction>`, and the negative
    second term is the whole of the rule.

    **A wrong seed here builds the wrong gear rather than failing to converge.** N's position is fixed
    by the toe line together with a LENGTH dimension on the front face, and a length is unsigned: the
    toe line meets the Toe Radius on BOTH sides of the shaft axis, so the solver takes whichever side
    the seed starts on. Seeded below the axis it converges happily onto the mirror, N comes out on the
    far side, the revolved hexagon crosses its own axis of revolution, and Fusion aborts the revolve
    with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`) at S17, pointing at the revolve rather than at the seed.

    Two earlier seeding rules do exactly that, so do not reinstate either: sliding from the M seed by
    the **Root Length**, and sliding by the **distance from the M seed to A**. Both were written for
    the scheme that pinned N to the A->Apex2 drop. Measured on the shipped default pair at a Toe
    Extension of 50%, the Root Length slide puts the N seed at a perpendicular distance of **-0.27 mm**
    from the shaft axis — past it — against a solved N at **+5.17 mm**. Do not seed M and N just
    `Face Width` away from C and H either; that starts N near H, far from its constraint target.

    Then apply **exactly these three constraints**, all of which are required:
    - `addCoincident(M, <the Pinion Root Axis Apex->C>)`;
    - `addParallel(<M->N>, <C->H>)`;
    - `addOffsetDimension(<C->H>, <M->N>, textPoint)` with `.parameter.value` set to the Root Length
      re-measured perpendicular to the pitch line, i.e. `Root Length * R / |Apex->C|`. An offset
      dimension controls a perpendicular distance, so it carries the root length in that form; at Toe
      Extension 0 the value is exactly the resolved Face Width. Place the text point in the gap between
      C->H and M->N on the Apex side — the midpoint of the M seed and point C — so the dimension reads
      cleanly (`[PB-OFFSET-DIM]`).

    **The toe's side relative to the heel is held by the M seed and by nothing else.**
    `addOffsetDimension` is unsigned, so a correctly built frame still admits M->N one root length on
    the *far* side of C->H, where the toe lands outside the heel and the revolved frustum is
    degenerate. The text point does not control it either.

    Let the line's start be **M** and its end **N**. Draw the reference line **M->C**.
29. **The front face N->A', which is what holds N.** N is **NOT** pinned to the A->Apex2 drop. Earlier
    revisions pinned it there, which fixed its station at A's; it now rides the **Pinion Gear Toe
    Radius** instead.
    - Draw a line from N to A' (both endpoints already exist, so one `addCoincident` per end).
    - `addCoincident(A', <the Apex->A shaft axis>)` — A' is the only toe-end point that touches that
      axis, and it is a *foot*, not a corner.
    - `addPerpendicular(<N->A'>, <the Apex->A shaft axis>)`, so the revolve sweeps the front face into
      a flat annulus.
    - An aligned distance dimension on the whole line N->A' set to the **resolved Pinion Gear Toe
      Radius**.

    **Pinning N itself to the Apex->A shaft axis remains forbidden**: that would put N *on the axis of
    revolution*, and the later conical split fails with `ASM_API_FAILED` for asymmetric tooth counts
    even though the symmetric case happens to survive. A' sits on the axis; N never does, because the
    Toe Radius is strictly positive. Those three rows plus the offset and the root-axis coincident
    fully constrain M, N and A' — six freedoms, six constraints.

    **A' replaces A as the hexagon's first vertex.** At Toe Extension 0 with a defaulted Toe Radius the
    two coincide exactly; a positive Toe Extension walks A' along the shaft axis toward the Apex and the
    shaft edge grows by that much.
30. **Point L and L'**, the driving twins of K and K': a construction line from I along Apex->B to L,
    pinned with `addCoincident(L, <line Apex->B>)` and `addCoincident(L, <the Driving Dedendum line
    Apex2->D>)` and no collinear; the reference line D->L; and, when Tooth Spacing is above 0, the
    L->L' line seeded at
    `L' = Apex2 + <unit Apex2->D> * (<the driving gear's virtual pitch radius> + Tooth Spacing)` with
    the same length dimension and the same unsigned-length hazard, taking the virtual pitch radius as
    the exact back-cone radius `(Driving Gear Pitch Diameter / 2) / cos(gamma_g)`. The reference line
    for S12 is **D->L'**.
31. **Line O->P**, the driving mirror of M->N, seeded the same way: O on Apex->D at the fraction
    `1 - <Root Length> / |Apex->D|`, then P slid from that O seed along **D->J** by
    `(<O seed's perpendicular distance from the Driving Gear Shaft Axis> - <Driving Gear Toe Radius>)
    / cos(gamma_g)`. The falling-distance rule carries over word for word, and reading it as a rise is
    the same wrong sign. Then `addCoincident(O, <the Driving Root Axis Apex->D>)`,
    `addParallel(<O->P>, <D->J>)` and `addOffsetDimension(<D->J>, <O->P>, textPoint)` set to the same
    perpendicular form of the Root Length, with its text point in the gap on the Apex side of D->J.
    The start is **O**, the end **P**. Draw the reference line **O->D**.
32. **The driving front face P->B'**, exactly as N->A' with B for A, P for N and the **Driving Gear
    Toe Radius**: the line P->B', `addCoincident(B', <the Apex->B shaft axis>)`,
    `addPerpendicular(<P->B'>, <the Apex->B shaft axis>)` and a length dimension on P->B'. P is never
    pinned to the shaft axis; only B' touches it. Then draw the line **B'->I**.

### The two end-of-step gates

First, the full-constraint gate: raise if `sketch.isFullyConstrained` is false, naming
`Gear Profiles` (`[BEVEL-F-FULL-CONSTRAINT]`). Do **not** reach full constraint by dimensioning the
driven lengths — Apex->A, Apex->B and the extension lines are determined by the perpendicular,
collinear and closing constraints (`[BEVEL-F-DRIVEN-DIMS]`, `[PB-NO-OVERCONSTRAIN]`).

Then **gate the solved figure against its own seeds** (`[BEVEL-F-SEED-HELD]`). Compare every named
point's solved `.geometry` against the closed-form position this step seeded it at, **in the sketch's
own 2-D frame with no world round-trip**, at a tolerance of **0.001 mm (1e-4 cm in internal units)**,
and **raise** naming the first point that has moved with both positions. Check them in creation
order:

    Apex, B, A, Apex 2, C, D, E, F, G, H, I, J, K, K', M, N, A', L, L', O, P, B'

so the message names the earliest site that flipped rather than a downstream symptom. **The list is
22 points only when Tooth Spacing is above zero. At Tooth Spacing 0 — the default — K' and L' are not
built at all, so drop those two and compare 20**; comparing a K' that was never created is the one
way this gate can raise on a correct figure.

This gate is the only measure that catches all 1024 figures the default configuration admits (8192 on
a 43/31 pair at Shaft Angle 75 with Tooth Spacing above zero). **Never treat the revolve's
`ASM_WIRE_X_AXIS` as the tripwire**: several of the flips build a valid-looking gear on the wrong side
and reach no error at all.

`addVertical`, `addCollinear`, `addSymmetry` and `isFixed` are named in this step only to forbid them
at the sites that name them; `addCollinear` is required at steps 13, 15, 16, 17 and 19 and is
therefore a real call there.

<!-- check-step-calls: ignore addVertical addSymmetry isFixed -->

Proved by `stepGearProfiles`, which builds this lattice in the standalone sketch engine, solves it,
and holds it to this same seed gate over the whole admitted regime. Three constraints Fusion needs
are left out there, each because the engine's counterpart carries rows Fusion's does not and writing
Fusion's arity makes the lattice redundant at DOF 0: the two base-height perpendiculars of steps 18
and 19, the two toe-line parallels of steps 28 and 31, and the second row of step 23's coincidence.
The proof's own file says so at each site. **What no proof can reach is the seed**: it seeds at the
closed form, which is the rule above, so it proves the constraints solve from a correct seed and never
that the generated module's seed is correct — which is why this gate has to live inside the module.

Shaft Angle 30 and Shaft Angle 150 stay in the proof's case table as **declared refusals**: this
particular net's conditioning reads 2.93e-5 and 3.99e-5 there, below the sketch engine's 4e-5 trust
floor, and the step requires exactly that verdict rather than avoiding the case. That is a property
of the net and not a bound on this spec, so **do not narrow the Shaft Angle range on it**.

<!-- proof-run: proofkit.RunParallel(latticeCases, stepGearProfiles) -->

**From:** `spec/bevelgear/instructions.md` L1-28, L99-221, L561-587, L654-806, L1297-1323, L1388-1436;
`spec/bevelgear/fusion.md` L1-18, L69-206; `.claude/skills/generate-gear/PLAYBOOK.md` L359-431, L441-516, L591-651

## S7 `[PROSE]` Resolve the virtual-spur dimensions for one gear

Run S7 through S15 **once per gear, pinion first, then driving**, with that gear's own parameters:
the pinion uses tooth centre **K'**, reference line **C->K'** and **gamma_p**; the driving gear uses
**L'**, **D->L'** and **gamma_g**. At Tooth Spacing 0 those are exactly K, C->K, L and D->L.

Compute this gear's **virtual (back-cone, Tredgold) tooth number from the closed form**, never by
measuring Apex2->K':

    virtual pitch radius = (<this gear's Pitch Diameter> / 2) / cos(gamma)
    virtual tooth number = 2 * virtualPitchRadius / Module        (equivalently teeth / cos(gamma))

**It is a real number and is NEVER rounded** — not floored, not ceiled, not cast to an int. The
Tredgold construction puts the equivalent spur gear's pitch radius exactly at the back-cone distance
`r / cos(gamma)` with this gear's own module, and `z_v = z / cos(gamma)` is a real number in every
published form of it. Rounding rebuilds every drawn circle from the rounded count, which draws the
tooth smaller than the back cone places it: on the shipped default — 31 teeth, Module 1, Shaft Angle
90, gamma 45 — the exact virtual pitch radius is 21.9203 mm, a floored count of 43 draws 21.5 mm, and
the working addendum falls to 0.5797 mm against a nominal 1.0 module.

The real count reaches the spur drawer only as an angular half-thickness: the drawer reads
`ToothNumber` as a float and uses it in one place, `pi / (2 * toothNumber)`. With
`z_v = 2 * r_v / Module` that angle gives a tooth thickness of `pi * Module / 2` at the pitch circle,
the standard thickness. An integer count drawn at the exact radius gives `pi * r_v / round(z_v)`
instead, which misses nominal by a different amount on each member of an unequal pair.

**Root sink.** Draw the root circle one root sink `0.05 * 2.25 * Module` **inside** the dedendum
corner rather than at it. At the corner exactly, the tooth's root arc touches the gear body's root
cone only where the arc crosses the tooth's own centreline — the tooth is drawn on the back-cone
plane, so only a point on that centreline rides the cone its own polar radius names — and the arc's
two corners stand outside it, by 0.002 module on the default pair and 0.027 module on a 4/4 pair, the
largest of any pair this spec admits. The sink pushes the whole arc inside, so the Combine-Join meets
the gear body across the root rather than along one line.

So the four circles the proxy is asked for are:

| circle | radius |
|---|---|
| pitch | `virtualPitchRadius` |
| base | `virtualPitchRadius * cos(20 deg)` |
| tip | `virtualPitchRadius + Module` |
| root | `virtualPitchRadius - 1.25 * Module - rootSink` |

**Units — pin the cm-to-mm conversion.** The stashed pitch diameters are internal **cm** while Module
is the raw **mm** value, so compute
`virtualPitchRadius_mm = (pitchDia_cm * 10 / 2) / cos(gamma)` — the `* 10` converts cm to mm — and
then `virtualTeeth = 2 * virtualPitchRadius_mm / Module`. Skipping the times ten makes the virtual
tooth count about ten times off.

Neither the virtual tooth number nor the root sink enters the per-gear dict: both are computed here
and consumed in S13, inside the same per-gear pass.

**From:** `spec/bevelgear/instructions.md` L807-834, L1258-1296; `.claude/skills/generate-gear/PLAYBOOK.md` L188-203

## S8 `[PROSE]` `{gearLabel} Plane` — the tooth plane

Create a construction plane that includes this gear's tooth-centre reference line — C->K' on the
pinion, D->L' on the driving gear — perpendicular to the Gear Profiles sketch plane. Use the
framework helper `plane_by_angle(designComponent, toothCenterRefLine, gearProfilesPlane, 90)` from
`.solids`, which builds it with `setByAngle`; do **not** re-implement the pattern. Name it
`{gearLabel} Plane` and write it into the per-gear dict as `toothPlane`.

Pass the sketch line **directly**; never wrap it in `Path.create` first
(`[PB-CONSTRUCTION-PLANES]`).

<!-- check-step-calls: ignore Path.create -->

**From:** `spec/bevelgear/instructions.md` L531-545, L809-810, L835; `.claude/skills/generate-gear/PLAYBOOK.md` L159-186, L775-786

## S9 `[GO]` `{gearLabel} Tooth` sketch

Create a sketch named `{gearLabel} Tooth` on the `{gearLabel} Plane`, and draw the virtual spur
tooth into it with the **borrowed** spur tooth generator:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth, rootSink_mm=rootSink)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))
```

`anchorPoint` is this gear's `toothCenterPoint` — K' on the pinion, L' on the driving gear. Import
`VirtualSpurProxy` from `.spurproxy` and `SpurGearInvoluteToothDesignGenerator` from `.spurgear`;
bevel defines **no** local proxy or value-wrapper class of its own. The proxy's defaults already
match bevel — pressure angle 20 degrees, which is not a bevel dialog input, and 15 involute steps.

**The 180 degree rotation is delivered through the `draw()` angle argument**, not through a post-hoc
Move or sketch rotation: the spur generator rotates the whole tooth by that angle, which relies on
spur's radial flank-to-root pinning so the connecting lines rotate with the tooth.

**After `draw()` returns, read `proxy._lastToothEmbedded` back** and write it into the per-gear dict
as `toothEmbedded`. The spur generator decides during `draw()` whether the tooth is *embedded* — tip,
root and flanks meeting with no connecting lines — and records it on the proxy, which pre-initialises
the slot to absorb that write. **This flag is not optional bookkeeping**: it is the deterministic
selector for the tooth loop's line count in S17, and skipping it grabs an unrelated loop and the
apex-to-tooth loft dies with `LOFT_NO_TOOLBODY`.

**Do NOT hard-gate this sketch.** Log it with `futil.log` if `toothSketch.isFullyConstrained` is
false, and never raise: the two tooth-profile sketches are the declared exemption to the
full-constraint gate, because the drawer labels each of its four circles with along-path sketch text
and sketch text holds a degree of freedom (`[PB-TEXT-HOLDS-DOF]`, `[BEVEL-F-FULL-CONSTRAINT]`). **That
exemption covers the labels and nothing else** — it is never licence for loose geometry. The
tooth-top arc is a centre-point arc created with `addByCenterStartEnd` whose copied centre the drawer
pins with `addCoincident(arc.centerSketchPoint, localOrigin)`; that is the shared generator's own
construction (`[SPUR-F-TOOTHTOP-ARC]`) and bevel authors no arc of its own here. Never write that
Fusion draws this arc as a three-point arc and dimensions its radius — that sentence sat in a proof
comment once and sent an investigation chasing a reflected centre the construction has no room for.

Write the sketch into the per-gear dict as `toothSketch`.

`isFullyConstrained` appears here only as a logged read, and the two names below are the borrowed
generator's surface rather than calls this module makes into Fusion.

<!-- check-step-calls: ignore addByCenterStartEnd -->

Proved by `stepToothProfile`, which reproduces the drawn tooth at each gear's exact virtual tooth
count with the sink applied, takes the embedded flag from the sunk root radius, writes the drawer's
own centre coincidence — which the engine solves, so the cost at that site is nil — and requires the
tooth loop to carry the curve counts S17 selects on. What it cannot reach is Fusion's own profile
finder selecting the loop and the loft consuming it.

<!-- proof-run: proofkit.RunParallel(toothCases, stepToothProfile) -->

**From:** `spec/bevelgear/instructions.md` L600-635, L836-837, L1324-1361; `spec/bevelgear/fusion.md` L31-58;
`.claude/skills/generate-gear/PLAYBOOK.md` L517-532, L682-692

## S10 `[PROSE]` `{gearLabel} Tooth Axis`

Create a construction axis through this gear's tooth-centre point, normal to the plane the tooth
profile was drawn on: `designComponent.constructionAxes.createInput()`, then
`setByTwoPlanes(planeOne, planeTwo)`, then `constructionAxes.add(axisInput)`. Name it
`{gearLabel} Tooth Axis` and write it into the per-gear dict as `toothAxis`.

The two planes are the **Gear Profiles plane** and a **helper plane** built with
`setByDistanceOnPath(<this gear's tooth-centre reference line>, 1.0)` — perpendicular to that line at
its far end, the tooth-centre point. Their intersection is the line through the tooth centre normal
to the tooth plane.

Use `setByTwoPlanes` and not `setByPerpendicularAtPoint`, which would need a `BRepFace` this step
does not have (`[PB-CONSTRUCTION-AXES]`). Creating this axis in the never-activated Design component
is proven to work — it does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here — so keep the axis. Pass
the sketch line directly to `setByDistanceOnPath`, never through `Path.create`.

No step reads `toothAxis` back; cleanup hides the axis by entity kind rather than through the dict.

<!-- check-step-calls: ignore setByPerpendicularAtPoint Path.create -->

**From:** `spec/bevelgear/instructions.md` L456-465, L809-810, L838-839; `.claude/skills/generate-gear/PLAYBOOK.md` L787-799

## S11 `[PROSE]` Create the `{gearLabel} Gear` component

Create a new component as a child of the **Bevel Gear** component — the same component that owns
Design, **not** the user's Parent Component — named `{gearLabel} Gear`, i.e. `Pinion Gear` or
`Driving Gear`. This intentionally overrides the looser "child of Parent Component" phrasing so the
pair nests cleanly inside Bevel Gear. Write the occurrence into the per-gear dict as
`gearOccurrence`.

The finished bodies for this gear end up here, but the feature operations do not run here: Fusion
rejects cross-sibling sketch and project calls even when the target is activated or the entities are
wrapped in assembly-context proxies (`[PB-NO-CROSS-SIBLING]`), so every feature below runs in the
Design component and the finished bodies are moved across in S26. The visible end state is identical.

**From:** `spec/bevelgear/instructions.md` L971-990; `.claude/skills/generate-gear/PLAYBOOK.md` L811-834

## S12 `[GO]` `{gearLabel} Profile` sketch — the hexagon

Open a **fresh sketch on the axial Gear Profiles plane**, named `{gearLabel} Profile` — one profile
sketch per gear, so `sketch.profiles` holds exactly this one hexagon loop. Do not draw both gears'
hexagons in the shared Gear Profiles sketch; that would leave two identically-shaped loops to
disambiguate.

The six vertices, in draw order:

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A' -> G -> H -> C -> M -> N -> A' | B' -> I -> J -> D -> O -> P -> B' |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A'->G | B'->I |
| toe cut edge | M->N | O->P |
| heel cut edge | C->H | D->J |
| S6 shaft construction line (NOT usable as the axis) | Apex->A | Apex->B |

Build it on fixed vertices by the recreate-share-fix recipe (`[PB-PROJECT-NOT-FIXED]`), and **in this
order**:

1. Recreate the six S6 vertices as **new** points at their exact world-mapped positions —
   `sketch.sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))` for each — which is valid
   because S6 is fully constrained by now.
2. Draw the closed hexagon in the table's draw order as six `SketchLine`s **sharing** those points.
3. Fix the lines and their endpoints **after** the lines exist: `edge.startSketchPoint.isFixed = True`
   and `edge.endSketchPoint.isFixed = True`. Setting `isFixed` on a bare point *before* it is consumed
   as a line endpoint does not leave the sketch fully constrained.

Write the sketch into the per-gear dict as `profileSketch` and its **first edge** — A'->G on the
pinion, B'->I on the driving gear — as `shaftAxisEdge`.

**The shaft axis every body operation below uses is this sketch's first edge, NOT the S6 Apex->A /
Apex->B construction line.** The edge is collinear with the shaft axis but lives in the *same* sketch
as the profile, which is what Fusion's revolve, pattern and path builders accept; reusing the S6
construction line fails or misbuilds. Fixed endpoints are what give that edge a well-defined
`worldGeometry` (`[PB-WORLDGEO-CONSTRAINED]`); a free edge resolves against a default world-XY frame
and silently moves the body onto world XY — observed on the driving gear, with the pinion looking fine
only because it never read the edge's `worldGeometry`.

Gate the sketch on `sketch.isFullyConstrained`, raising and naming it
(`[BEVEL-F-FULL-CONSTRAINT]`).

Proved by `stepProfileSketch`, which recreates the six vertices, draws the loop sharing them, fixes
the endpoints only once the lines exist, and then reads back that the sketch closes exactly one
region of the hexagon's own area, that the first edge lies on the shaft axis, that the toe corner N
sits strictly off it, and that the toe station is inboard of the heel corner — the reading a
collapsed dedendum seed would fail.

<!-- proof-run: proofkit.RunParallel(profileCases, stepProfileSketch) -->

**From:** `spec/bevelgear/instructions.md` L971-990; `.claude/skills/generate-gear/PLAYBOOK.md` L458-472, L598-605

## S13 `[GO]` Revolve the Gear Body

This sketch holds exactly one hexagon loop, so take its single profile with
`sketch.profiles.item(0)` and do not filter (`[PB-SINGLE-PROFILE]`). Revolve it about the
**shaft-axis edge**:

```python
revolveInput = designComponent.features.revolveFeatures.createInput(
    profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
revolve = designComponent.features.revolveFeatures.add(revolveInput)
```

The result is the **Gear Body**, the frustum. Write it into the per-gear dict as `gearBody`.

Because the toe edge is one edge of the revolved profile, the body already carries the conical face
that edge sweeps; likewise the heel edge's cone. Those faces are reused as the cutting tools in S18
and are not built again.

**The profile must not cross the axis of revolution.** If it does, Fusion aborts with
`ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`) — which is exactly what the Maximum Face Width cap of S6 step 26,
the Maximum Base Height of S2, and the strictly positive Toe Radius of S6 step 29 exist to prevent.

Proved by `stepRevolveGearBody`, which revolves the same hexagon a full turn about the station axis
with the evaluator's own revolve and asserts its volume against Pappus on that same hexagon — no
polygon correction, no decomposition into bands — then asserts its **five faces**, matched by surface
kind and by their own readings rather than by the order the face selector returns them: the flat heel
face at the back is a disc of the heel radius, the flat toe face at the front a disc of the Toe
Radius, and the three cone faces are the frusta the profile edges C->H, M->C and N->M sweep. Each
cone's half-angle is read off its own face rather than derived from two cap radii and a height: the
heel cone and the toe dish read 90 degrees minus gamma, and the root cone reads this gear's own root
cone angle. **This step costs the proof nothing** — the frustum is one watertight body with the right
volume, the right two flat faces and the right three cone faces, and there is no union left to owe.

The gear body is the one body no boolean consumes. Every boolean operand in the proof is built with
Loft and never with Revolve: measured over this gear's own bodies across the whole solid table, every
boolean whose operands were both Lofts verified Sound while every boolean with a Revolve operand
verified Suspect, which the solid gate admits no more than a failure.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepRevolveGearBody, assertRevolveGearBody) -->

**From:** `spec/bevelgear/instructions.md` L991-992, L1039-1094, L1275-1282; `.claude/skills/generate-gear/PLAYBOOK.md` L494-500, L716-723

## S14 `[GO]` Loft the Apex sketch point to the tooth profile

Loft the **S6 Apex sketch point** — the `centerToApex.endSketchPoint` from the Gear Profiles sketch,
used as a degenerate point section — to this gear's `{gearLabel} Tooth` profile. The result is the
**Tooth Body**.

```python
loftInput = designComponent.features.loftFeatures.createInput(
    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
loftInput.loftSections.add(apexSketchPoint)
loftInput.loftSections.add(toothProfile)
toothBody = designComponent.features.loftFeatures.add(loftInput).bodies.item(0)
```

Use the S6 Apex **SketchPoint** directly and do **NOT** create a construction point for it: the
Design component is never active and construction geometry needs an active component
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`, `[PB-LOFT]`). The order of `loftSections.add` is the loft order.

Select the tooth cross-section loop with
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)` from `.utilities`,
where the line count is **determined by** the embedded flag read back in S9:
`wantLines = 0 if toothEmbedded else 2`. **Do not accept "0 or 2 lines".** For a given gear only one
of those is the real tooth; an unrelated loop — an inter-tooth or annular region between the drawn
circles — can also have 2 NURBS and 2 arcs with the *other* line count, and selecting it makes this
loft fail with `RuntimeError ... ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`, because the impostor loop
cannot form a loft tool body. Embedded means tip, root and flanks meet with no connecting lines — 4
curves; non-embedded means 2 connecting lines — 6 curves.

Proved by `stepLoftTooth`. The loft's degenerate **point** section is the one thing the proof cannot
build, so a shrunken copy of the tooth stands in for it; the tooth **plane** is not substituted — the
proof builds the real back-cone plane, tilted out of the axis-perpendicular by gamma, and takes the
Apex's perpendicular distance to the section as the Pitch Cone Distance R rather than the tooth
centre's station. The cost is the point section, and what the volume and the taper prove is the shape
it has to produce.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepLoftTooth, assertLoftTooth) -->

**From:** `spec/bevelgear/instructions.md` L489-560, L993-994, L1095-1099; `.claude/skills/generate-gear/PLAYBOOK.md` L672-681, L724-728, L791-799

## S15 `[PROSE]` The tooth-body hook, and its straight-bevel gate

`_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane, gearLabel,
teethNumber, gamma)` is the single tooth-body hook, called after the loft of S14 and before the
pattern of S23.

**Its first line is the gate:** `if self._spiralAngle_rad <= 0: return cut_conical_ends(...)`. A
Mean Spiral Angle of 0 means a **straight** bevel gear and the hook returns immediately with the
framework's two conical trims — byte-for-byte the prior behaviour — and every spiral input is
ignored. Any value above 0 builds a curved tooth through S19 to S22, which run **in place of** the
straight tooth's two conical trims and before pattern, combine and bore.

**Build the four toe and heel arguments exactly per this table. Mislabelling them silently inverts
the spiral, and it is the single biggest hazard in this build.**

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M->N** | **C->H** | **M** | **C** |
| Driving | **O->P** | **D->J** | **O** | **D** |

From those S6 sketch points' **world** geometry, `_createGearBody` computes and passes positionally
in the order `toeMid, heelMid, toeConeWorld, heelConeWorld`:

- `toeMid` = the world **midpoint of the TOE edge** — half of (M + N) on the pinion, half of (O + P)
  on the driving gear;
- `heelMid` = the world **midpoint of the HEEL edge** — half of (C + H), half of (D + J);
- `toeConeWorld` = the toe edge's inner endpoint, **M** or **O**, which lies on the root cone element
  at the toe end;
- `heelConeWorld` = the **dedendum corner**, **C** or **D**, the outer end of that same root cone
  element.

**Two scrambles to avoid, both of which a fresh regen has made.** Do **not** pass the two endpoints of
a *single* edge as `toeMid` and `heelMid`: M and N both sit at the toe, so the span collapses to about
zero or goes negative and the spiral inverts. And `heelConeWorld` is the dedendum corner C or D,
**never H or J** — H and J lie on the Apex2->C / Apex2->D dedendum line, one Module beyond C and D and
off the root cone element, so using them skews the cone vector away from Apex->C / Apex->D.

`_pinionMeshPhase(pinionTeeth)` returns the pinion's extra mesh rotation about its own shaft axis in
**radians**, `_PINION_MESH_PHASE_TEETH * 2 * pi / pinionTeeth`, which is 0 for a straight bevel and 0
by default for a spiral one.

Two constants are declared on `BevelGearGenerator` rather than at module level:

| constant | declared on | default | what it does |
|---|---|---|---|
| `_CROWN_PER_RAD` | `BevelGearGenerator` | `0.5` | scales the spiral lengthwise crown — S21 |
| `_PINION_MESH_PHASE_TEETH` | `BevelGearGenerator` | `0.0` | the pinion's extra mesh rotation in tooth-fractions |

Both are class attributes because both are read through `self` from inside the tooth-body build, and
neither is a reproduced API string the way the input ids are. **`_CROWN_PER_RAD` reaches built
geometry and its value is not derived from anything** — it multiplies the relief every crowned slab
takes, and `0.5` was hand-entered in 2026 on the hand-coded generator that preceded this spec,
replacing a `0.0` that had the crown switched off. No measurement, published source or Fusion load
stands behind it. Treat it as an unverified tuning value: keep it at `0.5` so a regen reproduces
today's gear, and do not write a derivation the repository does not have.

`_transformToothBody`, `_createGearBody` and `_pinionMeshPhase` are this module's own methods, named
here to fix the call graph rather than as calls into Fusion.

<!-- check-step-calls: ignore _transformToothBody _createGearBody _pinionMeshPhase -->

**From:** `spec/bevelgear/instructions.md` L310-331, L489-560, L841-861, L969-970; `spec/bevelgear/spiral-tooth-trace.md` L1-31

## S16 `[PROSE]` `{gear} Cone Element` sketch and `{gear} Trace Plane` — spiral only

Runs only when the Mean Spiral Angle is above 0.

Build the world frame the spiral construction needs, from geometry this gear already has:

- `axisDir` = the **shaft axis** direction, from the two **world** endpoints of `shaftAxisEdge` — the
  in-sketch profile edge A'->G / B'->I — normalized.
- `coneVec` = the **dedendum (root) cone element** Apex->C / Apex->D, realized as
  `normalize(heelConeWorld - apexWorld)`.
- `v` = `axisDir x coneVec`, normalized — the **circumferential** direction.
- `tpNormal` = `coneVec x v`, normalized — the tangent-plane normal. It completes the frame and
  **nothing consumes it**: the projection that once used it is gone, so it is computed and left
  unread.
- the cone distance of a point p, written distAlong below, is `(p - apexWorld) . coneVec` — its
  distance from the Apex measured along the cone element.

**The heel MUST be the outer end so `coneVec` points outward and the span is positive.** Before
building `coneVec`, check the passed midpoints and fix a swapped pair: if
`apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid` with `heelMid` **and**
`toeConeWorld` with `heelConeWorld`, then build `coneVec` from the corrected heel. A negative span
silently inverts the entire spiral frame — the cutter-arc direction, the slice direction and the
per-segment twist all flip — and the gear comes out completely wrong with no error raised.

Then read R_toe as the distAlong of `toeMid`, R_heel as the distAlong of `heelMid`, R_mean as
`(R_toe + R_heel) / 2`, and span as `R_heel - R_toe`, the face width, now positive. Those are
the only quantities the rest of the spiral build needs.

Draw a **cone-element construction line** from the Apex to `apex + R_heel * coneVec` in a sketch on
the axial Gear Profiles plane, named `{gear} Cone Element`. Then build the tangent plane as that
axial plane rotated **90 degrees** about the cone-element line:
`plane_by_angle(designComponent, coneElementLine, gearProfilesPlane, 90)`, named
`{gear} Trace Plane`.

**Coordinates — this rule governs both this sketch and the trace sketch of S17.** The world `Point3D`
values here are passed **directly** into the sketch calls, where they are consumed as **sketch-space**
input: **no `modelToSketchSpace` conversion is applied**, even though `adsk.fusion.Sketch` offers
exactly that call and the points really are model-space coordinates. That is deliberate and it is
worth stating why it is harmless, because the reasoning is not the obvious one. The trace sketch is
construction and reference only — no downstream feature ever consumes it, since the twist is computed
analytically in S20 — and it exists only so the genuine cutter arc is inspectable before cleanup hides
it. The cone-element line is the one that needs the extra sentence: it *is* consumed, by
`plane_by_angle`, so an unconverted line places the Trace Plane somewhere other than the true tangent
plane. That still reaches no feature, because the only thing built on the Trace Plane is the
inspection-only trace sketch, and the chain ends there. **If a later revision ever makes any feature
consume the trace sketch or the Trace Plane, this shortcut stops being safe and both sketches need
`modelToSketchSpace` on every point.**

These transient sketches are **exempt** from the full-constraint gate and must not be gated
(`[BEVEL-F-FULL-CONSTRAINT]`).

`modelToSketchSpace` is named here only to say that this step does not call it.

<!-- check-step-calls: ignore modelToSketchSpace -->

**From:** `spec/bevelgear/instructions.md` L862-873, L890-895; `spec/bevelgear/spiral-tooth-trace.md` L32-106; `spec/bevelgear/fusion.md` L59-67

## S17 `[GO]` `{gear} 2D Tooth Trace` sketch — the cutter arc

Runs only when the Mean Spiral Angle is above 0. Work in the tangent-plane 2-D frame with the origin
at the Apex, **x = `coneVec`** (so a point's x is its cone distance) and **y = `v`**
(circumferential).

The cutter radius `r_c` is the Cutter Radius input when it is non-zero, **else `R_mean`** — the auto
default. The hand sign is `+1` for `Right` and `-1` for `Left`, then **negated for the pinion**,
because the pair meshes with opposite hands. The cutter-circle centre is

```
Cx = R_mean - r_c * sin(psi)
Cy = handSign * r_c * cos(psi)
```

**The hand sign goes on the `cos` / `Cy` term and NOT on the `sin` / `Cx` term.** This was a real bug.
Opposite hand mirrors the cutter centre **across the cone element**, which flips `Cy`; putting
`handSign` on `Cx` mirrors about `x = R_mean` instead, a *different* curve that gives the two gears
unequal twist. For equal teeth the driving and pinion traces must come out as exact mirror images.

The trace's toe and heel endpoints are circle-circle intersections taken a hair **past** the face, so
the kept arc reaches cleanly past the end trims:
`toe2d = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)` and
`heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)`, the framework helper from
`.solids`, with `R_lo = R_toe - 0.06 * span` and `R_hi = R_heel + 0.06 * span`. It intersects the apex
circle of radius R with the cutter circle and keeps the solution nearest the mean point — the branch
the mean point sits on.

Add a sketch on the Trace Plane named `{gear} 2D Tooth Trace`, and with
`tanW(px, py) = combine_point(apexWorld, px, coneVec, py, v)` mapping 2-D coordinates to world, draw:

- the **cutter circle** — `sketch.sketchCurves.sketchCircles.addByCenterRadius(tanW(Cx, Cy), r_c)` —
  marked `isConstruction`, with its centre pinned by `circle.centerSketchPoint.isFixed = True`
  (`[PB-CIRCLE-CENTER]`) and a diameter dimension of `2 * r_c` placed with
  `addDiameterDimension(circle, textPoint)`;
- the **trace arc** — `sketch.sketchCurves.sketchArcs.addByThreePoints(tanW(toe2d), tanW(R_mean, 0),
  tanW(heel2d))` — with its **centre coincident to the cutter circle's centre** and a **radius
  dimension of `r_c`** from `addRadialDimension(arc, textPoint)`, so it is the genuine cutter circle
  and not a look-alike spline.

Text points per `[PB-RADIAL-DIM]`: off-centre and on or near the curve. Use the mean point
`tanW(R_mean, 0)` for the arc's radius dimension and a point on the cutter circle such as
`tanW(Cx + r_c, Cy)` for the circle's diameter dimension. A text point at the curve's centre is
rejected outright.

**This sketch is deliberately left with free degrees of freedom** — the arc's endpoints are pinned by
the three-point construction, not by endpoint dimensions, because dimensioning them over-constrains
the solve against the cone-element plane — and it is therefore exempt from the full-constraint gate.
Do not gate it.

**This arc really is a three-point arc with a radius dimension**, which is what separates it from the
tooth-top arc of S9; the two must not be written from one template.

Proved by `stepTraceSketch`, which makes two substitutions and says so. The harness gates on DOF 0 and
waives nothing, so the arc's two ends are pinned there rather than left free; and because each end is
a circle-circle intersection with two solutions and carries exactly one freedom, no single unsigned
row separates them — the proof pins each by its signed angle about the cutter centre, which is
monotonic in the freedom it removes, and asserts the circle-circle conditions on the solved arc
instead. It then reads back the derivation's own invariants: the arc is one circle of the cutter
radius, its centre is `r_c` from the mean point so the arc passes through it, its ends sit on the toe
and heel circles about the Apex, its tangent at the mean point makes the Mean Spiral Angle with the
cone element, and the hand sign sits on the `cos` term.

<!-- proof-run: proofkit.RunParallel(traceCases, stepTraceSketch) -->

**From:** `spec/bevelgear/instructions.md` L874-895; `spec/bevelgear/spiral-tooth-trace.md` L107-203, L239-289;
`.claude/skills/generate-gear/PLAYBOOK.md` L451-457, L509-516, L652-656

## S18 `[GO]` Slice the straight tooth into slabs — spiral only

Runs only when the Mean Spiral Angle is above 0. Split the uncut Apex-to-heel `toothBody` into
cross-section slabs by planes **parallel to the parent transverse tooth plane** — `parentToothPlane`,
the `{gearLabel} Plane` of S8 — via a **fixed** scheme of **exactly 8 planes**. The count is not user
configurable.

**The slice planes are NOT perpendicular to the cone element.** The parent plane carries the
tooth-centre line C->K' / D->L', which is the back-cone line and so perpendicular to the Pitch Line,
so **the parent plane's normal runs along the PITCH element** while `coneVec` is the **ROOT** element.
The two differ by the dedendum angle `delta_f = atan(1.25 * Module / R)`, equivalently
`atan(2.5 * sin(gamma_p) / N_p) = atan(2.5 * sin(gamma_g) / N_g)`: Module cancels, so it depends only
on the tooth counts and the Shaft Angle and is the **same for both members** — 3.26 degrees on the
default 31/31 pair at Shaft Angle 90, growing as the tooth counts fall.

The parallel family is what the build **requires** rather than what it happens to use:
`slice_body_by_offset_planes` offsets the parent plane with `setByOffset`, which produces parallel
planes; the sign test below reads the **parent plane's own normal**, which is meaningful only for that
plane's own offsets; and the tooth is lofted from the Apex to the profile drawn in the parent plane,
so **the heel-most slab's heel face IS the parent plane** and a consistent family has to contain it.

**A build that follows "perpendicular to the cone element" instead is wrong and silent.** It tilts
every cut face by `delta_f` and nothing in the pipeline fails: parallel planes cut a cone in similar
sections whatever their orientation, so the loft still reproduces the taper; the piece count, the
retry gate and the conical trims are all indifferent to slab orientation; the proof builds its own
slabs and never sees the module's plane; and the runtime gate only counts pieces. What moves is the
geometry — on the default pair at Module 4 a face corner lands `1.125 * Module * tan(delta_f)` =
0.26 mm along the cone from where the parallel cut puts it, and the twist keyed across one face
mismatches by up to 0.0078 rad.

The first cut plane is the parent plane offset **toward the Apex** by `span / 6`. **The offset sign is
chosen per gear**, because the parent plane's normal points opposite ways for the two gears: pick
`sign` so `sign * normal` points Apex-ward, testing `(apex - planeOrigin) . normal`. The other seven
step further toward the Apex in `span / 6` increments, so
`offsets = [sign * (k + 1) * span / 6 for k in 0..7]`.

**Where the eight land: the first sits `span / 6` inside the HEEL and none of them lies past it** —
the parent plane is already the heel end, so there is no heel overshoot to give — **the sixth lands at
the toe, and the last two sit `span / 6` and `2 * span / 6` PAST the toe.** The two segments beyond
the toe are what the toe cone of S22 trims away. The first sits a hair more than `span / 6` inside the
heel and the sixth a fraction of a millimetre inside the toe, because `R_heel` and `R_toe` are read at
the two edge midpoints rather than on the root element.

Split with the framework helper
`slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)` from `.solids`,
which splits piece by piece and keeps a piece whole when a plane misses it. Do not re-implement it.

**The slice MUST actually split the tooth.** After the cut loop, if the body is still in **one
piece** — no plane cut it — the offset sign was wrong or `parentToothPlane` sits outside the tooth's
span: **retry the whole cut once with the opposite sign**. If it is still one piece, **raise** a clear
self-diagnosing error naming the gear, the final piece count, the span and the sign tried
(`[PB-SELF-DIAGNOSING]`, `[PB-EMPTY-RESULT]`). Do **not** return an unsliced single-piece result: S19
then drops that one piece as the Apex scrap, leaving the segment list **empty**, and the crown later
crashes with `ValueError: max() iterable argument is empty` far from the cause.

Proved by `stepSliceToothSlabs`. The evaluator has no split-by-plane, so each slab is **built**
between its two planes rather than cut out of one body, and the slabs are laid apart so no pair of
them touches — the evaluator verifies every pair of live bodies in a document, and two slabs sharing a
face resolve neither way. Laying them apart changes no volume. **The cost is the split**: the proof
does not show the evaluator dividing one body into these pieces. What it does assert is the runtime
gate the step carries — the piece count is nine, eight working segments and the Apex-side scrap, and
never one — and every slab's volume against the closed form for the frustum between its two planes,
summing to the whole tooth. **The proof builds its own slabs from the offsets this step fixes and
never reads the plane the generated module constructs, so the module's choice of family reaches
Fusion untested.** That is the same shape of gap as the S6 seed, and the only thing that closes it is
a face corner's position measured on a loaded spiral gear.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepSliceToothSlabs, assertSliceToothSlabs) -->

**From:** `spec/bevelgear/instructions.md` L896-931, L1220-1223; `.claude/skills/generate-gear/PLAYBOOK.md` L159-186, L440, L762-769

## S19 `[GO]` Order the segments and drop the Apex scrap — spiral only

Runs only when the Mean Spiral Angle is above 0. Sort the segments by the `distAlong` of their
centroid, read from `body.physicalProperties.centerOfMass`. The first — the apex-most — is the long
**Apex-side scrap** below the toe: remove it and keep the rest as the working `segments`.

**Drop the scrap by re-slicing the list FIRST and deleting it second** — `segments = segments[1:]`
before `designComponent.features.removeFeatures.add(scrap)`. Use a remove feature, which is
timeline-visible, and not a bare `deleteMe` (`[PB-REMOVE-PIECES]`).

After dropping the scrap, **`segments` must be non-empty** — at least one cross-section. If it is
empty the slice failed in S18: **raise** a clear error rather than proceeding into the twist and the
crown, which both assume at least one segment (`[PB-EMPTY-RESULT]`).

`deleteMe` is named only to forbid it here.

<!-- check-step-calls: ignore deleteMe -->

Proved by `stepDropApexScrap`, which builds the same nine pieces, sorts them by the cone distance of
their centroid, drops the first, and asserts that eight remain, that they sum to the whole tooth less
the scrap, and that the scrap has volume — so the drop removed something.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepDropApexScrap, assertDropApexScrap) -->

**From:** `spec/bevelgear/instructions.md` L932-933; `.claude/skills/generate-gear/PLAYBOOK.md` L440, L770-774

## S20 `[GO]` Twist each segment about the shaft axis — spiral only

Runs only when the Mean Spiral Angle is above 0. Rotate each segment about the **shaft axis** —
`axisDir` through `apexWorld` — so the tooth follows the trace, **centred on `R_mean` so the mid-face
section stays unrotated**. That section then meshes exactly like the straight tooth, which is why the
pinion needs no extra mesh phase.

The total toe-to-heel twist comes from the **conjugate crown-gear generation law** — the standard
Gleason and Litvin model, in which a spiral bevel is generated by an imaginary flat crown gear and the
work gear's shaft rotation relates to the developed crown-plane azimuth by the roll ratio
`1 / sin(gamma)`. Compute it **analytically — no projection, no curve sampling:**

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`phi_crown` is the angle the cutter arc's toe and heel endpoints subtend at the Apex in the flat 2-D
crown frame — exactly the `toe2d` and `heel2d` pairs from S17. `gamma` is this gear's **pitch** cone
angle, `self._gamma_p` or `self._gamma_g`, already computed in S6 and passed into the hook.

**The two halves of this law are taken on two different cones, and that is deliberate rather than an
oversight.** `phi_crown` is measured in the frame S16 builds, whose x axis is `coneVec`, the **ROOT**
cone element. The divisor `sin(gamma)` is the **PITCH** cone's roll ratio, because the crown gear the
law generates against is tangent to the pitch cone. Written consistently on the root cone the divisor
would be `sin(gamma_root)` for `gamma_root = gamma - delta_f`, which is a real difference and not a
rounding: at the default pair the dedendum angle is 3.26 degrees, so the ratio of the two divisors is
1.062 and the root divisor would twist the tooth about 6% further. **Keep the pitch angle.**

**Use the pitch cone angle from S6 and NOT `acos(coneVec . axisDir)`**, which is the root cone angle,
smaller by the dedendum angle — 24.7 degrees against the pitch 28.7 for a 17-tooth pinion meshing a
31-tooth gear — and yields a twist about 1.15 times too large. That was the defect that kept ratio
pairs from meshing at all. The two members of a meshing pair **legitimately get different twists**:
same cutter, same spiral angle, but gamma differs, so `1 / sin(gamma)` differs — about 2.08 for a
17-tooth pinion against about 1.14 for a 31-tooth gear. That is *why* equal-teeth pairs always meshed
while ratio pairs failed under any method that gets the roll ratio wrong. Do **not** measure the twist
off a projected 3-D cone trace: `projectToSurface` wraps the arc around the cone for ratio pairs and
the measurement collapses to a fraction of the true sweep.

`handSign` sets the direction and `total` is the magnitude. Each segment's rotation is a **linear
share** keyed to the cone distance of its **HEEL FACE** — the segment's farthest-along-the-element
face, which is the exact section the later loft samples:

```
ang = -handSign * total * (R_mean - R_heelFace(seg)) / span
```

**Define a slab's heel face precisely: the face whose centroid has the GREATEST
distAlong of `face.centroid`, searched across ALL of the slab's faces with NO surface-type filter.** Its
toe-side face is the least-centroid one. **Do not restrict this search to `PlaneSurfaceType` or any
other surface type** — a sliced slab is bounded by a mix of the two planar cut faces and ruled side
faces, and a type filter can pick the wrong face or miss the cut face, which makes the S22 loft fail
with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this same all-faces-by-centroid rule
everywhere a slab end face is needed: here, in S21's crown base, and in S22's loft sections.

**Key the twist on the segment's heel-face cone distance, NOT on its centroid.** The loft samples each
segment's heel face, so that face is what must land at the right azimuth; centroid-keying leaves the
loft's mid-face section rotated by half a segment and the faces overlap.

Apply the rotation as a free move: build
`matrix = adsk.core.Matrix3D.create()`, `matrix.setToRotation(ang, axisVector, apexPoint)`, then
`designComponent.features.moveFeatures.createInput2(bodyCollection)`,
`moveInput.defineAsFreeMove(matrix)` and `moveFeatures.add(moveInput)` (`[PB-MOVE-ROTATE]`). Use
`defineAsFreeMove` with a matrix and not `defineAsRotate`, which rejects a `SketchLine` axis.

`projectToSurface` and `defineAsRotate` are named only to forbid them.

<!-- check-step-calls: ignore projectToSurface defineAsRotate -->

Proved by `stepTwistSegments`, which builds the same eight segments, rotates each by the same law
about the shaft axis, and asserts that the total is `abs(phi_crown) / sin(gamma)` and specifically
**not** the root-cone divisor, that the twist at `R_mean` is zero, that the toe and heel ends turn
opposite ways about it, and that the rotation changes each segment's azimuth and nothing else.
**Whether the frame should move to the pitch cone to match the divisor is not settled, and nothing in
this repository measures it**: the proof checks the same formula the module computes, so it confirms
the arithmetic and says nothing about which cone the frame belongs on. Moving the frame would change
`R_toe` and `R_heel` and the element the Mean Spiral Angle is measured against, so it is its own
derivation and its own change. The proof file records that beside the twist assertion.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepTwistSegments, assertTwistSegments) -->

**From:** `spec/bevelgear/instructions.md` L934-951; `spec/bevelgear/spiral-tooth-trace.md` L204-238; `.claude/skills/generate-gear/PLAYBOOK.md` L439, L800-809

## S21 `[GO]` Lengthwise crown — spiral only

Runs only when the Mean Spiral Angle is above 0. Crown the tooth by scaling each segment **except the
outermost, heel one** down by a **monotonic** factor — full at the heel, growing smoothly toward the
toe — **about a sketch point on the ROOT edge of its heel face**.

For each segment compute its **heel-distance fraction** `u = (R_heel - R_heelFace) / span`, where
`R_heelFace` is the `distAlong` of that segment's heel face — found by S20's all-faces-by-centroid
rule but **RECOMPUTED here, AFTER the twist has moved the slabs**; do not reuse pre-twist values.
`R_heel` and `span` come from S16.

**`u` runs 0 at the held-full heel to 1 at the toe, and PAST 1 on the two segments beyond the toe.**
Those two segments' heel faces sit `6 * span / 6` and `7 * span / 6` in from the parent plane, so the
toe-most one reads about 1.18 before the twist and a little more after this recompute, at the default
Mean Spiral Angle on the default pair. **Do not treat `8 / 6` = 1.33 as a ceiling.** That figure is
the last plane's offset, and that plane is a toe face rather than any segment's heel face, so nothing
ever evaluates `u` there — but the twist moves the heel faces, so the recomputed `u` climbs with the
Mean Spiral Angle and is measured at 1.351 at 55 degrees, which the `[0, 60)` range admits. Nothing
reads an upper bound on `u`; the factor below stays positive for every value it takes, and what the
slab count rests on is the structural fact that the last cut plane is never a heel face. The heel
segment reads a few hundredths rather than exactly 0, because `R_heel` is read at the heel edge's
midpoint rather than on the parent plane; that segment is skipped anyway.

**"Outermost (heel) segment" is the one with the GREATEST post-twist heel-face `distAlong`** — sort
the segments by their recomputed heel-face distance and skip the last. Then:

```
factor = 1 - _CROWN_PER_RAD * (abs(total) / 2) * u
```

`total` is the full toe-to-heel twist from S20, so `abs(total) / 2` is the per-end peak twist
magnitude and the maximum relief — now at the **toe** — keeps the same magnitude the old per-end peak
had, just relocated. This makes relief grow **monotonically from the full heel to the toe**, so slab
heights stay strictly ordered heel to toe and the natural cone taper is never reversed. If a computed
`factor` comes out at or below 0, **raise** a self-diagnosing error naming the gear, the segment's `u`
and the factor; never scale by a non-positive factor.

**Do NOT key the relief on `abs(ang)`, the twist magnitude.** That is symmetric about the mid-face —
maximal at BOTH ends — so, because the heel slab is held full, the slab *just inside* the heel becomes
the most-relieved one and dips below both its neighbours, reversing the heel-to-toe taper. That was
the observed bug: the heel-adjacent slab came out at factor 0.932 while the next slab inward was
0.972, taller. Key the relief on the monotonic `u`, never on `abs(ang)`.

**Three gotchas.**

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). `scaleFeatures` is the **one** exception to never
   activating: it needs the Design occurrence as the active edit target, so call
   `designOccurrence.activate()` before the crown scales and restore afterwards — in a `finally` —
   with `design.activateRootComponent()`. **Do NOT write `design.rootComponent.activate()` or
   `someComponent.activate()`**: a `Component` has **no** `activate` method and raises
   `AttributeError`. Only an `Occurrence` has it, and the root is re-activated through `Design`.
2. **Skip the outermost, heel segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone of S22 trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, NOT its centroid.** `scaleFeatures` shrinks
   **uniformly** toward the base point, so a base point at the heel face's centroid — mid tooth-height
   — pulls the tooth's **root** edge *upward* by `(1 - factor) * (half the tooth height)`: the tooth no
   longer seats on the gear body's root cone, floats above the base, and the Combine-Join leaves a gap,
   clearly visible on ratio pairs. Put the base point on the **root** instead: of the heel face's
   vertices — `heelFace.vertices`, each `.geometry` a world `Point3D` — take the **two with the
   smallest perpendicular distance to the shaft axis**, computed as
   `abs((p - apex) - ((p - apex) . axisDir) * axisDir)` for the line through `apexWorld` along
   `axisDir`; those are the two **root corners**, since the tip corners are the farthest from the
   axis. Place the base sketch point at their **midpoint**, mapped into the heel-face sketch with
   `modelToSketchSpace`. The heel face is a planar cut, so that midpoint lies on it. A uniform scale
   about a point keeps every line through that point invariant, so anchoring on the root keeps the
   root edge on the seating cone while the tip is relieved progressively toward the toe — which is
   exactly the lengthwise crown intended. Finding the heel face itself is unchanged; only the point
   *on* it changes from centroid to root-edge midpoint.

Build the feature with
`designComponent.features.scaleFeatures.createInput(bodyCollection, baseSketchPoint,
adsk.core.ValueInput.createByReal(factor))` and `scaleFeatures.add(scaleInput)`.

`activate` on a `Component` is named only to forbid it.

Proved by `stepCrownSegments`, which makes two substitutions and states both. The evaluator has no
scale feature, so the relief is **built** into each slab's two sections rather than applied to a
finished slab; and the scale is taken **in section** rather than uniformly in three dimensions, so the
slab keeps its length along the cone and its cross-section alone is relieved — its volume is therefore
the plain slab's times factor **squared** and not cubed. The cost is the axial component, which the
S22 loft never samples because it takes each slab's heel *face*. Each section is relieved about its
own root anchor, so the two stay similar about the plane origin and the relieved slab is still a cone
frustum. What the assertion pins is the rule this step exists for: the factor is monotonic in `u`, the
segments run heel to toe in order, the factor stays positive, the heel segment is the one with the
smallest `u` and is held full, and anchoring on the centroid instead would lift the root by a
computed, positive amount.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepCrownSegments, assertCrownSegments) -->

**From:** `spec/bevelgear/instructions.md` L310-331, L953-966; `spec/bevelgear/fusion.md` L208-215; `.claude/skills/generate-gear/PLAYBOOK.md` L585-590, L791-799

## S22 `[GO]` Loft the crowned segments into the spiral tooth — spiral only

Runs only when the Mean Spiral Angle is above 0. **Re-sort the segments by their heel-face cone
distance HERE, AFTER the twist and the crown — do NOT reuse the pre-twist slice or centroid order
from S19.** The twist rotates each slab about the shaft axis, and for high-twist unequal-ratio pairs
that rotation changes the slabs' along-cone order enough to **reorder adjacent slabs**; lofting in
the stale order assembles the cross-sections out of sequence and the crowned tooth comes out
distorted, so the two gears interfere. For equal or low-twist pairs the two orders coincide, which is
why equal-teeth gears mesh even with the stale order while unequal ratios distort — this is the single
thing that makes a ratio pair like 31/17 fail while 31/31 looks fine.

So sort the segment indices **now**, by the distAlong of each segment's own heel-face centroid, and
loft a new body through, in that order:

1. first the **toe-most segment's apex-side (toe-facing) face** — the toe segment is `order[0]`, and
   its toe face goes in first to push the loft past the toe cone so the toe trim bites;
2. then the **heel-facing face of every segment, iterated in `order`** — each segment's
   farthest-along-the-element face by post-twist centroid, the last of which reaches past the heel
   cone.

Name the resulting body **`{gear} Spiral Tooth`**. Then remove the segment scaffolding with
`removeFeatures.add(...)`; the loft has captured their faces.

Proved by `stepLoftSpiralTooth`. The evaluator's loft takes exactly **two** sections, so the single
multi-section loft is built there as the chain of adjacent two-section lofts, laid apart. **The cost
is the one body**: the proof shows the sections in the right order and the volume they enclose, not
the evaluator running one loft through all of them. The order is recomputed in the assertion the same
way it is in the step, and the chain's sections are required to be strictly increasing in heel-face
cone distance, which a stale pre-twist order would not be.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepLoftSpiralTooth, assertLoftSpiralTooth) -->

**From:** `spec/bevelgear/instructions.md` L967-968; `.claude/skills/generate-gear/PLAYBOOK.md` L724-728, L770-774

## S23 `[GO]` Conical end cuts — trim the tooth to a flush band

Trim the Tooth Body to a flush band with the framework helper and do **not** re-implement the cut
machinery:

```python
return cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)
```

For a straight bevel this is what the hook of S15 returns immediately; for a spiral one it is what S22
hands its `{gear} Spiral Tooth` to, and the same two-cone trim either way, so the curved tooth's ends
sit flush on the gear base.

**Two distinct bodies are involved — do not conflate them.** The cutting **tools** are
`ConeSurfaceType` faces of the **Gear Body**, the revolved-hexagon frustum, because the lofted Tooth
Body has no cone faces and searching *it* for one finds none. The **target** being split is the
**Tooth Body**.

The helper implements the pinned cut behaviour: the **toe cut first**, its cone face identified by the
toe edge's world **MIDPOINT** best-first across the frustum's cone faces (`[PB-FACE-BY-MIDPOINT]` —
endpoints sit near the apex singularity and report an unevaluable distance), each candidate tried as
the actual split tool and the first that splits kept; **keeper selection after each cut**, removing
apex-containing pieces and keeping the largest (`[PB-REMOVE-PIECES]`); then the **heel cut on the
keeper alone**, since removing the apex tip first is what makes it deterministically two split
features for every gear ratio. A heel cone that does not intersect the keeper at all — common on ratio
pairs, where the heel cone never overshoots the tooth — is raised by the helper as the typed
`solids.NonIntersectError` and caught, and the keeper is returned whole. Every failure is
self-diagnosing with the per-face distance and error history (`[PB-SELF-DIAGNOSING]`), and each cut's
outcome is logged with `force_console=True`.

**Caller obligations, which stay in the generator:** pass `toeMid` as the toe edge's world midpoint —
half of (M + N) on the pinion, half of (O + P) on the driving gear — and `heelMid` as the heel edge's
world midpoint, half of (C + H) or (D + J); `apexWorld` as the S6 Apex sketch point's world geometry;
and `gearBody` as the revolved frustum, the cone-face source. **The toe cut must split**, and its
failure propagates and crashes the build, which is correct because an uncut tooth is unusable; only
the heel cut is lenient, and only through the typed `NonIntersectError`.

Proved by `stepConicalEndCuts`, which **performs the toe cut** and substitutes for the heel one. It
builds the toe cutting cone as the solid inside the cone — the whole body on the discard side, with
its apex on the shaft axis at the station the toe edge's own lattice point M/O puts it, rather than a
band spanning that one profile edge, which is enough to read an angle off and is not a tool a cut can
use — cuts the tooth with it for one piece and intersects for the other, and asserts that the two add
back to the whole tooth and that each piece is one solid lump. It tolerates exactly one typed refusal
from the intersect, the empty-result code, which says the cone took nothing off that tooth — the
condition the module raises as `solids.NonIntersectError` — and records that it built no split rather
than passing silently; any other error fails.

**It performs no heel cut, because that cone is tangent to the tooth plane.** The dedendum corner C/D
and the tooth centre K'/L' both sit on this gear's back-cone dedendum line, so the tooth plane contains
a generator of the heel cone and the two touch along the tooth's own centreline instead of crossing
it; the evaluator refuses exactly that, and rebuilding the cone as a revolve replaces the refusal with
a verdict the gate does not admit either. **The cost is the heel split**: for that end the proof does
not show the evaluator dividing the tooth, selecting the keeper, or leaving a watertight body. That
same tangency is why only the toe cut must split: at Tooth Spacing 0 the heel cone passes exactly
through the tooth's heel-end centreline and takes only the two corners, and a cut that removes that
little is a cut that can miss the keeper altogether on a ratio pair.

Both half-angles are read off the **revolved gear body's own cone faces**, which are the faces the
`ConeSurfaceType` search finds at this step, and each is required to equal this gear's back-cone
half-angle. Where each cut lands is solved from those readings and the root ray's own slope: the toe
cone meets the root cone at M and the heel cone at C, which is what makes the band flush. The proof
also requires each cut to cross the tooth's **tip inboard of its root**. It asserts **nothing** about
the two crossings merely differing, and carries no message for that case: both cones have their apex
on the shaft axis, so a cone of wall slope `k` and apex station `a` crosses a tooth surface of slope
`m` at `a * k / (m + k)`, positive for every `m > 0` and different for the tooth's two surfaces
whenever the tooth has height, so such an assertion passes on any figure this spec can build.

**What neither half reaches**, recorded in the proof file beside the cut assertions: a cone and a
tilted plane read identically in everything this step measures, because in the axial section a cone of
half-angle 90 minus gamma and a plane tilted by gamma through the same generator are the same line.
What makes the face conical is that it is a surface of revolution about the shaft axis — its crossing
with the tooth's tip surface sits at one station at every azimuth, while a tilted plane's crossing
moves with azimuth — and nothing here measures that azimuthal crossing. The flush-band check cannot
see the angle either: any band through M crosses the root ray at M whatever slope it has.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepConicalEndCuts, assertConicalEndCuts) -->

**From:** `spec/bevelgear/instructions.md` L995-1021, L1100-1146, L1224-1257; `.claude/skills/generate-gear/PLAYBOOK.md` L159-170, L733-774

## S24 `[GO]` Circular-pattern the tooth

Circular-pattern the remaining tooth piece around the **shaft-axis edge** — the same in-sketch profile
edge the revolve used, not the S6 construction line. **Pin all three inputs explicitly and do not rely
on Fusion's defaults staying equal to them** (`[PB-CIRCULAR-PATTERN]`):

```python
patternInput = designComponent.features.circularPatternFeatures.createInput(bodyCollection, shaftAxisEdge)
patternInput.quantity = adsk.core.ValueInput.createByReal(teethNumber)
patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
patternInput.isSymmetric = False
pattern = designComponent.features.circularPatternFeatures.add(patternInput)
```

The number of copies equals this gear's Teeth Number. Although the pitch diameter shrinks from the
heel toward the Apex, the *angular* spacing around the shaft axis stays constant at `360 / N` for the
entire face width: the radial taper is already produced by the loft from the Apex to the heel-end
tooth profile, so the pattern just rotates that one tapered tooth into N evenly spaced copies.

`pattern.bodies` already includes the seed body plus the copies, so **do not re-add the seed**, and
copy them into a fresh `adsk.core.ObjectCollection.create()` before handing them to the combine —
`pattern.bodies` is a `BRepBodies` and the combine input rejects it (`[PB-PATTERN-BODIES]`).

Proved by `stepCircularPattern`, **and this is the one step in the bevel proof that stays serial.**
The pattern increment retires the seed, so the seed cannot be measured after the step runs and its
volume, bounding box and azimuth have to be read during the build and handed to the assertion. That
hand-off leaves the case, and two cases running at once overwrite each other's readings and the proof
reports a wrong verdict rather than failing loudly — the two gear sides differ enough in volume that
the overwrite was caught when it happened, and a pair of cases whose seeds measured alike would have
passed on each other's numbers instead. No other bevel step carries a reading from its build into its
assertion; a step that acquires one moves to the serial runner in the same change.

The assertion pins the three inputs: the first copy is the increment, the middle one the half turn,
and the last sits one increment short of the full turn — which is what a total angle of 360 degrees
with `isSymmetric = False` produces and a symmetric pattern or a different total angle would not. Each
copy carries the seed's own volume, compared as two readings rather than against a formula, and spans
the same stations, so the pattern turned the tooth about the shaft axis and did not move it along.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCircularPattern, assertCircularPattern) -->

**From:** `spec/bevelgear/instructions.md` L1023-1024, L1362-1387; `.claude/skills/generate-gear/PLAYBOOK.md` L693-703

## S25 `[GO]` Combine-Join the teeth into the Gear Body

Join all patterned tooth pieces with the Gear Body in a **single** Combine-Join: the Gear Body is the
target and the patterned tooth bodies are the tools.

```python
combineInput = designComponent.features.combineFeatures.createInput(gearBody, toolCollection)
combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
designComponent.features.combineFeatures.add(combineInput)
```

The tool collection is the `ObjectCollection` built in S24 from `pattern.bodies`.

The root circle drawn in S9 sits one root sink inside the dedendum corner, which is what makes this
join overlap along the whole root rather than along the tooth's centreline alone.

Proved by `stepCombineJoin`, which **performs no join**, because the two operands are not in one
frame: the proof builds the tooth on the back-cone section at the Pitch Cone Distance from the Apex,
scaled about the Apex — the Tredgold mapping — while the gear body is written about the shaft axis,
and the tooth's own seating on that body is derived nowhere in this proof. The two therefore do not
meet when they are put in one document, and neither sign of the rotation that relates the two planes
seats them. **The engine is not what blocks this join**: given operands that do overlap it performs
the union, returns one lump and publishes a tight volume bound. What is missing is the tooth's real
back-cone placement, and deriving it is its own change. **The cost is the stitch**: the proof cannot
show the evaluator making one boundary out of two.

What it asserts instead are the join's two consequences, from the operands' own measured geometry: a
join leaves **one** lump when the tooth's root is below the gear body's root cone — seated, not
floating — and the joined body reaches further out than the frustum when the tooth's tip stands proud
of it. Both are read at the **toe, the middle and the heel** of the band the join would cover, and the
first of them is read at the **root arc's OUTERMOST point and at both root corners, never at the
tooth's centreline**: the centreline sits inside both corners, so a reading taken there passes a tooth
whose corners float outside the cone, which is exactly the defect the root sink exists to remove. The
proof applies the same sink the module does — it is one figure, not a proof-only offset.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCombineJoin, assertCombineJoin) -->

**From:** `spec/bevelgear/instructions.md` L820-822, L1025-1026, L1147-1205; `.claude/skills/generate-gear/PLAYBOOK.md` L693-697

## S26 `[GO]` `{gearLabel} Bore` sketch

Skip this step **entirely** when Enable Bore is unchecked: no bore is cut on either gear and the
per-gear bore diameter inputs are ignored.

Build the bore plane normal to the shaft at its start:
`constructionPlanes.createInput()`, then `setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))`,
then `constructionPlanes.add(planeInput)`. Pass the in-sketch edge, not the S6 construction line, and
never wrap it in `Path.create`.

In a sketch named `{gearLabel} Bore` on that plane, sketch the bore circle **centred at the sketch
origin** — the plane is rooted at the shaft start, so the origin is already on the axis. **Fix the
circle's centre and add a diameter dimension** set to the bore diameter:
`circle.centerSketchPoint.isFixed = True` plus `addDiameterDimension(circle, textPoint)`. A circle
created at (0, 0, 0) does **not** reuse the sketch's `originPoint` — its centre is a free point that
happens to sit there — and constraining it to the origin has been observed to throw
`VCS_SKETCH_SOLVING_FAILED` on exactly this kind of plane (`[PB-CIRCLE-CENTER]`).

The diameter is this gear's `boreDiameter_cm` from the per-gear dict: already resolved and already
bounded by that gear's Maximum Bore Diameter in S6. **Take that number; do not re-derive it here**, or
the cap is lost and the bore deletes the body's back face.

Gate the sketch on `sketch.isFullyConstrained`, raising and naming it — the Bore sketch is one of the
four permanent sketches the gate covers (`[BEVEL-F-FULL-CONSTRAINT]`).

<!-- check-step-calls: ignore Path.create -->

Proved by `stepBoreSketch`, which draws the same circle with a fixed centre and a driving diameter
dimension, reaches DOF 0, and reads back that the sketch closes exactly one region of the bore disc's
own area. A case where Enable Bore is unchecked draws no sketch at all and is recorded as unmodelled
rather than returning quietly.

<!-- proof-run: proofkit.RunParallel(boreCases, stepBoreSketch) -->

**From:** `spec/bevelgear/instructions.md` L137-188, L1027-1028; `.claude/skills/generate-gear/PLAYBOOK.md` L451-457, L775-786

## S27 `[GO]` Cut the bore

Skip this step entirely when Enable Bore is unchecked.

Extrude-cut the bore circle as a **symmetric through cut** restricted to this Gear Body:

```python
extrudeInput = designComponent.features.extrudeFeatures.createInput(
    boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(2 * coneDistance_cm), False)
extrudeInput.participantBodies = [gearBody]
designComponent.features.extrudeFeatures.add(extrudeInput)
```

`isFullLength = False` means the distance is the half-length **per side**, so `2 * Cone Distance` per
side is generously past any face width (`[PB-THROUGH-CUT]`). Do not pass a third, taper argument.
Restrict the cut with `participantBodies` so it reaches this gear's body and nothing else.

Proved by `stepBoreCut`, which asserts the tool first, from its own measured geometry — its diameter,
that its two ends sit exactly `2 * Cone Distance` either side of the shaft edge's start, and that both
clear the frustum, which is what makes it a through cut — and then performs the cut and asserts the
pierced body's volume against the target's own closed form less the prism the bore removes over that
height, that the result is **one lump**, which is what a through hole leaves, and that it is solid,
which an enclosed void would not be. It also requires the resolved bore diameter to be inside that
gear's Maximum Bore Diameter.

**The target is the heel cone band, lofted for this step**: it is the section of the gear body the
bore passes through, and it is a Loft, which is the form a boolean operand takes here. The revolved
gear body cannot be the target, for the reason the operand rule gives. **What this does not reach is
the rest of the body** — the bore is pierced through the band that stands for the heel section, not
through the whole frustum — and the proof file records that beside the assertion.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepBoreCut, assertBoreCut) -->

**From:** `spec/bevelgear/instructions.md` L1027-1028, L1206-1219; `.claude/skills/generate-gear/PLAYBOOK.md` L729-732

## S28 `[GO]` Meshing rotation — driving gear only

Do this **here, in the Design component, before the body is moved out.** Rotate the driving body by
`180 degrees / Driving Gear Teeth Number` — half a tooth pitch — about its shaft axis:

```python
rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)
```

the framework helper from `.solids`, which takes the rotation axis and origin from the profile edge's
**world** endpoints (`[PB-MOVE-ROTATE]`).

Both gears are patterned from a starting tooth in the axial plane, so without this offset a driving
tooth and a pinion tooth would both sit at the axial-plane crossing and visually collide. With it, a
driving valley sits where the pinion tooth crosses, giving the interlocked meshing look.

This runs in Design before `moveToComponent` because a construction axis cannot be added in the
moved-out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation must use the edge's world
geometry while the body is still in Design.

The pinion additionally gets `_pinionMeshPhase(pinionTeeth)`, which is 0 unless a spiral pair needs
it — and it is 0 by default, because S20 leaves the mid-face section unrotated precisely so that none
is needed. **A zero angle is a no-op, not a move**: `setToRotation(0, axis, origin)` builds the
identity and Fusion refuses to move a body by it with `RuntimeError: 3 : invalid transform`. Any
caller that *computes* an angle can legitimately arrive at zero, so `rotate_body_about_edge` absorbs
it and returns early rather than each call site guarding it.

Proved by `stepMeshRotation`, which pins both sides of the branch: the driving body turns by half a
tooth pitch about its own shaft axis and the pinion does not turn at all. The rotated body is compared
against the unrotated one as two readings, so the rotation is shown to change where the tooth sits and
nothing else.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepMeshRotation, assertMeshRotation) -->

**From:** `spec/bevelgear/instructions.md` L528-530, L1029-1032; `.claude/skills/generate-gear/PLAYBOOK.md` L800-809

## S29 `[PROSE]` Move the finished bodies into the gear component

Relocate this gear's finished body into its own `{gearLabel} Gear` component with
`body.moveToComponent(gearOccurrence)`, which preserves world position and needs no activation
(`[PB-NO-CROSS-SIBLING]`). Every feature ran in the Design component precisely so that no
cross-sibling reference was ever needed; the visible end state is identical.

This closes the per-gear pass. Run S7 through S29 again for the driving gear, then continue to S30.

**From:** `spec/bevelgear/instructions.md` L505-516, L985-986; `.claude/skills/generate-gear/PLAYBOOK.md` L829-834

## S30 `[PROSE]` Cleanup — hide the construction geometry

Call the framework helper `hide_construction_geometry(self.bevelComponent)` from `.solids`. It
recursively walks the Bevel Gear component tree, deduping by `entityToken`, and hides every sketch,
construction plane and construction axis by setting `isLightBulbOn = False`. Construction planes and
axes are **not** hidden by `isVisible` — that is a Fusion gotcha, and `isVisible` hides sketches while
`isLightBulbOn` hides construction planes and axes; do not cross them (`[PB-HIDE-AFTER-USE]`,
`[BEVEL-F-CLEANUP]`, `[PB-TREE-CLEANUP]`). Do not re-implement the walk.

There is no sketch-only mode and no per-mode guard — bevel always builds solids. Leave only the two
finished gear bodies visible.

The driving gear's half-tooth-pitch meshing rotation is **not** a cleanup step; it is performed in S28,
in the Design component, before the body is moved out.

Do not add a settle-the-display call here. `commands/_gear_command.py` calls
`geargen.settle_sketch_display` once after `generate()` returns, every gear command runs through that
one call, and nothing about it belongs in a generated module (`[PB-SETTLE-DISPLAY]`).

`isVisible` and `settle_sketch_display` are named only to say what this step does not do, and
`generate` is the module's own entry method — `commands/bevelgear/entry.py` calls
`BevelGearGenerator(design).generate(inputs)`, and the module defines it rather than calling it — so
naming it here to fix when the settle runs is a mention and not a call this module makes.

<!-- check-step-calls: ignore isVisible settle_sketch_display generate -->

**From:** `spec/bevelgear/instructions.md` L1033-1038; `spec/bevelgear/fusion.md` L216-221; `.claude/skills/generate-gear/PLAYBOOK.md` L534-555, L659-671, L835-837