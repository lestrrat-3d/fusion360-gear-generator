# Bevel Gear — compiled step list

The proof for this step list is `proof/bevelgear/geometry_test.go`, `proof/bevelgear/sketches_test.go`, `proof/bevelgear/solids_test.go`, `proof/bevelgear/spiral_test.go` and the generated `proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `77c90b20c527148248f154b934320a3d5f09a7ce` |
| `spec/bevelgear/fusion.md` | `efa49ddcd40c9d796c687097ff9fa620c1bda325` |
| `spec/bevelgear/spiral-tooth-trace.md` | `84474c55a77775fba98991437ef77a0fa812bd12` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9ee2dcbaed7b5480aa69e9295e8b61acaea081f3` |

## S01 `[PROSE]` Command dialog inputs

Add the **20** dialog inputs to `cmd.commandInputs` in exactly the display order of the table below,
from `@classmethod def configure(cls, cmd)` on `BevelGearCommandInputsConfigurator`. Target Plane is
added first so Fusion's auto-focus lands on it (`[PB-AUTOFOCUS-FIRST]`), Center Point second, the
pre-selected Parent Component third, then the numeric/bool fields.

Module-level constants hold the id strings, in row order, named exactly:
`INPUT_ID_PLANE`, `INPUT_ID_CENTER_POINT`, `INPUT_ID_PARENT`, `INPUT_ID_MODULE`,
`INPUT_ID_SHAFT_ANGLE`, `INPUT_ID_DRIVING_TEETH`, `INPUT_ID_PINION_TEETH`,
`INPUT_ID_DRIVING_BASE_HEIGHT`, `INPUT_ID_PINION_BASE_HEIGHT`, `INPUT_ID_BORE_ENABLE`,
`INPUT_ID_DRIVING_BORE`, `INPUT_ID_PINION_BORE`, `INPUT_ID_FACE_WIDTH`, `INPUT_ID_TOOTH_SPACING`,
`INPUT_ID_SPIRAL_ANGLE`, `INPUT_ID_HAND`, `INPUT_ID_CUTTER_RADIUS`, `INPUT_ID_TOE_EXTENSION`,
`INPUT_ID_DRIVING_TOE_RADIUS`, `INPUT_ID_PINION_TOE_RADIUS`.

Two further module constants carry the dropdown's item strings: `_HAND_RIGHT = 'Right'` and
`_HAND_LEFT = 'Left'`.

The whole table is reproduced here because the emit stage cannot open the prose spec. Every id,
label, unit string, default and tooltip below is literal surface — use it verbatim.

| # | Dialog input (label) | input id | input type | unit | default | selection filters / tooltip |
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

Calls: `cmd.commandInputs`, `inputs.addSelectionInput(id, label, tooltip)`,
`selectionInput.addSelectionFilter(filter)`, `selectionInput.setSelectionLimits(1, 1)`,
`selectionInput.addSelection(entity)`, `inputs.addValueInput(id, label, unit, initialValue)`,
`inputs.addBoolValueInput(id, label, True, '', True)`,
`inputs.addDropDownCommandInput(id, label, style)`, `dropdown.listItems.add(name, isSelected)`,
`adsk.core.ValueInput.createByReal(value)`, `adsk.core.ValueInput.createByString(expression)`,
`adsk.core.DropDownStyles.TextListDropDownStyle`,
`adsk.core.SelectionCommandInput.ConstructionPlanes`,
`adsk.core.SelectionCommandInput.PlanarFaces`,
`adsk.core.SelectionCommandInput.ConstructionPoints`,
`adsk.core.SelectionCommandInput.SketchPoints`,
`adsk.core.SelectionCommandInput.Occurrences`,
`adsk.core.SelectionCommandInput.RootComponents`.

Selection filters are written as the named constants, never quoted literals
(`[PB-SELECTION-FILTER-ENUM]`), and each selection input declares its filter set and
`setSelectionLimits(1, 1)` (`[PB-SELECTION-DECL]`). The Parent input pre-selects
`get_design().rootComponent` with `addSelection`. The `mm`/`deg` defaults go in as internal units
(`[PB-DIALOG-DEFAULT-UNITS]`): `to_cm(...)` for every length, `createByString('90 deg')` and
`createByString('35 deg')` for the two angles so the expression engine parses them. `toeExtension`
is a plain unitless percentage and takes no `to_cm`. The Hand dropdown uses
`DropDownStyles.TextListDropDownStyle` with `Right` added selected and `Left` added unselected.

Bevel registers **no** Fusion user parameters — every value is precomputed in Python in internal cm
and written into geometry numerically (`[PB-PRECOMPUTED-MODE]`). There are therefore no `PARAM_*`
strings; the 20 `INPUT_ID_*` constants plus `_HAND_RIGHT`/`_HAND_LEFT` are the only module-level
constants.

`configure()` calls `cls._updateSpiralInputVisibility(inputs)` as its **last** step (see S02).

This step is `[PROSE]` because a command dialog is not geometry: neither harness can hold a
`CommandInputs` collection, and there is no geometric substitute for a list of dialog rows. The
proof records that limit beside the §2 lattice in `proof/bevelgear/sketches_test.go`.

<!-- check-step-calls: ignore configure _updateSpiralInputVisibility -->
`configure` and `_updateSpiralInputVisibility` are named above as the methods the module defines for
the framework and for S02 to call, not as calls this step makes.

**From:** `spec/bevelgear/instructions.md` L25–36, L100–105, L142, L160–180, L181–262, L310–336;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-AUTOFOCUS-FIRST]`, `[PB-SELECTION-DECL]`,
`[PB-SELECTION-FILTER-ENUM]`, `[PB-DIALOG-DEFAULT-UNITS]`, `[PB-PRECOMPUTED-MODE]`.

## S02 `[PROSE]` Conditional visibility of the spiral-only inputs

Hand of Spiral and Cutter Radius are relevant only to a curved bevel, so they are **hidden whenever
Mean Spiral Angle ψ = 0 and shown when ψ > 0**. Mean Spiral Angle itself is the controller and is
**always visible**. There is no declarative show-if in the Fusion API, so this is realized with
`commandInput.isVisible`.

Add `@classmethod def _updateSpiralInputVisibility(cls, inputs)`. It reads the `spiralAngle` input's
**`.expression`** and evaluates it through
`unitsManager.evaluateExpression(spiral.expression, 'rad')` — internal **radians**; it does **not**
read the input's `.value` — then sets `inputs.itemById(INPUT_ID_HAND).isVisible` and
`inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible` to `(value > 0)`. Guard it: if any of the three
inputs is `None`, return early; wrap the evaluation in `try/except`, because a half-typed expression
can raise mid-edit, and on failure leave **both** inputs shown.

Add `@classmethod def handle_input_changed(cls, args)`, which calls
`cls._updateSpiralInputVisibility(args.inputs)` on **every** input change — cheap and robust, with no
branch on which input changed. `commands/bevelgear/entry.py` binds it by name as the dialog's
`inputChanged` handler.

`isVisible` only hides the dialog row. The input still exists, `_readInputs` reads it normally, and a
ψ = 0 build ignores Hand and Cutter anyway, so hiding is purely cosmetic and cannot affect
generation.

Calls: `unitsManager.evaluateExpression(expression, 'rad')`, `inputs.itemById(id)`,
`commandInput.isVisible`, `adsk.fusion.Design.unitsManager`.

<!-- check-step-calls: ignore handle_input_changed _updateSpiralInputVisibility -->
<!-- check-compile: ignore handle_input_changed _updateSpiralInputVisibility -->
Both names are methods this module **defines** — `handle_input_changed` for the framework to call
from `commands/bevelgear/entry.py`, `_updateSpiralInputVisibility` for `configure()` and
`handle_input_changed` to call — rather than calls this step makes on the Fusion API. Neither is a
Fusion name, so neither is one the API database can back.

This step is `[PROSE]` for the reason S01 gives: dialog state is not geometry, and no substitute
survives either harness gate.

**From:** `spec/bevelgear/instructions.md` L176–180, L226–247, L319–323, L419–426;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-EVAL-EXPRESSION]`.

## S03 `[PROSE]` Read and validate every input

`generate(inputs)` calls `_readInputs(inputs)` **first**, before anything creates an occurrence. It
returns the 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
shaftAngle_deg)` and stashes the rest on `self`: `self._drivingBaseHeight_cm`,
`self._pinionBaseHeight_cm`, `self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`,
`self._faceWidth_cm`, `self._toothSpacing_cm`, `self._spiralAngle_rad`, `self._hand`,
`self._cutterRadius_cm`, `self._toeExtension_pct`, `self._drivingToeRadius_cm`,
`self._pinionToeRadius_cm`.

**How each input is read** (`[PB-INPUT-READ]`). Selections go through `get_selection(inputs, id)`;
the Enable Bore checkbox through `get_boolean(inputs, id)`; the dropdown through
`inputs.itemById(INPUT_ID_HAND).selectedItem` — read its `.name`, defaulting to `_HAND_RIGHT` when
none. Every numeric and angular input is read by evaluating its expression with
`unitsManager.evaluateExpression(input.expression, units)` using `''` / `'mm'` / `'deg'` as the table
declares, which always returns Fusion internal units — cm for length, radians for angle
(`[PB-EVAL-EXPRESSION]`).

**Units, and this is the one that makes a gear come out ten times too big.** The `'mm'` inputs
(both base heights, both bore diameters, Face Width, Tooth Spacing, both toe radii) and the two
`'deg'` inputs come back **already internal** — use them as-is, never `to_cm` them again. `Module` is
read with unit `''`, so it comes back a raw number meaning **millimetres**: a module of `1` is 1 mm.
Every length derived from Module therefore needs `to_cm` before it touches geometry — Pitch Diameter
`to_cm(Module * teeth)`, Cone Distance, the dedendum `to_cm(1.25 * Module)`, every §2 seed length,
and the default Face Width `Cone Distance / 6`. `toeExtension` is a unitless percentage and takes no
conversion. Both tooth counts are coerced with `int(round(...))` before validation.

**Range checks, in this order.**

1. `module > 0`; both teeth counts `>= 3`; non-negative base heights, bore diameters, Face Width,
   Tooth Spacing, Cutter Radius and both Toe Radii; Toe Extension in `[0, 100]`; Mean Spiral Angle
   in `[0, 60)` degrees.
2. Shaft Angle **at least 30°** and **at most the Maximum Shaft Angle**. Convert to degrees with
   `math.degrees(...)` before the check. The Maximum Shaft Angle depends on both tooth counts, so it
   is checked after both are read and coerced, and the computed limit goes into the message. It is

       min( degrees(acos(-min(DPD, PPD) / max(DPD, PPD))), 150 )

   with the `acos` half **exclusive** (reject at or above it) and the 150° half **inclusive**. A
   31/17 pair gives `acos(-17/31) = 123.26°`; equal tooth counts give `acos(-1) = 180°`, which is no
   constraint at all and leaves the flat 150° cap. The `acos` limit is where a pitch cone angle
   reaches 90°, which turns that gear's cone inside out: `R * cos γ` passes through zero and changes
   sign, so the along-shaft seed points backwards and the back-cone virtual radius is unbounded.
3. Compute `γ_p` and `γ_g` from the closed form, now that both counts and Σ are known:
   `tan γ_p = sin Σ * PPD / (DPD + PPD * cos Σ)`, `γ_g = Σ − γ_p`.
4. **Minimum Teeth**, per gear, with that gear's own `γ`, on top of the blanket `teeth >= 3`:

       teeth >= 5.27 * cos γ

   The constant is `2 * (1.05 * 1.25 / 0.95 + 1.25) = 5.2632`, rounded **up** to 5.27 so the
   published floor stays at or above the exact crossing. Do not round it down and do not substitute
   the exact value. Name the computed floor in the message. At Σ = 90° the floor is 3.72, i.e. 4
   teeth: an equal 4-tooth pair solves and a 3-tooth pair fails on the heel edge.
5. **Base heights**, per gear, both bounds closed-form:

       Minimum Base Height = 1.05 * 1.25 * Module * sin γ
       Maximum Base Height = 0.95 * (r - 1.25 * Module * cos γ) * tan γ

   with `r` that gear's own pitch radius. Raise a fallback that falls below the minimum, cap a
   fallback that exceeds the maximum, and **reject** a user value outside either end naming the
   bound it broke. The driving fallback is `Module * Driving Gear Teeth Number / 8`; the pinion
   fallback is the **resolved** driving base height times `Pinion Teeth / Driving Teeth`, then
   passed through the **pinion's own** bounds, because the two gears have different cone angles
   whenever the tooth counts differ.

   Read the Maximum Base Height's origin carefully: the base height is measured from **Apex 2's
   plane**, not from the dedendum point, so the true heel crossing is at `r * tan γ` and this bound
   sits `1.25 * Module * sin γ` below it. It is deliberately conservative, refusing a band of base
   heights that would in fact still build. Past the true crossing the hexagonal frustum profile has
   crossed its own axis of revolution and the revolve fails with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`).

The Minimum Teeth check runs **before** the base-height resolution because it is exactly the
statement that the base-height window is non-empty, so step 5 never has to describe what to do when
the minimum exceeds the maximum.

**The bore bound is deliberately not part of this pass.** Validate the two bore diameters here only
as non-negative numbers. Each gear's Maximum Bore Diameter resolves in S07, where the Maximum Face
Width is applied: its heel term would resolve here, but its toe term needs the Root Length, hence
the resolved Face Width, hence solved §2 geometry, and the bound is the minimum of the two.

Calls: `get_selection(inputs, id)`, `get_boolean(inputs, id)`, `inputs.itemById(id)`,
`unitsManager.evaluateExpression(expression, units)`, `adsk.fusion.Design.unitsManager`,
`to_cm(value)`.

<!-- check-step-calls: ignore _readInputs generate -->
`_readInputs` and `generate` name this module's own methods, not Fusion API calls.

This step is `[PROSE]`. Its arithmetic has no geometry to build: a case table that drew a
stand-in figure in order to check a range message would be asserting against a stand-in rather than
against the real construction. The closed forms themselves are not left unproven — every bound
above is computed and asserted against solved §2 geometry in S07's proof
(`stepGearProfiles`), and against built solids in S27's (`stepBoreCut`). What no case can reach is
the generated module raising on a user's value, because the proof never runs that module; that gap
is recorded in `proof/bevelgear/sketches_test.go` beside the lattice assertion.

**From:** `spec/bevelgear/instructions.md` L35–52, L54–99, L100–141, L142–151, L263–309, L337–362;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-INPUT-READ]`, `[PB-EVAL-EXPRESSION]`,
`[PB-REVOLVE]`.

## S04 `[PROSE]` Build the component tree

With every selection already read (S03), create the occurrence tree directly — bevel does **not**
subclass `base.Generator` and does not use `getOccurrence` (`[PB-OCCURRENCE-TREE]`):

1. `Bevel Gear` as a child of the user's Parent Component:
   `parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`, then
   `occurrence.component.name = 'Bevel Gear'`. Stash the occurrence on `self.bevelOccurrence` for
   rollback and `self.bevelComponent` for cleanup.
2. `Design` as a child of `Bevel Gear`, the same way, named `Design`. Stash
   `self.designOccurrence` and `self.designComponent`. Every sketch, construction plane, construction
   axis and feature in this build runs in **this one component**.

**Never call `occurrence.activate()`** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`). The
Anchor Sketch is created on the user's **external**, root-owned target plane; an activated occurrence
resolves that external plane in its own local frame and the whole build collapses onto world XY no
matter what plane the user picked. Because every feature runs in Design, no cross-sibling reference
is ever needed (`[PB-NO-CROSS-SIBLING]`). The sole exception in the whole module is the spiral
crown's `scaleFeatures` step (S20).

`deleteComponent()` is the rollback the entry point calls on any exception: it calls `deleteMe()` on
`self.bevelOccurrence`. There are no user parameters to clean up.

Calls: `parent.occurrences.addNewComponent(transform)`, `adsk.core.Matrix3D.create()`,
`occurrence.component`, `component.name`, `occurrence.deleteMe()`.

<!-- check-step-calls: ignore activate deleteComponent -->
`activate` is named only to forbid it, and `deleteComponent` is this module's own rollback method
rather than a Fusion call.

This step is `[PROSE]`: an occurrence tree is assembly bookkeeping with no geometry in it, and
neither harness models components at all. The proof builds every case in one flat document and says
so in `proof/bevelgear/solids_test.go`.

**From:** `spec/bevelgear/instructions.md` L19–23, L310–336, L419–445, L518–529, L566–577;
`spec/bevelgear/fusion.md` L199–206; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-OCCURRENCE-TREE]`, `[PB-NEVER-ACTIVATE]`, `[PB-NO-CROSS-SIBLING]`.

## S05 `[GO]` Anchor sketch

One Fusion timeline entry: the sketch named `Anchor`.

Start it **directly on the user-selected target plane**, whether that selection is a
`ConstructionPlane` or a `PlanarFace` — `designComponent.sketches.add(targetPlane)`. Do not
re-derive it and do not build a coplanar offset plane first (`[PB-USE-SELECTED-PLANE]`): a
construction plane offset from a face in another component resolves in the sub-component's own frame
and silently loses the selected plane's world orientation, collapsing the gear onto XY.

Then, in this order:

1. Project the user-specified center point into the sketch with `sketch.project(centerPoint)`. Keep
   the returned projected `SketchPoint`; this is the anchor sketch's center point and **§2 projects
   this point, not the raw user selection**, so stash it on `self._anchorCenterPoint`.
2. Draw the Anchor Line through it with `sketch.sketchCurves.sketchLines.addByTwoPoints(p1, p2)`,
   seeding its two endpoints at **exactly ±0.5 cm from the projected center along the sketch-local
   X**, so the seeded length is 10 mm.
3. Apply **both** `sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)` — the
   intersection, which pins the center onto the line — **and**
   `sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)`, which makes the center
   bisect it. Both, never midpoint alone.
4. Add an **aligned** distance dimension and do **not** assign `.parameter.value`:
   `sketch.sketchDimensions.addDistanceDimension(anchorLine.startSketchPoint, anchorLine.endSketchPoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`.
   The dimension simply locks the length at the seeded 10 mm; the value is arbitrary because this is
   only a reference line.
5. Pin the direction so the sketch ends fully constrained:
   `sketch.geometricConstraints.addHorizontal(anchorLine)` — sketch-local, per
   `[PB-REFLINE-DIRECTION]`, which works on any tilted target plane where a world-axis lock would
   mis-orient the figure.

The anchor line's absolute direction is arbitrary — nothing downstream depends on it, because §2
derives every direction *relative* to the projected anchor line — but it must not be a free degree of
freedom. With midpoint plus length plus Horizontal the line has zero DOF.

Gate it: `if not sketch.isFullyConstrained: raise` naming the sketch
(`[BEVEL-F-FULL-CONSTRAINT]`, `[PB-FULL-CONSTRAINT]`). This sketch carries no text, so it gates
normally.

`sketch.project` is written as `project`, not `project2`. The compiled Fusion API reference declares
`project2(entities, isLinked)` and no `project`, so every gate in this repo reports the call as
unverified. That report is expected and is not a defect to fix here: `project` is what the shipped
add-ins call, it sits on `fusion_api.py`'s `UNVERIFIED_CALLS`, which is reported rather than
blocking, and the two are not interchangeable anyway — `project2` takes a list and returns a list.

Calls: `component.sketches.add(planarEntity)`, `sketch.name`, `sketch.project(entity)`,
`sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`,
`adsk.core.Point3D.create(x, y, z)`,
`sketch.geometricConstraints.addCoincident(point, entity)`,
`sketch.geometricConstraints.addMidPoint(point, midPointCurve)`,
`sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, orientation, textPoint)`,
`adsk.fusion.DimensionOrientations.AlignedDimensionOrientation`,
`sketch.geometricConstraints.addHorizontal(line)`,
`sketch.isFullyConstrained`,
`sketchLine.startSketchPoint`, `sketchLine.endSketchPoint`.

<!-- check-step-calls: ignore project2 -->
`project2` is named only to forbid substituting it for `project`; the module must not call it.

Proof: `stepAnchorSketch`.

<!-- proof-run: proofkit.RunParallel(anchorCases, stepAnchorSketch) -->

**From:** `spec/bevelgear/instructions.md` L491–517, L578–583, L592–595;
`spec/bevelgear/fusion.md` L19–30; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-USE-SELECTED-PLANE]`, `[PB-REFLINE-DIRECTION]`, `[PB-FULL-CONSTRAINT]`,
`[PB-SHARE-XOR-COINCIDENT]`, `[PB-SKETCHCURVES]`, `[PB-API-SPELLING]`.

## S06 `[PROSE]` Gear Profiles Plane

One timeline entry: the construction plane named `Gear Profiles Plane`.

Build it with `setByAngle` through the Anchor Line at **90°**, measured off the **original
`targetPlane`** as the reference — by default the plane would lie flush to the anchor line's own
plane, and 90° stands it perpendicular. Do not re-derive or offset the target plane
(`[PB-USE-SELECTED-PLANE]`): this is the second place the target-plane orientation reaches the
bodies, and substituting a different reference here also collapses the gear onto XY.

Pass the `SketchLine` **directly** to `setByAngle`; never wrap it in `adsk.fusion.Path.create` first
(`[PB-CONSTRUCTION-PLANES]`).

Calls: `component.constructionPlanes.createInput()`,
`planeInput.setByAngle(linearEntity, angle, planarEntity)`,
`component.constructionPlanes.add(input)`, `adsk.core.ValueInput.createByString('90 deg')`,
`constructionPlane.name`.

<!-- check-step-calls: ignore Path.create -->
`Path.create` is named only to forbid it.

This step is `[PROSE]`. A construction plane is a frame, not geometry: the sketch engine is planar
and has no second plane to tilt against, and `decad` would need a body to hang it on, so there is no
substitute that the gate accepts. What the plane is *for* — that §2's figure lives in a plane
perpendicular to the target plane containing the anchor line, so that the in-plane perpendicular to
the projected anchor line **is** the target-plane normal — is exactly the assumption
`stepGearProfiles` builds on, and that is where the limit is recorded.

**From:** `spec/bevelgear/instructions.md` L584–586; `spec/bevelgear/fusion.md` L162–189;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-USE-SELECTED-PLANE]`, `[PB-CONSTRUCTION-PLANES]`.

## S07 `[GO]` Gear Profiles sketch (the §2 lattice)

One timeline entry: the sketch named `Gear Profiles`, on the Gear Profiles Plane. It carries the
whole two-gear lattice — 22 named points and the lines between them — and every bound that needs
solved geometry.

### Sketch-wide rules

- **Every line drawn in this sketch is a construction line** — `line.isConstruction = True` — the
  lattice lines, the toe lines M->N / O->P, and the short reference/connector lines M->C, N->A′,
  O->D, P->B′, A′->G, B′->I, C->K/K′, D->L/L′ alike. The solid features later consume only the
  per-gear Profile sketches of S12, never a §2 curve directly.
- **Every length dimension in this sketch is `AlignedDimensionOrientation`.** This figure has no
  axis-aligned line in it — the shaft axes sit at the Shaft Angle to each other, the lattice tilts
  with the target plane, and the sketch is not world-aligned — so a Horizontal or Vertical
  orientation would dimension the line's *projection* onto a sketch axis instead of its length. That
  covers the PPD/2 and DPD/2 drops to Apex 2, the two `Module * 1.25` dedendum lines, the Tooth
  Spacing dimension on the K′ / L′ lines, and the Toe Radius dimension on the two front faces N->A′
  and P->B′. The offset dimensions are a different call, `addOffsetDimension`, which takes no
  orientation.
- **Coincident style, never sharing** (`[BEVEL-F-COINCIDENT-STYLE]`, a stricter delta to
  `[PB-SHARE-XOR-COINCIDENT]`). Build **every** §2 line — lattice or reference — from raw
  `adsk.core.Point3D` coordinates, then `addCoincident` **each** endpoint to its existing point, one
  per end. Never pass an existing `SketchPoint` into `addByTwoPoints` to share it. Sharing without a
  coincident leaves the sketch under-constrained; sharing **and** coinciding is redundant and the
  solve fails outright with `VCS_SKETCH_SOLVING_FAILED`. This covers the short connector lines whose
  both endpoints already exist; a regen that shared only those came out about 14 coincidents short.
- **Each named line is created once and reused** (`[BEVEL-F-LINE-ONCE]`). The extension lines
  A->E, B->F, E->G, F->I and the dedendum/closing lines C->H, D->J, G->H, I->J are *named*
  construction lines: the helper that creates one must return the line object and later steps use
  that reference. Drawing a second line over the same segment to obtain a reference over-determines
  the coupled net and fails with `VCS_SKETCH_OVER_CONSTRAINTS`.
- **The driven lengths are not dimensioned** (`[BEVEL-F-DRIVEN-DIMS]`, `[PB-NO-OVERCONSTRAIN]`).
  Apex->A, Apex->B and the extension lines are driven by the closing and collinear constraints.
  Undimensioned does not mean unpinned: each carries a closed-form seed, and the seed is what picks
  the figure.
- **Every seed is load-bearing geometry** (`[BEVEL-F-MIRROR-FIGURE]`). Fifteen constraint sites admit
  a mirrored solution that satisfies every constraint, and Fusion offers no constraint that pins any
  of them — 13 independent binary choices on a 43/31 pair at Σ = 75° with Tooth Spacing above zero,
  so 8192 distinct figures satisfy every constraint; 10 choices and 1024 figures for the default pair
  at Σ = 90° with Tooth Spacing 0. Never add a Fusion constraint to pin a side: `addSymmetry` on C
  and D against the Pitch Line rules out the collapse but not the swap and is a net redesign the
  `[PB-SKETCH-FIRST]` waiver forbids, and `SketchPoint.isFixed` over-constrains and turns the
  parametric lattice into placed geometry. Seed correctly and gate the result.
- **The figure's position is sketch-local** (`[BEVEL-F-APEX-LOCAL]`). Never compute a §2 position
  from a world round-trip. The one permitted world use is reading the target normal as a *direction*
  to choose `perp`'s sign (`[BEVEL-F-GROW-SIDE]`) — a one-bit comparison, not a position.

### Derived quantities this step needs

With `Σ` the Shaft Angle in radians, `m` the raw-mm Module, `N_d` / `N_p` the tooth counts,
`DPD = m·N_d`, `PPD = m·N_p`, and both base heights already resolved in S03:

    tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)
    γ_g     = Σ − γ_p
    R       = (PPD / 2) / sin γ_p                          # Pitch Cone Distance
    CD      = sqrt(DPD² + PPD²)                            # Cone Distance (the diagonal)

**`Cone Distance` and `Pitch Cone Distance` are two different lengths and both are used.** `CD` is
the diagonal of the two pitch diameters and depends on the tooth counts alone, never on Σ. `R` is
the real apex-to-heel length along the pitch cone. They coincide as `CD = 2·R` **exactly when
Σ = 90°** and diverge everywhere else: an equal 31/31 pair at Σ = 30° has `CD = 43.84 mm` against
`R = 59.89 mm`, and at 140° `R = 16.49 mm`. Where this step says "Cone Distance" it means the
diagonal; the pitch cone distance is always written `R`.

### Frame

Project **the Anchor Sketch's center SketchPoint** — the one stashed in S05, not the raw user
selection. Both happen to be coincident, but projecting the anchor-sketch point keeps the chain
inside the Design component; projecting the raw external point is a cross-component reference and
can resolve inconsistently. Project the anchor line too, for its direction.

Let `c` be the projected center and `d` the projected anchor line's 2-D unit direction. The in-plane
perpendicular is `perp = (-d.y, d.x)`. **Choose `perp`'s sign by the target-plane normal, not by the
sketch's local +Y** (`[BEVEL-F-GROW-SIDE]`): the sketch's local +Y maps to different world sides
depending on how the plane was oriented, so a `perp.y >= 0` rule grows the gear inconsistently. Read
the normal as **`targetPlane.geometry.normal`** for **both** selection kinds — a `BRepFace`'s
`geometry` and a `ConstructionPlane`'s `geometry` are each a `core.Plane` carrying `.normal` — and
pick the sign that points toward it.

### The lattice, in creation order

Each point below is given with the closed-form seed §2 requires. Seed at the value, not near it
(`[PB-SEED-NEAR]`).

1. **centerToApex**, from `c`, perpendicular to the projected anchor line (apply
   `addPerpendicular`). Its far end is the **Apex**, seeded at

       Apex = c + perp · (R · cos γ_g + <resolved Driving Gear Base Height>)

   Not `c + perp · DPD`, which earlier revisions said: the net closes this line at `R·cos γ_g` above
   point I plus the resolved driving base height, so on the default 31/31 pair at Σ = 90° the old
   seed sat 11.6 mm past where the solve puts it. Add **no** length constraint on this line. Pin its
   start with exactly one `addCoincident(centerToApex.startSketchPoint, projectedCenter)`.
2. **Driving Gear Shaft Axis**, from the Apex toward the anchor line, i.e. in `-perp`. Seed its far
   end at `apex - perp·(R · cos γ_g)`, which is `c + perp·(<resolved Driving Gear Base Height>)` —
   measure from the **apex**, not from `c`. Apply `addParallel(drivingShaftAxis, centerToApex)`.
   **Do not use `addVertical`**: that forces the line to the sketch's world-vertical, which is wrong
   on a tilted target plane and over-constrains the figure. Coincident its start to the Apex. Do not
   dimension its length. Its end is **point B**.
3. **Pinion Gear Shaft Axis**, from the Apex, the driving direction rotated about the apex by Σ.
   The rotation has two senses and they place point A on opposite sides. **Form both candidate A
   positions — the driving direction rotated by +Σ and by −Σ — and keep the candidate whose endpoint
   has the greater X coordinate in this sketch.** Compare the two and take the larger; do not rotate
   one fixed sense and flip only when its X comes out negative, because when *both* candidates have
   positive X that shortcut keeps the wrong one. Call the chosen unit Apex→A direction `pinionDir`.
   Seed the far end at `Apex + pinionDir · (R · cos γ_p)`. Coincident its start to the Apex; do not
   dimension its length. Its end is **point A**.
4. **Angular dimension** between the Pinion Gear Shaft Axis and the Driving Gear Shaft Axis, equal to
   the Shaft Angle. Place its text point **inside the Σ wedge** so it measures Σ and not 180−Σ
   (`[PB-ANGULAR-DIM]`) — on the interior bisector, `apex + normalize(pinionDir + drivingDir) · (PPD/4)`,
   with `drivingDir` the unit Apex→B direction. The angular dimension fixes the angle *magnitude*
   only; it does not pin which side the pinion lies on, and the text point does not prevent a frame
   flip. The pinion side is held by the seed in 3 together with the Apex 2 closure.
5. **A->Apex2**, the PPD/2 drop, from A, perpendicular to the Pinion Gear Shaft Axis
   (`addPerpendicular`), with an aligned distance dimension of `Pinion Gear Pitch Diameter / 2`.
   Coincident its start to A. ⚠️ **Apex 2 sits in the interior wedge *between* the two shaft axes, so
   this drop points toward the OTHER (Driving) shaft axis / point B, not "toward the anchor line".**
   Pick the perpendicular sense by the sign of its dot product with the A→B direction.

   **Naming convention used throughout: "A->Apex2" always means this perpendicular drop line, never
   the Apex->A shaft axis.** The two share point A but are different lines. The same holds for
   "B->Apex2" (the DPD/2 drop) against the Apex->B shaft axis.
6. **B->Apex2**, the DPD/2 drop, from B, perpendicular to the Driving Gear Shaft Axis, aligned
   distance dimension `Driving Gear Pitch Diameter / 2`, start coincident to B. ⚠️ **It must point
   toward the OTHER (Pinion) shaft axis / point A** — pick the sense by the sign of its dot product
   with the B→A direction. **Do not choose this sense against a "toward the anchor line" reference**
   (the −perp grow direction): the Driving Gear Shaft Axis is itself parallel to that direction, so
   the perpendicular's dot with it is ≈ 0, a degenerate test that silently selects an arbitrary and
   usually wrong side. Both drops must aim at the *same* interior-wedge point; if this one seeds Apex
   2 on the wrong side while the pinion's seeds it correctly, the closing coincidence makes the solver
   flip the whole figure to its mirror and A, C, D, G, H, K, M, N and A′ all land at negative X.
   Nothing in the build refuses that figure — the end-of-step gate is what catches it.
7. **Apex 2** — `addCoincident` on the end points of the two drops. At Σ = 90° the four points Apex,
   A, Apex 2, B form a rectangle; at other shaft angles a non-rectangular quadrilateral whose
   Apex→A and Apex→B lengths adjust so the two drops coincide.

   The along-shaft seeds `|Apex→A| = R · cos γ_p` and `|Apex→B| = R · cos γ_g` are what put the
   solver on the right branch for any Σ. Both cosines are positive for every Shaft Angle the range
   check admits, which is what the Maximum Shaft Angle exists to guarantee. Seeding A and B merely by
   a pitch diameter is wrong for Σ ≠ 90° and can send the solver to the wrong branch.
8. **Pitch Line**, Apex → Apex 2, both ends coincident to their points.
9. **Dedendum lines**, both from Apex 2, each perpendicular to the Pitch Line (`addPerpendicular`)
   with an aligned distance dimension of `Module * 1.25`, start coincident to Apex 2. Seed the two
   ends **by dot product against the shaft axes**, never by "towards / away from the anchor line".
   Let `u` be either unit perpendicular to the Pitch Line. The **pinion** dedendum direction is the
   `u` with `u · <unit Apex->A> > 0`, and the **driving** dedendum direction is its negation, which
   satisfies `(−u) · <unit Apex->B> > 0`. Those two dot products are exactly `sin γ_p` and
   `sin γ_g`, strictly positive for every admitted configuration. So

       C = Apex2 + 1.25 · Module · <pinion dedendum direction>      # Pinion Gear Dedendum end
       D = Apex2 + 1.25 · Module · <driving dedendum direction>     # Driving Gear Dedendum end

   ⚠️ **These two sites are where "C collapses onto D" lives, and each is held by its seed alone.**
   The perpendicular constrains direction and the dimension constrains magnitude; neither picks a
   side. Flip the pinion seed and C solves exactly onto D; flip the driving seed and D solves onto C.
   The collapsed figure inverts that gear — the toe ends up *outside* the heel, the revolved frustum
   is degenerate, and the conical end cut finds no cone face at the toe midpoint (`face dist = inf`).
10. **Root Axes**: a construction line Apex → D and another Apex → C, coincident at both ends. These
    are the Driving and Pinion Root Axis. They stay inside §2 and never enter the per-gear dict.
11. **A->E**, from A, collinear with the line Apex->A (`addCollinear`), start coincident to the end
    of Apex->A. No dimension. Seed

        E = A + <unit Apex->A> · (1.25 · Module · sin γ_p)

    E is the foot of the perpendicular dropped from C onto the Pinion Gear Shaft Axis, which is what
    `C->E ⊥ A->E` closes it on, so `|Apex->E| = R · cos γ_p + 1.25 · Module · sin γ_p`. Earlier
    revisions seeded this one Module long — the correct side but not the solved position, and a seed
    that is not the solved position cannot be gated.
12. **C->E**, from C to E, each end coincident to its point, with
    `addPerpendicular(A->E, C->E)`.
13. **B->F**, the driving twin of A->E: from B, collinear with Apex->B, start coincident, no
    dimension, seeded

        F = B + <unit Apex->B> · (1.25 · Module · sin γ_g)

    so `|Apex->F| = R · cos γ_g + 1.25 · Module · sin γ_g`.
14. **D->F**, from D to F, ends coincident, with `addPerpendicular(B->F, D->F)`.
15. **E->G**, from E, collinear with **line A->E** — the collinear names A->E, **never the Apex->A
    shaft axis further up the chain**, even though both describe the same infinite line
    (`[BEVEL-F-COLLINEAR-CHAIN]`, `[PB-COLLINEAR-CHAIN]`; naming the axis raises
    `VCS_SKETCH_OVER_CONSTRAINTS`). Start coincident to E, no dimension. Seed

        G = A + <unit Apex->A> · <resolved Pinion Gear Base Height>

    i.e. `|E->G| = <resolved Pinion Gear Base Height> − 1.25 · Module · sin γ_p`, strictly positive
    because the Minimum Base Height keeps every resolved base height above `1.25 · Module · sin γ`
    with a 1.05 margin.
16. **C->H**, from C, collinear with **line Apex2->C**, the Pinion Dedendum line C is the endpoint of
    (`[BEVEL-F-COLLINEAR-CHAIN]`). Start coincident to C, no dimension. Seed

        H = Apex2 + <unit Apex2->C> · (<resolved Pinion Gear Base Height> / sin γ_p)

    i.e. `|C->H| = <resolved Pinion Gear Base Height> / sin γ_p − 1.25 · Module`, positive by the
    same Minimum Base Height. ⚠️ **These two seeds are what pick the side of the pinion base-height
    offset dimension below, which is unsigned.** Flip them and G sits one base height on the Apex
    side of A instead of beyond it, H follows, and the pinion's heel end folds back inside the figure.
17. **G->H**, connecting G and H, ends coincident. **In Fusion, also
    `addPerpendicular(E->G, G->H)`.**

    ⚠️ **That perpendicular is required in Fusion and must be omitted in the proof harness, and the
    reason is a difference between the two engines rather than a choice.** `addOffsetDimension` in
    Fusion is a distance dimension whose documentation requires the second entity to be "a line that
    is parallel to the first", and it controls only the perpendicular distance — so the parallelism
    has to exist before the pinion base-height offset can be applied at all, and this perpendicular
    supplies it: E->G runs along the pinion shaft, so making G->H perpendicular to it makes G->H
    parallel to the A->Apex2 drop. Perpendicular plus offset is two equations for two freedoms and
    nothing is redundant. The proof harness's offset constraint is a different shape — it emits two
    residual rows holding both endpoints of the target line at the same signed perpendicular distance
    from the source, so it carries the parallelism itself, and adding this perpendicular there is a
    third row for the same two freedoms: measured, the lattice comes back overconstrained at DOF 0
    with 2 redundant constraints, and the engine names the two base-height offsets as the redundant
    pair. The proof leaves it out and says so (`[PB-NO-OVERCONSTRAIN]`).
18. **F->I**, from F, collinear with **line B->F** (never Apex->B), start coincident to F, no
    dimension. Seed

        I = B + <unit Apex->B> · <resolved Driving Gear Base Height>

    i.e. `|F->I| = <resolved Driving Gear Base Height> − 1.25 · Module · sin γ_g`.
19. **D->J**, from D, collinear with **line Apex2->D**, start coincident to D, no dimension. Seed

        J = Apex2 + <unit Apex2->D> · (<resolved Driving Gear Base Height> / sin γ_g)

    i.e. `|D->J| = <resolved Driving Gear Base Height> / sin γ_g − 1.25 · Module`. ⚠️ **The driving
    pair's seeds carry the whole figure**, because "Constrain Point I with center point" below hangs
    everything off I: flip the driving base-height offset's side and the entire lattice drops by twice
    the resolved Driving Gear Base Height, gear and pinion together, with every relative length still
    correct — which is why nothing downstream refuses it.
20. **I->J**, connecting I and J, ends coincident. **In Fusion, also
    `addPerpendicular(F->I, I->J)`** — the driving-side twin of 17, required in Fusion and omitted in
    the proof for the reason 17 gives in full.
21. **Driving base-height offset dimension**, between the **B->Apex2 perpendicular drop line** (the
    DPD/2 drop, **not** the Apex->B shaft axis) and **I->J**. I->J is already parallel to the drop by
    construction (I->J ⊥ F->I, which runs along the driving shaft), so add **no** extra parallel
    constraint (`[PB-OFFSET-DIM]`). Set its value to the **resolved** Driving Gear Base Height — the
    user's value if non-zero, otherwise `Module * Driving Gear Teeth Number / 8`, in either case
    after the driving Maximum Base Height has been applied. `addOffsetDimension` is unsigned and does
    not pick which side of the drop I->J lands on; the I and J seeds are the only thing that does.
22. **Pinion base-height offset dimension**, between the **A->Apex2 perpendicular drop line** (the
    PPD/2 drop, not the Apex->A shaft axis) and **G->H** — already parallel by construction
    (G->H ⊥ E->G), so again no parallel constraint. Its value is the **resolved** Pinion Gear Base
    Height: the user's value if non-zero, otherwise the **RESOLVED** Driving Gear Base Height
    `* (Pinion Gear Teeth Number / Driving Gear Teeth Number)`, then passed through the **pinion's
    own** Maximum and Minimum Base Height. "Resolved" means the value the driving offset above
    actually used, after the driving fallback **and** after the driving cap — never the raw driving
    input. The two gears have different pitch cone angles whenever the tooth counts differ, so the
    driving cap does not imply the pinion's.
23. **A'->G**, the hexagon's shaft-axis edge, ends coincident. It starts at the front face's foot
    **A′**, not at A; the two coincide at Toe Extension 0. **This line is what creates A′** — nothing
    above it does — so draw it with its start seeded at

        A' = Apex + <unit Apex->A> · <the along-shaft coordinate of N>

    the foot of the perpendicular from N onto the pinion shaft axis. The front face N->A′ in 30
    pins A′ to that axis; until then A′ is a free endpoint sitting at its seed. Draw the line **here**
    rather than after the front face, so the hexagon's edges are created in the walk order
    `A' -> G -> H -> C -> M -> N` that S12's first-edge rule depends on.
24. **Constrain Point I with the projected center point** — `addCoincident(I, projectedCenter)`.
    This is what hangs the lattice off the anchor.
25. **G->K**, a construction line away from the Apex starting at G, extending along Apex->A, ending
    at **K**. **Pin K with two point-on-line coincident constraints** —
    `addCoincident(K, line Apex->A)` and `addCoincident(K, the Pinion Dedendum line Apex2->C extended)`
    — rather than `addCollinear` on the connecting lines. By the time K is added, G and C are already
    fixed, so a collinear here over-constrains the sketch and Fusion errors; the two point-on-line
    coincidents locate K exactly, at the intersection of the two lines, without over-constraining
    (`[BEVEL-F-COLLINEAR-CHAIN]`). Then draw the reference line **C->K**.
26. **Tooth-center point K′ (Tooth Spacing offset).** The §3 tooth is centered at K′, which is K
    shifted outward along the dedendum line by **Tooth Spacing**, *away from the lower corner C*.

    **When Tooth Spacing is 0 (the default), build nothing here — set K′ ≡ K and reuse the C->K
    reference line.** A zero-length dimensioned line is degenerate, and one segment gets one line
    (`[BEVEL-F-LINE-ONCE]`).

    When Tooth Spacing > 0: draw a construction line starting at K, far end seeded at

        K' = Apex2 + <unit Apex2->C> · (<the pinion's virtual pitch radius> + Tooth Spacing)

    **"Virtual pitch radius" here is the exact back-cone radius `(Pinion Gear Pitch Diameter / 2) / cos γ_p`
    that S09 defines, never a radius rebuilt from a tooth count.** That is what `|Apex2 -> K|`
    measures: the dedendum line Apex2->C is perpendicular to the Pitch Line, which meets the pinion
    shaft axis at γ_p, so walking `r_p / cos γ_p` along it from Apex 2 lands exactly on the axis, at
    K. Reading the term as a rounded count times half a Module puts the seed 0.4203 mm short on the
    shipped default geometry — 31 teeth, Module 1, Σ = 90° — which is 420 times the
    `[BEVEL-F-SEED-HELD]` tolerance.

    Pin the far end the same way K is pinned: `addCoincident(start, K)` and
    `addCoincident(K', the Pinion Dedendum line Apex2->C extended)`, then add an **aligned length
    dimension on this line = Tooth Spacing**. Do not use `addCollinear`, for the same over-constraint
    reason as K. ⚠️ **That length dimension is unsigned, so the point-on-line pin plus the length
    admit K′ one Tooth Spacing on the C side of K just as readily — the two candidates sit
    `2 × Tooth Spacing` apart — and this seed is the only thing that rules the wrong one out.** A
    flipped K′ tightens the mesh by the clearance the input asked to add and builds a gear that looks
    right.

    Build it **here, inside this sketch, before the end-of-step full-constraint gate**, so the gate
    covers it. Then draw the tooth-center reference line **C->K′** for S09 to use in place of C->K.
    Only the tooth's center moves; the virtual tooth number and drawn tooth size are unchanged.
27. **Maximum Face Width — resolve and apply it here.** All of A, B, C, D, H, J now exist and are
    solved. It is `0.95 *` the smaller of
    - the perpendicular distance from point A to the line through C and H (the Pinion Gear Dedendum
      line, i.e. Apex2->C extended), and
    - the perpendicular distance from point B to the line through D and J (the Driving Gear Dedendum
      line, i.e. Apex2->D extended).

    **Compute both distances from the points' SOLVED sketch geometry** — `pointA.geometry`,
    `pointB.geometry`, `pointC.geometry`, `pointD.geometry`, `pointH.geometry`, `pointJ.geometry` —
    **not from the pre-solve seed coordinates** (`[PB-SOLVED-GEOMETRY]`). By now the network has
    located all six, so `.geometry` is exact; seeds diverge substantially for asymmetric tooth counts
    or non-90° shaft angles, making a seed-based bound too loose on the binding side, and the toe
    still crosses the axis.

    Apply it: cap the auto default `Cone Distance / 6` to it, and **reject** a user Face Width that
    exceeds it with a message stating the maximum. The pinion is normally the binding side (its
    smaller pitch radius gives the smaller distance), but compute both and take the minimum, because
    either gear can carry the smaller tooth count. At Σ = 90° this limit equals
    `0.95 * min(DPD, PPD)² / (2 * Cone Distance)` — the **smaller** pitch diameter and never the
    pinion's by name. Written with the pinion's diameter it is wrong whenever the driving gear
    carries the smaller count: on a Driving 17 / Pinion 31 pair at Module 1 the real bound is
    3.883 mm and the pinion form gives 13.591, so the naive `Cone Distance / 6` default exceeds it
    and the gear fails to generate for any ratio above roughly √2.

    The frustum profile — the hexagon A′, G, H, C, M, N built here and revolved in S13 — is revolved
    about the shaft axis, so a profile that has crossed that axis self-intersects the axis of
    revolution and Fusion aborts the revolve with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`). The `0.95`
    factor keeps N clearly off A, since a near-coincident N ≈ A degenerates the toe edge before it
    strictly crosses.
28. **Root Length and the Maximum Bore Diameter — resolve and apply them here too.** With the Face
    Width resolved, the Root Length follows:

        |Apex->Ded| = sqrt(R² + (1.25 · Module)²)
        Root Length at Toe Extension 0 = Face Width · |Apex->Ded| / R
        γ_root = γ − atan(1.25 · Module / R)
        Toe Radius Ceiling (per gear) = (r − 1.25 · Module · cos γ) · (1 − Face Width / R)
        Toe Limit (per gear) = sqrt(R² + (1.25 · Module)²) − Toe Radius / sin γ_root
        Root Length = RL0 + (Toe Extension / 100) · 0.99 · (min(Toe Limit_p, Toe Limit_g) − RL0)

    **Toe Extension 100 stops at 0.99 of the way** from the Toe Extension 0 root length to the
    smaller of the two gears' Toe Limits, not at the Toe Limit itself. The smaller limit wins because
    the pair shares one root length. The 0.99 is there because *at* the limit the toe face has zero
    length, so the revolved body carries **no cone at its toe end** and S22's toe cut — which must
    split or the build fails — has no `ConeSurfaceType` face to find. The last percent is worth well
    under a tenth of a millimetre of root length on every case in the proof's table. Do not drop it.

    **Reject a Toe Extension above 0 when a defaulted Toe Radius leaves no room.** On a driving gear
    with a large pitch cone angle the inner toe corner already sits at a *larger* radius than the
    outer one — the toe dish leans toward the heel rather than away from it — so X falls behind the
    toe corner and the Toe Limit comes out below the Toe Extension 0 root length. Measured over gear
    ratio against Shaft Angle it is a diagonal band that crosses 90° for every ratio from about 2.75
    up, and Module does not move its boundary. Reject with a message naming the gear and the Toe
    Radius Ceiling it needs to come below. Toe Extension 0 still resolves, so the gear stays
    buildable exactly as before. Do **not** silently substitute a smaller Toe Radius: that would
    change the toe end of a gear whose inputs asked for no change.

    Each gear's **resolved Toe Radius** is the user's value when non-zero, else the auto value
    `this gear's Pitch Radius − Face Width / sin γ`, the inner toe corner radius at Toe Extension 0,
    which is what makes Toe Extension 0 reproduce today's profile exactly. A user value must be
    **strictly below that gear's Toe Radius Ceiling**; reject it naming the ceiling.

    Then, per gear and only when Enable Bore is checked:

        r_heel = r − <this gear's RESOLVED Base Height> / tan γ
        r_toe  = (|Apex->Ded| − Root Length) · sin γ_root
        Maximum Bore Diameter = 2 · 0.95 · min(r_heel, r_toe)

    Cap an auto-calculated bore (`min(this gear's Pitch Diameter / 4, Maximum Bore Diameter)`) and
    **reject** a user value above the maximum with a message naming it. The bore is a through cut on
    the shaft axis, so past the heel term it takes the **entire flat back face** and past the toe
    term it takes the whole toe dish and bites into the root cone — either way the revolved frustum
    comes out of S27 with no end face on that side. The flat **front** face is deliberately not
    protected: its radius is the Toe Radius, which is not on the body's outer envelope, so a bore
    wider than it only exits through the toe cone and the frustum stays whole.

    This is the only step at which the whole bore bound can resolve, because its toe term needs the
    Root Length. Do this for both gears here, before either body is revolved, and skip it when Enable
    Bore is unchecked.
29. **M->N**, the pinion toe line. **Seed BOTH ends at their closed-form solved positions, not near
    them** (`[PB-SEED-NEAR]`). Seed M on `Apex->C` at the fraction `1 - <Root Length> / |Apex->C|`
    from the Apex. Then seed N by sliding from that M seed along the `C->H` direction by exactly

        (<M seed's perpendicular distance from the Pinion Gear Shaft Axis> - <resolved Pinion Gear Toe Radius>) / cos γ_p

    ⚠️ **A seed that merely lands somewhere plausible is not enough, and a wrong one builds the wrong
    gear rather than failing to converge.** N's position is fixed by the toe line together with a
    LENGTH dimension on the front face, and a length is unsigned: the toe line meets the Toe Radius
    on BOTH sides of the shaft axis, so the solver takes whichever side the seed starts on. Seeded
    below the axis it converges happily onto the mirror, N comes out on the far side, and the
    revolved hexagon crosses its own axis of revolution — Fusion then aborts the revolve with
    `ASM_WIRE_X_AXIS`, pointing at the revolve rather than at the seed that caused it.

    **Two earlier seeding rules are known to do exactly that; do not reinstate either**: sliding from
    the M seed by the **Root Length**, and sliding by the **distance from the M seed to A**. Both
    were written for the scheme that pinned N to the `A->Apex2` drop. Measured on the shipped default
    pair — module 1, 31/31, Σ = 90°, Toe Extension 50% — the Root Length slide puts the N seed at a
    perpendicular distance of **−0.27 mm** from the shaft axis, past it, against a solved N at
    **+5.17 mm**, and Fusion refuses the revolve. The slide above puts it at 5.17 mm exactly. Do not
    seed M/N just `Face Width` away from C/H either — that starts N near H, far from its constraint
    target.

    Then apply **exactly these three constraints**:
    - `addCoincident(M, Pinion Root Axis)` — M lies on the Apex->C root axis;
    - `addParallel(M->N, C->H)` — the toe line is parallel to C->H;
    - `addOffsetDimension(C->H, M->N, textPoint)` with
      `.parameter.value = <Root Length · R / |Apex->C|>` — the Root Length re-measured perpendicular
      to the pitch line, because an offset dimension controls a perpendicular distance. At Toe
      Extension 0 that value is exactly the resolved Face Width, which is what this dimension has
      always been. Place the `textPoint` in the gap between C->H and M->N on the Apex side, e.g.
      `(M_seed + C)/2` (`[PB-OFFSET-DIM]`).

    ⚠️ **The toe's side relative to the heel (`toe→Apex < heel→Apex`) is held by the M seed and by
    nothing else.** `addOffsetDimension` is unsigned, so a correctly built frame still admits M->N
    one root length on the *far* side of C->H, where the toe lands outside the heel and the revolved
    frustum is degenerate. The text point does not control it either.

    Let the beginning of this line be **M** and the end be **N**. Then draw the reference line
    **M->C**.
30. **The front face N->A′, which is what holds N.** ⚠️ **N is NOT pinned to line A->Apex2.** It
    rides the **resolved Pinion Gear Toe Radius** instead:
    - draw a line from N to **A′**, the point created in 23, seeding A′ at N's station on the shaft
      axis;
    - `addCoincident(A', line Apex->A)` — A′ lies on the **Apex->A shaft axis**; it is a *foot*, not
      a corner, and the only toe-end point that touches that axis;
    - `addPerpendicular(N->A', line Apex->A)` — the front face stands square to the shaft, so the
      revolve sweeps it into a flat annulus;
    - an aligned distance dimension on the whole line `N->A'` equal to the **resolved Pinion Gear Toe
      Radius**.

    ⚠️ **Pinning N itself to the Apex->A shaft axis remains forbidden** — that would put N on the axis
    of revolution, and the later conical split fails with `ASM_API_FAILED` for asymmetric tooth counts
    even though the symmetric 45° case happens to survive. A′ sits on the axis; N never does, because
    the Toe Radius is strictly positive. Those three rows plus the offset in 29 and
    `addCoincident(M, Pinion Root Axis)` fully constrain M, N and A′ — six freedoms, six constraints.

    **A′ replaces A as the hexagon's first vertex.** At Toe Extension 0 with a defaulted Toe Radius
    the two coincide exactly, so nothing moves; a positive Toe Extension walks A′ along the shaft axis
    toward the Apex and the shaft edge grows by that much.
31. **I->L**, a construction line away from the Apex starting at I, extending along Apex->B, ending
    at **L**. **Pin L the same way as K** — `addCoincident(L, line Apex->B)` and
    `addCoincident(L, the Driving Dedendum line Apex2->D extended)`; do not use `addCollinear`. Then
    draw the reference line **D->L**.
32. **Tooth-center point L′**, exactly as K′ with L for K, D for C and the Driving Dedendum line
    Apex2->D for the pinion's; the reference line for S09 is **D->L′**. Same single Tooth Spacing
    value, same gate, same reuse-the-existing-line rule at 0. Seed

        L' = Apex2 + <unit Apex2->D> · (<the driving gear's virtual pitch radius> + Tooth Spacing)

    taking "virtual pitch radius" as the same exact back-cone radius
    `(Driving Gear Pitch Diameter / 2) / cos γ_g`, which is `|Apex2 -> L|`. The flipped twin, one
    Tooth Spacing on the D side of L, is ruled out by that seed alone.
33. **O->P**, the driving toe line, the mirror of M->N. Seed O on `Apex->D` at the fraction
    `1 - <Root Length> / |Apex->D|`, then P slid from that O seed along `D->J` by
    `(<O seed's perpendicular distance from the Driving Gear Shaft Axis> - <resolved Driving Gear Toe Radius>) / cos γ_g`.
    The ⚠️ on 29 applies unchanged. Then:
    - `addCoincident(O, Driving Root Axis)` — O on the Apex->D root axis;
    - `addParallel(O->P, D->J)`;
    - `addOffsetDimension(D->J, O->P, textPoint)` with
      `.parameter.value = <Root Length · R / |Apex->D|>`, text point in the gap on the Apex side of
      D->J, e.g. `(O_seed + D)/2` (`[PB-OFFSET-DIM]`). Unsigned exactly as the pinion's is, so the O
      seed is what keeps the driving toe inside its heel.

    Beginning is **O**, end is **P**. Draw the reference line **O->D**.
34. **The driving front face P->B′**, exactly as the pinion's N->A′ with B for A, P for N and the
    **resolved Driving Gear Toe Radius**: the line `P->B'`, `addCoincident(B', line Apex->B)`,
    `addPerpendicular(P->B', line Apex->B)`, and an aligned distance dimension on `P->B'`. P is never
    pinned to the Apex->B shaft axis; only B′ touches it. Then draw the line **B'->I**.

### End of step

Gate the sketch: `if not sketch.isFullyConstrained: raise` naming it (`[BEVEL-F-FULL-CONSTRAINT]`).
Do **not** reach full constraint by dimensioning the driven lines.

Then **gate the solved figure against its own seeds** (`[BEVEL-F-SEED-HELD]`). Compare every named
point's solved `.geometry` against the closed-form position this step seeded it at, **in the sketch's
own 2-D frame with no world round-trip**, tolerance **0.001 mm** (1e-4 cm internal), and **raise**
naming the first point that has moved, with its solved position and its seeded one. Check the points
in creation order:

    Apex, B, A, Apex 2, C, D, E, F, G, H, I, J, K, K', M, N, A', L, L', O, P, B'

so the message names the earliest site that flipped rather than a downstream symptom. **The list is
22 points only when Tooth Spacing is above zero. At Tooth Spacing 0 — the default — K′ ≡ K and
L′ ≡ L are not built at all, so drop those two and compare 20**; comparing a K′ that was never
created is the one way this gate can raise on a correct figure. This gate is the only measure that
catches all 8192 figures. **Never treat the revolve's `ASM_WIRE_X_AXIS` as the tripwire**: several
flips build a valid-looking gear on the wrong side and reach no error at all.

Finally, build each gear's plain context dict — `pinionCtx` then `drivingCtx` — with the first ten of
the eighteen keys: `label` (`'Pinion'` / `'Driving'`), `teeth`, `gamma`, `pitchDiameter_cm`,
`toothCenterPoint` (K′ / L′), `toothCenterRefLine` (C->K′ / D->L′), `hexVertices` (the six vertices in
draw order, A′, G, H, C, M, N / B′, I, J, D, O, P), `toeEdgePoints` ((M, N) / (O, P)),
`heelEdgePoints` ((C, H) / (D, J) — the FIRST element is the dedendum corner C / D, **never** H / J),
and `boreDiameter_cm` (resolved **and** already bounded above). Use these key strings verbatim; never
rename one, split the dict, or wrap it in a class.

Calls: `component.sketches.add(planarEntity)`, `sketch.name`, `sketch.project(entity)`,
`sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`,
`adsk.core.Point3D.create(x, y, z)`, `sketchLine.isConstruction`,
`sketch.geometricConstraints.addCoincident(point, entity)`,
`sketch.geometricConstraints.addPerpendicular(lineOne, lineTwo)`,
`sketch.geometricConstraints.addParallel(lineOne, lineTwo)`,
`sketch.geometricConstraints.addCollinear(lineOne, lineTwo)`,
`sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, orientation, textPoint)`,
`adsk.fusion.DimensionOrientations.AlignedDimensionOrientation`,
`sketch.sketchDimensions.addAngularDimension(lineOne, lineTwo, textPoint)`,
`sketch.sketchDimensions.addOffsetDimension(line, entityTwo, textPoint)`,
`sketchDimension.parameter`, `sketchPoint.geometry`, `sketchLine.geometry`,
`sketchLine.startSketchPoint`, `sketchLine.endSketchPoint`,
`targetPlane.geometry`, `plane.normal`, `sketch.isFullyConstrained`.

<!-- check-step-calls: ignore addVertical addSymmetry isFixed addHorizontal -->
`addVertical`, `addSymmetry` and `SketchPoint.isFixed` are named only to forbid them in this sketch,
and `addHorizontal` is named only to say the direction lock belongs to S05's anchor line rather than
here.

Proof: `stepGearProfiles`.

<!-- proof-run: proofkit.RunParallel(gearProfileCases, stepGearProfiles) -->

**From:** `spec/bevelgear/instructions.md` L37–52, L58–99, L100–141, L142–174, L364–418, L491–517,
L584–712, L1142–1190; `spec/bevelgear/fusion.md` L69–117, L119–160, L162–197;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-SEED-NEAR]`, `[PB-SOLVED-GEOMETRY]`,
`[PB-OFFSET-DIM]`, `[PB-ANGULAR-DIM]`, `[PB-COLLINEAR-CHAIN]`, `[PB-NO-OVERCONSTRAIN]`,
`[PB-SHARE-XOR-COINCIDENT]`, `[PB-FULL-CONSTRAINT]`, `[PB-REVOLVE]`, `[PB-DIM-VALUE-SEMANTICS]`.

## S08 `[PROSE]` `{gearLabel} Plane` — the tooth plane

One timeline entry per gear: a construction plane named `Pinion Plane` / `Driving Plane`.

Create it through the **tooth-center reference line** — `C->K′` on the pinion, `D->L′` on the
driving gear — with `setByAngle`, perpendicular to the Gear Profiles sketch plane. Use the framework
helper `plane_by_angle(designComponent, toothCenterRefLine, gearProfilesPlane, 90)` from `.solids`
rather than re-implementing it. Pass the **sketch line directly**; never wrap it in
`adsk.fusion.Path.create` first (`[PB-CONSTRUCTION-PLANES]`).

Stash the plane in this gear's dict as `toothPlane`. `_transformToothBody` reads it back as
`parentToothPlane`, and §3a step E offsets **it** to make the slice planes.

Calls: `plane_by_angle(component, line, refPlane, angleDeg)`, `constructionPlane.name`.

<!-- check-step-calls: ignore Path.create setByAngle -->
`Path.create` is named only to forbid it, and `setByAngle` is named as the mechanism the framework
helper uses rather than as a call this step makes directly.

`[PROSE]` for the reason S06 gives: a construction plane is a frame. The proof does build the
back-cone plane's own consequence — the tooth section lies at the apex's perpendicular distance
`sK · cos γ` rather than at `sK` — and `proof/bevelgear/solids_test.go` records that the tilt itself
is not built.

**From:** `spec/bevelgear/instructions.md` L364–395, L713–721, L741;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-CONSTRUCTION-PLANES]`.

## S09 `[GO]` `{gearLabel} Tooth` sketch — the virtual spur tooth

One timeline entry per gear: a sketch named `Pinion Tooth` / `Driving Tooth`, on that gear's
`{gearLabel} Plane`, with the tooth-center point as its center.

### The virtual tooth number

Compute this gear's virtual (back-cone, Tredgold) tooth number from the closed form, **not** by
measuring `Apex2->K′` / `Apex2->L′`:

    virtual pitch radius = (this gear's Pitch Diameter / 2) / cos γ
    virtual tooth number = 2 · virtualPitchRadius / Module          # equivalently teeth / cos γ

**It is a real number and is NEVER rounded** — not floored, not ceiled, not cast to an int. The
Tredgold construction puts the equivalent spur gear's pitch radius exactly at the back-cone distance
`r / cos γ` with this gear's own module, and `z_v = z / cos γ` is a real number in every published
form of it (NPTEL Machine Design II ch. 13 eq. 13.1–13.2; Osakue et al., *FME Transactions* 49(3),
2021, §2.2; the KHK gear technical reference eq. 11.6). Rounding it rebuilds every drawn circle from
the rounded count, which draws the tooth smaller than the back cone places it and shortens the
working addendum: on the shipped default — 31 teeth, Module 1, Shaft Angle 90°, γ = 45° — the exact
virtual pitch radius is 21.9203 mm, a floored count of 43 draws 21.5 mm, and the addendum the tooth
works over falls to 0.5797 mm against a nominal 1.0 module.

**The real count reaches the spur drawer only as an angular half-thickness.** The drawer reads
`ToothNumber` as a float and uses it in one place, `π / (2 · toothNumber)`, the angle it rotates the
flank to so the pitch crossing lands there. With `z_v = 2 · r_v / Module` that angle gives a tooth
thickness of `π · Module / 2` at the pitch circle, which is the standard tooth thickness. An integer
count drawn at the exact radius gives `π · r_v / round(z_v)` instead, which misses nominal by a
different amount on each member of an unequal pair, so the two teeth of one pair no longer carry the
same thickness.

**Root sink.** Draw the root circle one **root sink** `0.05 · 2.25 · Module` **inside** the dedendum
corner rather than at it. At the dedendum corner exactly, the tooth's root arc touches the gear
body's root cone only where the arc crosses the tooth's own centreline: the tooth is drawn on the
back-cone plane, so only a point on that centreline rides the cone its own polar radius names, and
the arc's two corners stand outside it — by 0.002 module on the default pair and 0.027 module on a
4/4 pair, the largest of any pair the spec admits. The sink pushes the whole arc inside, so the
Combine-Join meets the gear body across the root rather than along one line.

So the four circles the proxy is asked for are:

| circle | radius |
|---|---|
| pitch | `virtualPitchRadius` |
| base | `virtualPitchRadius · cos(20°)` |
| tip | `virtualPitchRadius + Module` |
| root | `virtualPitchRadius − 1.25 · Module − rootSink` |

**Units — pin the cm→mm conversion.** The stashed pitch diameters are internal **cm** while Module
is the raw **mm** value, so compute `virtualPitchRadius_mm = (pitchDia_cm · 10 / 2) / cos(γ)` — the
`· 10` converts cm to mm — and then `virtualTeeth = 2 · virtualPitchRadius_mm / Module`. Skipping the
×10 makes the virtual tooth count about ten times off.

### Drawing it

Borrow the spur tooth generator, once per gear:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth, rootSink_mm=rootSink)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))   # the 180 degree tooth rotation IS the draw() angle
```

`VirtualSpurProxy` comes from the framework (`from .spurproxy import VirtualSpurProxy`); bevel
defines no local proxy or value-wrapper class. It precomputes, in internal cm, exactly the parameter
keys the spur drawer reads, with its own defaults matching bevel — pressure angle 20°, which is not
a bevel dialog input, and `InvoluteSteps` 15 — and returns each wrapped in a `.value` carrier.
`virtualTeeth` is a **real** number: the proxy computes the pitch diameter as
`virtualTeeth · module_mm`, so passing the exact `2 · r_v / Module` is what makes the drawn pitch
circle reach the back cone. `rootSink_mm` shortens the root diameter by twice its value and leaves
pitch, base and tip alone.

The **180° rotation is delivered through the `draw()` angle argument** — the spur generator rotates
the whole tooth by `angle` — never as a post-hoc Move or sketch rotation. This relies on spur's
radial flank-to-root pinning so the connecting lines rotate with the tooth.

**After `draw(...)` returns, read `proxy._lastToothEmbedded` back and thread it forward.** The spur
generator decides during `draw()` whether the tooth is *embedded* — tip, root and flanks meeting
with no connecting lines — and records it with `self.parent._lastToothEmbedded = <bool>`; the
framework proxy pre-initialises the slot to absorb that write. This flag is **not optional
bookkeeping**: it is the deterministic selector for the tooth loop's line count in S14,
`wantLines = 0 if embedded else 2`. Stash it in this gear's dict as `toothEmbedded`, alongside
`toothSketch`.

**Do not hard-gate this sketch.** Log if `not toothSketch.isFullyConstrained`, never raise: the
tooth-profile sketches are exempt from the full-constraint gate (`[BEVEL-F-FULL-CONSTRAINT]`), and
the exemption is about the four circle labels the drawer places with along-path sketch text, which
hold a DOF (`[PB-TEXT-HOLDS-DOF]`). The exemption covers the labels and nothing else.

Calls: `component.sketches.add(planarEntity)`, `sketch.name`,
`VirtualSpurProxy(module_mm, virtualTeeth, rootSink_mm)`,
`SpurGearInvoluteToothDesignGenerator(sketch, parent)`, `drawer.draw(anchorPoint, angle)`,
`math.radians(180)`, `sketch.isFullyConstrained`.

<!-- check-step-calls: ignore round floor ceil int -->
`round`, `floor`, `ceil` and `int` are named only to forbid applying any of them to the virtual
tooth number.

Proof: `stepToothProfile`.

<!-- proof-run: proofkit.RunParallel(perGearCases, stepToothProfile) -->

**From:** `spec/bevelgear/instructions.md` L364–395, L476–490, L530–565, L713–740, L743;
`spec/bevelgear/fusion.md` L31–58; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-PRECOMPUTED-MODE]`, `[PB-TEXT-HOLDS-DOF]`, `[PB-PROFILE-MATCH]`, `[PB-SHARE-XOR-COINCIDENT]`.

## S10 `[PROSE]` `{gearLabel} Tooth Axis`

One timeline entry per gear: a construction axis named `Pinion Tooth Axis` / `Driving Tooth Axis`,
through the tooth-center point and normal to the plane the tooth profile was drawn on.

Build it with `setByTwoPlanes` (`[PB-CONSTRUCTION-AXES]`); `setByPerpendicularAtPoint` would need a
`BRepFace` this build does not have. The two planes are the **Gear Profiles plane** and a **helper
plane built `setByDistanceOnPath(<tooth-center reference line>, 1.0)`** — perpendicular to that line
at its far end, the tooth-center point. Their intersection is the line through the tooth center
normal to the tooth plane. Pass the sketch line directly to `setByDistanceOnPath`, never wrapped in
`Path.create` (`[PB-CONSTRUCTION-PLANES]`).

Creating this axis in the never-activated Design component is proven to work —
`constructionAxes.add` via `setByTwoPlanes` does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here — so
keep the axis.

Stash it as `toothAxis`. **No step reads this key back**; Cleanup hides the axis by entity kind
rather than through the dict. It is the one dict entry with no reader, and it is listed so a regen
that stashes the axis is not read as having invented a key.

Calls: `component.constructionAxes.createInput()`,
`axisInput.setByTwoPlanes(planarEntityOne, planarEntityTwo)`,
`component.constructionAxes.add(input)`, `component.constructionPlanes.createInput()`,
`planeInput.setByDistanceOnPath(pathEntity, distance)`,
`component.constructionPlanes.add(input)`, `adsk.core.ValueInput.createByReal(1.0)`,
`constructionAxis.name`.

<!-- check-step-calls: ignore setByPerpendicularAtPoint Path.create -->
Both are named only to forbid them.

`[PROSE]` for the reason S06 gives.

**From:** `spec/bevelgear/instructions.md` L364–395, L713–716, L745;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-CONSTRUCTION-AXES]`,
`[PB-CONSTRUCTION-PLANES]`, `[PB-CONSTRUCTION-NEEDS-ACTIVE]`.

## S11 `[PROSE]` `{gearLabel} Gear` component

One timeline entry per gear: a new component named `Pinion Gear` / `Driving Gear`, created as a
child of the **Bevel Gear** component — the same component that owns Design, **not** the user's
Parent Component. This intentionally overrides the looser "child of Parent Component" phrasing so
the pair nests cleanly inside Bevel Gear.

Stash the occurrence as `gearOccurrence`. The finished bodies land here at the end of S29.

Fusion rejects cross-sibling sketch and `project` calls even when the target is activated or the
entities are wrapped in `createForAssemblyContext` proxies (`[PB-NO-CROSS-SIBLING]`), so every
feature operation runs in the Design component and the finished bodies are `moveToComponent`'d here
at the end. The visible end state is identical.

Calls: `bevelComponent.occurrences.addNewComponent(transform)`, `adsk.core.Matrix3D.create()`,
`occurrence.component`, `component.name`.

`[PROSE]` for the reason S04 gives.

**From:** `spec/bevelgear/instructions.md` L364–395, L857–871;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-OCCURRENCE-TREE]`, `[PB-NO-CROSS-SIBLING]`.

## S12 `[GO]` `{gearLabel} Profile` sketch — the hexagon

One timeline entry per gear: a **fresh sketch on the axial (Gear Profiles) plane**, named
`Pinion Profile` / `Driving Profile`. **One profile sketch per gear**, so `sketch.profiles` holds
exactly this one hexagon loop. Do not draw both gears' hexagons in the shared Gear Profiles sketch;
that would leave two identically shaped loops to disambiguate.

Build the hexagon on **fixed** vertices per the `[PB-PROJECT-NOT-FIXED]` recreate-share-fix recipe:

1. Recreate the six §2 vertices as new points at their exact world-mapped positions —
   `sketch.sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))` for each — valid because
   §2 is fully constrained by now.
2. Draw the closed hexagon as six `SketchLine`s **sharing** those points, in the draw order below.
3. Fix the lines and their endpoints **after** the lines exist, never before. Setting `isFixed` on a
   bare point before it is consumed as a line endpoint does **not** leave the sketch fully
   constrained.

Then gate it: `if not sketch.isFullyConstrained: raise` naming the sketch
(`[BEVEL-F-FULL-CONSTRAINT]`).

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A' -> G -> H -> C -> M -> N -> A' | B' -> I -> J -> D -> O -> P -> B' |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A'->G | B'->I |
| toe cut edge | M->N | O->P |
| heel cut edge | C->H | D->J |
| teeth / bore / pitch-diameter inputs | Pinion Gear … | Driving Gear … |
| §2 shaft construction line (NOT usable as the axis) | Apex->A | Apex->B |

**The hexagon's FIRST edge is the gear's shaft axis** for the revolve, the pattern, the bore plane
and the meshing-rotation axis, so it must be fixed well enough to carry a trustworthy world
position: fixed endpoints give that edge a well-defined `worldGeometry`
(`[PB-WORLDGEO-CONSTRAINED]`). A free edge resolves against a default or world-XY frame and silently
moves the body onto world XY — observed on the driving gear, where the pinion looked fine only
because it never read the edge's `worldGeometry`.

**The shaft axis used by every body operation below is this profile sketch's first edge, NOT the §2
`Apex->A` / `Apex->B` construction line.** The edge is collinear with the shaft axis but lives in the
*same* sketch as the profile, which is what Fusion's revolve, pattern and path builders accept;
reusing the §2 construction line, which lives in a different sketch, fails or misbuilds.

Stash `profileSketch` and `shaftAxisEdge` in this gear's dict.

Calls: `component.sketches.add(planarEntity)`, `sketch.name`,
`sketch.sketchPoints.add(point)`, `sketch.modelToSketchSpace(modelCoordinate)`,
`sketchPoint.worldGeometry`, `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`,
`sketchLine.startSketchPoint`, `sketchLine.endSketchPoint`, `sketchPoint.isFixed`,
`sketch.isFullyConstrained`, `sketch.profiles`.

<!-- check-step-calls: ignore project project2 -->
Both are named only to say this sketch does **not** project the §2 points: it recreates them, which
is what `[PB-PROJECT-NOT-FIXED]` requires.

Proof: `stepProfileSketch`.

<!-- proof-run: proofkit.RunParallel(perGearCases, stepProfileSketch) -->

**From:** `spec/bevelgear/instructions.md` L364–395, L491–517, L656–657, L857–876;
`spec/bevelgear/fusion.md` L21–30; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-PROJECT-NOT-FIXED]`, `[PB-WORLDGEO-CONSTRAINED]`, `[PB-SPACE-METHODS]`,
`[PB-FULL-CONSTRAINT]`, `[PB-SINGLE-PROFILE]`.

## S13 `[GO]` Revolve the Gear Body

One timeline entry per gear. This sketch holds exactly one hexagon loop, so take its single profile
with `sketch.profiles.item(0)` and do not filter (`[PB-SINGLE-PROFILE]`): a curve-type filter has
spuriously rejected a valid all-line loop and made the revolve fail with "could not find profile".

Revolve that profile around the **shaft-axis edge** — the hexagon's first edge, not the §2
construction line — a full turn. Let the result be this gear's **Gear Body**, the frustum, and stash
it as `gearBody`.

Because the toe edge is one edge of the revolved profile, the body already carries the conical face
that edge sweeps; that face is reused as the cutting tool in S22, as is the heel edge's cone.

**The profile must not cross the axis of revolution.** If it does, Fusion aborts with
`RuntimeError … ASM_WIRE_X_AXIS … the profile crosses the axis of revolution` (`[PB-REVOLVE]`). That
is what the Maximum Face Width of S07 step 27 and the Maximum Base Height of S03 exist to prevent,
and it is why both caps are applied before anything is revolved.

Calls: `sketch.profiles`, `profiles.item(0)`,
`component.features.revolveFeatures.createInput(profile, axis, operation)`,
`adsk.fusion.FeatureOperations.NewBodyFeatureOperation`,
`revolveInput.setAngleExtent(isSymmetric, angle)`,
`adsk.core.ValueInput.createByString('360 deg')`,
`component.features.revolveFeatures.add(input)`.

Proof: `stepRevolveGearBody`, asserted by `assertRevolveGearBody`.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepRevolveGearBody, assertRevolveGearBody) -->

**From:** `spec/bevelgear/instructions.md` L152–159, L857–877, L925–959;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-SINGLE-PROFILE]`, `[PB-REVOLVE]`.

## S14 `[GO]` Loft the Tooth Body

One timeline entry per gear. Loft the **§2 Apex sketch point** — `centerToApex.endSketchPoint` from
the Gear Profiles sketch, the degenerate point section — to this gear's §3 Tooth profile. Let the
result be the **Tooth Body**.

Use the §2 Apex **SketchPoint** directly. Do **not** create a construction point for it: construction
geometry needs an active component and the Design component is never active
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). A `SketchPoint` is a valid loft section
(`[PB-LOFT]`), and the order of `loftSections.add(...)` is the loft order.

**Select the tooth cross-section loop with
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)`** — the framework
helper from `.utilities` — where the line count is **determined by the `toothEmbedded` flag S09
stashed, not guessed and not accepted as either**: `wantLines = 0 if toothEmbedded else 2`.
⚠️ Do **not** accept "0 **or** 2 lines". For a given gear only one of those is the real tooth; an
unrelated loop — an inter-tooth or annular region between the drawer's circles — can also carry
2 NURBS and 2 arcs with the *other* line count, and selecting it makes this loft fail with
`RuntimeError … ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`, because the impostor loop cannot form a loft
tool body. Embedded means tip, root and flanks meet with no connecting lines (4 curves);
non-embedded means 2 connecting lines (6 curves).

Calls: `find_profile_by_curve_counts(sketch, nurbs, arcs, lines)`,
`component.features.loftFeatures.createInput(operation)`,
`adsk.fusion.FeatureOperations.NewBodyFeatureOperation`,
`loftInput.loftSections`, `loftSections.add(entity)`,
`component.features.loftFeatures.add(input)`, `sketchLine.endSketchPoint`.

<!-- check-step-calls: ignore constructionPoints.add -->
Named only to forbid it: the apex loft section is the §2 sketch point, never a construction point.

Proof: `stepToothLoft`, asserted by `assertToothLoft`.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepToothLoft, assertToothLoft) -->

**From:** `spec/bevelgear/instructions.md` L419–490, L878–879, L960–964;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-LOFT]`, `[PB-CONSTRUCTION-NEEDS-ACTIVE]`,
`[PB-PROFILE-MATCH]`, `[PB-EMPTY-RESULT]`.

## S15 `[PROSE]` `{gear} Cone Element` sketch and `{gear} Trace Plane` (ψ > 0 only)

Two timeline entries per gear, and **only when Mean Spiral Angle ψ > 0**. At ψ = 0 the tooth-body
hook returns immediately with `cut_conical_ends` and nothing here is built.

1. In a sketch on the **axial (Gear Profiles) plane**, named `{gear} Cone Element`, draw a
   construction line from the Apex to `Apex + R_heel · coneVec`.
2. Make the tangent plane as that axial plane rotated **90°** about the cone-element line:
   `plane_by_angle(designComponent, coneElementLine, axialPlane, 90)`. Name it `{gear} Trace Plane`.

`coneVec` is the dedendum (root) cone element `Apex->C` (pinion) / `Apex->D` (driving), realized as
`normalize(heelConeWorld − apexWorld)`, and `R_heel` is the heel edge midpoint's cone distance —
both defined in S17's frame section, which runs first in the hook.

**Coordinates.** The raw `apex` and cone-end `Point3D`s are passed **directly** into
`addByTwoPoints`, where they are consumed as **sketch-space** input — **no `modelToSketchSpace`
conversion is applied**, even though `adsk.fusion.Sketch` offers exactly that call and the points
really are model-space coordinates. This is deliberate. An unconverted cone-element line does place
the Trace Plane somewhere other than the true tangent plane, and that still reaches no feature,
because the only thing built on the Trace Plane is the inspection-only trace sketch of S16 and the
chain ends there. **If a later revision ever makes any feature consume the trace sketch or the Trace
Plane, this shortcut stops being safe and both sketches need `modelToSketchSpace` on every point.**

Both sketches are **exempt from the full-constraint gate** — they are transient construction the
build consumes and cleanup hides (`[BEVEL-F-FULL-CONSTRAINT]`).

Calls: `component.sketches.add(planarEntity)`, `sketch.name`,
`sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`, `sketchLine.isConstruction`,
`plane_by_angle(component, line, refPlane, angleDeg)`, `constructionPlane.name`.

<!-- check-step-calls: ignore modelToSketchSpace -->
Named only to say this step does **not** call it, which is the whole of the coordinate rule above.

`[PROSE]`: the Trace Plane is a frame, and the spec itself records that its placement reaches no
feature. `proof/bevelgear/sketches_test.go` builds the trace geometry the plane would carry, in the
flat tangent-plane frame, and records there that the plane's own placement is not reached.

**From:** `spec/bevelgear/instructions.md` L747–801; `spec/bevelgear/spiral-tooth-trace.md`
L32–65; `spec/bevelgear/fusion.md` L59–67; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-CONSTRUCTION-PLANES]`, `[PB-SPACE-METHODS]`.

## S16 `[GO]` `{gear} 2D Tooth Trace` sketch (ψ > 0 only)

One timeline entry per gear, only when ψ > 0: a sketch named `{gear} 2D Tooth Trace` on the
`{gear} Trace Plane`, carrying the genuine cutter arc.

### The cutter-arc geometry

Work in the tangent-plane 2-D frame with origin at the apex, **x = coneVec** (so a point's x is its
cone distance) and **y = v** (circumferential, `v = normalize(axisDir × coneVec)`). With `R_toe`,
`R_heel`, `R_mean = ½(R_toe + R_heel)` and `span = R_heel − R_toe` from S17's frame section:

    r_c = Cutter Radius if non-zero, else R_mean          # the auto default
    handSign = +1 for `Right` else −1, then NEGATED for the pinion
    Cx = R_mean − r_c · sin ψ
    Cy = handSign · r_c · cos ψ

⚠️ **The hand sign goes on the `cos`/`Cy` term, NOT the `sin`/`Cx` term.** This was a real bug.
Opposite hand mirrors the cutter centre **across the cone element (y = 0)**, which flips `Cy`.
Putting `handSign` on `Cx` mirrors about `x = R_mean` instead — a *different* curve that gives the
two gears **unequal twist**; for equal teeth the driving and pinion traces must come out as exact
mirror images.

The arc's endpoints are taken a hair **past** the face so the kept arc reaches cleanly past the
end-trims:

    R_lo = R_toe − 0.06 · span
    R_hi = R_heel + 0.06 · span
    toe2d  = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)
    heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)

`circle_intersect_nearest` is the framework helper from `.solids`: it intersects the apex circle of
radius R with the cutter circle of centre `(Cx, Cy)` and radius `r_c` and keeps the solution nearest
`(R_mean, 0)` — the branch the mean point sits on. A non-overlapping pair clamps to tangency.

### What the sketch carries

Map 2-D coordinates to world with the framework helper
`combine_point(apexWorld, px, coneVec, py, v)`, and pass the resulting `Point3D`s **directly** into
the sketch calls, with no `modelToSketchSpace` conversion, for the reason S15 gives in full.

- the **cutter circle** — centre at `tanW(Cx, Cy)`, radius `r_c` — `isConstruction`, with its centre
  pinned via `centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`) and a **diameter dimension of
  `2 · r_c`**, whose text point is off-centre and on the curve, e.g. `tanW(Cx + r_c, Cy)`
  (`[PB-RADIAL-DIM]`);
- the **trace arc** — a three-point arc through `tanW(toe2d)`, `tanW(R_mean, 0)` (the mean point on
  the cone element) and `tanW(heel2d)` — with its **centre coincident to the cutter circle's centre**
  and a **radius dimension of `r_c`**, so it is the genuine cutter circle and not a look-alike
  spline. Its radius dimension's text point is the mean point `tanW(R_mean, 0)` (`[PB-RADIAL-DIM]`).

⚠️ **`addByThreePoints` copies the arc's centre rather than sharing it**, so the coincidence to the
cutter circle's centre is required and is the one place `[PB-SHARE-XOR-COINCIDENT]`'s "share **or**
coincident" rule asks for a coincident on a point that was passed in. A copied centre is a free
point: on the bevel tooth-top arc a stranded centre gave a 0.5743 mm arc where 22.5 mm was intended,
on a sketch that raised no error.

This sketch is **deliberately left with free DOF** — the arc's endpoints are pinned by the three-point
construction, not by endpoint dimensions, and dimensioning them over-constrains the solve against the
cone-element plane. It is therefore **exempt from the full-constraint gate**
(`[BEVEL-F-FULL-CONSTRAINT]`); do not gate it.

Calls: `component.sketches.add(planarEntity)`, `sketch.name`,
`combine_point(base, a, e1, b, e2)`, `circle_intersect_nearest(R, Cx, Cy, r_c, refX, refY)`,
`sketch.sketchCurves.sketchCircles.addByCenterRadius(centerPoint, radius)`,
`sketch.sketchCurves.sketchArcs.addByThreePoints(startPoint, point, endPoint)`,
`sketchCircle.centerSketchPoint`, `sketchArc.centerSketchPoint`, `sketchPoint.isFixed`,
`sketchCurve.isConstruction`,
`sketch.geometricConstraints.addCoincident(point, entity)`,
`sketch.sketchDimensions.addDiameterDimension(entity, textPoint)`,
`sketch.sketchDimensions.addRadialDimension(entity, textPoint)`,
`adsk.core.Point3D.create(x, y, z)`.

Proof: `stepTraceSketch`.

<!-- proof-run: proofkit.RunParallel(traceCases, stepTraceSketch) -->

**From:** `spec/bevelgear/instructions.md` L176–180, L747–801; `spec/bevelgear/spiral-tooth-trace.md`
L18–29, L68–89, L92–148, L151–185, L220–252; `spec/bevelgear/fusion.md` L59–67;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-CIRCLE-CENTER]`, `[PB-RADIAL-DIM]`,
`[PB-SHARE-XOR-COINCIDENT]`, `[PB-SKETCHCURVES]`.

## S17 `[GO]` Slice the straight tooth into slabs (ψ > 0 only)

One timeline entry per gear, only when ψ > 0. This is the first geometric step of the spiral branch
of `_transformToothBody`, which **replaces** the straight tooth's two conical trims.

### The hook's gate and hand-off

`_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane, gearLabel,
teethNumber, gamma)`. Its **first line** is the gate
`if self._spiralAngle_rad <= 0: return cut_conical_ends(...)` — straight bevels are byte-for-byte
the prior behavior.

`_createGearBody` builds the four toe/heel world points **exactly** per this table. **Mislabeling
them silently inverts the spiral; this is the single biggest spiral-regen hazard.**

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

- `toeMid` = the world **midpoint of the TOE edge** — ½(M+N) pinion, ½(O+P) driving.
- `heelMid` = the world **midpoint of the HEEL edge** — ½(C+H) pinion, ½(D+J) driving.
- `toeConeWorld` = the toe edge's inner endpoint, **M** / **O**, which lies on the root cone element.
- `heelConeWorld` = the **dedendum corner**, **C** / **D**, the outer end of that same element.

⚠️ **Two scrambles to avoid**, both of which a fresh regen has made:
- Do **not** pass the two endpoints of a *single* edge as `toeMid` / `heelMid`. M and N both sit at
  the toe, so `span` collapses to about zero or goes negative and the spiral inverts. `toeMid` is the
  midpoint of the **toe** edge; `heelMid` is the midpoint of the **heel** edge — two different edges.
- `heelConeWorld` is the **dedendum corner C/D**, **never H/J**. H and J lie on the `Apex2→C` /
  `Apex2→D` dedendum line, one Module beyond C/D and **off** the root cone element; using them skews
  `coneVec` away from `Apex→C` / `Apex→D`.

### A. Gate and frame

- `axisDir` = the shaft axis direction, from the two **world** endpoints of `shaftAxisEdge` (the
  in-sketch profile edge A′→G / B′→I), normalized.
- `coneVec` = `normalize(heelConeWorld − apexWorld)`, the root cone element.
- `v` = `normalize(axisDir × coneVec)` — the **circumferential** direction.
- `tpNormal` = `normalize(coneVec × v)` — the tangent-plane normal. It completes the frame and
  **nothing consumes it**: step D removed the projection that once used it, so it is computed and
  left unread.
- `distAlong(p)` = `(p − apexWorld) · coneVec` — a point's cone distance.

⚠️ **The heel MUST be the OUTER end** so `coneVec` points outward and `span > 0`. Before building
`coneVec`, check the passed midpoints and **fix swapped toe/heel**: if
`apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid ↔ heelMid` **and**
`toeConeWorld ↔ heelConeWorld`, then build `coneVec = apex → heelConeWorld`. A negative `span`
silently inverts the entire spiral frame — the cutter-arc direction, the slice direction and the
per-segment twist all flip — and the gear comes out completely wrong with no error.

Then `R_toe = distAlong(toeMid)`, `R_heel = distAlong(heelMid)`, `R_mean = ½(R_toe + R_heel)` and
`span = R_heel − R_toe`, now positive. These are the only quantities the rest of the build needs.

### E. The slice

Split the uncut apex→heel `toothBody` into cross-section slabs by planes **parallel to the parent
transverse tooth plane** (`parentToothPlane`, the `{gearLabel} Plane` of S08), via a **fixed** scheme
of **exactly 8 planes** — the count is not user-configurable.

⚠️ **The slice planes are NOT perpendicular to the cone element.** The parent plane carries the
tooth-center line C->K′ / D->L′, which is the back-cone line and so perpendicular to the Pitch Line,
so **the parent plane's normal runs along the PITCH element** — while `coneVec` is the **ROOT**
element. The two differ by the dedendum angle

    δ_f = atan(1.25 · Module / R) = atan(2.5 · sin γ_p / N_p) = atan(2.5 · sin γ_g / N_g)

Module cancels, so δ_f depends only on the tooth counts and the Shaft Angle and is the **same for
both members** — 3.26° on the default 31/31 pair at Shaft Angle 90°, growing as the tooth counts
fall. The parallel family is what the build **requires**, not what it happens to use:
`slice_body_by_offset_planes` offsets the parent plane with `setByOffset`, which produces parallel
planes; the sign test below reads the parent plane's own normal, which is meaningful only for that
plane's own offsets; and the tooth is lofted from the Apex to the profile drawn in the parent plane,
so **the heel-most slab's heel face IS the parent plane** and a consistent family has to contain it.

⚠️ **A build that follows "perpendicular to the cone element" instead is wrong and silent.** It tilts
every cut face by δ_f and nothing in the pipeline fails: parallel planes cut a cone in similar
sections whatever their orientation, so the loft still reproduces the taper; the piece count, the
retry gate and the conical trims of S22 are all indifferent to slab orientation; the proof builds its
own slabs and never sees the module's plane; and the runtime gate only counts pieces. What moves is
the geometry — on the default pair at Module 4 a face corner lands `1.125 · Module · tan δ_f` =
0.26 mm along the cone from where the parallel cut puts it, and the step-G twist keyed across one
face mismatches by up to 0.0078 rad.

The **first cut plane is the parent plane offset toward the apex by `span/6`**. The offset **sign is
chosen per gear** so it moves toward the apex — the parent plane's normal points opposite ways for
the two gears, so pick `sign` by testing `(apex − planeOrigin) · normal`. The other seven step
further toward the apex in `span/6` increments:

    offsets = [sign · (k + 1) · span / 6 for k in 0…7]

**Where the eight land.** The first sits `span/6` inside the **HEEL** and none of them lies past it
— the parent plane is already the heel end, so there is no heel overshoot to give. The **sixth lands
at the toe**, and the **last two sit `span/6` and `2·span/6` PAST the toe**; those two segments are
what S22's toe cone trims away. (The first sits a hair more than `span/6` inside the heel, and the
sixth a fraction of a millimetre inside the toe, because `R_heel` and `R_toe` are read at the two
edge midpoints rather than on the root element.)

Split with `slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)` — the
framework helper, which splits piece-by-piece and keeps a piece whole when a plane misses it.

⚠️ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in **one
piece**, the offset sign was wrong or `parentToothPlane` sits outside the tooth's span — **retry the
whole cut once with the opposite sign**. If it is still one piece, **`raise` a clear self-diagnosing
error** naming the gear, the final piece count, `span` and the sign tried (`[PB-SELF-DIAGNOSING]`,
`[PB-EMPTY-RESULT]`). Do **not** return an unsliced single-piece result: S18 then drops that one
piece as the apex scrap, leaving `segments` empty, and the crown later crashes with
`ValueError: max() iterable argument is empty` far from the cause.

Calls: `slice_body_by_offset_planes(component, body, basePlane, offsets)`,
`constructionPlane.geometry`, `plane.normal`, `plane.origin`, `sketchLine.worldGeometry`,
`adsk.core.Vector3D.crossProduct(vector)`, `adsk.core.Point3D.vectorTo(point)`,
`adsk.core.Point3D.distanceTo(point)`.

<!-- check-step-calls: ignore setByOffset cut_conical_ends distAlong vectorTo -->
<!-- check-compile: ignore setByOffset cut_conical_ends distAlong vectorTo -->
Four mentions rather than requirements. `setByOffset` names the mechanism inside
`slice_body_by_offset_planes` rather than a call this step makes, and `cut_conical_ends` names the
ψ = 0 early return, which belongs to S22. **`distAlong` is a name this spec gives its own local
helper** — the one-line projection `(p − apexWorld) · coneVec` — so it is not a Fusion API call and
the database cannot back it; the module is free to write it inline instead of as a function.
`Point3D.vectorTo` is **one of several alternatives**: the difference of two world points can be
taken with `vectorTo`, with `asVector` and `subtract`, or by reading the three coordinates, and the
spec fixes the arithmetic rather than the call, so the module may use any of them.

Proof: `stepSliceTooth`, asserted by `assertSliceTooth`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepSliceTooth, assertSliceTooth) -->

**From:** `spec/bevelgear/instructions.md` L419–460, L518–529, L747–804, L1033–1035;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-EMPTY-RESULT]`, `[PB-SELF-DIAGNOSING]`,
`[PB-CONSTRUCTION-PLANES]`, `[PB-WORLD-FRAME]`.

## S18 `[GO]` Order the segments and drop the apex scrap (ψ > 0 only)

One timeline entry per gear, only when ψ > 0.

Sort the segments by the `distAlong` of their centroid — `body.physicalProperties.centerOfMass`. The
first, apex-most piece is the long **apex-side scrap** below the toe: **remove it**, and keep the
rest as the working `segments`.

**Drop the scrap by re-slicing the list, *then* delete it** — `segments = segments[1:]` before
`removeFeatures.add(scrap)`, never the other way round.

After dropping the scrap, **`segments` must be non-empty** (at least one cross-section). If it is
empty the slice failed in S17: `raise` a clear error rather than proceeding into the twist and the
crown, which both assume at least one segment (`[PB-EMPTY-RESULT]`).

Calls: `body.physicalProperties`, `physicalProperties.centerOfMass`,
`component.features.removeFeatures.add(itemToRemove)`.

Proof: `stepDropApexScrap`, asserted by `assertDropApexScrap`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepDropApexScrap, assertDropApexScrap) -->

**From:** `spec/bevelgear/instructions.md` L822, L1033–1035;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-EMPTY-RESULT]`, `[PB-REMOVE-PIECES]`.

## S19 `[GO]` Twist the segments (ψ > 0 only)

One timeline entry per gear, only when ψ > 0. Rotate each segment about the **shaft axis**
(`axisDir` through `apexWorld`) so the tooth follows the trace, **centred on `R_mean` so the mid-face
section stays unrotated**. That section then meshes exactly like the straight tooth, which is
critical: the pinion's zero mesh nudge depends on it.

The total toe→heel shaft-axis twist comes from the **conjugate crown-gear generation law** — the
standard Gleason/Litvin model. A spiral bevel is generated by an imaginary flat *crown gear*, and the
work gear's shaft rotation relates to the developed crown-plane azimuth by the roll ratio
`1 / sin γ`, because the generating crown gear has `N / sin γ` teeth. Compute it **analytically —
no projection, no curve sampling**:

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])   # developed azimuth at the apex
total     = abs(phi_crown) / math.sin(gamma)                          # shaft-axis twist magnitude
```

`toe2d` and `heel2d` are the S16 pairs. `gamma` is this gear's **pitch cone angle** — `self._gamma_p`
(Pinion) / `self._gamma_g` (Driving), already computed in S07 and passed into the hook. `handSign`
sets the direction; `total` is the magnitude.

⚠️ **Use the PITCH cone angle γ — NOT `acos(coneVec · axisDir)`**, which is the *root* cone angle,
smaller by the dedendum angle δ_f (24.7° against the pitch 28.7° for a 17-tooth pinion meshing a
31-tooth gear) and yields a twist about 1.15× too large.

⚠️ **The two members of a meshing pair legitimately get different twists**: same cutter, same spiral
angle ψ, but γ differs, so `1 / sin γ` differs — about 2.08× for a 17-tooth pinion against about
1.14× for a 31-tooth gear, a ratio near 1.83. This is *why* equal-teeth pairs always meshed while
ratio pairs failed under any method that gets `1 / sin γ` wrong.

⚠️ **Do NOT measure the twist off a projected 3-D cone trace.** `projectToSurface` wraps the arc
around the cone for ratio pairs and returns it as multiple disjoint fragments, so the measured
azimuth collapses to a fraction of the true sweep — the pinion comes out grossly under-twisted and
the pair interferes. The analytic law here is exact, deterministic and cannot wrap. There is
therefore **no `projectToSurface`, no root-cone-face search and no 3-D trace sketch** anywhere in
this build.

Each segment's rotation is a **linear share** keyed to the **cone distance of its HEEL FACE** — the
segment's farthest-along-the-element face, which is the exact section the S21 loft samples:

```
ang = −handSign · total · (R_mean − R_heelFace(seg)) / span
```

⚠️ **Key the twist on the segment's HEEL-FACE cone distance, NOT its centroid.** The loft samples
each segment's heel face, so that face is what must land at the right azimuth; centroid keying leaves
the loft's mid-face section rotated by half a segment, and the mid faces then overlap.

**Define a slab's heel face precisely: the face whose centroid has the GREATEST
`distAlong(face.centroid)`, searched across ALL of the slab's faces with NO surface-type filter.** Its
toe/apex-side face is the least-centroid one. ⚠️ Do **not** restrict this search to
`PlaneSurfaceType` or any other surface type — a sliced slab is bounded by a mix of the two planar
cut faces and ruled side faces, and a type filter can pick the wrong face or miss the cut face, which
makes the S21 loft fail with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this **same
all-faces-by-centroid** rule (max → heel, min → toe) everywhere a slab end face is needed: here, in
S20's crown base, and in S21's loft sections.

Apply the rotation with a free move by `Matrix3D.setToRotation(ang, axisDir, apexWorld)`
(`[PB-MOVE-ROTATE]`).

Calls: `adsk.core.Matrix3D.create()`, `matrix.setToRotation(angle, axis, origin)`,
`component.features.moveFeatures.createInput2(inputEntities)`,
`moveInput.defineAsFreeMove(transform)`, `component.features.moveFeatures.add(input)`,
`adsk.core.ObjectCollection.create()`, `body.faces`, `face.centroid`,
`math.atan2(y, x)`, `math.sin(gamma)`.

<!-- check-step-calls: ignore projectToSurface defineAsRotate PlaneSurfaceType distAlong -->
<!-- check-compile: ignore projectToSurface defineAsRotate PlaneSurfaceType distAlong -->
Three of these are named only to forbid them: no projection is performed, `defineAsRotate` rejects a
`SketchLine` axis, and a surface-type filter on the heel-face search picks the wrong face. The
fourth, `distAlong`, is the local helper S17 defines, not a Fusion call.

Proof: `stepTwistSegments`, asserted by `assertTwistSegments`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepTwistSegments, assertTwistSegments) -->

**From:** `spec/bevelgear/instructions.md` L802–803, L824–838, L1033–1035;
`spec/bevelgear/spiral-tooth-trace.md` L188–217; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-MOVE-ROTATE]`, `[PB-WORLD-FRAME]`, `[PB-EMPTY-RESULT]`.

## S20 `[GO]` Crown the segments (ψ > 0 only)

One timeline entry per gear, only when ψ > 0. Crown the tooth lengthwise by scaling each segment
**except the outermost (heel) one** down by a **monotonic** factor — full at the heel, growing
smoothly toward the toe — **about a sketch point on the ROOT edge of its heel face**.

For each segment compute its **heel-distance fraction**

    u = (R_heel − R_heelFace) / span

where `R_heelFace` is the `distAlong` of that segment's heel face, found by S19's all-faces-by-centroid
rule but **RECOMPUTED here, AFTER the twist has moved the slabs** — do not reuse pre-twist values —
and `R_heel` and `span` are from S17's frame section.

**`u` runs 0 at the held-full heel to 1 at the toe, and PAST 1 on the two segments that lie beyond
the toe.** Those two segments' heel faces sit `6·span/6` and `7·span/6` in from the parent plane, so
the toe-most one reads about `7/6` (≈1.18) before the twist and a little more (≈1.21) after this
recompute, at the default Spiral Angle 35° on the default 31/31 pair. **Do not treat `8/6` = 1.33 as
a ceiling.** That figure is the last plane's offset, and that plane is a toe face rather than any
segment's heel face, so nothing ever evaluates `u` there — but the twist moves the heel faces, so the
recomputed `u` climbs with the Spiral Angle and is **measured at 1.351 at Spiral Angle 55°**, which
the `[0, 60)` range admits. Nothing reads an upper bound on `u`; the crown factor stays positive for
every value it takes, and what the slab count actually rests on is the structural fact that the last
cut plane is never a heel face. (The heel segment reads a few hundredths rather than exactly 0,
because `R_heel` is read at the heel edge's midpoint rather than on the parent plane; that segment is
skipped anyway.)

**"Outermost (heel) segment" = the one with the GREATEST post-twist heel-face `distAlong`** — sort the
segments by their recomputed heel-face `distAlong` and skip the last. Then:

```
factor = 1 − _CROWN_PER_RAD · (abs(total) / 2) · u
```

`total` is the full toe→heel twist from S19; `abs(total)/2` is the per-end peak twist magnitude, so
the maximum relief — now at the **toe** — keeps the same magnitude the old per-end peak had, just
relocated. This makes relief **grow monotonically from the (full) heel to the toe**, so slab heights
stay **strictly ordered heel→toe** and the natural cone taper is never reversed.

If a computed `factor` comes out ≤ 0 (extreme twist), **`raise`** a self-diagnosing error naming the
gear, the segment's `u` and the factor — never scale by a non-positive factor.
**`_CROWN_PER_RAD` is a tunable class constant, default `0.5`** (0 disables the crown) — set it to
0.5, do not leave it unset or 0.

⚠️ **Do NOT key the relief on `abs(ang)`**, the twist magnitude. That is **symmetric** about the
mid-face — maximal at BOTH ends — so, because the heel slab is held full, the slab *just inside* the
heel becomes the **most**-relieved one and dips below both its neighbours, a notch that reverses the
heel→toe taper. This was the observed bug: the heel-adjacent slab came out at factor `0.932` while
the next slab inward was `0.972`, taller. Key on the monotonic heel-distance `u`, never on `abs(ang)`.

**Three gotchas.**

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). `scaleFeatures` is the **one** exception to
   never-activate: it needs the Design occurrence as the **active** edit target, so call
   **`designOccurrence.activate()`** before the crown scales and restore afterwards — in a
   `finally` — with **`design.activateRootComponent()`**. ⚠️ Do **not** write
   `design.rootComponent.activate()` or `someComponent.activate()`: a `Component` has **no**
   `.activate()` method and raises `AttributeError`. Only `Occurrence` has `.activate()`, and the
   root is re-activated through `Design.activateRootComponent()`.
2. **Skip the outermost (heel) segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone of S22 trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, NOT its centroid — otherwise the crowned tooth
   lifts off the gear base.** `scaleFeatures` shrinks **uniformly** toward the base point, so a base
   point at the heel-face **centroid** (mid tooth-height) pulls the tooth's **root** edge upward by
   `(1 − factor) · (½ tooth height)`: the tooth no longer seats on the gear body's root cone, floats
   above the base, and the Combine-Join leaves a gap — clearly visible for ratio pairs, e.g. module 2
   with driving 19 and pinion 13, which is the symptom that exposed this. Put the base point on the
   **root** instead: of the heel face's vertices (`heelFace.vertices`, each `.geometry` a world
   `Point3D`), take the **two with the smallest perpendicular distance to the shaft axis** — the line
   through `apexWorld` along `axisDir`, perpendicular distance
   `|(p − apex) − ((p − apex) · axisDir) · axisDir|` — those are the two **root corners**, since the
   tip corners are farthest from the axis — and place the base sketch point at their **midpoint**,
   mapped into the heel-face sketch via `modelToSketchSpace`. The heel face is a planar cut, so that
   midpoint lies on it. A uniform scale about a point keeps every line and plane through that point
   invariant, so anchoring on the root keeps the root edge on the seating cone while the tip is
   relieved progressively toward the toe — which is exactly the lengthwise crown intended. Finding the
   heel face itself is unchanged, still the max-`distAlong`-centroid face of S19; only the point *on*
   it changes from centroid to root-edge midpoint.

Calls: `designOccurrence.activate()`, `design.activateRootComponent()`,
`component.sketches.add(planarEntity)`, `sketch.sketchPoints.add(point)`,
`sketch.modelToSketchSpace(modelCoordinate)`, `face.vertices`, `brepVertex.geometry`,
`component.features.scaleFeatures.createInput(inputEntities, point, scaleFactor)`,
`component.features.scaleFeatures.add(input)`, `adsk.core.ValueInput.createByReal(factor)`,
`adsk.core.ObjectCollection.create()`, `face.centroid`, `body.faces`.

<!-- check-step-calls: ignore rootComponent.activate distAlong -->
<!-- check-compile: ignore rootComponent.activate distAlong -->
`rootComponent.activate` is named only to forbid it: a `Component` has no `.activate()`.
`distAlong` is the local helper S17 defines, not a Fusion call.

Proof: `stepCrownSegments`, asserted by `assertCrownSegments`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepCrownSegments, assertCrownSegments) -->

**From:** `spec/bevelgear/instructions.md` L839–851, L1033–1035; `spec/bevelgear/fusion.md`
L199–206; `.claude/skills/generate-gear/PLAYBOOK.md` `[PB-CONSTRUCTION-NEEDS-ACTIVE]`,
`[PB-NEVER-ACTIVATE]`, `[PB-SPACE-METHODS]`, `[PB-SELF-DIAGNOSING]`.

## S21 `[GO]` Loft the spiral tooth (ψ > 0 only)

One timeline entry per gear, only when ψ > 0.

⚠️ **Re-sort the segments by their heel-face cone distance HERE, AFTER the twist (S19) and the crown
(S20) — do NOT reuse the pre-twist slice or centroid order from S18.** The twist rotates each slab
about the shaft axis, and for high-twist *unequal-ratio* pairs that rotation changes the slabs'
along-cone `distAlong` order enough to **reorder adjacent slabs**; lofting in the stale pre-twist
order assembles the cross-sections out of sequence and the crowned tooth comes out distorted, so the
two gears interfere. For equal or low-twist pairs the two orders coincide, which is why equal-teeth
gears mesh even with the stale order while unequal ratios distort — this is the single thing that
makes a ratio pair like 31/17 fail while 31/31 looks fine.

So compute `order = sorted(segment indices, key = distAlong(slabHeelFace(seg).centroid))` **now**, and
loft a **NewBody** through, in that order:

1. first the **toe-most segment's apex-side (toe-facing) face** — the toe segment is `order[0]`, and
   its toe face goes in first to push the loft past the toe cone so the toe trim bites;
2. then the **heel-facing face of every segment, iterated in `order`** — each segment's
   farthest-along-the-element face by post-twist centroid, the last of which reaches past the heel
   cone.

Name the resulting body **`{gear} Spiral Tooth`**. Then remove the segment scaffolding; the loft has
captured their faces.

Calls: `component.features.loftFeatures.createInput(operation)`,
`adsk.fusion.FeatureOperations.NewBodyFeatureOperation`, `loftInput.loftSections`,
`loftSections.add(entity)`, `component.features.loftFeatures.add(input)`,
`component.features.removeFeatures.add(itemToRemove)`, `body.faces`, `face.centroid`, `body.name`.

<!-- check-step-calls: ignore sorted slabHeelFace distAlong -->
<!-- check-compile: ignore sorted slabHeelFace distAlong -->
Three mentions rather than requirements, all inside the ordering expression above. `sorted` is the
Python builtin, not a Fusion call. **`slabHeelFace` and `distAlong` are names this spec gives its
own local helpers** — the all-faces-by-centroid search of S19 and the projection on `coneVec` — so
the API database cannot back either, and the module may write either inline rather than as a
function.

Proof: `stepSpiralLoft`, asserted by `assertSpiralLoft`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepSpiralLoft, assertSpiralLoft) -->

**From:** `spec/bevelgear/instructions.md` L831, L853, L1033–1035;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-LOFT]`, `[PB-REMOVE-PIECES]`.

## S22 `[GO]` Conical end cuts — trim the Tooth Body flush

Two timeline entries per gear (the toe split and the heel split), performed by one framework call.
This is the whole of the tooth-body hook when ψ = 0, and the last thing the spiral branch does when
ψ > 0.

Return
`cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)` from
`.solids`. Do **not** re-implement the cut machinery.

**Two distinct bodies are involved — do not conflate them.** The cutting **TOOLS** are
`ConeSurfaceType` faces of the **Gear Body**, the revolved-hexagon frustum; the lofted Tooth Body has
no cone faces, so searching *it* for the cone face finds none. The **TARGET** being split is the
**Tooth Body**.

The helper implements the pinned cut behavior: the **toe cut first**, its cone face identified by the
toe edge's world **MIDPOINT** best-first across the frustum's cone faces (`[PB-FACE-BY-MIDPOINT]` —
endpoints sit near the apex singularity), each candidate tried as the actual split tool and the first
that splits kept; **keeper selection after each cut** (remove apex-containing pieces, keep the
largest — `[PB-REMOVE-PIECES]`); then the **heel cut on the keeper alone**, which is what makes it
deterministically two split features for every gear ratio. A heel cone that does not intersect the
keeper at all — common on ratio pairs, e.g. module 1 with driving 31 and pinion 43, where the heel
cone never overshoots the tooth — is raised by the helper as the typed `solids.NonIntersectError` and
caught, and the keeper is returned whole. Every failure is self-diagnosing with the per-face
distance and error history (`[PB-SELF-DIAGNOSING]`), and each cut's outcome is logged with
`force_console=True`.

**Caller obligations, which stay in the generator:** pass `toeMid` = the toe edge's world midpoint
(`(M_world + N_world)/2` pinion, `(O_world + P_world)/2` driving) and `heelMid` = the heel edge's
world midpoint (`(C_world + H_world)/2`, `(D_world + J_world)/2`) — the same edge-midpoint pairs as
S17's hand-off table; `apexWorld` = the §2 Apex sketch point's world geometry; `gearBody` = the
revolved frustum, the cone-face source. **The toe cut must split**, and its failure propagates and
crashes the build, which is correct because an uncut tooth is unusable; only the heel cut is lenient,
and only through the typed `NonIntersectError`.

**Why the heel cut is the lenient one.** The dedendum corner C/D and the tooth centre K′/L′ both sit
on this gear's back-cone dedendum line, so the tooth plane contains a generator of the heel cone and
the two touch along the tooth's own centreline instead of crossing it. That cone's apex on the shaft
axis is K/L, where the same dedendum line meets the axis. At Tooth Spacing 0 the tooth centre is that
apex, so the heel cone passes exactly through the tooth's heel-end centreline and takes only the two
corners, by `py² / (2 · r · cos γ)` for a corner lying `py` off the centreline at polar radius `r` in
the tooth plane. A cut that removes that little is a cut that can miss the keeper altogether on a
ratio pair.

Calls: `cut_conical_ends(component, toothBody, gearBody, toeMid, heelMid, apexWorld, label)`,
`sketchPoint.worldGeometry`.

<!-- check-step-calls: ignore apply_conical_cut select_keeper find_cone_faces_by_midpoint surface_distance splitBodyFeatures -->
Each names machinery inside `cut_conical_ends`, which this step calls instead of re-implementing;
none is a call the module makes.

Proof: `stepConicalEndCut`, asserted by `assertConicalEndCut`.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepConicalEndCut, assertConicalEndCut) -->

**From:** `spec/bevelgear/instructions.md` L419–475, L747–762, L855, L880–907, L965–982, L1037–1067;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-FACE-BY-MIDPOINT]`, `[PB-REMOVE-PIECES]`,
`[PB-SPLIT-BODY]`, `[PB-SELF-DIAGNOSING]`.

## S23 `[GO]` Circular pattern

One timeline entry per gear. Circular-pattern the remaining tooth piece around the **shaft-axis
edge** — the same in-sketch profile edge S13's revolve used, not the §2 construction line.

Pin all three inputs explicitly; do not rely on Fusion's defaults staying equal to them
(`[PB-CIRCULAR-PATTERN]`):

- `quantity = <this gear's Teeth Number>`
- `totalAngle = ValueInput.createByString('360 deg')`
- `isSymmetric = False`

Although the pitch diameter shrinks from heel toward apex, the *angular* spacing around the shaft
axis stays constant at `360° / N` for the entire face width: the radial taper is already produced by
the loft from the Apex to the heel-end tooth profile, so the pattern just rotates that single tapered
tooth into N evenly spaced copies.

`CircularPatternFeature.bodies` already includes the seed body plus the copies, so do not re-add the
seed, and **copy them into an `ObjectCollection` first** before handing them to the Combine —
`pattern.bodies` is a `BRepBodies` and `combineFeatures.createInput` rejects it
(`[PB-PATTERN-BODIES]`).

Calls: `component.features.circularPatternFeatures.createInput(inputEntities, axis)`,
`patternInput.quantity`, `patternInput.totalAngle`, `patternInput.isSymmetric`,
`component.features.circularPatternFeatures.add(input)`,
`adsk.core.ValueInput.createByReal(teeth)`, `adsk.core.ValueInput.createByString('360 deg')`,
`adsk.core.ObjectCollection.create()`, `patternFeature.bodies`.

Proof: `stepCircularPattern`, asserted by `assertCircularPattern`. **This step is registered
SERIALLY**, on `proofkit3d.RunSolid`, while every other `[GO]` step here runs its cases in parallel.
The pattern increment retires the seed tooth, so the seed cannot be measured after the step runs and
its azimuth, radius, height and volume have to be read during the build and handed to the assertion.
That hand-off leaves the case, and two cases sharing one set of seed readings overwrite each other.
It is not a hazard that announces itself — a pair of cases whose seeds measured alike would pass on
each other's numbers — and the proof file records beside the carried readings that this step is
serial because of them.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCircularPattern, assertCircularPattern) -->

**From:** `spec/bevelgear/instructions.md` L908–909, L1116–1141;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-CIRCULAR-PATTERN]`, `[PB-PATTERN-BODIES]`.

## S24 `[GO]` Combine-Join

One timeline entry per gear. Join all patterned tooth pieces with the Gear Body in a **single**
Combine-Join: the Gear Body is the target and the patterned tooth bodies are the tools.

Calls: `component.features.combineFeatures.createInput(targetBody, toolBodies)`,
`combineInput.operation`, `adsk.fusion.FeatureOperations.JoinFeatureOperation`,
`component.features.combineFeatures.add(input)`, `adsk.core.ObjectCollection.create()`.

Proof: `stepCombineJoin`, asserted by `assertCombineJoin`.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCombineJoin, assertCombineJoin) -->

**From:** `spec/bevelgear/instructions.md` L910–911, L983–1025;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-PATTERN-BODIES]`.

## S25 `[PROSE]` The bore plane

One timeline entry per gear, and only when Enable Bore is checked: a construction plane normal to the
shaft at its start, `setByDistanceOnPath(<shaft-axis edge>, 0.0)`. Pass the **in-sketch profile
edge**, not the §2 construction line, and pass it directly rather than wrapped in `Path.create`
(`[PB-CONSTRUCTION-PLANES]`).

The plane is rooted at the shaft start, so its origin sits on the axis, which is what lets S26 centre
the bore circle on the sketch origin.

Calls: `component.constructionPlanes.createInput()`,
`planeInput.setByDistanceOnPath(pathEntity, distance)`,
`component.constructionPlanes.add(input)`, `adsk.core.ValueInput.createByReal(0.0)`.

<!-- check-step-calls: ignore Path.create -->
Named only to forbid it.

`[PROSE]` for the reason S06 gives.

**From:** `spec/bevelgear/instructions.md` L912–913;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-CONSTRUCTION-PLANES]`.

## S26 `[GO]` `{gearLabel} Bore` sketch

One timeline entry per gear, only when Enable Bore is checked: a sketch named `Pinion Bore` /
`Driving Bore` on the bore plane.

Sketch the bore circle centred at the **sketch origin** — the plane is rooted at the shaft start, so
the origin is on the axis. **Fix the circle's centre and add a diameter dimension** set to this
gear's resolved bore diameter (`[PB-CIRCLE-CENTER]`): a circle's centre is free even when created at
(0,0,0), because `addByCenterRadius` does not reuse the sketch's `originPoint`, and
`addCoincident(circle.centerSketchPoint, sketch.originPoint)` has been observed to throw
`VCS_SKETCH_SOLVING_FAILED` on a `setByDistanceOnPath` plane. `isFixed` on the centre plus a diameter
dimension is the reliable pin, and it leaves the sketch fully constrained.

The diameter is this gear's `boreDiameter_cm`, **already resolved and already bounded** by S07 step
28 — the user's value if non-zero, otherwise `this gear's Pitch Diameter / 4`, in either case capped
to that gear's Maximum Bore Diameter. Take that resolved number; do not re-derive it here, or the cap
is lost and the bore deletes the body's back face.

Gate it: `if not sketch.isFullyConstrained: raise` naming the sketch
(`[BEVEL-F-FULL-CONSTRAINT]`).

Calls: `component.sketches.add(planarEntity)`, `sketch.name`,
`sketch.sketchCurves.sketchCircles.addByCenterRadius(centerPoint, radius)`,
`sketchCircle.centerSketchPoint`, `sketchPoint.isFixed`,
`sketch.sketchDimensions.addDiameterDimension(entity, textPoint)`, `sketchDimension.parameter`,
`adsk.core.Point3D.create(x, y, z)`, `sketch.isFullyConstrained`.

<!-- check-step-calls: ignore addCoincident -->
Named only to forbid coincidenting the bore circle's centre to `sketch.originPoint`.

Proof: `stepBoreSketch`.

<!-- proof-run: proofkit.RunParallel(perGearCases, stepBoreSketch) -->

**From:** `spec/bevelgear/instructions.md` L100–141, L491–517, L666–668, L912–913;
`spec/bevelgear/fusion.md` L21–30; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-CIRCLE-CENTER]`, `[PB-FULL-CONSTRAINT]`, `[PB-SKETCHCURVES]`.

## S27 `[GO]` Bore through-cut

One timeline entry per gear, only when Enable Bore is checked. Extrude-cut the bore circle as a
**symmetric through-cut restricted to this Gear Body**:

- `extentInput.setSymmetricExtent(ValueInput.createByReal(2 × Cone Distance), False)` — the second
  argument is `isFullLength=False`, so the value is the **half-length per side**, generously past any
  face width. Do not pass a third, taper argument (`[PB-THROUGH-CUT]`).
- `participantBodies = [this Gear Body]`, taken from the dict's `gearBody`.
- operation `CutFeatureOperation`.

Skip this step entirely when Enable Bore is unchecked.

The bore is a through cut on the shaft axis, so it removes every ring of material inside its own
radius and reaches the two ends of the body first: past the heel term of the Maximum Bore Diameter it
takes the **entire flat back face**, and past the toe term it takes the whole toe dish and bites into
the root cone. Either way the revolved frustum comes out of this step with no end face on that side,
which is what the bound applied in S07 step 28 exists to prevent.

Calls: `component.features.extrudeFeatures.createInput(profile, operation)`,
`adsk.fusion.FeatureOperations.CutFeatureOperation`,
`extrudeInput.setSymmetricExtent(distance, isFullLength)`,
`extrudeInput.participantBodies`, `component.features.extrudeFeatures.add(input)`,
`adsk.core.ValueInput.createByReal(distance)`, `sketch.profiles`, `profiles.item(0)`.

Proof: `stepBoreCut`, asserted by `assertBoreCut`.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepBoreCut, assertBoreCut) -->

**From:** `spec/bevelgear/instructions.md` L100–141, L912–913, L1026–1032;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-THROUGH-CUT]`.

## S28 `[GO]` Meshing rotation

One timeline entry, on the **driving gear only**, performed **here in the Design component, before
the body is moved out**.

Rotate the driving body by `180° / Driving Gear Teeth Number` — half a tooth pitch — about its shaft
axis with `rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)` from `.solids`,
which takes the rotation axis and origin from the B′→I profile edge's **world** endpoints
(`[PB-MOVE-ROTATE]`). A driving valley then sits where the pinion tooth crosses the axial plane,
giving the interlocked meshing look. Both gears are patterned from a starting tooth in the axial
plane, so without the offset a driving tooth and a pinion tooth would both sit at that crossing and
visually collide.

This runs in Design before `moveToComponent` because a construction axis cannot be added in the
moved-out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation must use the edge's
world geometry while the body is still in Design.

The pinion additionally receives `_pinionMeshPhase(pinionTeeth)`, which returns the pinion's extra
mesh rotation in **radians**, `_PINION_MESH_PHASE_TEETH · 2π / pinionTeeth`, with
`_PINION_MESH_PHASE_TEETH` defaulting to **0**. **A zero angle is a no-op, not a move:**
`setToRotation(0, axis, origin)` builds the identity and Fusion refuses it with
`RuntimeError: 3 : invalid transform` (`[PB-MOVE-ROTATE]`). `rotate_body_about_edge` absorbs that, so
the call site does not guard it.

Calls: `rotate_body_about_edge(component, body, edge, angleRad)`, `math.radians(angle)`.

<!-- check-step-calls: ignore _pinionMeshPhase setToRotation -->
`_pinionMeshPhase` is this module's own helper, and `setToRotation` is named as the mechanism inside
`rotate_body_about_edge` rather than a call this step makes.

Proof: `stepMeshRotation`, asserted by `assertMeshRotation`.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepMeshRotation, assertMeshRotation) -->

**From:** `spec/bevelgear/instructions.md` L447–475, L914–915, L919–923;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-MOVE-ROTATE]`, `[PB-CONSTRUCTION-NEEDS-ACTIVE]`.

## S29 `[PROSE]` Move the finished body into its gear component

One timeline entry per gear. Relocate this gear's finished body into its `{gearLabel} Gear`
occurrence with `body.moveToComponent(gearOccurrence)`, which preserves world position and needs no
activation (`[PB-NO-CROSS-SIBLING]`).

Every feature operation ran in the Design component, because Fusion rejects cross-sibling sketch and
`project` calls even when the target is activated or the entities are wrapped in
`createForAssemblyContext` proxies. The visible end state is identical to having built in place.

Calls: `body.moveToComponent(target)`.

<!-- check-step-calls: ignore createForAssemblyContext -->
Named only to say that wrapping entities in it does not help and is not done.

`[PROSE]`: relocating a body between components is assembly bookkeeping. `decad` has no component
tree, so there is no substitute geometry a gate would accept; the proof builds every case in one flat
document and says so in `proof/bevelgear/solids_test.go`.

**From:** `spec/bevelgear/instructions.md` L364–395, L857–871;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-NO-CROSS-SIBLING]`.

## S30 `[PROSE]` Cleanup — hide the construction geometry

One timeline entry. Call `hide_construction_geometry(bevelComponent)` from `.solids`. It recursively
walks the Bevel Gear component tree, dedupes by `entityToken`, and hides every sketch, construction
plane and construction axis with `isLightBulbOn = False` — construction planes and axes are **not**
hidden by `isVisible` (`[PB-HIDE-AFTER-USE]`, `[BEVEL-F-CLEANUP]`). Leave only the two finished gear
bodies visible.

There is no sketch-only mode and no per-mode guard: bevel always builds solids.

The driving gear's half-tooth-pitch meshing rotation is **not** a cleanup step; it is performed
earlier, in S28, in the Design component before the body is moved out.

Nothing in a generated module calls `settle_sketch_display`: `commands/_gear_command.py` calls it
once after `generate()` returns, and every gear command runs through that one call
(`[PB-SETTLE-DISPLAY]`).

Calls: `hide_construction_geometry(component)`.

<!-- check-step-calls: ignore isLightBulbOn isVisible entityToken settle_sketch_display -->
The first three name the mechanism inside the framework helper, which this step calls instead of
re-implementing the walk; `settle_sketch_display` is named only to say a generated module must not
call it.

`[PROSE]`: display state is not geometry, and neither harness models visibility at all.

**From:** `spec/bevelgear/instructions.md` L919–923; `spec/bevelgear/fusion.md` L207–212;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-HIDE-AFTER-USE]`, `[PB-TREE-CLEANUP]`,
`[PB-SETTLE-DISPLAY]`.
