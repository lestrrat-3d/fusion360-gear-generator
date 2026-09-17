# Bevel gear — compiled step list

The proof for these steps is `proof/bevelgear/geometry_test.go`,
`proof/bevelgear/sketches_test.go`, `proof/bevelgear/solids_test.go`,
`proof/bevelgear/spiral_test.go` and the generated
`proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `52dfe41c1d1cc7bf9536e491e0d643634bcd006f` |
| `spec/bevelgear/fusion.md` | `efa49ddcd40c9d796c687097ff9fa620c1bda325` |
| `spec/bevelgear/spiral-tooth-trace.md` | `84474c55a77775fba98991437ef77a0fa812bd12` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9ee2dcbaed7b5480aa69e9295e8b61acaea081f3` |

## S01 `[PROSE]` Command dialog inputs

Add the **20** dialog inputs to `cmd.commandInputs` in this display order, in
`BevelGearCommandInputsConfigurator.configure(cls, cmd)`. Target Plane is added first so it wins
Fusion's auto-focus (`[PB-AUTOFOCUS-FIRST]` — Fusion focuses the FIRST `SelectionCommandInput` and
ignores a later `hasFocus`), then Center Point, then the pre-selected Parent Component, then the
numeric and boolean fields.

Reproduce this table verbatim. Every id string, label, unit string, default and tooltip below is
part of the surface.

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

The module-level constants holding those id strings, in row order, named exactly:
`INPUT_ID_PLANE = 'targetPlane'`, `INPUT_ID_CENTER_POINT = 'centerPoint'`,
`INPUT_ID_PARENT = 'parentComponent'`, `INPUT_ID_MODULE = 'module'`,
`INPUT_ID_SHAFT_ANGLE = 'shaftAngle'`, `INPUT_ID_DRIVING_TEETH = 'drivingTeeth'`,
`INPUT_ID_PINION_TEETH = 'pinionTeeth'`, `INPUT_ID_DRIVING_BASE_HEIGHT = 'drivingBaseHeight'`,
`INPUT_ID_PINION_BASE_HEIGHT = 'pinionBaseHeight'`, `INPUT_ID_BORE_ENABLE = 'boreEnable'`,
`INPUT_ID_DRIVING_BORE = 'drivingBore'`, `INPUT_ID_PINION_BORE = 'pinionBore'`,
`INPUT_ID_FACE_WIDTH = 'faceWidth'`, `INPUT_ID_TOOTH_SPACING = 'toothSpacing'`,
`INPUT_ID_SPIRAL_ANGLE = 'spiralAngle'`, `INPUT_ID_HAND = 'spiralHand'`,
`INPUT_ID_CUTTER_RADIUS = 'cutterRadius'`, `INPUT_ID_TOE_EXTENSION = 'toeExtension'`,
`INPUT_ID_DRIVING_TOE_RADIUS = 'drivingToeRadius'`, `INPUT_ID_PINION_TOE_RADIUS = 'pinionToeRadius'`.
Plus `_HAND_RIGHT = 'Right'` and `_HAND_LEFT = 'Left'`. There are no `PARAM_*` strings: bevel
registers **no** Fusion user parameters at all (`[PB-PRECOMPUTED-MODE]`), so nothing here is a live
link and re-running the dialog is how a gear changes (`[PB-NUMERIC-SNAPSHOT]`). Look every
`adsk.*` name up rather than guessing it, and get the submodule right — the geometry and value types
live in `adsk.core` and the feature and topology enums in `adsk.fusion`, and the wrong one is a
runtime `AttributeError` rather than a parse error (`[PB-API-LOOKUP]`, `[PB-ADSK-MODULES]`).

Each selection input takes `addSelectionInput(id, label, tooltip)` with the tooltip string above
used verbatim — the filter set and the limits are contract surface this table declares per input
(`[PB-SELECTION-DECL]`) — then `addSelectionFilter` once per filter — written as the named constant
`adsk.core.SelectionCommandInput.ConstructionPlanes`, never a quoted literal
(`[PB-SELECTION-FILTER-ENUM]`) — then `setSelectionLimits(1, 1)`. Parent pre-selects
`get_design().rootComponent`. The Hand dropdown is
`adsk.core.DropDownStyles.TextListDropDownStyle`; add `Right` selected and `Left` unselected
through the returned input's `listItems.add(name, isSelected)`. The `mm` and `deg` defaults are in
Fusion INTERNAL units regardless of the unit string (`[PB-DIALOG-DEFAULT-UNITS]`), which is why the
lengths pass through `to_cm` and the angle uses `createByString` so the expression engine parses it.

**Conditional visibility.** Hand of Spiral and Cutter Radius are relevant only for curved bevels,
so they are hidden when ψ = 0 and shown when ψ > 0; Mean Spiral Angle is the controller and is
always visible. Add the classmethod `_updateSpiralInputVisibility`: it reads the
`spiralAngle` input's **`.expression`** through
`unitsManager.evaluateExpression(spiral.expression, 'rad')` — internal radians, and NOT the input's
`.value` — and assigns `inputs.itemById(INPUT_ID_HAND).isVisible` and
`inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible` to `(value > 0)`. Guard it: return early if any
of the three inputs is `None`, and wrap the evaluation in `try/except`, leaving both inputs SHOWN on
failure, because a half-typed expression can raise mid-edit. `configure()` calls it as its last
step. `handle_input_changed` calls `cls._updateSpiralInputVisibility(args.inputs)` on
every input change — cheap and robust, with no branch on which input changed.
`isVisible` only hides the dialog row; the input still exists and `_readInputs` reads it normally.

`configure` and `handle_input_changed` are bound **by name** from `commands/bevelgear/entry.py` and
are never called by the generated module itself, which is why they carry the exemption below.

<!-- check-step-calls: ignore configure handle_input_changed _updateSpiralInputVisibility -->

**From:** `spec/bevelgear/instructions.md` L19-34 L181-265,
`.claude/skills/generate-gear/PLAYBOOK.md` L38-41 L53-61 L103-143 L291-357 L355-357
L557-568 L850-867

## S02 `[GO]` Resolve and validate the input values

Read every input up front in one `_readInputs(inputs)` pass, before anything creates an occurrence.
It returns the 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth,
pinionTeeth, shaftAngle_deg)` and stashes the rest on `self` as `self._drivingBaseHeight_cm`,
`self._pinionBaseHeight_cm`, `self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`,
`self._faceWidth_cm`, `self._toothSpacing_cm`, `self._spiralAngle_rad`, `self._hand`,
`self._cutterRadius_cm`, `self._toeExtension_pct`, `self._drivingToeRadius_cm`,
`self._pinionToeRadius_cm`.

Read each input with the helper that matches the type it was declared with (`[PB-INPUT-READ]`):
the three selections with `get_selection`, the checkbox with `get_boolean` — never `get_value` on a
bool input, which has no `.expression` — and the dropdown
through `itemById(INPUT_ID_HAND).selectedItem.name` (default `Right` when none). Read every numeric
and angle input by evaluating its expression with
`design.unitsManager.evaluateExpression(input.expression, units)` using `''` / `'mm'` / `'deg'`
(`[PB-EVAL-EXPRESSION]`): the value always comes back in Fusion internal units — cm for length,
RADIANS for angle — whatever the unit string says. Convert the shaft angle with `math.degrees` before
any degree-range check. Coerce both tooth inputs with `int(round(...))` before validating.

**Units, and this is where a whole gear goes 10× wrong.** The `'mm'` inputs and the `'deg'` input
come back already internal, so use them as they are and do **not** `to_cm` them again. `Module` is
read with `''`, so it comes back as a raw number meaning MILLIMETRES, and every length derived from
it must be `to_cm`-converted before it touches geometry: the pitch diameters, the Cone Distance, the
dedendum `1.25 * Module`, every §2 seed length, and the default Face Width. Mixing a raw-mm
Module-derived length with an already-cm `'mm'` input makes the Face-Width bound meaningless.

Validate, in this order:

1. Range checks: module > 0, each tooth count ≥ 3, the Shaft Angle at or above 30° and below the
   **Maximum Shaft Angle**, and non-negative heights, bores, face width, tooth spacing, toe
   extension and toe radii. Reject a negative Cutter Radius. The Mean Spiral Angle's range is
   **[0, 60)** degrees and the Toe Extension's is **[0, 100]**.
2. **Maximum Shaft Angle** = `min(degrees(acos(-min(DPD, PPD) / max(DPD, PPD))), 150)`. The
   cone-angle half is EXCLUSIVE — reject at it as well as above it, because a pitch cone angle
   reaching 90° turns that gear's cone inside out and `R * cos γ` passes through zero and changes
   sign — while the 150° half is inclusive and is a practical ceiling rather than a measured one.
   It depends on both tooth counts, so check it after both are read, and name the computed limit in
   the message.
3. Compute `tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)` and `γ_g = Σ − γ_p`, with
   `PPD = Module · Pinion Gear Teeth Number` and `DPD = Module · Driving Gear Teeth Number`.
4. **Minimum Teeth**, per gear with that gear's own γ: `teeth >= 5.27 * cos γ`, on top of the
   blanket `teeth >= 3`. Name the computed floor in the message. The constant is
   `2 * (1.05 * 1.25 / 0.95 + 1.25) = 5.2632` rounded UP to 5.27 so the published floor stays at or
   above the exact crossing; do not round it down. Run this check BEFORE the base heights, because
   it is exactly the statement that the base-height window is non-empty.
5. **Base heights**, per gear, both closed-form:
   `Minimum Base Height = 1.05 * 1.25 * Module * sin γ` and
   `Maximum Base Height = 0.95 * (r - 1.25 * Module * cos γ) * tan γ`, with `r` that gear's pitch
   radius. The driving fallback is `Module * Driving Gear Teeth Number / 8`; the pinion fallback is
   the **RESOLVED** driving height times `Pinion Teeth / Driving Teeth`, and then the PINION's own
   Maximum Base Height is applied to it, because the two gammas differ whenever the tooth counts do.
   Raise a fallback below the minimum, cap one above the maximum, and REJECT a user value outside
   either end naming the bound it broke.

The base height is measured from **Apex 2's plane**, not from the dedendum point: walking out the
dedendum line the perpendicular distance to the shaft axis falls at `cos γ` per unit while the
along-shaft coordinate rises at `sin γ`, so the heel corner reaches the axis at `r * tan γ`. The
bound above sits `1.25 * Module * sin γ` below that true crossing and is deliberately conservative.

**The bore bound is NOT part of this pass.** Validate the two bore diameters here only as
non-negative numbers. Its heel term would resolve here, but its toe term needs the Root Length,
hence the resolved Face Width, hence solved §2 geometry, and the bound is the minimum of the two —
so the whole bound resolves in §2, at S07.

Reading every selection before anything creates an occurrence is what keeps the
selection-context-shift hazard away (`[PB-SELECTION-STASH]`); bevel registers no parameters, so
nothing creates an occurrence during this pass anyway, but keep the order so it stays that way.
`_readInputs` is the generator's own helper and is called by `generate`, so it is a real call; the
framework readers it uses are `get_selection` and `get_boolean` from `base.py`.

The proof function is `stepDerivedValues`. Its case table walks the Shaft Angle range end to end,
both directions of the gear ratio, the tooth counts the virtual count needs, and each input that
moves the lattice, and it checks every bound above against the solved lattice rather than against
the arithmetic alone.

<!-- proof-run: proofkit.RunParallel(sketchCases, stepDerivedValues) -->

**From:** `spec/bevelgear/instructions.md` L35-105 L263-308 L337-362,
`.claude/skills/generate-gear/PLAYBOOK.md` L99-127 L205-237 L326-353 L863-867

## S03 `[PROSE]` Create the Bevel Gear component

Create the Bevel Gear component as a child of the user's Parent Component with
`parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` and name it `Bevel Gear`
(`[PB-OCCURRENCE-TREE]`). Stash the occurrence on `self.bevelOccurrence` so `deleteComponent()` can
delete it on failure, and the component on `self.bevelComponent`.

**NEVER call `activate` on any occurrence** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`).
Bevel's own reason: the Anchor Sketch is created on the user's EXTERNAL, root-owned target plane,
and an activated occurrence resolves that external plane in its own local frame, so the whole build
collapses onto world XY regardless of the real plane tilt. The sole exception is the spiral crown's
scale in S20.

Bevel uses a **standalone generator**: `BevelGearGenerator` does not subclass `base.Generator`,
carries no `GenerationContext`, and does not use `getOccurrence`, `addParameter`, `parameterName` or
`createSketchObject`. Its `__init__(self, design)` stores `self.design` and sets
`self.bevelOccurrence = None`. From `base.py` import only `get_selection` and `get_boolean`; imports
are explicit, with no `import *` anywhere in the module.

<!-- check-step-calls: ignore activate deleteComponent -->

**From:** `spec/bevelgear/instructions.md` L310-336 L419-428 L566-571,
`spec/bevelgear/fusion.md` L1-18 L199-206,
`.claude/skills/generate-gear/PLAYBOOK.md` L1-16 L17-48 L75-98 L254-290 L811-828 L869-874

## S04 `[PROSE]` Create the Design component

Create the Design component as a child of the **Bevel Gear** component, again with
`parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`, and name it `Design`. Hold the
occurrence as `self.designOccurrence` and the component as `self.designComponent`. Every sketch,
construction plane, construction axis and feature in this build runs in this one component, because
Fusion rejects cross-sibling sketch and `project` references even through
`createForAssemblyContext` proxies (`[PB-NO-CROSS-SIBLING]`); the finished bodies are moved out at
the end with `moveToComponent`.

**From:** `spec/bevelgear/instructions.md` L337-362 L572-577 L871,
`.claude/skills/generate-gear/PLAYBOOK.md` L829-834

## S05 `[GO]` Anchor sketch

Start the sketch **directly on the user-selected target plane** — `sketches.add(targetPlane)` —
whether the selection is a `ConstructionPlane` or a `PlanarFace`, and name it `Anchor`. Do not
re-derive or offset it (`[PB-USE-SELECTED-PLANE]`): a coplanar plane built inside the Design
component resolves in that component's own frame and silently loses the selected plane's world
orientation, collapsing the build onto XY.

Mark the centre by projecting the user-specified centre point into the sketch with
`sketch.project(centerPoint)`.

Draw a line through the projected centre, seeding its two endpoints at exactly **±0.5 cm** from the
projected centre along the sketch-local X, so the seeded length is 10 mm. Apply **BOTH**
`addCoincident(projectedCenter, anchorLine)` — the intersection, which pins the centre onto the
line — **and** `addMidPoint(projectedCenter, anchorLine)`, which makes the centre bisect it. Use
both; midpoint alone is not enough. Add an aligned distance dimension with
`addDistanceDimension(startPoint, endPoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
and do **not** assign its `.parameter.value`: it simply locks the length at the seeded 10 mm, which
is arbitrary because this is only a reference line. Then pin the direction with
`addHorizontal(anchorLine)` — sketch-local, per `[PB-REFLINE-DIRECTION]`, so it survives any tilted
target plane; a world-axis lock would mis-orient it. The line's absolute direction is arbitrary,
since §2 derives every direction relative to the projected anchor line, but it must not be a free
degree of freedom: midpoint plus length plus horizontal leaves zero DOF.

**Stash the projected-centre `SketchPoint` on `self._anchorCenterPoint`** so §2 re-projects THIS
point rather than the raw user-selected centre.

Gate the sketch at the end of the step: raise, naming the sketch, if `isFullyConstrained` is false
(`[PB-FULL-CONSTRAINT]`, `[BEVEL-F-FULL-CONSTRAINT]` — a free DOF is a generation defect, not a
warning).

`project` is named here deliberately. The compiled Fusion API reference declares `project2(entities,
isLinked)` and no `project`, so every gate in this repo reports this call as unverified; that report
is expected and is not a defect to fix. `project` is what the shipped add-ins call, and the two are
not interchangeable in any case, since `project2` takes a list and returns a list.

The proof function is `stepAnchorSketch`. It records one deviation at its own site: Fusion needs
both the coincident and the midpoint, while the bench engine's midpoint is a two-row constraint that
already pins both coordinates, so the proof carries the midpoint alone and says so.

<!-- proof-run: proofkit.RunParallel(sketchCases, stepAnchorSketch) -->

**From:** `spec/bevelgear/instructions.md` L491-499 L578-583 L590-594,
`spec/bevelgear/fusion.md` L19-30, `.claude/skills/generate-gear/PLAYBOOK.md` L239-253
L433-450 L838-848

## S06 `[PROSE]` Gear Profiles Plane

Create the plane with `constructionPlanes.createInput()` then
`setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)` and
`constructionPlanes.add(planeInput)`. Name it `Gear Profiles Plane` and stash it on
`self._gearProfilesPlane`.

Build it off the **original `targetPlane`** as the reference, not a re-derived or offset copy
(`[PB-USE-SELECTED-PLANE]`): this is the second place the target-plane orientation reaches the
bodies, and substituting a different plane here also collapses the gear onto XY. Pass the
`SketchLine` DIRECTLY to `setByAngle`; never wrap it in `adsk.fusion.Path.create` first, which
raises an `InternalValidationError` whenever the curve's owner sketch is not trivially resolvable in
a multi-component context (`[PB-CONSTRUCTION-PLANES]`).

**From:** `spec/bevelgear/instructions.md` L584-589 L713-717,
`.claude/skills/generate-gear/PLAYBOOK.md` L708-716 L775-786

## S07 `[GO]` Gear Profiles sketch

One sketch, named `Gear Profiles`, on the Gear Profiles Plane, holding the whole §2 lattice. Stash
it on `self._gpSketch`.

**Every line in this sketch is a construction line** — set `isConstruction = True` on the lattice
lines, the toe lines and the short reference and connector lines alike. The solid features later
consume only the per-gear Profile sketches, never a §2 curve.

**Every length dimension in this sketch is aligned**: `addDistanceDimension(pointOne, pointTwo,
adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`. This figure has no
axis-aligned line in it — the shaft axes sit at the Shaft Angle to each other and the whole lattice
tilts with the target plane — so a horizontal or vertical orientation would dimension the line's
PROJECTION onto a sketch axis instead of its length. The offset dimensions are a different call,
`addOffsetDimension`, which takes no orientation.

The §2 constraint scheme is exercised only inside Fusion: bevel has **no** `spec/bevelgear/sketch/`
bench proof, so the `[PB-SKETCH-FIRST]` gate is **waived** for it as it stands. A regen keeps the
scheme exactly as stated here — it may not invent a new one, and may not claim bench-proven status.

Geometric-constraint method names are exact: `addCollinear` carries a double L and `addColinear`
does not exist, and `addMidPoint` carries a capital P (`[PB-API-SPELLING]`). Sketch curve
collections live under `sketch.sketchCurves.sketchLines`, never on the sketch directly
(`[PB-SKETCHCURVES]`), while `sketch.sketchPoints`, `sketch.sketchDimensions`,
`sketch.geometricConstraints` and `sketch.profiles` are direct members. Every dimension added here
is DRIVING: never pass a trailing `isDriven` (`[PB-DRIVING-DIM]`). A seed-coordinate helper must
tolerate both a raw `(x, y)` tuple and an object with `.x` / `.y`, because §2 mixes seed tuples with
solved `.geometry` points and feeds both to the same helper (`[PB-POINT-HELPER]`).

**Build every line in the COINCIDENT style** (`[BEVEL-F-COINCIDENT-STYLE]`, a stricter delta to
`[PB-SHARE-XOR-COINCIDENT]`): create the line from raw `adsk.core.Point3D.create` coordinates for
BOTH endpoints and pin each endpoint that already exists with exactly one
`addCoincident(line.startSketchPoint, existingPoint)`. Never pass an existing `SketchPoint` into
`addByTwoPoints` to share it. This covers the short reference and connector lines too — `C→K`,
`D→L`, `C→K′`, `D→L′`, `M→C`, `O→D`, `A′→G`, `B′→I` — with no exemption: sharing without a
coincident leaves the sketch under-constrained and the gate fails, and sharing AND coinciding is
redundant and the solve dies with `VCS_SKETCH_SOLVING_FAILED`.

**Each named line is created ONCE and reused** (`[BEVEL-F-LINE-ONCE]`). A helper that creates an
extension line must RETURN it and the caller keeps the reference; drawing a second line between the
same two points to obtain a reference over-determines the coupled net and the solve fails with
`VCS_SKETCH_OVER_CONSTRAINTS`.

**The driven lengths are NOT dimensioned** (`[BEVEL-F-DRIVEN-DIMS]`): the along-shaft lengths
Apex→A and Apex→B and every extension line are driven by the closing, perpendicular and collinear
constraints. Undimensioned does not mean unpinned — each has a closed-form SEED below, and the seed
is the only thing that picks the figure.

**Place the figure in the sketch's own 2-D coordinates** (`[BEVEL-F-APEX-LOCAL]`), never through a
world-coordinate round trip, which is what caused the XY collapse. The single permitted world use is
reading the target-plane normal as a DIRECTION to choose the grow side.

### The frame

Project the Anchor sketch's stashed centre point with `sketch.project(self._anchorCenterPoint)` —
not the raw user-selected centre, which is a cross-component reference and can resolve
inconsistently. Let `c` be the projected centre and `d` the projected anchor line's 2-D unit
direction; the in-plane perpendicular is `perp = (-d.y, d.x)`, and **its sign is chosen so it points
toward the target-plane normal** (`[BEVEL-F-GROW-SIDE]`), read as `targetPlane.geometry.normal` for
BOTH selection kinds — a `BRepFace`'s geometry and a `ConstructionPlane`'s geometry are each a
`core.Plane` carrying `.normal`. A sketch-local rule such as `perp.y >= 0` is deterministic but not
tied to a physical side and grows the gear inconsistently.

The closed forms every seed below is written from, with `Σ` the Shaft Angle:

- `tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`, `γ_g = Σ − γ_p`
- **Pitch Cone Distance** `R = (PPD / 2) / sin γ_p` — this `R`, never the Cone Distance parameter
- `|Apex→A| = R · cos γ_p`, `|Apex→B| = R · cos γ_g`; both cosines are positive for every Shaft
  Angle the range check admits, which is what the Maximum Shaft Angle exists to guarantee
- `Cone Distance = sqrt(PPD² + DPD²)`, the DIAGONAL of the two pitch diameters, which equals `2R`
  exactly at Shaft Angle 90° and diverges everywhere else

Then, in creation order:

1. **`c→Apex`**: a construction line from the projected centre, perpendicular to the projected
   anchor line — `addPerpendicular(centerToApex, projectedAnchorLine)` — with its far end seeded at
   `c + perp·(R·cos γ_g + <resolved Driving Gear Base Height>)`. Seed it at THAT distance and not at
   the Driving Gear Pitch Diameter: the net closes this line at `R·cos γ_g` above point I plus the
   resolved driving base height, and for the default 31/31 pair at 90° the old seed sat 11.6 mm past
   where the solve puts it (`[PB-SEED-NEAR]`). Add no length constraint. Pin the start with exactly
   one `addCoincident(centerToApex.startSketchPoint, projectedCenter)`.
2. **`Apex→B`**, the Driving Gear Shaft Axis, pointing from the apex back toward the anchor line in
   the `-perp` direction, with its far end seeded at `apex - perp·(R·cos γ_g)`, which is
   `c + perp·<resolved Driving Gear Base Height>` — measure from the APEX, not from `c`. Apply
   `addParallel(drivingShaftAxis, centerToApex)`. **Do NOT use `addVertical`**, which forces the
   line to the sketch's world-vertical and mis-orients the figure on a tilted target plane. Coincide
   the start with the apex. Do not dimension the length. The end is point **B**.
3. **`Apex→A`**, the Pinion Gear Shaft Axis: the driving direction rotated about the apex by the
   Shaft Angle. Form BOTH candidate point-A positions — rotated by +Σ and by −Σ — and keep the one
   whose endpoint has the **greater X** in this sketch. Compare the two and take the larger; do not
   rotate one fixed sense and flip only when its X comes out negative, because when both candidates
   have a positive X that shortcut keeps the wrong one. Call the chosen unit direction `pinionDir`.
   Apply `addAngularDimension(pinionShaftAxis, drivingShaftAxis, textPoint)` equal to the Shaft
   Angle, with the **text point inside the Σ wedge** so it measures Σ and not 180−Σ
   (`[PB-ANGULAR-DIM]`) — for example `apex + normalize(pinionDir + drivingDir) · (PPD/4)`. Coincide
   the start with the apex; do not dimension the length. The end is point **A**.
4. **`A→Apex2`**, the PPD/2 drop: from A, perpendicular to the Pinion Gear Shaft Axis, drawn toward
   the side where Apex 2 lies. ⚠️ Apex 2 sits in the interior wedge BETWEEN the two shaft axes, so
   this drop points toward the OTHER shaft axis, toward point B — pick the perpendicular sense by
   the sign of its dot with the A→B direction, not against a generic "toward the anchor line"
   reference. `addPerpendicular` against the Pinion Gear Shaft Axis, plus an aligned dimension of
   **Pinion Gear Pitch Diameter / 2**. Coincide the start with A. Throughout this step, "A->Apex2"
   always means THIS drop line and never the Apex→A shaft axis; the two share point A and are
   different lines. The same holds for "B->Apex2" against Apex→B.
5. **`B→Apex2`**, the DPD/2 drop: from B, perpendicular to the Driving Gear Shaft Axis, toward point
   A — pick the sense by the sign of its dot with the B→A direction. ⚠️ Do **NOT** choose this sense
   by a "toward the anchor line" reference: the Driving Gear Shaft Axis is itself parallel to that
   grow direction, so the perpendicular's dot with it is about zero, a degenerate test that silently
   picks an arbitrary side. If this drop seeds Apex 2 on the wrong side of the driving shaft while
   the pinion's drop seeds it on the correct side, the coincidence that closes them flips the WHOLE
   figure to the mirror solution: A, C, D, G, H, K, M, N and A′ all land at negative X and the pair
   comes out mirrored about the driving shaft axis. Nothing in the build refuses that figure; the
   end-of-§2 gate is what catches it. `addPerpendicular` against the Driving Gear Shaft Axis, plus
   an aligned dimension of **Driving Gear Pitch Diameter / 2**. Coincide the start with B.
6. Close the two drops with `addCoincident` on their far endpoints. That point is **Apex 2**. At
   Shaft Angle 90° the four points Apex, A, Apex 2, B form a rectangle; at other angles the
   quadrilateral is not rectangular and the lengths of Apex→A and Apex→B adjust so the two drops
   coincide.
7. **`Apex→Apex2`**, the **Pitch Line**, with both ends coincident to their points.
8. **`Apex2→C`** and **`Apex2→D`**, the two dedendum lines: each perpendicular to the Pitch Line
   with an aligned dimension of `Module * 1.25`. **Seed their ends by dot product against the shaft
   axes**, not by "towards or away from the anchor line". Let `u` be either unit perpendicular to
   the Pitch Line: the PINION dedendum direction is the `u` with `u · <unit Apex→A> > 0`, and the
   DRIVING dedendum direction is its negation, which satisfies `(−u) · <unit Apex→B> > 0`. Those two
   dot products are exactly `sin γ_p` and `sin γ_g`, strictly positive for every admitted
   configuration. Seed `C = Apex2 + 1.25·Module·<pinion dedendum direction>` and
   `D = Apex2 + 1.25·Module·<driving dedendum direction>`. ⚠️ These two sites are among the **15 whose
   side is decided by the SEED alone**, with no Fusion constraint able to pin any of them
   (`[BEVEL-F-MIRROR-FIGURE]`): 13 independent binary choices on a 43/31 pair at Shaft Angle 75°
   with Tooth Spacing above zero, so 8192 distinct figures satisfy every constraint, and 10 choices
   with 1024 figures for the default pair, where at 90° the shaft-angle dimension's two senses are
   the same line and the two Tooth Spacing sites do not exist. Never add a Fusion constraint to pin
   a side: every geometric constraint Fusion offers is unsigned or undirected, and the two
   mechanisms that would fix one — `addSymmetry` on C and D against the Pitch Line, and
   `SketchPoint.isFixed` — are both refused, the first as a net redesign the sketch-first waiver
   forbids and the second because the closure already determines every core point. These two are
   where the "C
   collapses onto D" symptom lives, and each is held by its seed alone: the perpendicular fixes the
   direction and the dimension fixes the magnitude, and neither picks a side. Flip the pinion seed
   and C solves exactly onto D; flip the driving seed and D solves onto C. The collapsed figure
   inverts that gear — the toe ends up outside the heel, the revolved frustum is degenerate, and the
   conical end-cut finds no cone face at the toe midpoint.
9. **`Apex→C`** and **`Apex→D`**, the two **Root Axes**, both ends coincident.
10. **`A→E`**, collinear with `Apex→A` and coincident at A, seeded at
    `E = A + <unit Apex→A> · (1.25 · Module · sin γ_p)`. E is the foot of the perpendicular dropped
    from C onto the pinion shaft axis, so `|Apex→E| = R·cos γ_p + 1.25·Module·sin γ_p`. No
    dimension.
11. **`C→E`**, both ends coincident, with `addPerpendicular(cToE, aToE)`.
12. **`B→F`**, collinear with `Apex→B` and coincident at B, seeded at
    `F = B + <unit Apex→B> · (1.25 · Module · sin γ_g)`. No dimension.
13. **`D→F`**, both ends coincident, with `addPerpendicular(dToF, bToF)`.
14. **`E→G`**, collinear with **`A→E`** and never with the Apex→A shaft axis further up the chain
    (`[BEVEL-F-COLLINEAR-CHAIN]`, `[PB-COLLINEAR-CHAIN]`): `addCollinear` carries two point-on-line
    rows, and naming the farther line asserts one of them twice and raises
    `VCS_SKETCH_OVER_CONSTRAINTS`. Seed `G = A + <unit Apex→A> · <resolved Pinion Gear Base Height>`,
    so `|E→G| = <resolved Pinion Gear Base Height> − 1.25·Module·sin γ_p`, which is strictly
    positive because the Minimum Base Height keeps every resolved height above
    `1.25·Module·sin γ` with a 1.05 margin. Coincide E; no dimension.
15. **`C→H`**, collinear with **`Apex2→C`**, coincident at C, seeded at
    `H = Apex2 + <unit Apex2→C> · (<resolved Pinion Gear Base Height> / sin γ_p)`, so
    `|C→H| = <resolved Pinion Gear Base Height> / sin γ_p − 1.25·Module`. No dimension. ⚠️ These two
    seeds pick the side of the pinion base-height offset below, which is unsigned: flip them and G
    sits one base height on the Apex side of A instead of beyond it, H follows, and the pinion's
    heel end folds back inside the figure.
16. **`G→H`**, both ends coincident, plus `addPerpendicular(eToG, gToH)`. That perpendicular is
    REQUIRED in Fusion: `addOffsetDimension` requires its second entity to be a line already
    parallel to the first, and E→G runs along the pinion shaft, so making H→G perpendicular to it
    makes H→G parallel to the A→Apex2 drop. Perpendicular plus offset is two equations for two
    freedoms and nothing is redundant.
17. **`F→I`**, collinear with **`B→F`**, coincident at F, seeded at
    `I = B + <unit Apex→B> · <resolved Driving Gear Base Height>`. No dimension.
18. **`D→J`**, collinear with **`Apex2→D`**, coincident at D, seeded at
    `J = Apex2 + <unit Apex2→D> · (<resolved Driving Gear Base Height> / sin γ_g)`. No dimension.
    ⚠️ The driving pair's seeds carry the whole figure, because "Constrain Point I with centre"
    below hangs everything off I: flip the driving side and the entire lattice drops by twice the
    resolved Driving Gear Base Height, gear and pinion together, with every relative length still
    correct — which is why nothing downstream refuses it.
19. **`I→J`**, both ends coincident, plus `addPerpendicular(fToI, iToJ)` for the same reason as
    G→H.
20. `addOffsetDimension(bDropLine, iToJ, textPoint)` with `.parameter.value` set to the **resolved**
    Driving Gear Base Height — the value AFTER its fallback and cap. The first entity is the
    B→Apex2 drop, NOT the Apex→B shaft axis. J→I is already parallel to that drop by construction,
    so add **no** extra parallel constraint (`[PB-OFFSET-DIM]` — a redundant `addParallel`
    over-constrains and throws). The dimension is UNSIGNED and does not pick which side J→I lands
    on; the I and J seeds are the only thing that does.
21. `addOffsetDimension(aDropLine, gToH, textPoint)` with `.parameter.value` set to the **resolved**
    Pinion Gear Base Height, again with no extra parallel constraint and again unsigned.
22. **`A′→G`**, the pinion hexagon's shaft-axis edge. This line is what CREATES A′ — nothing above
    it does — so seed its start at `A' = Apex + <unit Apex→A> · <the along-shaft coordinate of N>`,
    the foot of the perpendicular from N onto the pinion shaft axis. The front face N→A′ below is
    what PINS A′ to that axis; until then A′ is a free endpoint sitting at its seed. Draw it HERE
    rather than after the front face, so the hexagon's edges are created in the walk order
    `A' -> G -> H -> C -> M -> N` the Profile sketch's first-edge rule depends on. It starts at the
    front face's foot A′, not at A; the two coincide at Toe Extension 0.
23. `addCoincident(pointI, projectedCenter)` — constrain point I with the centre point. This is what
    closes the figure's one remaining freedom.
24. **`G→K`**: from G, away from the Apex along Apex→A. Pin K with **two point-on-line coincidents**
    — `addCoincident(K, line Apex→A)` and `addCoincident(K, the Pinion Dedendum line Apex2→C
    extended)` — rather than `addCollinear` on the connecting lines: by the time K is added G and C
    are already fixed, so a collinear here over-constrains and Fusion errors, while the two
    point-on-line coincidents locate K exactly. Then draw **`C→K`** for reference.
25. **`I→L`** and **`D→L`**, the driving twins, pinned the same way against Apex→B and the Driving
    Dedendum line Apex2→D.
26. **Tooth-centre points K′ and L′.** When Tooth Spacing is 0 — the default — build NOTHING here:
    K′ ≡ K and L′ ≡ L, and the existing `C→K` and `D→L` reference lines are reused, because a
    zero-length dimensioned line is degenerate and one segment gets one line. When Tooth Spacing
    > 0, draw `K→K′` starting at K with its far end seeded at
    `K' = Apex 2 + <unit Apex2→C> · (<the pinion's virtual pitch radius> + Tooth Spacing)`, where
    the **virtual pitch radius is the exact back-cone radius `(PPD / 2) / cos γ_p`** that S09 step 1
    defines and never a radius rebuilt from a tooth count — reading it as a rounded count times half
    a Module puts the seed 0.4203 mm short on the shipped default, which is 420 times the gate's
    tolerance. Pin K′ the same way K is pinned: `addCoincident(start, K)` plus
    `addCoincident(K′, the Pinion Dedendum line extended)`, then an aligned length dimension on this
    line equal to Tooth Spacing. ⚠️ That dimension is unsigned, so the point-on-line pin plus the
    length admit K′ one Tooth Spacing on the C side of K just as readily — the two candidates sit
    `2 × Tooth Spacing` apart — and the seed is the only thing that rules the wrong one out. A
    flipped K′ tightens the mesh by exactly the clearance the input asked to add, and builds a gear
    that looks right. Then draw the tooth-centre reference line **`C→K′`**. Build L′ identically,
    substituting L for K, D for C and the Driving Dedendum line, with the driving gear's virtual
    pitch radius `(DPD / 2) / cos γ_g`, and draw **`D→L′`**.
27. **Resolve the Maximum Face Width and apply it.** All of A, B, C, D, H, J now exist and are
    SOLVED, so read `pointA.geometry`, `pointB.geometry`, `pointC.geometry`, `pointD.geometry`,
    `pointH.geometry` and `pointJ.geometry` (`[PB-SOLVED-GEOMETRY]` — NOT the pre-solve seed
    coordinates, which diverge substantially for asymmetric tooth counts and non-90° shaft angles
    and make the bound too loose on the binding side). The bound is `0.95 *` the smaller of the
    perpendicular distance from A to the line through C and H, and from B to the line through D and
    J. Cap the auto default `Cone Distance / 6` to it, and REJECT a user Face Width above it with a
    message stating the maximum. Compute BOTH distances and take the minimum: the pinion is only
    usually the smaller, binding side, and written with the pinion's diameter the bound is wrong
    whenever the driving gear carries the smaller tooth count — on a Driving 17 / Pinion 31 pair at
    Module 1 the real bound is 3.883 mm against the pinion form's 13.591.
28. **Resolve each gear's Maximum Bore Diameter and apply it, here and nowhere else.** With the Face
    Width resolved the Root Length follows, so this is the only step at which the whole bound can
    resolve. Per gear, with `r` its pitch radius, `γ` its pitch cone angle,
    `|Apex→Ded| = sqrt(R² + (1.25 · Module)²)` and `γ_root = γ − atan(1.25 · Module / R)`:
    `r_heel = r − <resolved Base Height> / tan γ`,
    `r_toe = (|Apex→Ded| − Root Length) · sin γ_root`, and
    `Maximum Bore Diameter = 2 * 0.95 * min(r_heel, r_toe)`. Cap an auto-calculated bore — the input
    is 0, so the value is `min(<this gear's Pitch Diameter / 4>, Maximum Bore Diameter)` — and
    REJECT a user value above the maximum naming it. Skip this entirely when Enable Bore is
    unchecked. A bore past the heel term takes the ENTIRE flat back face and past the toe term the
    whole toe dish; the flat FRONT face is deliberately not protected, because its radius is not on
    the body's outer envelope.
29. **`M→N`**, the pinion toe line. **Seed BOTH ends at their closed-form solved positions, not near
    them** (`[PB-SEED-NEAR]`). Seed M on `Apex→C` at the fraction `1 - <Root Length> / |Apex→C|`
    from the Apex. Then seed N by sliding from that M seed along the `C→H` direction by exactly
    `(<M seed's perpendicular distance from the Pinion Gear Shaft Axis> - <Pinion Gear Toe Radius>)
    / cos γ_p`. ⚠️ A seed that merely lands somewhere plausible is not enough, and a wrong one
    builds the wrong gear rather than failing to converge: N's position is fixed by the toe line
    together with an unsigned LENGTH on the front face, and the toe line meets the Toe Radius on
    BOTH sides of the shaft axis, so the solver takes whichever side the seed starts on. Seeded
    below the axis it converges happily onto the mirror, N comes out on the far side, and the
    revolved hexagon crosses its own axis of revolution — Fusion then aborts the revolve with
    `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`), pointing at the revolve rather than at the seed. Two earlier
    seeding rules do exactly that and must not be reinstated: sliding from the M seed by the Root
    Length, and sliding by the distance from the M seed to A. Measured on the shipped default pair
    at a Toe Extension of 50%, the Root Length slide puts the N seed at −0.27 mm from the shaft
    axis, past it, against a solved N at +5.17 mm; the slide above puts it at 5.17 mm exactly. Do
    not seed M and N just one Face Width away from C and H either.

    Then apply exactly these three constraints, all of which are required:
    `addCoincident(M, Pinion Root Axis)`, so M lies on the Apex→C root axis;
    `addParallel(mToN, cToH)`; and
    `addOffsetDimension(cToH, mToN, textPoint)` with `.parameter.value` set to the Root Length
    re-measured perpendicular to the pitch line, `Root Length * R / |Apex→C|` — an offset dimension
    controls a perpendicular distance, so it carries the root length in that form, and at Toe
    Extension 0 the value is exactly the resolved Face Width. Place the text point in the gap
    between C→H and M→N on the Apex side, for example the midpoint of the M seed and point C. ⚠️
    The toe's side relative to the heel is held by the M seed and by nothing else: the offset is
    unsigned, so a correctly built frame still admits M→N one root length on the FAR side of C→H,
    where the toe lands outside the heel and the revolved frustum is degenerate. The text point does
    not control it either. Let the start be **M** and the end **N**, then draw **`M→C`**.
30. **The pinion front face `N→A′`.** ⚠️ N is NOT pinned to line A→Apex2; earlier revisions did that
    and it fixed N's station at A's. N now rides the Pinion Gear Toe Radius. Draw the line from N to
    A′, then `addCoincident(A′, line Apex→A)` so A′ lies on the shaft axis — A′ is the only toe-end
    point that touches that axis, and it is a FOOT, not a corner; `addPerpendicular(nToAprime, line
    Apex→A)` so the front face stands square to the shaft and the revolve sweeps it into a flat
    annulus; and an aligned distance dimension on the whole line equal to the **resolved Pinion Gear
    Toe Radius**. ⚠️ Pinning N itself to the shaft axis remains forbidden: that would put N on the
    axis of revolution and the later conical split fails with `ASM_API_FAILED` for asymmetric tooth
    counts even though the symmetric 45° case happens to survive. Those three rows plus the offset
    and `addCoincident(M, Pinion Root Axis)` fully constrain M, N and A′ — six freedoms, six
    constraints.
31. **`O→P`** and **`P→B′`**, the driving mirrors, built exactly as the pinion's: seed O on
    `Apex→D` at the fraction `1 - <Root Length> / |Apex→D|`, then P slid from that O seed along
    `D→J` by `(<O seed's perpendicular distance from the Driving Gear Shaft Axis> - <Driving Gear
    Toe Radius>) / cos γ_g`; `addCoincident(O, Driving Root Axis)`, `addParallel(oToP, dToJ)`, and
    `addOffsetDimension(dToJ, oToP, textPoint)` carrying the same re-measured Root Length with its
    text point at `(O_seed + D)/2`. The unsigned-offset warning applies word for word. Then draw
    `O→D`, the front face P→B′ with `addCoincident(B′, line Apex→B)`,
    `addPerpendicular(pToBprime, line Apex→B)` and an aligned dimension equal to the resolved
    Driving Gear Toe Radius, and finally **`B′→I`**.

### The resolved toe values this step needs

- **Toe Radius**, per gear, 0 meaning auto: `this gear's Pitch Radius - Face Width / sin γ`, which
  is that gear's inner toe corner at Toe Extension 0 and is what makes Toe Extension 0 reproduce
  today's profile exactly. A user value must be **strictly below** that gear's Toe Radius Ceiling;
  reject it naming the ceiling.
- **Toe Radius Ceiling**, per gear:
  `(this gear's Pitch Radius - 1.25 * Module * cos γ) * (1 - Face Width / R)`, that gear's OUTER toe
  corner at Toe Extension 0.
- **Toe Limit**, per gear:
  `sqrt(R**2 + (1.25 * Module)**2) - Toe Radius / sin(γ_root)`, with
  `γ_root = γ - atan(1.25 * Module / R)`.
- **Root Length**: at Toe Extension 0 it is `Face Width * |Apex->Ded| / R` with
  `|Apex->Ded| = sqrt(R**2 + (1.25 * Module)**2)`. At a positive Toe Extension add that percentage's
  share of the window, and **Toe Extension 100 stops at 0.99 of the way** from the Toe Extension 0
  root length to the SMALLER of the two gears' Toe Limits, never at the limit itself: at the limit
  the toe face has zero length, the revolved body carries no cone at its toe end, and the conical
  end-cut has no `ConeSurfaceType` face to find. Do not drop that 0.99 factor. Face Width still
  resolves exactly as it always did and still carries its own cap — the Toe Extension ADDS to what
  Face Width resolved.
- **A defaulted Toe Radius can leave no room at all, and that is a real configuration.** On a
  driving gear with a large pitch cone angle the inner toe corner already sits at a LARGER radius
  than the outer one, so the Toe Limit comes out below the Toe Extension 0 root length. Reject a Toe
  Extension above 0 on such a pair, naming the gear and the Toe Radius Ceiling it needs to come
  below; Toe Extension 0 still resolves, so the gear stays buildable. Do **not** silently substitute
  a smaller Toe Radius, which would change the toe end of a gear whose inputs asked for no change.

### The two gates that close this step

Gate the sketch on `isFullyConstrained` and raise naming it if it is false
(`[BEVEL-F-FULL-CONSTRAINT]`). Do **not** reach full constraint by dimensioning a driven length:
full constraint comes from the missing constraint, never from piling on dimensions, and a dimension
on an already-driven length throws `VCS_SKETCH_OVER_CONSTRAINTS` (`[PB-NO-OVERCONSTRAIN]`). A linear
dimension's value is a magnitude plus a direction captured at creation, so a point's side is chosen
by seeding the geometry on the intended side and only `abs(Δ)` may go into `parameter.value`
(`[PB-DIM-VALUE-SEMANTICS]`).

Then gate the solved figure against its own seeds (`[BEVEL-F-SEED-HELD]`): compare every named
point's solved `.geometry` against the closed-form position this step seeded it at, **in the sketch's
own 2-D frame with no world round trip**, at a tolerance of **0.001 mm (1e-4 cm internal)**, and
RAISE naming the first point that has moved, with its solved position and its seeded one. Check them
in creation order — Apex, B, A, Apex 2, C, D, E, F, G, H, I, J, K, K′, M, N, A′, L, L′, O, P, B′ —
so the message names the earliest site that flipped rather than a downstream symptom. **The list is
22 points only when Tooth Spacing is above zero; at Tooth Spacing 0, K′ and L′ are not built at all,
so drop those two and compare 20** — comparing a K′ that was never created is the one way this gate
can raise on a correct figure. This gate is the only measure that catches all 8192 figures the 15
seed-decided sites admit, and **never** treat the revolve's `ASM_WIRE_X_AXIS` as the tripwire:
several of the flips build a valid-looking gear on the wrong side and reach no error at all.

The proof function is `stepGearProfiles`. It pins every seed-decided site with a constraint the
sketch engine signs and records four deviations at their own sites: the collinear chains are
modelled as the single point-on-line row that is not already implied, so the proof cannot tell a
correct collinear from an over-constraining one; the two base-height perpendiculars are omitted,
because the engine's offset already carries the parallelism and adding them leaves the lattice DOF 0
with two redundant constraints; the closure on I is one point-on-line row rather than a
two-row coincident; and the Tooth Spacing and front-face sites take signed constraints in place of
Fusion's unsigned length. **The proof cannot catch a wrong seed** — it seeds at the closed form, so
what it proves is that the constraints solve FROM a correct seed, which is why the gate above has to
exist inside the module. The case table keeps Shaft Angle 30° as a declared refusal: this lattice's
conditioning reads 2.93e-5 there against the engine's 4e-5 floor, which is a property of THIS net
and not of the range the spec states.

<!-- check-step-calls: ignore addVertical addCollinear -->
<!-- proof-run: proofkit.RunParallel(sketchCases, stepGearProfiles) -->

**From:** `spec/bevelgear/instructions.md` L1-18 L106-180 L500-517 L584-712 L1210-1258,
`spec/bevelgear/fusion.md` L69-118 L119-161 L162-198,
`.claude/skills/generate-gear/PLAYBOOK.md` L359-432 L473-493 L591-651 L717-723

## S08 `[PROSE]` {gearLabel} Plane

Once per gear, pinion first. Create a construction plane that includes this gear's tooth-centre
reference line — `C→K′` for the pinion, `D→L′` for the driving gear — perpendicular to the Gear
Profiles sketch plane, through the framework helper `plane_by_angle(designComponent,
toothCenterRefLine, gearProfilesPlane, 90)` from `.solids`. Name it `{gearLabel} Plane`, so
`Pinion Plane` and `Driving Plane`, and write it into that gear's context dict under `toothPlane`.
Pass the sketch line DIRECTLY; never wrap it in `Path.create` (`[PB-CONSTRUCTION-PLANES]`).

**From:** `spec/bevelgear/instructions.md` L713-721 L740-742 L363-418,
`.claude/skills/generate-gear/PLAYBOOK.md` L178-187 L775-786

## S09 `[GO]` {gearLabel} Tooth sketch

Once per gear, on that gear's `{gearLabel} Plane`, in a sketch named `{gearLabel} Tooth` — so
`Pinion Tooth` and `Driving Tooth`.

**Step 1 — the virtual tooth count and the root sink.** Compute this gear's back-cone (Tredgold)
figures from the closed form, never by measuring Apex2→K′:

    virtualPitchRadius_mm = (pitchDia_cm * 10 / 2) / cos(γ)
    virtualTeeth          = 2 * virtualPitchRadius_mm / Module

The `* 10` converts the stashed internal-cm pitch diameter to millimetres; skipping it makes the
count about 10× wrong. `virtualTeeth` is equivalently `this gear's Teeth / cos(γ)`.

**It is a REAL number and is NEVER rounded** — not floored, not ceiled, not cast to an int. The
Tredgold construction puts the equivalent spur gear's pitch radius exactly at the back-cone distance
`r / cos γ`, and `z_v = z / cos γ` is a real number in every published form of it. Rounding it
rebuilds every drawn circle from the rounded count, which draws the tooth smaller than the back cone
places it: on the shipped default the exact virtual pitch radius is 21.9203 mm, a floored count of
43 draws 21.5 mm, and the working addendum falls to 0.5797 mm against a nominal 1.0 module. The real
count reaches the drawer only as the angular half-thickness `π / (2 · toothNumber)`, which at
`z_v = 2 · r_v / Module` gives the standard tooth thickness `π · Module / 2` at the pitch circle;
an integer count at the exact radius gives `π · r_v / round(z_v)` instead, which misses nominal by a
different amount on each member of an unequal pair.

The **root sink** is `0.05 * 2.25 * Module`, and the root circle is drawn one root sink INSIDE the
dedendum corner rather than at it. At the dedendum corner exactly, the tooth's root arc touches the
gear body's root cone only where the arc crosses the tooth's own centreline — the tooth is drawn on
the back-cone plane, so only a point on that centreline rides the cone its own polar radius names,
and the arc's two corners stand outside it, by 0.002 module on the default pair and 0.027 module on
a 4/4 pair. The sink pushes the whole arc inside, so the Combine-Join meets the gear body across the
root rather than along one line.

The four circles the proxy is asked for:

| circle | radius |
|---|---|
| pitch | `virtualPitchRadius` |
| base | `virtualPitchRadius · cos(20°)` |
| tip | `virtualPitchRadius + Module` |
| root | `virtualPitchRadius − 1.25 · Module − rootSink` |

**Step 3 — draw the tooth.** Construct the framework proxy and the borrowed drawer:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth, rootSink_mm=rootSink)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))
```

`VirtualSpurProxy` is imported from `.spurproxy` and the drawer from `.spurgear`; define no local
proxy or value-wrapper class. The proxy's defaults already match bevel — pressure angle 20°, which
is not a bevel dialog input, and 15 involute steps — and it serves each value in internal cm wrapped
in a `.value` carrier. The **180° rotation is delivered through the `draw()` angle argument**, not a
post-hoc move or sketch rotation; the spur generator rotates the whole tooth by that angle, which
relies on its radial flank-to-root pinning so the connecting lines rotate with the tooth. The anchor
point is this gear's tooth-centre point K′ / L′ from the context dict.

**After `draw()` returns, read `proxy._lastToothEmbedded` back** and stash it on the context dict as
`toothEmbedded`. It is an OUTPUT the spur generator writes during `draw()`, and it is not optional
bookkeeping: it is the deterministic selector for the tooth loop's line count.

**Do NOT hard-gate this sketch**: log with `futil.log` if `not toothSketch.isFullyConstrained`, and
never raise (`[PB-LOGGING]` — use `futil.log` for step progress and let the entry point's
try/except handle rollback rather than inventing a silent failure path). The tooth sketches are exempt from the full-constraint gate, and ONLY because the
drawer labels each of the four circles with along-path sketch text, which holds a DOF
(`[PB-TEXT-HOLDS-DOF]`; the three-call along-path shape is `[PB-SKETCH-TEXT]`'s and is the drawer's
to make, not bevel's). ⚠️ That exemption covers the labels and nothing else — never read it as
licence for loose geometry. The reading is also not stable between runs, which is why the
instruction is to log either way. Nothing here calls `settle_sketch_display`; the shared command
wrapper does that once after `generate()` returns, and a generated module must not add its own
(`[PB-SETTLE-DISPLAY]`). `generate` is named here only to say WHEN that happens: the module defines
it for `commands/_gear_command.py` to call, and the module never calls it itself, so it is a mention
rather than a call this step requires.

<!-- check-step-calls: ignore generate -->

The proof function is `stepToothProfile`. It draws both members at the exact virtual count with the
sink applied and requires the four radii, the embedded flag and the loop's curve counts, including
the two connecting lines the non-embedded default pair carries. Its substitution, recorded at the
site: the involute flanks are laid down as fixed samples rather than through the spur family's own
constraint scheme, which that family's own proof owns, and each arc's centre is pinned with one
signed component because the unsigned alternatives leave a mirror twin the ambiguity probe names.

<!-- check-step-calls: ignore draw -->
<!-- proof-run: proofkit.RunParallel(toothCases, stepToothProfile) -->

**From:** `spec/bevelgear/instructions.md` L529-565 L718-744 L1136-1182,
`spec/bevelgear/fusion.md` L31-58, `.claude/skills/generate-gear/PLAYBOOK.md`
L144-204 L494-516 L517-556 L672-692

## S10 `[PROSE]` {gearLabel} Tooth Axis

Once per gear. Create a construction axis through the tooth-centre point, normal to the plane the
tooth profile was drawn on, with `constructionAxes.createInput()` then
`setByTwoPlanes(gearProfilesPlane, helperPlane)` and `constructionAxes.add(axisInput)`
(`[PB-CONSTRUCTION-AXES]`; `setByPerpendicularAtPoint` would need a `BRepFace` that does not exist
here). The helper plane is built `setByDistanceOnPath(<this gear's tooth-centre reference line>,
1.0)` — perpendicular to that line at its far end, the tooth-centre point — and the sketch line is
passed directly, never through `Path.create`. Name the axis `{gearLabel} Tooth Axis` and stash it on
the context dict as `toothAxis`.

Creating this axis in the never-activated Design component is proven to work here, so keep the axis.
No step reads `toothAxis` back; Cleanup hides the axis by entity kind rather than through the dict,
and the key is listed so that a regen that stashes the axis is not read as having invented one.

**From:** `spec/bevelgear/instructions.md` L744-746 L363-418,
`.claude/skills/generate-gear/PLAYBOOK.md` L787-799

## S11 `[PROSE]` {gearLabel} Gear component

Once per gear. Create a new component as a child of the **Bevel Gear** component — the same
component that owns Design, and NOT the user's Parent Component; this intentionally overrides the
looser "child of Parent Component" phrasing so the pair nests inside Bevel Gear — with
`parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`, named `{gearLabel} Gear`, so
`Pinion Gear` and `Driving Gear`. Stash the occurrence on the context dict as `gearOccurrence`.

The actual feature operations still run in the Design component and the finished bodies are
`moveToComponent`'d here at the end (`[PB-NO-CROSS-SIBLING]`); the visible end state is identical.

### The per-gear context dictionary

State is threaded through plain per-gear dicts, `pinionCtx` and `drivingCtx`, built in
`_buildGearProfiles` and passed to `_buildVirtualSpurProfile` and `_createGearBody`. **These key
strings are reproduced surface — never rename a key, split the dict, wrap it in a class, or carry
one of these values under a different shape.** Both gears carry the same **18** keys and no others:

| key | value it carries | type / unit | written | read by |
|---|---|---|---|---|
| `label` | `'Pinion'` or `'Driving'` | `str` | `_buildGearProfiles` | every `{gearLabel}` name and the `gearLabel` argument of `_transformToothBody` / `cut_conical_ends` |
| `teeth` | this gear's Teeth Number | `int` | `_buildGearProfiles` | Pattern quantity; the Meshing rotation angle; `_transformToothBody`'s `teethNumber` |
| `gamma` | this gear's pitch cone angle | `float`, radians | `_buildGearProfiles` | S09 step 1; `_transformToothBody`'s `gamma` |
| `pitchDiameter_cm` | this gear's Pitch Diameter | `float`, internal cm | `_buildGearProfiles` | S09 step 1, the virtual pitch radius |
| `toothCenterPoint` | the tooth-centre point K′ / L′ | `SketchPoint` | `_buildGearProfiles` | S09 step 3, as the drawer's anchor |
| `toothCenterRefLine` | the tooth-centre reference line C→K′ / D→L′ | `SketchLine` | `_buildGearProfiles` | S08; S10's helper plane |
| `hexVertices` | the six profile vertices in draw order — A′, G, H, C, M, N / B′, I, J, D, O, P | `list[SketchPoint]`, length 6 | `_buildGearProfiles` | S12 |
| `toeEdgePoints` | the toe edge's two endpoints, M and N / O and P, in that order | `tuple[SketchPoint, SketchPoint]` | `_buildGearProfiles` | the `toeMid` midpoint; its FIRST element is `toeConeWorld` |
| `heelEdgePoints` | the heel edge's two endpoints, C and H / D and J, in that order | `tuple[SketchPoint, SketchPoint]` | `_buildGearProfiles` | the `heelMid` midpoint; its FIRST element is `heelConeWorld` — the dedendum corner C / D, **NEVER** H / J |
| `boreDiameter_cm` | this gear's Bore Diameter, already resolved AND already bounded | `float`, internal cm | `_buildGearProfiles` | S25 |
| `toothPlane` | the `{gearLabel} Plane` construction plane | `ConstructionPlane` | S08 | `_transformToothBody`'s `parentToothPlane` |
| `toothSketch` | the `{gearLabel} Tooth` sketch | `Sketch` | S09 | the tooth-profile selection |
| `toothEmbedded` | the drawer's `_lastToothEmbedded`, read back off the proxy | `bool` | S09 | the tooth-profile selection, as `wantLines = 0 if toothEmbedded else 2` |
| `toothAxis` | the `{gearLabel} Tooth Axis` construction axis | `ConstructionAxis` | S10 | nothing; it is the one entry with no reader |
| `gearOccurrence` | the `{gearLabel} Gear` occurrence | `Occurrence` | this step | `moveToComponent`'s destination |
| `profileSketch` | the `{gearLabel} Profile` sketch | `Sketch` | S12 | the Revolve's single profile |
| `shaftAxisEdge` | that sketch's first edge, A′→G / B′→I | `SketchLine` | S12 | the Revolve axis; the Pattern axis; the Bore plane's `setByDistanceOnPath`; the Meshing rotation; `_transformToothBody`'s `shaftAxisEdge` |
| `gearBody` | the revolved Gear Body | `BRepBody` | S13 | the Bore extrude's `participantBodies` |

**Six values are used where they are made and NEVER enter the dict**, so a regen that stashes one
has invented an entry: the Root Axis, consumed inside §2 itself; the toe and heel cone points, which
are the first elements of `toeEdgePoints` / `heelEdgePoints`; the shaft-edge point pair, which would
only duplicate the first two `hexVertices`; the virtual tooth number; the root sink; and the meshing
rotation angle.

**From:** `spec/bevelgear/instructions.md` L363-418 L856-872,
`.claude/skills/generate-gear/PLAYBOOK.md` L264-290 L811-820 L829-837

## S12 `[GO]` {gearLabel} Profile sketch

Once per gear, a FRESH sketch on the axial Gear Profiles plane named `{gearLabel} Profile`, so
`Pinion Profile` and `Driving Profile` — one profile sketch per gear, so `sketch.profiles` holds
exactly this one hexagon loop. Do not draw both gears' hexagons in the shared Gear Profiles sketch,
which would leave two identically shaped loops to disambiguate. Stash it on the context dict as
`profileSketch`.

The hexagon vertices, in draw order:

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A' -> G -> H -> C -> M -> N -> A' | B' -> I -> J -> D -> O -> P -> B' |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A'->G | B'->I |
| toe cut edge | M->N | O->P |
| heel cut edge | C->H | D->J |
| §2 shaft construction line (NOT usable as the axis) | Apex->A | Apex->B |

Build it on fixed vertices with the `[PB-PROJECT-NOT-FIXED]` recreate-share-fix recipe, and the
order is load-bearing: recreate the six §2 vertices as new points at their exact world-mapped
positions — `sketch.sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))`, valid because
§2 is fully constrained by now — then draw the closed hexagon in the table's draw order as six
`addByTwoPoints` lines SHARING those points, and only THEN set `isFixed = True` on the lines'
endpoints. Fixing a bare point before it is consumed as a line endpoint does not leave the sketch
fully constrained. `modelToSketchSpace` is a point-transforming METHOD, not a matrix
(`[PB-SPACE-METHODS]`): call it directly on the `Point3D` rather than passing it to `transformBy`.

**The hexagon's first edge is the gear's shaft axis** for the revolve, the pattern, the bore plane
AND the meshing-rotation axis, so it must be fixed well enough to carry a trustworthy world
position: fixed endpoints give that edge a well-defined `worldGeometry` (`[PB-WORLDGEO-CONSTRAINED]`
— a fully unconstrained sketch line has an UNDEFINED `worldGeometry`, which Fusion resolves against
a default frame, silently moving the body onto world XY; this was observed on the driving gear,
while the pinion looked fine only because it never read the edge's `worldGeometry`). Stash that
first edge on the context dict as `shaftAxisEdge`. **Every body operation below uses THIS edge, not
the §2 `Apex→A` / `Apex→B` construction line**: the edge is collinear with the shaft axis but lives
in the same sketch as the profile, which is what Fusion's revolve, pattern and path builders accept;
reusing the §2 construction line fails or misbuilds.

Gate the sketch on `isFullyConstrained` and raise naming it if it is false.

The proof function is `stepProfileSketch`. It recreates each gear's six vertices, shares them,
fixes the endpoints after the lines exist, and requires the first edge to run along the shaft axis
away from the apex and every vertex to stay on one side of the axis of revolution — a profile that
crosses its own axis fails the revolve with `ASM_WIRE_X_AXIS`.

<!-- proof-run: proofkit.RunParallel(sketchCases, stepProfileSketch) -->

**From:** `spec/bevelgear/instructions.md` L856-876 L491-517,
`spec/bevelgear/fusion.md` L19-30,
`.claude/skills/generate-gear/PLAYBOOK.md` L433-440 L458-472 L576-613

## S13 `[GO]` Revolve the Gear Body

Once per gear. The Profile sketch holds exactly one hexagon loop, so take its single profile with
`sketch.profiles.item(0)` and do not filter (`[PB-SINGLE-PROFILE]` — a curve-type filter has
spuriously rejected a valid all-line loop and made the revolve fail with "could not find profile").
Revolve it around the shaft-axis edge: `revolveFeatures.createInput(profile, shaftAxisEdge,
adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`, then
`setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))`, then
`revolveFeatures.add(revolveInput)`. The result is the **Gear Body**, the frustum; stash it on the
context dict as `gearBody`.

Because the toe edge is one edge of the revolved profile, the body already carries the conical face
that edge sweeps, and likewise the heel edge's cone. Those faces are reused as the cutting tools in
S15 — the body is the cone-face source, and searching the lofted Tooth Body for a cone face finds
none.

**The profile must not cross the axis of revolution** (`[PB-REVOLVE]`): if it does, Fusion aborts
with `ASM_WIRE_X_AXIS`. That is what the Maximum Face Width cap in S07 and the strictly positive Toe
Radius are for; reproduce both exactly.

The proof function is `stepRevolveGearBody` and its assertion is `assertRevolveGearBody`. It
substitutes nothing: decad's own Revolve returns the whole frustum as one body, its volume is
compared against Pappus on that same hexagon with a relative tolerance because the published bound
is Approximate, and its **five faces** are matched by surface kind and by their own readings rather
than by the order the face selector returns — a flat heel disc, a flat toe disc of the Toe Radius,
and three cone faces whose areas are those of the frusta the edges C→H, M→C and N→M sweep. Each
cone's half-angle is read off its own face, which a `decad.Cone` publishes, instead of deriving a
tangent from two cap radii and a height: the heel cone and the toe dish read `90° − γ` and the root
cone reads this gear's root cone angle. This step now costs nothing. The gear body is also the one
body no boolean consumes, because every boolean measured with a Revolve operand verified Suspect.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepRevolveGearBody, assertRevolveGearBody) -->

**From:** `spec/bevelgear/instructions.md` L877-880 L925-980,
`.claude/skills/generate-gear/PLAYBOOK.md` L494-500 L708-723

## S14 `[GO]` Loft the Tooth Body

Once per gear. Loft the **§2 Apex sketch point** — the `centerToApex.endSketchPoint` from the Gear
Profiles sketch, the degenerate point-section — to this gear's `{gearLabel} Tooth` profile:
`loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`, then
`loftSections.add(apexSketchPoint)` and `loftSections.add(toothProfile)` in that order, then
`loftFeatures.add(loftInput)`. The result is the **Tooth Body**. Use the §2 Apex SKETCH point
directly and do NOT create a construction point for it (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`: a
`ConstructionPoint` needs an active component and the Design component is never activated, while a
`SketchPoint` works as a loft point-section).

Loft sections are added in loft order and a section may be a single `SketchPoint` for a degenerate
end (`[PB-LOFT]`).

**Select the tooth cross-section with
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)`** from `.utilities`,
where **`wantLines = 0 if toothEmbedded else 2`** — the line count is DETERMINED by the embedded
flag read back off the proxy in S09, never guessed and never accepted either way. ⚠️ Do not accept
"0 **or** 2 lines": for a given gear only one of those is the real tooth, and an unrelated loop —
an inter-tooth or annular region between the drawn circles — can carry the same 2 NURBS and 2 arcs
with the other line count, and selecting it makes this loft fail with
`RuntimeError ... ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`, because the impostor loop cannot form a loft
tool body. `embedded` means the tip, root and flanks meet with no connecting lines, four curves;
non-embedded means two connecting lines, six curves. Profiles are matched by curve count and type,
never by index, and a curve's type is read as `curve.geometry.curveType` against
`adsk.core.Curve3DTypes` (`[PB-PROFILE-MATCH]`).

The proof function is `stepLoftTooth` and its assertion is `assertLoftTooth`. It substitutes a
shrunken section for the degenerate apex point and nothing else: the tooth plane is NOT substituted,
the proof building the real back-cone plane tilted out of the axis-perpendicular by γ and taking the
Apex's perpendicular distance to the section as `sK · cos γ` rather than `sK`, which at Tooth
Spacing 0 is the Pitch Cone Distance itself. The cost is the point section — the loft's degenerate
end is not built, and what the volume and the reach to the virtual tip radius prove is the taper it
has to produce.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepLoftTooth, assertLoftTooth) -->

**From:** `spec/bevelgear/instructions.md` L471-490 L877-880 L981-985,
`.claude/skills/generate-gear/PLAYBOOK.md` L144-158 L724-728 L791-799

## S15 `[GO]` Conical end cuts (the straight tooth)

Once per gear, and this is the ψ = 0 path of the tooth-body hook. `_transformToothBody`'s first line
is the gate `if self._spiralAngle_rad <= 0: return cut_conical_ends(...)`, so a straight bevel is
byte-for-byte the prior behaviour and every spiral input is ignored.

Trim the Tooth Body to a flush band with the framework helper:
`cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)` from
`.solids`. Do **not** re-implement the cut machinery — `apply_conical_cut`, `select_keeper`,
`find_cone_faces_by_midpoint` and `surface_distance` are the framework's and are named here only as
the behaviour the helper encodes.

**Two distinct bodies are involved and must not be conflated:** the cutting TOOLS are
`ConeSurfaceType` faces of the **Gear Body**, the revolved-hexagon frustum, and the TARGET being
split is the **Tooth Body**, the loft. The lofted tooth has no cone faces, so searching it finds
none.

The helper implements the pinned behaviour: the **toe cut first**, its cone face identified by the
toe edge's world **MIDPOINT** best-first across the frustum's cone faces (`[PB-FACE-BY-MIDPOINT]` —
endpoints sit near the apex singularity, where `getParameterAtPoint` returns no result, so an
endpoint distance cannot see the right face and an unevaluable distance is "try last" rather than
"disqualified"), each candidate tried as the actual split tool and the first that splits kept;
keeper selection after each cut, dropping apex-containing pieces and keeping the largest
(`[PB-REMOVE-PIECES]`); each candidate is passed with `isSplittingToolExtended=True` so the tool
surface extends to fully bisect the target, and a tool that does not intersect raises
`SPLIT_TARGET_TOOL_NOT_INTERSECT`, whose message text may be localized (`[PB-SPLIT-BODY]`); then the **heel cut on the keeper alone**, removing the apex tip first being
what makes it deterministically two split features for every gear ratio. A heel cone that does not
intersect the keeper at all — common on ratio pairs — is raised by the helper as the typed
`solids.NonIntersectError` and caught, and the keeper is returned whole. Every failure is
self-diagnosing with the per-face distance and error history (`[PB-SELF-DIAGNOSING]`).

**Caller obligations, which stay in the generator:** pass `toeMid` = the toe edge's world midpoint,
`(M_world + N_world)/2` for the pinion and `(O_world + P_world)/2` for the driving gear; `heelMid` =
the heel edge's world midpoint, `(C_world + H_world)/2` and `(D_world + J_world)/2`; `apexWorld` =
the §2 Apex sketch point's world geometry; and `gearBody` = the revolved frustum. The toe cut MUST
split — its failure propagates and crashes the build, which is correct, since an uncut tooth is
unusable — and only the heel cut is lenient, and only through the typed error.

The proof function is `stepConicalEndCut` and its assertion is `assertConicalEndCut`. It PERFORMS
the toe cut on the tooth and substitutes for the heel cut. Both half-angles are taken off the
revolved gear body's own cone faces, which are the faces the `ConeSurfaceType` search finds at this
step; the step then asserts that each cut lands where the flush band requires — the toe cone meets
the gear body's own root cone at M and the heel cone at C — that each cone's half-angle equals
`90° − γ`, and that each cone crosses the tooth's tip inboard of where it crosses its root. It does
NOT assert merely that the two crossings differ, and carries no message for that case: both cutting
cones have their apex on the shaft axis, so a cone of wall slope `k` and apex station `a` crosses a
tooth surface of slope `m` at `a · k / (m + k)`, which differs for the tooth's two surfaces whenever
the tooth has height, so such an assertion passes on any figure this spec can build. The toe cone is
built as the SOLID inside the cone — an n-gon loft, because the cut consumes it — with its apex on
the shaft axis at the station M puts it, and the keeper and the offcut are asserted to add back to
the whole tooth. One typed refusal is tolerated and any other error fails: an empty toe trim is the
same condition the module raises as `NonIntersectError`. **No heel cut is performed**, because its
cone is tangent to the tooth plane — the dedendum corner and the tooth centre both sit on this
gear's back-cone dedendum line, so the plane contains a generator of the heel cone and the two touch
along the tooth's own centreline — and decad refuses exactly that contact. The cost is the heel
split. The proof also records what these readings cannot tell apart: a cone and a TILTED PLANE read
identically in everything this step measures, since in the axial section they are the same line;
what makes the face conical is that it is a surface of revolution about the shaft axis, and nothing
here measures the azimuthal crossing of the tool the toe cut consumes.

<!-- check-step-calls: ignore apply_conical_cut select_keeper find_cone_faces_by_midpoint surface_distance -->
<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepConicalEndCut, assertConicalEndCut) -->

**From:** `spec/bevelgear/instructions.md` L419-470 L881-908 L986-1032 L1103-1135,
`.claude/skills/generate-gear/PLAYBOOK.md` L159-177 L733-774

## S16 `[PROSE]` {gear} Cone Element sketch and {gear} Trace Plane

Everything from here to S21 runs **only when ψ > 0**, inside `_transformToothBody`, on the freshly
lofted uncut apex→heel Tooth Body, before pattern, combine and bore.

First build the frame (§3a step A) from geometry already constructed for this gear:
`axisDir` is the shaft axis direction from the two WORLD endpoints of `shaftAxisEdge`, normalized;
`coneVec` is `normalize(heelConeWorld − apexWorld)`, the dedendum root cone element Apex→C for the
pinion and Apex→D for the driving gear; `v = axisDir × coneVec`, normalized, the circumferential
direction; and `tpNormal = coneVec × v`, normalized, which completes the frame and which **nothing
consumes** — step D removed the projection that once used it, so it is computed and left unread.
A point's **cone distance** is its distance from the apex measured along the cone
element, `(p − apex) · coneVec`.

Every one of those world quantities is read in WORLD space, from `worldGeometry` rather than
`geometry`: mixing a sketch-local curve with a world axis is valid Python that silently returns
wrong numbers, and a wrong spiral-twist magnitude is exactly the failure it produces
(`[PB-WORLD-FRAME]`).

⚠️ **Before building `coneVec`, fix a swapped toe and heel.** If
`apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid` with `heelMid` AND
`toeConeWorld` with `heelConeWorld`, then build `coneVec` from the corrected heel. A negative span
silently inverts the entire spiral frame — the cutter-arc direction, the slice direction and the
per-segment twist all flip — and the gear comes out completely wrong with no error.

**The caller hand-off `_createGearBody` builds and passes positionally as
`toeMid, heelMid, toeConeWorld, heelConeWorld`, which is the single biggest spiral-regen hazard:**

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

`toeMid` is the world MIDPOINT of the TOE edge and `heelMid` the world midpoint of the HEEL edge —
two different edges. ⚠️ Do NOT pass the two endpoints of a single edge as the two midpoints: M and N
both sit at the toe, so the span collapses to about zero or goes negative and the spiral inverts.
⚠️ `heelConeWorld` is the dedendum corner C / D and **never** H / J, which lie on the Apex2→C /
Apex2→D dedendum line one Module beyond C / D, off the root cone element, and skew `coneVec`.

Then, in a sketch named `{gear} Cone Element` on the **axial Gear Profiles plane**, draw a
construction line from the apex to `apex + R_heel·coneVec`. Make the tangent plane by rotating that
axial plane 90° about the cone-element line —
`plane_by_angle(designComponent, coneElementLine, axialPlane, 90)` — and name it
`{gear} Trace Plane`.

**Coordinates, and this rule governs both this sketch and the trace sketch in S17.** The world
`Point3D`s these two sketches are built from are passed **directly** into the sketch calls, where
they are consumed as SKETCH-space input, with **no `modelToSketchSpace` conversion applied**, even
though the points really are model-space coordinates. That is deliberate and it is harmless for a
reason that is not the obvious one: the trace sketch is construction and reference only and no
downstream feature ever consumes it, because the twist is computed analytically in S19. The
cone-element line IS consumed, by `plane_by_angle`, so an unconverted line does place the Trace
Plane somewhere other than the true tangent plane — and that still reaches no feature, because the
only thing built on the Trace Plane is the inspection-only trace sketch and the chain ends there.
**If a later revision ever makes any feature consume the trace sketch or the Trace Plane, this
shortcut stops being safe and both sketches need `modelToSketchSpace` on every point.**

Both sketches are exempt from the full-constraint gate and must not be gated.

**From:** `spec/bevelgear/instructions.md` L518-528 L747-779 L791-802,
`spec/bevelgear/spiral-tooth-trace.md` L1-30 L32-66 L253-270, `spec/bevelgear/fusion.md` L59-68

## S17 `[GO]` {gear} 2D Tooth Trace sketch

A sketch named `{gear} 2D Tooth Trace` on the Trace Plane, holding the genuine cutter arc.

Work in the tangent-plane 2-D frame with the origin at the apex, **x = coneVec** so a point's x is
its cone distance, and **y = v**, circumferential. From `toeMid` and `heelMid`, after the swap guard:
`R_toe` is the cone distance of `toeMid`, `R_heel` that of `heelMid`,
`R_mean = ½(R_toe + R_heel)` and `span = R_heel − R_toe`, the face width, now positive. These are the only quantities the rest of the
spiral build needs.

The cutter radius `r_c` is the Cutter Radius input if non-zero, **else `R_mean`**, the auto default.
The hand sign is `handSign = +1` for `Right` else `−1`, then **negated for the pinion**, because the
pair meshes with opposite hands. The cutter-circle centre is

```
Cx = R_mean − r_c · sin ψ
Cy = handSign · r_c · cos ψ
```

⚠️ **The hand sign goes on the `cos` / `Cy` term, NOT the `sin` / `Cx` term.** Opposite hands mirror
the cutter centre across the cone element `y = 0`, which flips `Cy`; putting `handSign` on `Cx`
mirrors about `x = R_mean` instead, a different curve that gives the two gears unequal twist, where
for equal teeth the two traces must come out as exact mirror images. This was a real bug.

The trace's endpoints are circle–circle intersections taken a hair PAST the face so the kept arc
reaches cleanly past the end trims: `toe2d = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)`
and `heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)`, the framework helper from
`.solids`, with `R_lo = R_toe − 0.06·span` and `R_hi = R_heel + 0.06·span`. The helper intersects
the apex circle of radius R with the cutter circle and keeps the solution nearest `(R_mean, 0)`, the
branch the mean point sits on.

Draw, with `tanW(px, py) = combine_point(apexWorld, px, coneVec, py, v)` from `.solids` mapping 2-D
coordinates to world:

- the **cutter circle**, centre at `tanW(Cx, Cy)`, radius `r_c`, `isConstruction`, with its centre
  pinned by `centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]` — a circle's centre is free
  even when created at the origin, and a coincident to the sketch origin has thrown
  `VCS_SKETCH_SOLVING_FAILED` on exactly this kind of plane) and a diameter dimension of `2·r_c`;
- the **trace arc**, a three-point arc through `tanW(toe2d)`, `tanW(R_mean, 0)` and `tanW(heel2d)`,
  with its centre coincident to the cutter circle's centre and a **radius dimension of `r_c`**, so
  it is the genuine cutter circle and not a look-alike spline.

Text points per `[PB-RADIAL-DIM]`, off-centre and on or near the curve, because a text point at the
curve's centre is rejected: use the mean point `tanW(R_mean, 0)` for the arc's radius dimension and
a point on the cutter circle such as `tanW(Cx + r_c, Cy)` for the circle's diameter dimension.

This sketch is **deliberately left with free DOF** — the arc's endpoints are pinned by the
three-point construction, not by endpoint dimensions, since dimensioning them over-constrains the
solve against the cone-element plane — so it is exempt from the full-constraint gate and must not be
gated.

**There is no 3-D projection.** The 2-D cutter arc is the only trace geometry needed, so there is no
`projectToSurface`, no root-cone-face search and no 3-D trace sketch. Earlier versions projected the
arc onto the root cone and measured the trace azimuth there; that projection is fragile — for
unequal-ratio pairs the arc wraps around the cone and comes back as multiple disjoint fragments, the
measured azimuth collapses to a fraction of the true sweep, and the pinion comes out grossly
under-twisted. Do not reintroduce it.

The proof function is `stepSpiralTrace`. It proves the invariants `spiral-tooth-trace.md` §9 lists:
the loci are apex-centred, the arc's radius is `r_c` everywhere, the centre is `r_c` from the mean
point so the circle passes through it, the angle between the arc's tangent and the element at the
mean point is ψ, swapping the hand mirrors the construction across the cone element and changes
nothing else, and each end sits on its own apex circle. Its ψ = 0 cases assert the straight path:
at ψ = 0 the centre sits straight north of the mean point, so the arc is tangent to the element
there and only there, and its ends still subtend a residual sweep at the apex — which no built tooth
carries, because the hook's ψ = 0 gate returns before any of this runs. Its substitution, recorded at
the site: this harness gates every sketch, so the proof pins the three through-points and holds the
arc's centre with one signed component rather than reproducing the free-DOF sketch Fusion authors.

<!-- check-step-calls: ignore projectToSurface -->
<!-- proof-run: proofkit.RunParallel(spiralCases, stepSpiralTrace) -->

**From:** `spec/bevelgear/instructions.md` L768-803,
`spec/bevelgear/spiral-tooth-trace.md` L67-252,
`.claude/skills/generate-gear/PLAYBOOK.md` L451-457 L652-658

## S18 `[GO]` Slice the tooth into slabs and drop the apex scrap

Split the uncut apex→heel Tooth Body into cross-section slabs with
`slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)` from `.solids`,
where `parentToothPlane` is this gear's `{gearLabel} Plane` from S08 and
`offsets = [sign·(k+1)·span/6 for k in 0…7]` — a **fixed** scheme of exactly **8** planes, a count
that is not user-configurable.

⚠️ **The slice planes are NOT perpendicular to the cone element.** The parent plane carries the
tooth-centre line, which is the back-cone line and so perpendicular to the Pitch Line, so the parent
plane's normal runs along the PITCH element while `coneVec` is the ROOT element; the two differ by
the dedendum angle `atan(1.25 · Module / R)`, equivalently `atan(2.5 · sin γ / N)`, in which Module
cancels, so it depends only on the tooth counts and the Shaft Angle and is the same for both
members — 3.26° on the default 31/31 pair at Shaft Angle 90°, growing as the tooth counts fall. The
parallel family is what the build REQUIRES rather than what it happens to use: the helper offsets the
parent plane with `setByOffset`, which produces parallel planes; the sign test below reads the parent
plane's own normal, which is meaningful only for that plane's own offsets; and the tooth is lofted to
the profile drawn in the parent plane, so the heel-most slab's heel face IS the parent plane and a
consistent family has to contain it. ⚠️ A build that follows "perpendicular to the cone element"
instead is **wrong and silent**: it tilts every cut face by the dedendum angle and nothing in the
pipeline fails, because parallel planes cut a cone in similar sections whatever their orientation.
What moves is the geometry — on the default pair at Module 4 a face corner lands `1.125 · Module ·
tan δ_f` = 0.26 mm along the cone from where the parallel cut puts it, and the step-G twist keyed
across one face mismatches by up to 0.0078 rad.

The **offset sign is chosen per gear** so the planes move toward the apex: the parent plane's normal
points opposite ways for the two gears, so pick `sign` such that `sign·normal` points apex-ward, by
testing `(apex − planeOrigin)·normal`. Where the eight land: the first sits `span/6` inside the HEEL
and none of them lies past it — the parent plane is already the heel end, so there is no heel
overshoot to give — the sixth lands at the toe, and the last two sit `span/6` and `2·span/6` PAST
the toe. The two segments beyond the toe are what S21's toe cone trims away. The first sits a hair
more than `span/6` inside the heel and the sixth a fraction of a millimetre inside the toe, because
`R_heel` and `R_toe` are read at the two edge midpoints rather than on the root element.

⚠️ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in one
piece, the offset sign was wrong or the parent plane sits outside the tooth's span: **retry the
whole cut once with the opposite sign**. If it is still one piece, `raise` a clear self-diagnosing
error naming the gear, the final piece count, `span` and the sign tried (`[PB-EMPTY-RESULT]`,
`[PB-SELF-DIAGNOSING]`). Do NOT return an unsliced result: the next step then drops that one piece as
the apex scrap, leaving `segments` empty, and the crown later crashes with
`ValueError: max() iterable argument is empty` far from the cause.

**Order and drop the scrap.** Sort the segments by the `distAlong` of their centroid, read from
`physicalProperties.centerOfMass`. The first, apex-most piece is the long apex-side scrap below the
toe: **remove it**, and keep the rest as the working `segments`. Drop it by re-slicing the list
FIRST and deleting it after — `segments = segments[1:]` before
`removeFeatures.add(scrap)` — so the list never holds a deleted body. After dropping the scrap,
`segments` must be **non-empty**; if it is empty the slice failed, so raise a clear error rather than
proceeding into the twist and the crown, which both assume at least one segment.

The proof function is `stepSpiralSlice` and its assertion is `assertSpiralSlice`. It builds the
segments directly between the consecutive section planes rather than by splitting one lofted tooth,
because this evaluator performs no split, and lays each apart along the shaft axis, because two
slabs sharing a cut face report a pair contact decad refuses to classify; neither changes a reading.
The cost is the split itself, so the retry-once-then-raise guard reaches Fusion untested. **The proof
also cannot catch the plane family**, for the reason above: it builds its own slabs from the offsets
this spec fixes and never reads the plane the module constructs.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralSolidCases, stepSpiralSlice, assertSpiralSlice) -->

**From:** `spec/bevelgear/instructions.md` L804-823 L1099-1102,
`.claude/skills/generate-gear/PLAYBOOK.md` L173-177 L433-440 L762-769

## S19 `[GO]` Twist the slabs

Rotate each segment about the **shaft axis** — `axisDir` through `apexWorld` — so the tooth follows
the trace, **centred on `R_mean` so the mid-face section stays unrotated**. That section then meshes
exactly like the straight tooth, which is what the pinion's zero mesh nudge depends on.

The total toe→heel twist comes from the conjugate crown-gear generation law, computed
**analytically, with no projection and no curve sampling**:

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`phi_crown` is the angle the cutter arc's toe and heel endpoints subtend at the apex in the flat 2-D
crown frame — exactly the `toe2d` and `heel2d` pairs from S17. ⚠️ **`gamma` is this gear's PITCH
cone angle**, `self._gamma_p` or `self._gamma_g`, and NOT `acos(coneVec·axisDir)`, which is the root
cone angle, smaller by the dedendum angle — 24.7° against a pitch 28.7° for a 17-tooth pinion
meshing a 31-tooth gear — and yields a twist about 1.15× too large. ⚠️ The two members of a meshing
pair legitimately get DIFFERENT twists: same cutter and same ψ, but γ differs, so `1/sin γ` differs,
about 2.08× for a 17-tooth pinion against about 1.14× for a 31-tooth gear. That is why equal-teeth
pairs always meshed while ratio pairs failed under any method that gets `1/sin γ` wrong.

Each segment's rotation is a **linear share keyed to the cone distance of its HEEL FACE**, the
segment's farthest-along-the-element face, which is the exact section the later loft samples:

```
ang = −handSign · total · (R_mean − R_heelFace(seg)) / span
```

⚠️ **Key the twist on the heel-face cone distance, NOT on the segment's centroid.** The loft samples
each segment's heel face, so that face is what must land at the right azimuth; centroid-keying
leaves the loft's mid-face section rotated by half a segment and the mid-faces overlap.

⚠️ **Define a slab's heel face precisely: the face whose centroid has the GREATEST
cone distance, searched across ALL of the slab's faces with NO surface-type filter.**
Its toe-side face is the least-centroid one. Do not restrict the search to `PlaneSurfaceType` or any
other type: a sliced slab is bounded by a mix of the two planar cut faces and ruled side faces, and
a type filter can pick the wrong face or miss the cut face, which makes the loft in S21 fail with
`ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this same all-faces-by-centroid rule everywhere
a slab end face is needed — the twist key here, the crown base in S20 and the loft sections in S21.

Apply the rotation as a free move with a matrix built by
`adsk.core.Matrix3D.setToRotation(ang, axisVector, originPoint)` (`[PB-MOVE-ROTATE]`), through
`moveFeatures.createInput2(bodyCollection)` and `defineAsFreeMove(matrix)`.

The proof function is `stepSpiralTwist` and its assertion is `assertSpiralTwist`. It builds each
slab already rotated, which is the same transform, and requires each one's azimuth to match its own
share of the total; it also requires the root cone angle to be strictly below the pitch cone angle,
the total to be `|phi_crown| / sin γ` on the pitch angle, a face at `R_mean` to twist by zero, and
the two ends to take half the total each in opposite senses.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralSolidCases, stepSpiralTwist, assertSpiralTwist) -->

**From:** `spec/bevelgear/instructions.md` L824-838,
`spec/bevelgear/spiral-tooth-trace.md` L186-219,
`.claude/skills/generate-gear/PLAYBOOK.md` L433-440 L800-810

## S20 `[GO]` Lengthwise crown

Crown the tooth by scaling each segment **except the outermost, heel one** down by a **monotonic**
factor — full at the heel, growing smoothly toward the toe. For each segment compute its
**heel-distance fraction** `u = (R_heel − R_heelFace) / span`, where `R_heelFace` is found by the
same all-faces-by-centroid rule as S19 but **RECOMPUTED HERE, AFTER the twist has moved the slabs**;
do not reuse pre-twist values. `u` runs 0 at the held-full heel to 1 at the toe, and PAST 1 on the
two segments beyond the toe: those two read about `7/6` before the twist and a little more after the
recompute at the default ψ on the default pair. **Do not treat `8/6` = 1.33 as a ceiling** — that
figure is the last plane's offset, and that plane is a toe face rather than any segment's heel face,
so nothing ever evaluates `u` there; the recomputed `u` climbs with the Mean Spiral Angle and is
measured at 1.351 at ψ = 55°, which the range admits. Nothing reads an upper bound on `u`; the crown
factor stays positive for every value it takes, and what the slab count rests on is the structural
fact that the last cut plane is never a heel face. The heel segment reads a few hundredths rather
than exactly 0, because `R_heel` is read at the heel edge's midpoint rather than on the parent
plane; that segment is skipped anyway. **"Outermost (heel) segment" is the one with the GREATEST
post-twist heel-face `distAlong`** — sort by that and skip the last. Then:

```
factor = 1 − _CROWN_PER_RAD · (|total| / 2) · u
```

`|total|/2` is the per-end peak twist magnitude, so the maximum relief, now at the toe, keeps the
magnitude the old per-end peak had, just relocated. Relief therefore grows monotonically from the
full heel to the toe, so slab heights stay strictly ordered heel→toe and the natural cone taper is
never reversed. If a computed factor comes out **≤ 0**, `raise` a self-diagnosing error naming the
gear, the segment's `u` and the factor; never scale by a non-positive factor.
**`_CROWN_PER_RAD` is a tunable class constant with default `0.5`** — 0 disables the crown; set it
to 0.5 and do not leave it unset.

⚠️ **Do NOT key the relief on `|ang|`, the twist magnitude.** That is symmetric about the mid-face,
maximal at BOTH ends, so because the heel slab is held full the slab just inside the heel becomes
the most-relieved one and dips below both its neighbours, reversing the heel→toe taper. This was the
observed bug: the heel-adjacent slab came out at factor 0.932 while the next slab inward was 0.972,
taller. Key on the monotonic `u`, never on `|ang|`.

Three further gotchas:

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). `scaleFeatures` is the ONE exception to
   never-activate: it needs the Design occurrence as the active edit target, so call
   `designOccurrence.activate()` before the crown scales and restore afterwards, in a `finally`,
   with `design.activateRootComponent()`. ⚠️ Do **NOT** write `design.rootComponent.activate()` or
   `someComponent.activate()`: a `Component` has no `activate` method and raises `AttributeError`.
   Only an `Occurrence` has it, and the root is re-activated through `Design.activateRootComponent`.
2. **Skip the outermost (heel) segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone in S21 trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, NOT its centroid, or the crowned tooth lifts off
   the gear base.** `scaleFeatures` shrinks uniformly toward the base point, so a base point at the
   heel-face centroid — mid tooth-height — pulls the tooth's root edge upward by
   `(1−factor)·(½ tooth height)`, the tooth no longer seats on the gear body's root cone, and the
   Combine-Join leaves a visible gap for ratio pairs. Put the base point on the root instead: of the
   heel face's vertices, each `.geometry` a world `Point3D`, take the **two with the smallest
   perpendicular distance to the shaft axis** — the line through `apexWorld` along `axisDir`, the
   distance being `|(p−apex) − ((p−apex)·axisDir)·axisDir|` — which are the two root corners, the
   tip corners being farthest from the axis, and place the base sketch point at their **midpoint**,
   mapped into the heel-face sketch with `modelToSketchSpace`. The heel face is a planar cut, so
   that midpoint lies on it. A uniform scale about a point keeps every line through that point
   invariant, so anchoring on the root keeps the root edge on the seating cone while the tip is
   relieved progressively toward the toe, which is exactly the lengthwise crown intended.

Apply the scale through `scaleFeatures.createInput(inputEntities, basePoint, scaleFactorValueInput)`
and `scaleFeatures.add(scaleInput)`.

The proof function is `stepSpiralCrown` and its assertion is `assertSpiralCrown`. This evaluator has
no scale operation, so each segment is BUILT at its crowned size about the same base point, which is
exactly what a uniform scale produces; the cost is that the operation itself and its one activation
are not shown. The case computes BOTH keys and requires the `u`-keyed factors and the measured slab
heights to be monotonic while the `|ang|`-keyed ones notch, so the case separates the two.

<!-- check-step-calls: ignore rootComponent -->
<!-- proof-run: proofkit3d.RunSolidParallel(spiralSolidCases, stepSpiralCrown, assertSpiralCrown) -->

**From:** `spec/bevelgear/instructions.md` L839-852,
`spec/bevelgear/fusion.md` L199-206,
`.claude/skills/generate-gear/PLAYBOOK.md` L576-590 L791-799

## S21 `[GO]` Loft the spiral tooth and trim it flush

⚠️ **Re-sort the segments by their heel-face cone distance HERE, AFTER the twist and the crown — do
NOT reuse the pre-twist slice order.** The twist rotates each slab about the shaft axis, and for
high-twist unequal-ratio pairs that rotation changes the slabs' along-cone order enough to reorder
adjacent slabs; lofting in the stale order assembles the cross-sections out of sequence, the crowned
tooth comes out distorted, and the two gears interfere. For equal or low-twist pairs the two orders
coincide, which is why equal-teeth gears mesh even with the stale order while ratio pairs distort —
this is the single thing that makes a ratio pair like 31/17 fail while 31/31 looks fine. So sort the segment
indices by the cone distance of each segment's own heel-face centroid **now**, and loft in that
order.

Loft a NewBody through, in that order: first the **toe-most segment's apex-side (toe-facing) face**
— the toe segment is `order[0]`, and its toe face is added first to push the loft past the toe cone
so the toe trim bites — then the **heel-facing face of every segment, iterated in `order`**, each
being that segment's farthest-along-the-element face by post-twist centroid, with the last reaching
past the heel cone. Name the resulting body **`{gear} Spiral Tooth`**. Then remove the segment
scaffolding with `removeFeatures.add(segment)`; the loft has captured their faces.

**Flush trim.** Return
`cut_conical_ends(designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)` —
the same toe-then-heel two-cone trim the straight tooth takes in S15 — so the curved tooth's ends sit
flush on the gear base. The toe and heel mesh phasing is handled outside this hook, by the mesh
rotate step S26; the pinion's extra phase is 0 by default because the mid-face section is unrotated
and already meshes.

The proof function is `stepSpiralLoft` and its assertion is `assertSpiralLoft`. This evaluator lofts
two sections at a time, so the chain is built as consecutive pairwise lofts, laid apart along the
shaft axis; the cost is that the single `{gear} Spiral Tooth` body made from all the sections at once
is not built. The assertion requires the order to be the post-twist one, every consecutive pair to
run outward along the element, and the toe-most segment's toe face to reach past its own heel face.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralSolidCases, stepSpiralLoft, assertSpiralLoft) -->

**From:** `spec/bevelgear/instructions.md` L419-446 L853-856,
`.claude/skills/generate-gear/PLAYBOOK.md` L704-707 L724-732 L770-774

## S22 `[GO]` Circular pattern

Once per gear. Circular-pattern the remaining tooth piece around the **shaft-axis edge**, the same
in-sketch profile edge the revolve used and never the §2 construction line. Copy the tooth body into
a fresh `adsk.core.ObjectCollection.create()`, then
`circularPatternFeatures.createInput(bodyCollection, shaftAxisEdge)`, and pin all three inputs
explicitly rather than relying on Fusion's defaults staying equal to them
(`[PB-CIRCULAR-PATTERN]`): `quantity = adsk.core.ValueInput.createByReal(<this gear's Teeth
Number>)`, `totalAngle = adsk.core.ValueInput.createByString('360 deg')` and
`isSymmetric = False`. Then `circularPatternFeatures.add(patternInput)`.

Although the pitch diameter shrinks from the heel toward the apex, the ANGULAR spacing around the
shaft axis stays constant at `360° / N` for the entire face width: the radial taper is already
produced by the loft from the Apex to the heel-end tooth profile, so the pattern just rotates that
one tapered tooth into N evenly spaced copies.

The pattern's `bodies` collection already includes the seed plus the copies, so do not re-add the
seed, and copy them into a fresh `ObjectCollection` before handing them to the Combine, which
rejects a `BRepBodies` (`[PB-PATTERN-BODIES]`).

The proof function is `stepCircularPattern` and its assertion is `assertCircularPattern`, and this
is **the one bevel step that stays serial**, on `proofkit3d.RunSolid`. The pattern increment retires
the seed tooth, so the seed cannot be measured after the step runs: its azimuth, radius, height and
volume are read during the build and handed to the assertion through package-level variables, and
that hand-off leaves the case. Two cases sharing one set of seed readings overwrite each other, and
it is not a hazard that announces itself — the two gear sides differ enough in volume that the
overwrite was caught when it happened, but a pair of cases whose seeds measured alike would have
passed on each other's numbers. No other bevel step carries a reading from its build into its
assertion; a step that acquires one moves to the serial runner in the same change. The step also
lays each copy apart along the shaft axis after its rotation, which leaves the azimuth, the distance
from the axis, the volume and the span unchanged — every quantity it measures — because left where
the pattern puts them the neighbours report an undecided pair from three teeth up. The cost is the
arrangement: the proof does not show the N copies standing clear of one another around one axis.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCircularPattern, assertCircularPattern) -->

**From:** `spec/bevelgear/instructions.md` L909-910 L1183-1209,
`.claude/skills/generate-gear/PLAYBOOK.md` L693-707

## S23 `[GO]` Combine-Join

Once per gear. Join all patterned tooth pieces with the Gear Body in a **single** Combine-Join, the
Gear Body as the target and the patterned tooth bodies as the tools:
`combineFeatures.createInput(gearBody, toolBodyCollection)` with
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`, then
`combineFeatures.add(combineInput)`.

The proof function is `stepCombineJoin` and its assertion is `assertCombineJoin`. It performs **no
join**, because the two operands are not in one frame: the proof builds the tooth on a section
perpendicular to the build axis at the Pitch Cone Distance from the apex, scaled about the apex —
the Tredgold mapping — while the gear body is written about the shaft axis, and the tooth's own
seating on that body is derived nowhere in the proof. The two do not meet when put in one document,
and neither sign of the rotation that relates the two planes seats them. **The engine is not what
blocks this join**: given operands that do overlap it performs the union, returns one lump and
publishes a volume bound of 8e-15 of the value. What is missing is the tooth's real back-cone
placement, and deriving it is its own change. So the operands are laid apart and the join's two
consequences are asserted from their own measured geometry: a join leaves ONE lump when the tooth's
root is below the body's root cone — seated, not floating — and the joined body reaches further out
than the frustum when the tooth's tip stands proud of it, both read at the toe, the middle and the
heel of the band the join would cover. ⚠️ The **root arc's OUTERMOST point** is what is read, never
the tooth's centreline: the centreline sits inside both root corners, so a reading taken there
passes a tooth whose corners float outside the cone, which is exactly the defect the root sink
exists to remove. The generated module draws its root circle one root sink inside the dedendum
corner, and the proof applies that same sink — it is one figure, not a proof-only offset. **The cost
is the stitch**: the proof cannot show the evaluator making one boundary out of two.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCombineJoin, assertCombineJoin) -->

**From:** `spec/bevelgear/instructions.md` L911-912 L1033-1084,
`.claude/skills/generate-gear/PLAYBOOK.md` L672-692 L693-703

## S24 `[GO]` {gearLabel} Bore sketch

Once per gear, and **skip this step entirely when Enable Bore is unchecked**; no bore is cut on
either gear and the per-gear bore diameter inputs are ignored.

Build the bore plane normal to the shaft at its start: `constructionPlanes.createInput()` then
`setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))` and
`constructionPlanes.add(planeInput)`. Pass the in-sketch edge directly, not the §2 construction
line, and never through `Path.create`.

In a sketch named `{gearLabel} Bore` — so `Pinion Bore` and `Driving Bore` — sketch the bore circle
centred at the sketch origin, since the plane is rooted at the shaft start and the origin is
therefore on the axis. **Fix the circle's centre and add a diameter dimension** set to this gear's
resolved bore diameter: `circle.centerSketchPoint.isFixed = True` plus
`addDiameterDimension(circle, textPoint)` with `.parameter.value` assigned (`[PB-CIRCLE-CENTER]` — a
circle's centre is free even when created at (0,0,0), and `addCoincident` to the sketch origin has
been observed to throw `VCS_SKETCH_SOLVING_FAILED` on exactly this kind of
`setByDistanceOnPath` plane; `isFixed` on the centre is the reliable pin). The text point must be
off-centre, on or near the curve (`[PB-RADIAL-DIM]`). Gate the sketch on `isFullyConstrained`.

The bore diameter is the value already resolved and already bounded in S07 — the user's value if
non-zero, otherwise this gear's Pitch Diameter / 4, in either case capped by that gear's Maximum
Bore Diameter. Take that number from the context dict's `boreDiameter_cm`; do **not** re-derive it
here, or the cap is lost and the bore deletes the body's back face.

The proof function is `stepBoreSketch`. It requires the diameter reaching the sketch to be strictly
positive and within that gear's Maximum Bore Diameter, and its bore-disabled case builds nothing.

<!-- proof-run: proofkit.RunParallel(sketchCases, stepBoreSketch) -->

**From:** `spec/bevelgear/instructions.md` L100-141 L913-914,
`.claude/skills/generate-gear/PLAYBOOK.md` L441-457 L652-658 L775-786

## S25 `[GO]` Bore extrude-cut

Once per gear, skipped with S24 when Enable Bore is unchecked. Cut a cylindrical through bore
through the Gear Body along the shaft axis:
`extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)`, then
`setSymmetricExtent(adsk.core.ValueInput.createByReal(2 * <Cone Distance>), False)` — the second
argument `isFullLength=False` means the distance is the half-length PER SIDE, and `2 × Cone
Distance` is generously past any face width (`[PB-THROUGH-CUT]`); do not pass a third taper
argument. Restrict the cut with `extrudeInput.participantBodies = [gearBody]`, then
`extrudeFeatures.add(extrudeInput)`.

The proof function is `stepBoreCut` and its assertion is `assertBoreCut`. It builds the tool as a
real extrude, which a symmetric extent produces as a prism, and performs the cut. The tool is
asserted first from its own geometry — its two ends sit exactly `2 * Cone Distance` either side of
the shaft edge's start and both clear the frustum, which is what makes it a through cut — and then
the pierced body's volume is asserted against the band's own closed form less the prism the bore
removes over that height. **The target is the heel cone band, lofted for this step**: it is the
section of the gear body the bore passes through, and it is a Loft, which is the form a boolean
operand takes here; the revolved gear body cannot be the target, for the reason S13 gives. **What
this does not reach is the rest of the body** — the bore is pierced through the band that stands for
the heel section, not through the whole frustum.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepBoreCut, assertBoreCut) -->

**From:** `spec/bevelgear/instructions.md` L913-914 L1085-1098,
`.claude/skills/generate-gear/PLAYBOOK.md` L729-732

## S26 `[GO]` Meshing rotation

**Driving gear only**, and here, in the Design component, BEFORE the body is moved out. Rotate the
driving body by `180° / Driving Gear Teeth Number` — half a tooth pitch — about its shaft axis with
`rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)` from `.solids`, which
takes the rotation axis and origin from the B→I profile edge's **world** endpoints
(`[PB-MOVE-ROTATE]`). Both gears are patterned from a starting tooth in the axial plane, so without
this offset a driving tooth and a pinion tooth would both sit at the axial-plane crossing and
visually collide; with it, a driving valley sits where the pinion tooth crosses, giving the
interlocked meshing look.

This runs in Design before `moveToComponent` because a construction axis cannot be added in the
moved-out gear component, so the rotation must use the edge's world geometry while still in Design.

The pinion additionally gets `_pinionMeshPhase(pinionTeeth)`, its extra rotation about its own shaft
axis in radians, `_PINION_MESH_PHASE_TEETH · 2π / pinionTeeth`, which is **0 by default**. **A zero
angle is a no-op, not a move**: `Matrix3D.setToRotation(0, axis, origin)` builds the identity and
Fusion refuses to move a body by it with `RuntimeError: 3 : invalid transform`. Any caller that
computes an angle can legitimately arrive at zero, so `rotate_body_about_edge` absorbs it and
returns early rather than making each call site guard it.

The proof function is `stepMeshingRotation` and its assertion is `assertMeshingRotation`. It rotates
the driving side by half a tooth pitch and applies no transform at all on the pinion side, then
requires the built body's centroid azimuth to match the closed form for that side.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepMeshingRotation, assertMeshingRotation) -->

**From:** `spec/bevelgear/instructions.md` L447-490 L915-918,
`.claude/skills/generate-gear/PLAYBOOK.md` L800-810

## S27 `[PROSE]` Move the bodies into the gear component

Once per gear, at the end of `_createGearBody`. Relocate the finished body with
`body.moveToComponent(gearOccurrence)`, the occurrence stashed on the context dict in S11.
`moveToComponent` preserves world position and needs no activation, which is what lets every feature
operation run in the single Design component while the visible end state has each gear's body in its
own `{gearLabel} Gear` component (`[PB-NO-CROSS-SIBLING]`).

Pinion is built first and driving second: `_buildGearProfiles` runs profile and body INTERLEAVED per
gear — pinion profile, pinion body, driving profile, driving body — and not both profiles followed
by both bodies.

**From:** `spec/bevelgear/instructions.md` L419-446 L518-528 L856-872,
`.claude/skills/generate-gear/PLAYBOOK.md` L811-837 L869-874

## S28 `[PROSE]` Cleanup

Call `hide_construction_geometry(bevelComponent)` from `.solids` as the last step of `generate()`.
`generate` is named here to place this step in the method, not to require a call: the module defines
`generate` as its entry point and the shared command wrapper is what invokes it, so nothing in the
module calls it.

<!-- check-step-calls: ignore generate -->
It recursively walks the Bevel Gear component tree, dedupes by `entityToken` (`[PB-TREE-CLEANUP]`), and hides every
sketch, construction plane and construction axis by setting `isLightBulbOn = False`. Construction
planes and axes are **not** hidden by `isVisible` (`[PB-HIDE-AFTER-USE]`, `[BEVEL-F-CLEANUP]`), so
do not cross the two properties; do not re-implement the walk. There is no sketch-only mode and no
per-mode guard — bevel always builds solids. Leave only the two finished gear bodies visible.

The driving gear's half-tooth-pitch meshing rotation is performed earlier, at S26, in the Design
component before the body is moved out; it is not a cleanup step.

`deleteComponent()` is the error rollback the command entry point calls on an exception: it calls
`deleteMe()` on `self.bevelOccurrence`. Bevel registers no user parameters, so there are none to
clean up.

<!-- check-step-calls: ignore deleteMe -->

**From:** `spec/bevelgear/instructions.md` L419-446 L500-517 L919-924,
`spec/bevelgear/fusion.md` L199-212, `.claude/skills/generate-gear/PLAYBOOK.md` L657-671
L835-837
