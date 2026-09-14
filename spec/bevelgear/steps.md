# Bevel Gear — compiled step list

The proof for these steps is `proof/bevelgear/geometry_test.go`, `proof/bevelgear/sketches_test.go`,
`proof/bevelgear/solids_test.go`, `proof/bevelgear/spiral_test.go`,
`proof/bevelgear/drawing_geometry_test.go` and the generated registration file
`proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `4ab5559ec7c59f3bf87cecadc65eeeb75d2b5955` |
| `spec/bevelgear/fusion.md` | `754729fc9a33ad9dbf93816e0430e4b94d82e05b` |
| `spec/bevelgear/spiral-tooth-trace.md` | `c9ec08561ced7975aa0ed9ad6a330186259c0d08` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `abb1123b5910f10c5c96c1ad936a38691ea3e7fb` |

## S1 `[PROSE]` Dialog inputs — `BevelGearCommandInputsConfigurator.configure`

<!-- check-step-calls: ignore configure handle_input_changed _updateSpiralInputVisibility -->
<!-- check-compile: ignore configure handle_input_changed _updateSpiralInputVisibility get_design -->
`configure`, `handle_input_changed` and `_updateSpiralInputVisibility` are methods the module
DEFINES rather than calls: `commands/bevelgear/entry.py` binds the first two by name and the third
is the private helper the second delegates to. They are named here so the surface is fixed, not
because the module calls them.

`BevelGearCommandInputsConfigurator` is a plain class — no base — with
`@classmethod def configure(cls, cmd)`, `@classmethod def handle_input_changed(cls, args)` and the
private `@classmethod def _updateSpiralInputVisibility(cls, inputs)`.

**Add the inputs to `cmd.commandInputs` in exactly this order.** Target Plane comes first so it wins
Fusion's auto-focus, which ignores a later `hasFocus` (`[PB-AUTOFOCUS-FIRST]`); Center Point follows
so the user flows from plane to point; Parent Component is third because it is already pre-selected
to the root component.

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
| 18 | Toe Extension | `toeExtension` | `addValueInput` | `''` | `createByReal(0)` | — |
| 19 | Driving Gear Toe Radius | `drivingToeRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 20 | Pinion Gear Toe Radius | `pinionToeRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |

There are **20** dialog inputs and **20 `INPUT_ID_*`** module constants, holding the table's id
strings in row order, named exactly: `INPUT_ID_PLANE`, `INPUT_ID_CENTER_POINT`, `INPUT_ID_PARENT`,
`INPUT_ID_MODULE`, `INPUT_ID_SHAFT_ANGLE`, `INPUT_ID_DRIVING_TEETH`, `INPUT_ID_PINION_TEETH`,
`INPUT_ID_DRIVING_BASE_HEIGHT`, `INPUT_ID_PINION_BASE_HEIGHT`, `INPUT_ID_BORE_ENABLE`,
`INPUT_ID_DRIVING_BORE`, `INPUT_ID_PINION_BORE`, `INPUT_ID_FACE_WIDTH`, `INPUT_ID_TOOTH_SPACING`,
`INPUT_ID_SPIRAL_ANGLE`, `INPUT_ID_HAND`, `INPUT_ID_CUTTER_RADIUS`, `INPUT_ID_TOE_EXTENSION`,
`INPUT_ID_DRIVING_TOE_RADIUS`, `INPUT_ID_PINION_TOE_RADIUS`. The only other module-level constants
are `_HAND_RIGHT = 'Right'` and `_HAND_LEFT = 'Left'`. There are **no** `PARAM_*` names, because
bevel registers no Fusion user parameters at all (`[PB-PRECOMPUTED-MODE]`).

**The three selection inputs.** Each is `cmd.commandInputs.addSelectionInput(id, name, commandPrompt)`
— the third argument is the tooltip string from the table and is reproduced surface — then
`selectionInput.addSelectionFilter(...)` once per filter, then
`selectionInput.setSelectionLimits(1, 1)`. Filters are the named constants, never quoted literals
(`[PB-SELECTION-FILTER-ENUM]`, and `[PB-SELECTION-DECL]` makes the filter set and the limits the
spec's own declaration):

- Target Plane: `adsk.core.SelectionCommandInput.ConstructionPlanes` and
  `adsk.core.SelectionCommandInput.PlanarFaces`.
- Center Point: `adsk.core.SelectionCommandInput.ConstructionPoints` and
  `adsk.core.SelectionCommandInput.SketchPoints`.
- Parent Component: `adsk.core.SelectionCommandInput.Occurrences` and
  `adsk.core.SelectionCommandInput.RootComponents`, pre-selected with
  `parentInput.addSelection(get_design().rootComponent)`.

**The value inputs.** `cmd.commandInputs.addValueInput(id, name, unitType, initialValue)` with the
defaults from the table. A `createByReal` default is in Fusion INTERNAL units whatever the unit
string says (`[PB-DIALOG-DEFAULT-UNITS]`), which is why every `mm` default is written
`adsk.core.ValueInput.createByReal(to_cm(0))` and the two `deg` defaults are written
`adsk.core.ValueInput.createByString('90 deg')` and
`adsk.core.ValueInput.createByString('35 deg')` so the expression engine parses them. `module`,
`drivingTeeth`, `pinionTeeth` and `toeExtension` take the unit string `''` and plain
`adsk.core.ValueInput.createByReal(...)` defaults of `1`, `31`, `31` and `0`.

**Enable Bore** is `cmd.commandInputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)`
— a check box, initially checked.

**Hand of Spiral** is
`cmd.commandInputs.addDropDownCommandInput(INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)`,
then `handInput.listItems.add(_HAND_RIGHT, True)` and `handInput.listItems.add(_HAND_LEFT, False)`.

**Conditional visibility — the spiral-only inputs show only when ψ > 0.** Hand of Spiral and Cutter
Radius are hidden whenever Mean Spiral Angle ψ = 0 and shown when ψ > 0; Mean Spiral Angle itself is
the controller and is always visible. There is no declarative show-if in the Fusion API, so this is
realised with `commandInput.isVisible`:

- `_updateSpiralInputVisibility(cls, inputs)` reads the `spiralAngle` input's **`.expression`** and
  evaluates it with `design.unitsManager.evaluateExpression(spiral.expression, 'rad')` — internal
  **radians**, and it does NOT read the input's `.value` — then sets
  `inputs.itemById(INPUT_ID_HAND).isVisible` and `inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible`
  to `(value > 0)`. Guard it: if any of the three inputs is `None` return early, and wrap the
  evaluation in `try/except`, because a half-typed expression can raise mid-edit; on failure leave
  both inputs **shown**.
- `configure()` calls `cls._updateSpiralInputVisibility(inputs)` as its **last step**, so the initial
  state is right for the default ψ = 35°.
- `handle_input_changed(cls, args)` is one line: `cls._updateSpiralInputVisibility(args.inputs)`,
  recomputed on every input change.

`isVisible` only hides the dialog row. The input still exists, S2 reads it normally, and the ψ = 0
build ignores Hand and Cutter anyway, so hiding cannot affect generation.

**From:** `spec/bevelgear/instructions.md` L27-34, L100-104, L140-144, L145-225;
`.claude/skills/generate-gear/PLAYBOOK.md` L128-143, L346-348, L548-559.

## S2 `[PROSE]` Read and validate every input — `_readInputs`

<!-- check-step-calls: ignore _readInputs generate deleteComponent -->
<!-- check-compile: ignore _readInputs generate deleteComponent get_selection get_boolean to_cm int round min max sqrt acos atan2 degrees radians cos sin tan hypot -->
`_readInputs`, `generate` and `deleteComponent` are methods the module DEFINES; `get_selection`,
`get_boolean` and `to_cm` are framework helpers and `int`, `round`, `min`, `max` and the `math`
functions are Python's, so none of them is an Autodesk name for the API-reality check.

`BevelGearGenerator` is a plain class — it does **not** subclass `base.Generator` and uses **no**
`GenerationContext`. `__init__(self, design)` stores `self.design` and `self.bevelOccurrence = None`.
`generate(inputs)` runs the whole build; `deleteComponent()` is the error rollback the entry point
calls on an exception. From `base.py` import only `get_selection` and `get_boolean`; the
`Generator` / `ParamNamePrefix` / `ComponentCleaner` machinery is unused. Imports are explicit, never
`import *`.

**Read every input first, in one pass, before anything creates an occurrence.**
`_readInputs(inputs)` returns the 7-tuple
`(parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth, shaftAngle_deg)`
and stashes the rest on `self` as `self._drivingBaseHeight_cm`, `self._pinionBaseHeight_cm`,
`self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`, `self._faceWidth_cm`,
`self._toothSpacing_cm`, `self._spiralAngle_rad`, `self._hand`, `self._cutterRadius_cm`.

**How each input is read.** Selections with `get_selection(inputs, id)`; the checkbox with
`get_boolean(inputs, INPUT_ID_BORE_ENABLE)` — never `get_value`, which reads `.expression` and
raises `AttributeError` on a `BoolValueCommandInput` (`[PB-INPUT-READ]`); the dropdown with
`inputs.itemById(INPUT_ID_HAND).selectedItem`, taking `.name` and defaulting to `_HAND_RIGHT` when
none is selected; every numeric and angle input with
`design.unitsManager.evaluateExpression(input.expression, units)` using `''`, `'mm'` or `'deg'` per
the S1 table (`[PB-EVAL-EXPRESSION]`).

**Units — critical.** `evaluateExpression` always returns Fusion internal units — cm for length,
radians for angle — whatever the unit string says. So the `mm` inputs (both base heights, both bore
diameters, Face Width, Tooth Spacing, both Toe Radii) and the `deg` inputs (Shaft Angle, Mean Spiral
Angle) come back already internal: use them as-is and do **not** `to_cm` them again. **`Module` is
read with unit `''`, so it comes back as a raw number that means MILLIMETRES**, and every length
derived from it must be `to_cm`-converted before it touches geometry: Pitch Diameter =
`to_cm(Module * teeth)`, Cone Distance, the dedendum `to_cm(1.25 * Module)`, the module-length
extensions, and the default Face Width. Mixing a raw-mm Module-derived length with an already-cm
`mm` input makes the gear come out about ten times off. `toeExtension` is a plain unitless
percentage and needs no conversion. Both teeth inputs are coerced with `int(round(...))` before
validation.

**Names and values every check below uses.** Write them out; none of them can be looked up
elsewhere.

- `Driving Gear Pitch Diameter (DPD) = Module * Driving Gear Teeth Number`
- `Pinion Gear Pitch Diameter (PPD) = Module * Pinion Gear Teeth Number`
- `Cone Distance = sqrt((Module * Driving Gear Teeth Number)**2 + (Module * Pinion Gear Teeth Number)**2)`
  — the diagonal of the two pitch diameters, depending on the tooth counts only and never on the
  Shaft Angle. **It is NOT the Pitch Cone Distance `R`.** `R = (PPD / 2) / sin(γ_p)` is the real
  apex-to-heel length along the pitch cone. The two coincide as `Cone Distance = 2 * R` exactly when
  the Shaft Angle is 90°, for any tooth counts, and diverge everywhere else: an equal 31/31 pair at
  30° has `Cone Distance = 43.84 mm` against `R = 59.89 mm`, and at 140° `R = 16.49 mm`.
- the closed-form cone angles: `tan γ_p = sin Σ * PPD / (DPD + PPD * cos Σ)`, `γ_g = Σ - γ_p`.

**Validation, in this order.** The order is load-bearing.

1. `module > 0`; both tooth counts `>= 3` as an absolute floor; every base height, bore diameter,
   Face Width, Tooth Spacing, Toe Radius and Cutter Radius non-negative; Toe Extension in `[0, 100]`;
   Mean Spiral Angle in `[0, 60)` degrees after `math.degrees(...)`.
2. **Shaft Angle**: at least **30°** and below the **Maximum Shaft Angle**, converted to degrees
   before the check. The Maximum Shaft Angle is the cone-angle limit capped at 150°, the cone-angle
   half EXCLUSIVE and the 150° half inclusive. Both cone angles stay below 90° exactly while
   `cos(Shaft Angle) > -min(DPD, PPD) / max(DPD, PPD)`, so reject a Shaft Angle **at or above**
   `degrees(acos(-smaller / larger))` and name the computed limit in the message. A 31/17 pair gives
   `acos(-17/31) = 123.26°`; equal tooth counts give `acos(-1) = 180°`, which is no constraint. A
   pitch cone angle reaching 90° turns that gear's cone inside out — `R * cos γ` passes through zero
   and changes sign — so the limit is a hard singularity, not a style choice. This check needs both
   tooth counts, so run it after both are read and coerced.
3. Compute `γ_p` and `γ_g` once, then check each gear's **Minimum Teeth** floor,
   `Driving/Pinion Gear Teeth Number >= 5.27 * cos γ` with that gear's own `γ`, on top of the
   blanket `teeth >= 3`, naming the computed floor. The constant is
   `2 * (1.05 * 1.25 / 0.95 + 1.25) = 5.2632`, **rounded UP to 5.27** so the published floor stays at
   or above the exact crossing; do not round it down. At Shaft Angle 90° the floor is 3.72, i.e. four
   teeth — measured, an equal 4-tooth pair solves and a 3-tooth pair still fails on the heel edge.
4. Resolve each gear's base height between its own two bounds, per gear, with that gear's own `r`
   (its Pitch Diameter / 2) and `γ`:
   - `Minimum Base Height = 1.05 * 1.25 * Module * sin γ`
   - `Maximum Base Height = 0.95 * (r - 1.25 * Module * cos γ) * tan γ`

   The base height is the offset dimension measured from **Apex 2's plane**, not from the dedendum
   point, so H (resp. J) reaches the shaft axis at `r * tan γ`; the bound above sits
   `1.25 * Module * sin γ` below that crossing and is deliberately conservative. Past the true
   crossing the hexagonal frustum profile has crossed its own axis of revolution and the revolve
   fails with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`). Apply both bounds in both directions: raise a
   fallback below the minimum, cap a fallback above the maximum, and reject a USER value outside
   either end with a message stating the bound it broke. The driving fallback is
   `Module * Driving Gear Teeth Number / 8`; the pinion fallback is the **resolved** driving height
   times `Pinion Gear Teeth Number / Driving Gear Teeth Number`, then held to the PINION's own
   bounds. Running the Minimum Teeth check first is what makes the window non-empty here.

Both bounds are closed-form and need no solved geometry, so resolve them during input validation.
The Face Width cap and the Toe Radius checks cannot be resolved yet — they need the §2 sketch — and
are applied in S6.

`generate(inputs)` then resolves the pitch diameters and bore diameters in Python (internal cm),
builds the component tree, and runs the geometry steps in order: S4 → S6 → per gear (S7, S8, S9,
S10, S11, S12, S13 or the spiral chain, S21, S22, S23, S24, S25) pinion first and driving second,
then the cleanup. Bevel registers no user parameters, so nothing creates an occurrence until every
selection is already read, and keeping that order is what keeps it so.

**From:** `spec/bevelgear/instructions.md` L35-98, L106-114, L226-265, L267-292, L294-315, L411-421;
`.claude/skills/generate-gear/PLAYBOOK.md` L103-118, L708-714, L843-858.

## S3 `[PROSE]` Build the component tree

Create the occurrences directly with `parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`
and name each through `occurrence.component.name` (`[PB-OCCURRENCE-TREE]`). Bevel does not use
`getOccurrence`, `addParameter`, `parameterName` or `createSketchObject`.

The tree is: the user's Parent Component → a component named `Bevel Gear` → a component named
`Design`. `self.bevelOccurrence` holds the top occurrence for rollback, and
`self.designOccurrence` / `self.designComponent` / `self.bevelComponent` hold the inner tree. Each
gear's own component (`Pinion Gear`, `Driving Gear`) is created later, in S21's own section, as a
child of **`Bevel Gear`** — not of the user's Parent Component.

**Never activate any occurrence** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`). The Anchor
sketch is created on the user's EXTERNAL root-owned target plane, and an activated occurrence
resolves that external plane in its own local frame, collapsing the whole build onto world XY no
matter what the user picked. All features run in the single Design component, so no cross-sibling
reference is ever needed (`[PB-NO-CROSS-SIBLING]`). The single exception is the spiral crown's
`scaleFeatures` step in S19, which activates the Design occurrence and restores the root afterwards.

This step creates occurrences and no measurable geometry, so no proof function realises it.

**From:** `spec/bevelgear/instructions.md` L19-23, L455-463, L701-713;
`spec/bevelgear/fusion.md` L155-160; `.claude/skills/generate-gear/PLAYBOOK.md` L808-825.

## S4 `[GO]` Anchor sketch — `stepAnchorSketch`

<!-- proof-run: proofkit.RunParallel(anchorCases, stepAnchorSketch) -->

Start the sketch **directly on the user-selected target plane**, whether the selection is a
`ConstructionPlane` or a `PlanarFace`, with `designComponent.sketches.add(targetPlane)`, and name it
`Anchor`. Do not re-derive or offset the plane (`[PB-USE-SELECTED-PLANE]`): a construction plane
built inside the sub-component from a face in another component resolves in the sub-component's own
frame and silently loses the selected plane's world orientation.

Mark the centre by projecting the user's Center Point in with `sketch.project(centerPoint)`.

<!-- check-step-calls: ignore project2 -->
`project2` is named only to forbid substituting it, so the module must NOT call it.

**Write the call as `sketch.project(entity)` and do not substitute `project2`.** The compiled Fusion
API reference declares `project2(entities, isLinked)` and no `project`, so every gate in this repo
reports the call as unverified; that report is expected and is not a defect to fix here. The two are
not interchangeable in any case — `project2` takes a list and returns a list — and only a Fusion
session can settle whether `project` exists at runtime.

Draw a line through the projected centre with
`sketch.sketchCurves.sketchLines.addByTwoPoints(pointOne, pointTwo)` — the curve collections live
under `sketch.sketchCurves` and never on the sketch directly (`[PB-SKETCHCURVES]`) — seeding its two endpoints at
**exactly ±0.5 cm from the projected centre** along the sketch-local X, so the seeded length is
10 mm. Then:

- `sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)` — the intersection, which
  pins the centre onto the line;
- `sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)` — the centre bisects the
  line. Use **both**, not the midpoint alone;
- `sketch.sketchDimensions.addDistanceDimension(anchorLine.startSketchPoint, anchorLine.endSketchPoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
  **without assigning `.parameter.value`** — the dimension simply locks the length at the seeded
  10 mm, and the value is arbitrary because this is only a reference line;
- `sketch.geometricConstraints.addHorizontal(anchorLine)` — sketch-local, per
  `[PB-REFLINE-DIRECTION]`. A world-axis lock would mis-orient the line on a tilted target plane.

The anchor line's absolute direction is arbitrary — §2 derives every direction relative to it — but
it must not be a free degree of freedom; with midpoint, length and Horizontal the line has zero.
**Stash the projected-centre `SketchPoint`** on `self._anchorCenterPoint` so S6 re-projects THIS
point rather than the raw user selection. After all constraints, gate the sketch: raise, naming the
sketch, if `sketch.isFullyConstrained` is false (`[BEVEL-F-FULL-CONSTRAINT]`,
`[PB-FULL-CONSTRAINT]`).

### What the proof establishes

`stepAnchorSketch` builds this sketch across three centre positions — on the sketch origin and well
off it, because nothing in the dialog requires the user to put the centre anywhere in particular —
and gates it at DOF 0 with nothing redundant. Beside that it reads back the two facts the line is
for: its length is the seeded 10 mm and its midpoint is the projected centre.

Two deviations are recorded in the proof file next to the geometry they belong to. The
`addCoincident` above is **omitted there**: the sketch engine's midpoint carries the point-on-line
row already, so writing both is a third row for two freedoms and the sketch comes back with a
redundant constraint; only a Fusion session settles whether that engine absorbs the pair the way it
absorbs an implied collinear row. And the aligned dimension crosses over as the SIGNED horizontal
distance, which is the playbook's own mapping for it (`[PB-DIM-VALUE-SEMANTICS]`): written unsigned,
the line has two discrete solutions — its endpoints swapped — that the seed alone resolves in
Fusion.

**From:** `spec/bevelgear/instructions.md` L465-469, L479-481;
`spec/bevelgear/fusion.md` L21-30; `.claude/skills/generate-gear/PLAYBOOK.md` L230-242, L432-441,
L500-507, L829-839.

## S5 `[PROSE]` Gear Profiles Plane

Create the plane the §2 figure is drawn on:
`planeInput = designComponent.constructionPlanes.createInput()`, then
`planeInput.setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)`,
then `designComponent.constructionPlanes.add(planeInput)`. Name it `Gear Profiles Plane`.

The angle is 90° because by default the plane would lie flush to the anchor line's own plane and the
figure has to stand perpendicular to it. **Build it off the original `targetPlane`** as the
reference (`[PB-USE-SELECTED-PLANE]`) — this is the second and last place the target plane's
orientation reaches the bodies, and substituting a different plane here also collapses the gear onto
XY. Pass the `SketchLine` DIRECTLY to `setByAngle`; never wrap it in `Path.create` first
(`[PB-CONSTRUCTION-PLANES]`).

This step creates a construction plane and no measurable geometry, so no proof function realises it.

**From:** `spec/bevelgear/instructions.md` L473;
`.claude/skills/generate-gear/PLAYBOOK.md` L766-777, L829-839.

## S6 `[GO]` Gear Profiles sketch — the §2 lattice — `stepGearProfiles`

<!-- proof-run: proofkit.RunParallel(latticeCases, stepGearProfiles) -->

Create the sketch on the Gear Profiles Plane with
`designComponent.sketches.add(gearProfilesPlane)` and name it `Gear Profiles`. Stash it as
`self._gpSketch`.

**Three rules govern every line in this sketch.**

**Everything is construction geometry.** Set `line.isConstruction = True` on every §2 line — the
lattice lines, the toe lines M→N and O→P, and the short reference and connector lines M→C, N→A,
O→D, P→B, A→G, B→I, C→K/K′, D→L/L′ alike. The solid features consume only the per-gear Profile
sketches of S10, never a §2 curve.

**Every length dimension is aligned.**
`sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
for every one of them: the PPD/2 and DPD/2 drops to Apex 2, the two `Module * 1.25` dedendum lines,
the Tooth Spacing dimension on the K′ / L′ lines, and the Toe Radius dimension on the two front faces
N→A′ and P→B′. This figure has no axis-aligned line in it — the shaft axes sit at the Shaft Angle to
each other and the whole lattice tilts with the target plane — so a Horizontal or Vertical
orientation would dimension a line's projection onto a sketch axis instead of its length. The offset
dimensions are a different call, `sketch.sketchDimensions.addOffsetDimension(line, entityTwo, textPoint)`,
which takes no orientation.

Every constraint name below is exact and the easy misspellings are real `AttributeError`s:
`addCollinear` carries a double "l", and `addCoincident`, `addPerpendicular`, `addParallel`,
`addMidPoint`, `addOffsetDimension` and `addDiameterDimension` are copied rather than inferred
(`[PB-API-SPELLING]`).

**Every line is built in the COINCIDENT style** (`[BEVEL-F-COINCIDENT-STYLE]`): create it from raw
`adsk.core.Point3D.create(x, y, 0)` coordinates and pin each endpoint that meets an existing point
with exactly one `sketch.geometricConstraints.addCoincident(endpoint, existingPoint)`. Never pass an
existing `SketchPoint` into `addByTwoPoints` to share it. Sharing without a coincident leaves the
sketch under-constrained; sharing AND coincidenting is redundant and the solve fails outright with
`VCS_SKETCH_SOLVING_FAILED - failed to create offset`. ⚠️ **This covers the short reference and
connector lines too**, the ones whose both endpoints already exist: a regen that shared only those
came out about fourteen coincidents short and the gate failed on `Gear Profiles`. **Each named line
is created ONCE and later references reuse that line object** (`[BEVEL-F-LINE-ONCE]`); a helper that
creates a module-extension must RETURN the line so the caller keeps the reference. **The driven
lengths carry no dimension** (`[BEVEL-F-DRIVEN-DIMS]`, `[PB-NO-OVERCONSTRAIN]`): |Apex→A|, |Apex→B|
and the four module-length extensions are fixed by the perpendicular, collinear and closing
constraints, and dimensioning any of them throws `VCS_SKETCH_OVER_CONSTRAINTS`.

### The figure, in build order

Project **the Anchor sketch's centre `SketchPoint`** — `self._anchorCenterPoint`, not the raw user
selection — with `sketch.project(self._anchorCenterPoint)`. Both happen to be coincident, but
projecting the anchor-sketch point keeps the chain inside the Design component; the raw point is a
cross-component reference and can resolve inconsistently. Let `c` be the projected centre and `d`
the projected anchor line's 2-D unit direction.

**The centre→apex line.** From `c`, draw a construction line and constrain it
`sketch.geometricConstraints.addPerpendicular(centerToApex, anchorLine)` in the sketch's own 2-D
frame. Its far end is the **Apex**, seeded in sketch-local coordinates at

    Apex = c + perp * (R * cos γ_g + <resolved Driving Gear Base Height>)

with `perp = (-d.y, d.x)` the in-plane unit perpendicular to the projected anchor line, `R` the
Pitch Cone Distance and `γ_g` the driving pitch cone angle. **Seed it at exactly that distance and
not at the Driving Gear Pitch Diameter**, which earlier revisions said: the constraint net closes
this line at `R * cos γ_g` above point I plus the resolved driving base height, so for the default
31/31 pair at 90° the old seed sat 11.6 mm past where the solve puts it — 31 mm seeded against
19.375 mm solved (`[PB-SEED-NEAR]`, `[BEVEL-F-APEX-LOCAL]`). The apex POSITION is sketch-local; do
NOT compute it from a world round-trip, which is what caused the XY collapse. **The SIGN of `perp`
is the one permitted world reading in §2**: pick it so `perp` points toward the target plane's
normal, read as `targetPlane.geometry.normal` for both selection kinds — a `BRepFace`'s `geometry`
and a `ConstructionPlane`'s `geometry` are each a `core.Plane` carrying `.normal`. The comparison is
between an in-plane 2-D direction and a 3-D world vector, and the spec states the intent without
naming a call, so write it as: map the two 2-D points `c` and `c + perp` into world with
`sketch.sketchToModelSpace(point)` — a point-transforming METHOD, not a matrix
(`[PB-SPACE-METHODS]`) — subtract to get the world direction, and keep the sign whose dot product
with `targetPlane.geometry.normal` is positive. That is a one-bit direction reading and not a
position round-trip, so it cannot reintroduce the XY collapse. Do not decide it by the sketch's
local +Y, which maps to different world sides depending on how the plane was oriented
(`[BEVEL-F-GROW-SIDE]`). Add **no** length constraint on this line.

**The Driving Gear Shaft Axis.** From the Apex, pointing back toward the anchor line in the `-perp`
direction. **Seed its far end at `apex - perp * (R * cos γ_g)`, which is
`c + perp * (<resolved Driving Gear Base Height>)`** — measured from the apex, not from `c`. Earlier
revisions said `c - perp * (some length)`, which puts B on the far side of the projected centre from
the apex, the wrong side of the figure entirely. Apply
`sketch.geometricConstraints.addParallel(drivingShaftAxis, centerToApex)` and
`sketch.geometricConstraints.addCoincident(drivingShaftAxis.startSketchPoint, apexPoint)`. **Do NOT
use `addVertical`** (named here only to forbid it — see the exemption at the end of this step), which
forces the line to the sketch's world-vertical and mis-orients the figure
on a tilted target plane. The far end is point **B**. Do not dimension its length.

**The Pinion Gear Shaft Axis.** The driving direction rotated about the apex by the Shaft Angle.
Rotating has two senses and they place point A on opposite sides. **Select the sense this way: form
BOTH candidate point-A positions — the driving-shaft direction rotated about the apex by +Shaft
Angle and by −Shaft Angle — and keep the candidate whose endpoint has the GREATER X coordinate in
the Gear Profiles sketch.** Compare the two and take the larger; do not rotate one fixed sense and
flip it only when its X comes out negative, because when both candidates have a positive X that
shortcut keeps the wrong one. Call the chosen unit Apex→A direction `pinionDir`. Coincident its
start with the apex, then apply
`sketch.sketchDimensions.addAngularDimension(pinionShaftAxis, drivingShaftAxis, textPoint)` set to
the Shaft Angle, with the **text point inside the Σ wedge** so it measures Σ and not 180−Σ
(`[PB-ANGULAR-DIM]`) — for example on the interior bisector,
`apex + normalize(pinionDir + drivingDir) * (PPD / 4)`, where `drivingDir` is the unit Apex→B
direction. The far end is point **A**. Do not dimension its length.

**Seed the along-shaft lengths with the closed-form cone geometry**, so the solver converges on the
right branch for any Σ: `|Apex→A| = R * cos γ_p` and `|Apex→B| = R * cos γ_g`. Both cosines are
positive for every Shaft Angle the range check admits, which is what the Maximum Shaft Angle
guarantees. Seeding A or B merely by a pitch diameter is wrong for Σ ≠ 90° and can send the solver to
the wrong branch.

**The two perpendicular drops, and Apex 2.** From A, a construction line perpendicular to the Pinion
Gear Shaft Axis; from B, one perpendicular to the Driving Gear Shaft Axis. ⚠️ **Apex 2 sits in the
interior wedge BETWEEN the two shaft axes, so each drop must point toward the OTHER shaft axis** —
the A drop toward B, the B drop toward A. Pick each perpendicular's sense by the sign of its dot
product with that direction. **Do NOT choose the B drop's sense by a "toward the anchor line"
reference**: the Driving Gear Shaft Axis is itself parallel to that grow direction, so the
perpendicular's dot with it is about zero — a degenerate test that silently selects an arbitrary,
usually wrong, side. That is the critical failure: with the B drop seeding Apex 2 on the wrong side
of the driving shaft while the A drop seeds it on the right one, the closing coincidence makes the
solver **flip the entire frame to the mirror solution** — A jumps to the opposite side, the pinion
dedendum C collapses onto the driving dedendum D, the toe ends up outside the heel, the revolved
frustum is degenerate and the conical end cut finds no cone face at the toe midpoint, reporting
`face dist = inf`.

Apply `addPerpendicular` against each drop's own shaft axis, `addCoincident` on each drop's start,
and an aligned distance dimension of `Pinion Gear Pitch Diameter / 2` on the A drop and
`Driving Gear Pitch Diameter / 2` on the B drop — each is that gear's pitch radius at the heel,
which is the perpendicular distance from Apex 2 to that shaft axis for any Shaft Angle. Then close
the figure with `sketch.geometricConstraints.addCoincident(dropA.endSketchPoint, dropB.endSketchPoint)`.
Let that point be **Apex 2**. At Σ = 90° the four points Apex, A, Apex 2, B form a rectangle; at other
angles a non-rectangular quadrilateral, with |Apex→A| and |Apex→B| adjusting so the two drops
coincide.

**Naming convention, used throughout.** "A→Apex2" always means this PPD/2 perpendicular DROP line,
never the Apex→A shaft axis. The two share point A and are different lines. The same holds for
"B→Apex2" against the Apex→B shaft axis.

**The Pitch Line and the two dedendum lines.** Draw Apex→Apex 2, coincident at each end; this is the
**Pitch Line**. From Apex 2 draw two construction lines perpendicular to it —
`addPerpendicular(dedendumLine, pitchLine)` — each with an aligned dimension of `Module * 1.25`. The
one drawn **towards** the anchor line is the **Driving Gear Dedendum**, ending at point **D**; the one
drawn **away** from it is the **Pinion Gear Dedendum**, ending at point **C**.

**The two root axes.** Apex→D and Apex→C, coincident at both ends. These are the Driving and Pinion
**Root Axis**.

**The pinion module-length chain.** From A, a construction line collinear with the line from Apex to
A, extended by one module as a SEED only and with NO dimensional constraint. Apply
`sketch.geometricConstraints.addCollinear(segmentAE, pinionShaftAxis)` and a coincident between the
end of Apex→A and the start of the new line. The far end is point **E**. Draw C→E, coincident at
both ends, and `addPerpendicular(segmentAE, segmentCE)`.

From E, a line collinear with **line A→E** — the collinear names A→E, **never the Apex→A shaft axis**
further up the chain, even though both describe the same infinite line (`[PB-COLLINEAR-CHAIN]`,
`[BEVEL-F-COLLINEAR-CHAIN]`; naming the axis raises `VCS_SKETCH_OVER_CONSTRAINTS`). Length one
module as a seed, no dimension. The far end is point **G**.

From C, a line of one module's seeded length, collinear with **line Apex2→C**, the Pinion Dedendum
line C is the endpoint of. The far end is point **H**. Connect G and H, coincident at both ends, and
`addPerpendicular(segmentEG, segmentGH)`.

⚠️ **That last perpendicular is required in Fusion and must be omitted in the proof harness, and the
reason is a difference between the two engines rather than a choice.** `addOffsetDimension` in
Fusion is a distance dimension whose documentation requires the second entity to be "a line that is
parallel to the first", and it controls only the perpendicular distance. So the parallelism has to
exist before the pinion base-height offset below can be applied at all, and this perpendicular is
what supplies it: E→G runs along the pinion shaft, so making H→G perpendicular to it makes H→G
parallel to the A→Apex2 drop. Perpendicular plus offset is two equations for two freedoms and
nothing is redundant. The proof harness's offset constraint emits **two** residual rows, holding
both endpoints of the target line at the same signed perpendicular distance from the source, so it
carries the parallelism itself; adding this perpendicular there is a third row for the same two
freedoms and the lattice comes back overconstrained at DOF 0 with two redundant constraints, the
engine naming the two base-height offsets as the redundant pair.

**The driving module-length chain** is the exact twin: B→F collinear with Apex→B, D→F with
`addPerpendicular(segmentBF, segmentDF)`, F→I collinear with **B→F**, D→J collinear with **Apex2→D**,
then I→J connected and `addPerpendicular(segmentFI, segmentIJ)` — required in Fusion, omitted in the
proof, for the reason just given.

**The two base-height offsets.** Both are
`sketch.sketchDimensions.addOffsetDimension(line, entityTwo, textPoint)` with the value set through
the returned dimension's `.parameter.value`, and **neither takes an extra `addParallel`** because
each target line is already parallel to its source by construction (`[PB-OFFSET-DIM]`).

- Between the **B→Apex2 perpendicular drop** (the DPD/2 drop, not the Apex→B shaft axis) and **J→I**:
  the value is the Driving Gear Base Height if specified, otherwise
  `module * Driving Gear Teeth Number / 8`, in either case AFTER the driving gear's Maximum Base
  Height has been applied.
- Between the **A→Apex2 perpendicular drop** (the PPD/2 drop) and **G→H**: the value is the Pinion
  Gear Base Height if specified, otherwise the **RESOLVED** Driving Gear Base Height
  `* (Pinion Gear Teeth Number / Driving Gear Teeth Number)`, then held to the **pinion's own**
  Maximum Base Height. "Resolved" means the value the driving offset actually used — after its own
  fallback and after its own cap — not the raw driving input.

**Constrain point I with the projected centre** — `addCoincident(pointI, projectedCenter)`. This is
what fixes the Apex's height above the anchor line, and with it the whole figure's station.

**The two tooth centres.** From G, a construction line along Apex→A, away from the apex, ending at
point **K**; then **pin K with two point-on-line coincidents** —
`addCoincident(pointK, pinionShaftAxis)` and `addCoincident(pointK, pinionDedendumLine)` — rather
than an `addCollinear`. By the time K is added, G and C are already fixed, so a collinear there
over-constrains and Fusion errors; the two point-on-line coincidents locate K exactly, at the
intersection of the two lines, without over-constraining. Draw C→K for reference. The driving twin
is I→L along Apex→B with `addCoincident(pointL, drivingShaftAxis)` and
`addCoincident(pointL, drivingDedendumLine)`, plus D→L for reference.

**The tooth-centre points K′ and L′ (Tooth Spacing).** The §3 tooth is centred not at K but at K′,
K shifted outward along the dedendum line by **Tooth Spacing**, away from the lower corner C — in
the C→K direction, beyond K. **When Tooth Spacing is 0, which is the default, build NOTHING here:
set K′ ≡ K and reuse the C→K reference line**, because a zero-length dimensioned line is degenerate
and one segment gets one line (`[BEVEL-F-LINE-ONCE]`). When Tooth Spacing > 0: draw a line starting
at K with its far end seeded on the FAR side of K from C along the dedendum direction, pin it the
same way K is pinned — `addCoincident(shift.startSketchPoint, pointK)` and
`addCoincident(pointKPrime, pinionDedendumLine)` — then add an aligned length dimension on that line
equal to Tooth Spacing. Do not use `addCollinear`, for the same over-constraint reason as K. Finally
draw the tooth-centre reference line **C→K′** for §3 to use in place of C→K. The driving side is
identical with L for K, D for C, and the Driving Dedendum line Apex2→D; its reference line is
**D→L′**. Build both here, inside this sketch, before this step's end-of-step full-constraint gate,
so the gate covers them. Only the tooth's centre moves; the virtual tooth number and the drawn tooth
size are unchanged.

**Resolve the Maximum Face Width here.** All of A, B, C, D, H, J now exist and are solved, so read
`pointA.geometry`, `pointB.geometry`, `pointC.geometry`, `pointD.geometry`, `pointH.geometry` and
`pointJ.geometry` — the SOLVED geometry, never the pre-solve seed coordinates
(`[PB-SOLVED-GEOMETRY]`) — and compute

    Maximum Face Width = 0.95 * min(
        perpendicular distance from A to the line through C and H,
        perpendicular distance from B to the line through D and J)

Cap the auto default to it and reject a user Face Width above it, naming the maximum. The auto
default is `min(Cone Distance / 6, Maximum Face Width)`. Seeds diverge substantially from the solved
positions for asymmetric tooth counts and non-90° shaft angles, making a seed-based bound too loose
on the binding side, and the toe then crosses the axis and the cap is defeated. The pinion is
normally the binding side because its smaller pitch radius gives the smaller distance, but compute
both and take the minimum: at Shaft Angle 90° this limit equals
`0.95 * min(Driving Gear Pitch Diameter, Pinion Gear Pitch Diameter)**2 / (2 * Cone Distance)`, the
SMALLER pitch diameter and never the pinion's by name. Written with the pinion's diameter it is
wrong whenever the driving gear carries the smaller tooth count: on a Driving 17 / Pinion 31 pair at
Module 1 the real bound is 3.883 mm and the pinion form gives 13.591. Stash the result as
`self._faceWidthResolved_cm`.

**Resolve the Root Length and the two Toe Radii.** These are what the toe lines are dimensioned
from, and every figure below is needed by name:

- `|Apex→Ded| = sqrt(R**2 + (1.25 * Module)**2)`, the same for both gears.
- **Root Length** at Toe Extension 0 is `Face Width * |Apex→Ded| / R` — the resolved Face Width
  re-measured along the root element rather than perpendicular to the pitch line. Face Width still
  resolves exactly as it always did and still carries its cap; the Toe Extension adds to what Face
  Width resolved and does not replace it.
- Each gear's **Toe Radius**, 0 meaning auto, auto being that gear's inner toe corner radius at Toe
  Extension 0: `this gear's Pitch Radius - Face Width / sin γ`. A user value must be **strictly
  below** that gear's **Toe Radius Ceiling**,
  `(this gear's Pitch Radius - 1.25 * Module * cos γ) * (1 - Face Width / R)`, which is that gear's
  OUTER toe corner radius at Toe Extension 0; reject it with a message naming the ceiling.
- Each gear's **Toe Limit**, `|Ded→X|` where X is the point on the root element Apex→Ded at this
  gear's Toe Radius: `sqrt(R**2 + (1.25 * Module)**2) - Toe Radius / sin(γ_root)`, with the root cone
  angle `γ_root = γ - atan(1.25 * Module / R)`.
- **Toe Extension 100 stops at 0.99 of the way from the Toe Extension 0 root length to the SMALLER
  of the two gears' Toe Limits**, not at the Toe Limit itself. The smaller limit wins because the
  pair shares one root length. The 0.99 is there because AT the limit the toe face has zero length,
  so the revolved gear body carries no cone at its toe end and the conical end cut in S13 — whose toe
  cut must split or the build fails — has no `ConeSurfaceType` face to find. Do not drop this factor.
  So `Root Length = base + (Toe Extension / 100) * 0.99 * (min(Toe Limit) - base)` with
  `base = Face Width * |Apex→Ded| / R`.
- **A defaulted Toe Radius can leave no room at all, and that is a real configuration rather than a
  defect.** On a driving gear with a large pitch cone angle the inner toe corner already sits at a
  LARGER radius than the outer one, so X falls behind the toe corner and the Toe Limit comes out
  below the Toe Extension 0 root length. **Reject a Toe Extension above 0 on such a pair**, naming
  the gear and the Toe Radius Ceiling it needs to come below. Toe Extension 0 still resolves, so the
  gear stays buildable exactly as before. Do **not** silently substitute a smaller Toe Radius.

**The pinion toe line M→N.** **Seed BOTH ends at their closed-form solved positions, not near them**
(`[PB-SEED-NEAR]`). Seed M on `Apex→C` at the fraction `1 - <Root Length> / |Apex→C|` from the Apex.
Then seed N by sliding from that M seed along the `C→H` direction by exactly

    (<M seed's perpendicular distance from the Pinion Gear Shaft Axis> - <Pinion Gear Toe Radius>)
    / cos γ_p

⚠️ **A seed that merely lands somewhere plausible is not enough here, and a wrong one builds the
wrong gear rather than failing to converge.** N's position is fixed by the toe line together with a
LENGTH dimension on the front face, and a length is unsigned: the toe line meets the Toe Radius on
BOTH sides of the shaft axis, so the solver takes whichever side the seed starts on. Seeded below the
axis it converges happily onto the mirror, N comes out on the far side, the revolved hexagon crosses
its own axis of revolution, and Fusion aborts the revolve with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`) at
S11 — pointing at the revolve rather than at the seed that caused it. Two earlier seeding rules do
exactly that and must not be reinstated: sliding from the M seed by the **Root Length**, and sliding
by the **distance from the M seed to A**. Measured on the shipped default pair, Module 1 with 31/31
teeth at Shaft Angle 90° and a Toe Extension of 50%, the Root Length slide puts the N seed at a
perpendicular distance of **−0.27 mm** from the shaft axis — past it — against a solved N at
**+5.17 mm**, and Fusion refuses the revolve; the slide above puts it at 5.17 mm exactly. Do not seed
M or N just `Face Width` away from C or H either, which starts N near H, far from its constraint
target.

Then apply **exactly these three constraints**:

- `addCoincident(pointM, pinionRootAxis)` — M lies on the Apex→C root axis;
- `addParallel(segmentMN, segmentCH)` — the toe line is parallel to C→H;
- `addOffsetDimension(segmentCH, segmentMN, textPoint)` with `.parameter.value` set to the Root
  Length re-measured perpendicular to the pitch line, `<Root Length> * R / |Apex→C|` — an offset
  dimension controls a perpendicular distance, so it carries the root length in that form. At Toe
  Extension 0 the value is exactly the resolved Face Width, which is what this dimension has always
  been. Place the `textPoint` in the gap between C→H and M→N on the Apex side, for example
  `(M_seed + C) / 2`, so it reads cleanly (`[PB-OFFSET-DIM]`).

The line's start is **M** and its end is **N**. Draw M→C.

**The front face A′→N, which is what holds N.** ⚠️ **N is NOT pinned to line A→Apex2.** Earlier
revisions pinned it there, which fixed its station at A's and made the Maximum Face Width the value
at which N reached A. It now rides the Pinion Gear Toe Radius instead:

- draw a line from N to a new point **A′**, seeding A′ at N's station on the shaft axis;
- `addCoincident(pointAPrime, pinionShaftAxis)` — A′ lies on the **Apex→A shaft axis**; it is the
  only toe-end point that touches that axis, and it is a foot, not a corner;
- `addPerpendicular(segmentNA, pinionShaftAxis)` — the front face stands square to the shaft, so the
  revolve sweeps it into a flat annulus;
- an aligned distance dimension on the whole line N→A′ equal to the resolved Pinion Gear Toe Radius.

⚠️ **Pinning N itself to the Apex→A shaft axis remains forbidden**: that would put N on the axis of
revolution and the later conical split fails with `ASM_API_FAILED` for asymmetric tooth counts even
though the symmetric 45° case happens to survive. A′ sits on the axis; N never does, because the Toe
Radius is strictly positive. Those three rows plus the offset and the coincident on M fully constrain
M, N and A′ — six freedoms, six constraints.

**A′ replaces A as the hexagon's first vertex.** At Toe Extension 0 with a defaulted Toe Radius the
two coincide exactly, so nothing moves; a positive Toe Extension walks A′ along the shaft axis toward
the Apex and the shaft edge grows by that much. Draw the line **A′→G**, the hexagon's shaft-axis
edge; it starts at the front face's foot A′, not at A.

**The driving toe line O→P** is the mirror. Seed O on `Apex→D` at the fraction
`1 - <Root Length> / |Apex→D|`, then P slid from that O seed along `D→J` by
`(<O seed's perpendicular distance from the Driving Gear Shaft Axis> - <Driving Gear Toe Radius>) / cos γ_g`.
The ⚠️ above applies unchanged. Apply `addCoincident(pointO, drivingRootAxis)`,
`addParallel(segmentOP, segmentDJ)`, and `addOffsetDimension(segmentDJ, segmentOP, textPoint)` with
the same `<Root Length> * R / |Apex→D|` value and its `textPoint` at `(O_seed + D) / 2`. Draw O→D.
Build the driving front face **B′→P** exactly as the pinion's: the line P→B′,
`addCoincident(pointBPrime, drivingShaftAxis)`, `addPerpendicular(segmentPB, drivingShaftAxis)` and
an aligned dimension equal to the resolved Driving Gear Toe Radius. Draw **B′→I**.

**Gate the sketch.** Raise, naming the sketch, if `sketch.isFullyConstrained` is false
(`[BEVEL-F-FULL-CONSTRAINT]`).

<!-- check-step-calls: ignore addVertical -->
`addVertical` is named only to forbid it on the Driving Gear Shaft Axis, so the module must NOT call
it.

### What the proof establishes

`stepGearProfiles` builds this whole lattice — both shaft axes, both dedendum chains, both heel
edges, both toe lines, both front faces, both tooth centres — in one sketch and gates it at DOF 0
with nothing redundant, nothing conflicting, valid profiles, a system that is not near-singular and
no discrete ambiguity. The table sweeps the Shaft Angle from the documented 30° floor up past 150°,
ratio pairs both ways round, the low tooth count the base-height bounds exist for, Toe Extension 0,
50 and 100, user-set toe radii, a positive Tooth Spacing, user-set base heights and Face Width, and
**both grow sides**, because the sign of `perp` is chosen from a world normal and a scheme that only
closes on one side mirrors the gear on half the target planes.

On the solved geometry it then reads back: every point at the position its seed rule states, which is
what makes the seeding rule checkable at all; both cone angles against the closed form; the Pitch
Cone Distance, and that it is twice the Cone Distance parameter at 90° and not otherwise; the Maximum
Face Width recomputed from the solved A, B, C, D, H and J; both toe corners riding at their gears'
Toe Radii and on the SAME side of the shaft axis as the rest of the figure; the toe end nearer the
Apex than the heel end, which is the mirrored frame's signature; |C→M| equal to the Root Length and,
at Toe Extension 0, equal to `Face Width * |Apex→Ded| / R`; the tooth centres one virtual pitch
radius from Apex 2; and that the sketch closes no region at all, every line being construction.

**The Shaft Angle floor of 30° is in the table as a DECLARED REFUSAL rather than removed.** This
lattice reads conditioning 2.83e-05 there against the engine's 4e-05 trust floor and first clears at
35°. That is a property of how this net is built — three independently written nets do not agree
about which end of the range is reachable — so the case stays and is marked, and the remedy is a
different construction, never a loosened gate and never a narrower advertised range.

Four substitutions are recorded in the proof file beside the geometry each belongs to. Every
`addCollinear` becomes the single point-on-line row it is not already implied by, which is what makes
the proof unable to tell the correct collinear from one naming a farther line up the chain. The two
`addPerpendicular` calls before the base-height offsets are omitted, and the two `addParallel` calls
on the toe lines with them, because this engine's offset carries the parallelism itself. The
`addCoincident(I, projected centre)` becomes one point-on-line row, because I already lies on the
line through the centre and the engine reports the second row as redundant. And the four
side-selecting perpendiculars — each Apex 2 drop against its shaft axis, each dedendum line against
the pitch line — plus the two front faces and the Tooth Spacing shift are written as SIGNED angles,
because perpendicular-plus-unsigned-length is satisfied on either side and the probe finds the
mirror; in Fusion the side comes from the seed, and that is exactly why the proof cannot catch a
wrong seed.

**From:** `spec/bevelgear/instructions.md` L106-138, L471-587;
`spec/bevelgear/fusion.md` L71-115, L119-151;
`.claude/skills/generate-gear/PLAYBOOK.md` L470-484, L492-499, L576-581, L582-604, L628-642.

## S7 `[PROSE]` `{gearLabel} Plane` — the tooth-profile plane

Run once per gear, pinion first. Create a plane that includes the tooth-centre reference line —
C→K′ for the pinion, D→L′ for the driving gear — perpendicular to the Gear Profiles sketch plane,
through the framework helper `plane_by_angle(designComponent, toothCentreLine, gearProfilesPlane, 90)`
from `.solids`. Name it `{gearLabel} Plane`, where `gearLabel` is `Pinion` or `Driving`. Pass the
sketch line DIRECTLY; never wrap it in `Path.create` first (`[PB-CONSTRUCTION-PLANES]`).

This step creates a construction plane and no measurable geometry, so no proof function realises it.

**From:** `spec/bevelgear/instructions.md` L590-591, L593-599;
`.claude/skills/generate-gear/PLAYBOOK.md` L766-777.

## S8 `[GO]` `{gearLabel} Tooth` sketch — the virtual spur tooth — `stepVirtualSpurTooth`

<!-- proof-run: proofkit.RunParallel(latticeCases, stepVirtualSpurTooth) -->

<!-- check-compile: ignore VirtualSpurProxy SpurGearInvoluteToothDesignGenerator draw floor cos -->
`VirtualSpurProxy` and `SpurGearInvoluteToothDesignGenerator` are this repository's own classes and
`draw` is the borrowed generator's own method, so none of them is an Autodesk name.

Run once per gear, pinion first. Compute this gear's virtual (back-cone, Tredgold) tooth number from
the closed form and **not** by measuring Apex2→K′/L′:

    virtualPitchRadius_mm = (pitchDia_cm * 10 / 2) / cos(γ)
    virtualTeeth          = floor(2 * virtualPitchRadius_mm / Module)

as an int. **The `* 10` is the cm→mm conversion and is load-bearing**: the stashed pitch diameters
are internal cm while Module is the raw mm value, and skipping it makes the virtual tooth count about
ten times off. The virtual tooth number is independent of Tooth Spacing; the spacing offset moves
only the centre, never the tooth size.

Create the sketch on `{gearLabel} Plane` and name it `{gearLabel} Tooth`. Then, with the tooth-centre
point K′ (pinion) / L′ (driving) as the anchor:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))
```

`VirtualSpurProxy` is imported from the framework — `from .spurproxy import VirtualSpurProxy` — and
bevel defines no local proxy or value-wrapper class. It precomputes, in internal cm, exactly the
parameter keys the spur drawer reads, and its defaults match bevel: pressure angle 20°, which is NOT
a bevel dialog input, and `InvoluteSteps` 15. The **180° rotation is delivered through the `draw()`
angle argument** — the spur generator rotates the whole tooth by that angle — and never as a
post-hoc Move or sketch rotation.

**After `draw()` returns, read `proxy._lastToothEmbedded` back.** The spur generator decides during
`draw()` whether the tooth is embedded — tip, root and flanks meeting with no connecting lines — and
records it on the proxy, which pre-initialises the slot to absorb that write. Stash the flag
alongside the tooth sketch and plane on this gear's context dict. It is **not optional
bookkeeping**: it is the deterministic selector for the tooth loop's line count in S12,
`wantLines = 0 if embedded else 2`.

**Do NOT gate this sketch.** Log it with `futil.log(...)` if `toothSketch.isFullyConstrained` is false; never raise
(`[PB-LOGGING]`: let the entry point's try/except and `deleteComponent()` handle rollback rather
than inventing a new silent failure path). The
tooth-profile sketches are exempt from the full-constraint gate because the borrowed generator labels
each of its four circles with along-path sketch text, and sketch text holds a degree of freedom
(`[PB-TEXT-HOLDS-DOF]`), so a sketch whose geometry is completely determined still reads `False`
purely because it is labelled. ⚠️ That exemption covers the labels and nothing else, and it is never
licence for loose geometry: measured in Fusion, the same labelled sketches read `False` in one run and
`True` in the two after it, so the answer cannot be relied on either way.

### What the proof establishes

`stepVirtualSpurTooth` draws the same tooth from the shared involute math the spur family uses — at
this gear's virtual tooth number, the Module and the proxy's 20° pressure angle, already rotated by
180° — and gates it at DOF 0. It then reads the curve counts the S12 profile selection keys on: one
region of 2 NURBS, 2 arcs and **exactly** `wantLines` lines, plus the root-circle disc, with the line
count determined by the embedded flag and never accepted as either. It also checks the virtual tooth
number against `floor(2 * virtual pitch radius / Module)` and that the drawn tooth's pitch radius
sits within half a module of the virtual pitch radius.

What the proof cannot reach is recorded beside it: this engine has no sketch text at all, so the
sketch here is the unlabelled geometry and reaches DOF 0, which shows the geometry is determined and
not that Fusion's `isFullyConstrained` would say so.

**From:** `spec/bevelgear/instructions.md` L374-384, L423-451, L588-604;
`spec/bevelgear/fusion.md` L31-58;
`.claude/skills/generate-gear/PLAYBOOK.md` L508-523, L663-672, L695-697.

## S9 `[PROSE]` `{gearLabel} Tooth Axis`

Run once per gear. Create a construction axis through the tooth-centre point, normal to the plane the
tooth profile was drawn on, and name it `{gearLabel} Tooth Axis`:
`axisInput = designComponent.constructionAxes.createInput()`, then
`axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)`, then
`designComponent.constructionAxes.add(axisInput)` (`[PB-CONSTRUCTION-AXES]`;
`setByPerpendicularAtPoint` would need a `BRepFace` that does not exist here). The helper plane is
built `planeInput.setByDistanceOnPath(toothCentreLine, adsk.core.ValueInput.createByReal(1.0))` —
perpendicular to the tooth-centre reference line at its far end, the tooth-centre point — and the two
planes' intersection is the line through that point normal to the tooth plane. Pass the sketch line
directly to `setByDistanceOnPath`, never through `Path.create`.

Creating this axis in the never-activated Design component is proven to work — `constructionAxes.add`
via `setByTwoPlanes` does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here — so keep the axis.

This step creates a construction axis and no measurable geometry, so no proof function realises it.

**From:** `spec/bevelgear/instructions.md` L590-591, L603;
`.claude/skills/generate-gear/PLAYBOOK.md` L766-790.

## S10 `[GO]` `{gearLabel} Profile` sketch — the hexagon — `stepGearProfileHexagon`

<!-- proof-run: proofkit.RunParallel(latticeCases, stepGearProfileHexagon) -->

Run once per gear. Open a **fresh sketch on the axial Gear Profiles plane** and name it per this
table, so `sketch.profiles` holds exactly this one hexagon loop. Do not draw both gears' hexagons in
the shared Gear Profiles sketch, which would leave two identically-shaped loops to disambiguate.

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A′ → G → H → C → M → N → A′ | B′ → I → J → D → O → P → B′ |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A′→G | B′→I |
| toe cut edge | M→N | O→P |
| heel cut edge | C→H | D→J |
| teeth / bore / pitch-diameter inputs | Pinion Gear … | Driving Gear … |
| §2 shaft construction line (NOT usable as the axis) | Apex→A | Apex→B |

Build the hexagon on fixed vertices by the `[PB-PROJECT-NOT-FIXED]` recreate-share-fix recipe, which
is valid because §2 is fully constrained by now:

1. recreate the six §2 vertices as new points at their exact world-mapped positions —
   `sketch.sketchPoints.add(sketch.modelToSketchSpace(source.worldGeometry))` for each
   (`[PB-SPACE-METHODS]`: `modelToSketchSpace` is a point-transforming METHOD, not a matrix);
2. draw the closed hexagon in the table's draw order as six
   `sketch.sketchCurves.sketchLines.addByTwoPoints(...)` calls **sharing** those points;
3. then, and only then, fix the lines' endpoints: `line.startSketchPoint.isFixed = True` and
   `line.endSketchPoint.isFixed = True` for every line. **Order matters** — setting `isFixed` on a
   bare point before it is consumed as a line endpoint does not leave the sketch fully constrained.

Projecting the §2 points instead would leave the sketch under-constrained, because a projection is
associative and not fixed. **The hexagon's first edge is the gear's shaft axis** for the revolve, the
pattern, the bore plane and the meshing rotation, so it must be fixed well enough to carry a
trustworthy world position: fixed endpoints give it a well-defined `worldGeometry`
(`[PB-WORLDGEO-CONSTRAINED]`), while a free edge resolves against a default world-XY frame and
silently moves the body onto world XY — observed on the driving gear, the pinion looking fine only
because it never read the edge's `worldGeometry`.

**The shaft axis used by every body operation below is this profile sketch's first edge, NOT the §2
`Apex→A` / `Apex→B` construction line.** The edge is collinear with the shaft axis but lives in the
SAME sketch as the profile, which is what Fusion's revolve, pattern and path builders accept;
reusing the §2 construction line fails or misbuilds.

Gate the sketch: raise if `sketch.isFullyConstrained` is false (`[BEVEL-F-FULL-CONSTRAINT]`).

### What the proof establishes

`stepGearProfileHexagon` recreates the six vertices, draws the closed hexagon sharing them, fixes the
endpoints AFTER the lines exist, and gates the sketch at DOF 0. It then reads back the one closed
region the revolve consumes, that it is extrudable, and that its loop holds exactly six lines. It
also checks the three things that keep the revolve legal: no vertex sits across the shaft axis, the
inner toe corner rides at a strictly positive radius so only the front face's foot touches the axis,
and the first edge really is on the axis and runs from the toe end toward the heel.

**From:** `spec/bevelgear/instructions.md` L699-718;
`spec/bevelgear/fusion.md` L28-30;
`.claude/skills/generate-gear/PLAYBOOK.md` L449-463, L485-491, L576-581, L589-596.

## S11 `[GO]` Revolve the Gear Body — `stepRevolveGearBody`

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepRevolveGearBody, assertRevolveGearBody) -->

Run once per gear. This sketch holds exactly one hexagon loop, so take its single profile directly —
`sketch.profiles.item(0)` (`[PB-SINGLE-PROFILE]`); do not invent a search that filters by
`profileLoops` or by curve type, which has spuriously rejected a valid all-line loop and made the
revolve fail with "could not find profile".

`revolveInput = designComponent.features.revolveFeatures.createInput(profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
then `revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))`, then
`designComponent.features.revolveFeatures.add(revolveInput)`. The axis is the profile sketch's FIRST
edge. The result is the **Gear Body**, the frustum.

**Hard failure to design around: the profile must not cross the axis of revolution**
(`[PB-REVOLVE]`). If it does, Fusion aborts with
`RuntimeError … ASM_WIRE_X_AXIS … the profile crosses the axis of revolution`. The Maximum Face
Width, the Maximum Base Height and the strictly positive Toe Radius are each there to stop that, and
the toe-line seeding rule in S6 is what keeps N on the right side to begin with.

Because the toe edge is one edge of the revolved profile, the body already carries the conical face
that edge sweeps, and likewise the heel edge's cone. Those faces are the cutting tools S13 uses.

### What the proof establishes

decad has a Revolve and it is not usable here: measured on this repository's pinned revision, a
revolved trapezoid publishes volume 8210.03 mm³ with a proven bound of 16420.06 mm³ — a bound equal
to twice the reading — so a revolved body is Suspect at any tolerance and cannot pass the harness
gate. `stepRevolveGearBody` therefore substitutes the **polygonal sweep** the spec calls for: the
three bands the hexagon's edges sweep — the root cone out to the dedendum corner, the heel cone out
to the heel end, and the toe-dish plug that hollows the front face — each built as a loft between
two coaxial regular polygons and **laid apart along the shaft axis**, because no boolean can join
them here and decad verifies every pair of live bodies in a document. The hexagon's other three
edges sweep nothing: two lie on the axis, and the front face's annulus is the plug's own end ring.

The assertion reads each band's two ring stations and radii off its own vertices and checks them
against the §2 points; reads each band's cone half-angle and checks that the heel band and the toe
plug come out parallel on the back-cone family at 90° minus the pitch cone angle while the root band
stands at the dedendum angle to them; and checks the SIGNED sum — root plus heel minus the plug —
against the solid-of-revolution integral taken edge by edge around the hexagon. The polygonal sweep
publishes every band's volume exactly, so that comparison carries no tolerance beyond floating-point
noise.

**The cost is the union**: the proof does not show the three bands closing into one watertight solid,
only that each is separately watertight and that together they have the frustum's volume, stations
and angles. The tables run at Module 4 to 8 and never at Module 1, because decad's mesh bound has an
absolute floor and a small enough figure brings every measurement inside it; Module is a pure scale
on this figure.

**From:** `spec/bevelgear/instructions.md` L715-719, L774-801, L832-838;
`.claude/skills/generate-gear/PLAYBOOK.md` L485-491, L708-714.

## S12 `[GO]` Loft the Apex to the tooth profile — `stepLoftTooth`

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepLoftTooth, assertLoftTooth) -->

<!-- check-compile: ignore find_profile_by_curve_counts -->
`find_profile_by_curve_counts` is this repository's own helper in `lib/geargen/utilities.py`.

Run once per gear. Select the tooth cross-section loop with
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)` where
**`wantLines = 0 if embedded else 2`**, the `embedded` flag read back from the proxy in S8. ⚠️ Do
**NOT** accept "0 **or** 2 lines": for a given gear only one of those is the real tooth, and an
unrelated loop — an inter-tooth or annular region between the drawn circles — can carry the same two
NURBS and two arcs with the other line count. Selecting it makes this loft fail with
`RuntimeError … ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`, because the impostor loop cannot form a loft
tool body. Embedded means tip, root and flanks meet with no connecting lines, which is four curves;
non-embedded adds two connecting lines, which is six.

Then loft the **§2 Apex sketch point** to that profile:
`loftInput = designComponent.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
then `loftInput.loftSections.add(apexSketchPoint)` and `loftInput.loftSections.add(toothProfile)` in
that order, then `designComponent.features.loftFeatures.add(loftInput)` (`[PB-LOFT]`). Use the §2
Apex SKETCH point directly — `centerToApex.endSketchPoint` from the Gear Profiles sketch, stashed as
`self._apexSketchPoint` — and do NOT create a construction point for it, because construction
geometry needs an active component and the Design component is never activated
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). The result is the **Tooth Body**, the uncut apex-to-heel tooth.

### What the proof establishes

Two substitutions, both forced. The degenerate apex point becomes a shrunken copy of the same
section, because decad has no point section; because a cone's sections are its end section scaled
linearly in station, the loft between them is exactly the piece of the real cone above that station.
And the tooth profile's own plane — the back-cone plane of S7 — becomes a plane PERPENDICULAR to the
shaft axis at the same station, because a loft here takes two parallel sections.

The assertion reads the tooth's reach from its own vertices and holds it to the virtual tip radius;
checks its span along the shaft and that its heel face sits at the tooth-centre station; and checks
its volume against the tapered cone's, the heel section's own area times the station over three, less
the fraction the shrunken end drops.

**The cost of the second substitution is the back cone's tilt**: the real tooth's heel face leans by
the dedendum angle, and what the loft proves is the taper from the apex rather than the lean of the
face it ends on.

**From:** `spec/bevelgear/instructions.md` L374-384, L720-721, L802-803;
`.claude/skills/generate-gear/PLAYBOOK.md` L715-719, L782-790.

## S13 `[GO]` Conical end cuts — `stepConicalEndCuts`

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepConicalEndCuts, assertConicalEndCuts) -->

<!-- check-compile: ignore cut_conical_ends apply_conical_cut select_keeper find_cone_faces_by_midpoint surface_distance NonIntersectError -->
These are all names in this repository's own `lib/geargen/solids.py`, not Autodesk's.

Run once per gear, and for BOTH branches: the straight tooth reaches it directly, and the spiral
tooth reaches it at the end of S20. Trim the Tooth Body to a flush band with the framework helper and
do **not** re-implement the cut machinery:

`cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)`

**Two distinct bodies are involved and must not be conflated.** The cutting TOOLS are
`ConeSurfaceType` faces of the **Gear Body**, the revolved-hexagon frustum; the lofted Tooth Body has
no cone faces at all, so searching IT for the cone face finds none. The TARGET being split is the
**Tooth Body**.

**Caller obligations, which stay in the generator.** Pass `toeMid` = the toe edge's world midpoint,
`(M_world + N_world) / 2` for the pinion and `(O_world + P_world) / 2` for the driving gear;
`heelMid` = the heel edge's world midpoint, `(C_world + H_world) / 2` and
`(D_world + J_world) / 2`; `apexWorld` = the §2 Apex sketch point's world geometry; and `gearBody` =
the revolved frustum, the cone-face source.

The helper implements the pinned behaviour: the **toe cut first**, its cone face identified by the toe
edge's world MIDPOINT best-first across the frustum's cone faces (`[PB-FACE-BY-MIDPOINT]` — endpoints
sit near the apex singularity and `getParameterAtPoint` returns no result there, so an endpoint
cannot see the right face), each candidate tried as the actual split tool and the first that splits
into more than one piece kept; **keeper selection after each cut**, dropping apex-containing pieces
and keeping the largest (`[PB-REMOVE-PIECES]`); then the **heel cut on the keeper alone**, which is
what makes it deterministically two split features for every gear ratio. A heel cone that does not
intersect the keeper at all — common on ratio pairs such as Module 1 with driving 31 and pinion 43,
where the heel cone never overshoots the tooth — is raised by the helper as the typed
`solids.NonIntersectError` and caught, and the keeper is returned whole. **The toe cut must split**:
its failure propagates and crashes the build, which is correct, since an uncut tooth is unusable.
Only the heel cut is lenient, and only through that typed error.

### What the proof establishes

`stepConicalEndCuts` performs NEITHER cut. Both operands are Lofts — the tooth and each cone alike —
so the split is unavailable. It builds the tooth and the two cones the toe and heel edges sweep, lays
them apart, and reads the cut off them.

The assertion checks each cone passes through the §2 points its edge was drawn between; that both
stand on the back-cone family and so are parallel, differing only in where their apexes sit on the
shaft, with the heel cone's apex further out; that the tooth's heel section runs from the virtual root
radius to the virtual tip radius; that each of the four cone-against-tooth-surface crossings falls
INSIDE the tooth's own span, which is what the cut needs in order to split at all; and that every toe
crossing sits strictly nearer the apex than every heel crossing, since an inverted frame is exactly
what makes the toe cone miss.

**The cost is the split**: the proof does not show the evaluator dividing the tooth, selecting the
keeper, or leaving a watertight body. Two further limits are recorded in the proof file. The spec
reads the cut's signature as the two ends landing on DIFFERENT surfaces of the tooth, the toe on its
tip and the heel on its root; measured here, each cone crosses BOTH surfaces, which is what a cut
through a solid does, and which surface carries the new face is decided by the keeper selection,
which needs the split. And the band's LENGTH is not checked against the hexagon's toe-to-heel run,
because S12's axis-perpendicular section means the tooth's root surface is not the gear body's root
cone; what survives is the ORDER of the stations and that each falls inside the tooth.

**From:** `spec/bevelgear/instructions.md` L723-749, L804-811;
`.claude/skills/generate-gear/PLAYBOOK.md` L159-172, L724-760.

## S14 `[PROSE]` `{gear} Cone Element` sketch and `{gear} Trace Plane` (ψ > 0 only)

<!-- check-compile: ignore plane_by_angle combine_point circle_intersect_nearest distAlong -->
<!-- check-step-calls: ignore distAlong -->
`distAlong` is written here as the frame's own shorthand for a point's cone distance, not as a call
the module must make; the module computes that dot product inline wherever it needs it.
`plane_by_angle`, `combine_point` and `circle_intersect_nearest` are framework helpers in
`lib/geargen/solids.py`.

Everything from here to S20 runs **only when the Mean Spiral Angle ψ > 0**. The tooth-body hook's
first line is the gate `if self._spiralAngle_rad <= 0: return cut_conical_ends(...)`, so a straight
bevel is byte-for-byte the prior behaviour and none of S14 to S20 runs.

**The world frame this branch works in**, built from geometry already constructed for this gear:

- `axisDir` — the shaft axis direction, from the two **WORLD** endpoints of the profile edge A′→G
  (pinion) / B′→I (driving), normalized. Every quantity in this frame is world, so every sketch
  entity it is read from must be sampled in world space (`[PB-WORLD-FRAME]`): a curve's `.geometry`
  is sketch-local and mixing it with a world axis is valid Python that silently returns wrong
  numbers — a wrong spiral-twist magnitude that makes meshing teeth interfere, with no exception and
  nothing a lint can catch;
- `coneVec` — the dedendum (root) cone element Apex→C (pinion) / Apex→D (driving), realized as
  `normalize(heelConeWorld - apex)`;
- `v = axisDir × coneVec`, normalized — the circumferential direction;
- `tpNormal = coneVec × v`, normalized — the tangent-plane normal;
- `distAlong(p) = (p - apex) · coneVec` — a point's cone distance.

**The four toe/heel world points `_createGearBody` builds and passes positionally into
`_transformToothBody` in the order `toeMid, heelMid, toeConeWorld, heelConeWorld`. Pin them exactly;
mislabelling them silently inverts the spiral, and this is the single biggest spiral-regen hazard:**

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

`toeMid` is the world MIDPOINT of the TOE edge and `heelMid` the world midpoint of the HEEL edge —
two different edges. Do **NOT** pass the two endpoints of a single edge as the pair: M and N both sit
at the toe, so the span collapses to about zero or goes negative and the spiral inverts.
`heelConeWorld` is the **dedendum corner C/D**, on the root axis Apex→C / Apex→D, and **never H/J**,
which lie on the Apex2→C / Apex2→D dedendum line one Module beyond C/D, off the root cone element.

⚠️ **The heel MUST be the OUTER end.** Before building `coneVec`, check the passed midpoints and fix
a swapped pair: if `apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid` with `heelMid`
AND `toeConeWorld` with `heelConeWorld`, then build `coneVec = apex → heelConeWorld`. A negative span
silently inverts the entire spiral frame — the cutter-arc direction, the slice direction and the
per-segment twist — and the gear comes out completely wrong with no error.

From the midpoints, after that guard: `R_toe = distAlong(toeMid)`, `R_heel = distAlong(heelMid)`,
`R_mean = ½(R_toe + R_heel)`, `span = R_heel - R_toe`, now positive. These are the only quantities the
rest of the build needs.

**Build the two pieces of scaffolding.** Draw a **cone-element construction line** from the Apex to
`Apex + R_heel * coneVec` in a sketch on the axial Gear Profiles plane and name that sketch
`{gear} Cone Element`. Then make the tangent plane as that axial plane rotated **90°** about the
cone-element line — `plane_by_angle(designComponent, coneElementLine, axialPlane, 90)` — and name it
`{gear} Trace Plane`.

Both are transient construction geometry, consumed by the build and hidden in cleanup, and both are
exempt from the full-constraint gate. This step draws one unconstrained construction line and creates
one plane, so no proof function realises it; the geometry it carries is asserted inside S15.

**From:** `spec/bevelgear/instructions.md` L339-357, L605-651;
`spec/bevelgear/fusion.md` L59-67;
`spec/bevelgear/spiral-tooth-trace.md` L30-63;
`.claude/skills/generate-gear/PLAYBOOK.md` L430.

## S15 `[GO]` `{gear} 2D Tooth Trace` sketch — the cutter arc — `stepSpiralTrace`

<!-- proof-run: proofkit.RunParallel(latticeCases, stepSpiralTrace) -->

Work in the tangent-plane 2-D frame with the origin at the apex, **x = `coneVec`** — so a point's x
IS its cone distance — and **y = `v`**, circumferential. The cutter radius is `r_c = Cutter Radius`
if non-zero, **else `R_mean`**, which is the auto default. The hand sign is `handSign = +1` for
`Right` else `−1`, then **negated for the pinion**, because the pair meshes with opposite hands. The
cutter-circle centre is

```
Cx = R_mean - r_c * sin ψ
Cy = handSign * r_c * cos ψ
```

⚠️ **The hand sign goes on the `cos` / `Cy` term, NOT the `sin` / `Cx` term.** This was a real bug.
Opposite hands mirror the cutter centre **across the cone element (y = 0)**, which flips `Cy`.
Putting `handSign` on `Cx` mirrors about `x = R_mean` instead — a different curve that gives the two
gears **unequal twist**, where for equal teeth the driving and pinion traces must come out as exact
mirror images.

The trace's toe and heel arc endpoints are circle-circle intersections taken a hair **past** the face
so the kept arc reaches cleanly past the end trims:
`toe2d = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)` and
`heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)`, with
`R_lo = R_toe - 0.06 * span` and `R_hi = R_heel + 0.06 * span`. The helper intersects the apex circle
of that radius with the cutter circle and keeps the solution nearest `(R_mean, 0)`, the branch the
mean point sits on.

Add a sketch on the `{gear} Trace Plane` named **`{gear} 2D Tooth Trace`**. With
`tanW(px, py) = combine_point(apex, px, coneVec, py, v)` mapping 2-D coordinates to world, draw:

- the **cutter circle** — `sketch.sketchCurves.sketchCircles.addByCenterRadius(tanW(Cx, Cy), r_c)`,
  with `circle.isConstruction = True`, its centre pinned by
  `circle.centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`: the centre is a free point even
  when created at the origin, and `addCoincident` to the sketch origin has thrown
  `VCS_SKETCH_SOLVING_FAILED`), and a diameter dimension
  `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` of `2 * r_c`;
- the **trace arc** — `sketch.sketchCurves.sketchArcs.addByThreePoints(tanW(toe2d), tanW(R_mean, 0), tanW(heel2d))`
  — a three-point arc through the toe end, the mean point on the cone element and the heel end, with
  its **centre coincident to the cutter circle's centre**,
  `sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, circle.centerSketchPoint)`, and a
  **radius dimension** `sketch.sketchDimensions.addRadialDimension(arc, textPoint)` equal to `r_c`,
  so it is the genuine cutter circle and not a look-alike spline. ⚠️ Both text points must be
  OFF-CENTRE (`[PB-RADIAL-DIM]`): use the mean point `tanW(R_mean, 0)` for the arc's radius dimension
  and a point on the cutter circle such as `tanW(Cx + r_c, Cy)` for the circle's diameter dimension.
  The explicit coincident is required and is not redundant: `addByThreePoints` shares an arc's start
  and end points but **COPIES** the centre, which is the one place `[PB-SHARE-XOR-COINCIDENT]` says
  passing a point and coincidenting to it is correct. A stranded centre silently deforms the curve —
  found in Fusion at 22.9 mm behind its origin, giving a 0.5743 mm arc where 22.5 mm was intended, on
  a sketch that raised no error.

**Coordinates — this rule governs the `{gear} Cone Element` sketch as well.** The world `Point3D`s
from `tanW(...)`, and the raw apex and cone-end points of the cone-element line, are passed
**directly** into the sketch calls, where they are consumed as **sketch-space** input; **no
`modelToSketchSpace` conversion is applied**, even though `adsk.fusion.Sketch` offers exactly that
call and the points really are model-space coordinates. This is deliberate and harmless for a reason
that is not the obvious one: the trace sketch is construction and reference only, no downstream
feature ever consumes it, and the twist is computed analytically in S18 from the 2-D endpoints. The
cone-element line IS consumed, by `plane_by_angle`, so an unconverted line places the Trace Plane
somewhere other than the true tangent plane — and that still reaches no feature, because the only
thing built on the Trace Plane is this inspection-only sketch and the chain ends there. **If a later
revision ever makes any feature consume the trace sketch or the Trace Plane, this shortcut stops
being safe and both sketches need `modelToSketchSpace` on every point.**

**There is no 3-D projection.** No `projectToSurface`, no root-cone-face search and no 3-D trace
sketch. An earlier version projected the 2-D arc onto the root cone along `tpNormal` and measured the
azimuth there; for unequal-ratio pairs the arc wraps around the cone and the projection comes back as
multiple disjoint fragments, so the measured azimuth collapses to a fraction of the true sweep, the
pinion comes out grossly under-twisted and the pair interferes. Do not reintroduce it.

This sketch is **deliberately left with free degrees of freedom** — the arc's endpoints are pinned by
the three-point construction, not by endpoint dimensions, and dimensioning them over-constrains the
solve against the cone-element plane — so it is **exempt from the full-constraint gate**. Do not gate
it.

### What the proof establishes

`stepSpiralTrace` draws the cutter circle with its centre fixed and its diameter dimensioned, and
carries the trace's toe, mean and heel stations as pinned reference points on it. It then checks every
invariant a correct trace satisfies: the radius is the cutter radius; the centre is exactly `r_c`
from the mean point, so the trace passes through it; the mean point sits on the cone element at the
mean cone distance; the two ends sit on the toe and heel circles about the APEX — the most natural
centre to get wrong — each a hair past the face, and each ON the cutter circle; the angle between the
trace's tangent at the mean point and the cone element is exactly ψ; flipping the hand mirrors the
centre ACROSS the cone element and not about `x = R_mean`; and the twist is taken with the pitch cone
angle rather than the root one.

**THE ARC ITSELF IS THE ONE PIECE THIS HARNESS REFUSES, and the substitute is the circle it is a
portion of.** This engine attaches an internal equal-radius row to every arc, so an arc whose centre
and both ends are pinned carries a row for no freedom and the sketch reads overconstrained, while
leaving a point free to absorb that row makes the arc's centre or its ends a circle-circle
intersection, which is two solutions and reads as a discrete ambiguity. Everything the arc's own two
rows assert is asserted on the circle instead; what is not shown is that Fusion keeps the piece
between the two ends, or that a three-point arc through those three points is that piece. That, and
the free degrees of freedom the Fusion sketch deliberately keeps, are recorded in the proof file.

**From:** `spec/bevelgear/instructions.md` L626-660;
`spec/bevelgear/spiral-tooth-trace.md` L18-28, L64-182, L218-236;
`.claude/skills/generate-gear/PLAYBOOK.md` L442-448, L605-625, L643-647.

## S16 `[GO]` Slice the tooth into slabs — `stepSliceTooth`

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepSliceTooth, assertSliceTooth) -->

<!-- check-compile: ignore slice_body_by_offset_planes -->
`slice_body_by_offset_planes` is a framework helper in `lib/geargen/solids.py`.

Split the uncut apex-to-heel Tooth Body into cross-section slabs with planes **perpendicular to the
cone element**, spanning a touch past toe and heel, on a **fixed** scheme of eight planes; the count
is not user configurable.

The first cut plane is the **parent transverse tooth plane** — `parentToothPlane`, the
`{gearLabel} Plane` of S7, passed into the hook — offset toward the apex by `span/6`. The offset
**sign is chosen per gear** so that it moves toward the apex, because the parent plane's normal points
opposite ways for the two gears: test `(apex - planeOrigin) · normal` and pick the sign for which
`sign * normal` points apex-ward. Then a sequence of about eight planes stepped further toward the
apex in `span/6` increments, `sign * (k+1) * span/6` for k = 0…7, k = 0 being the first cut plane.
Split with `slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)` where
`offsets = [sign * (k+1) * span/6 for k in 0…7]`; it splits piece by piece and keeps a piece whole
when a plane misses it.

⚠️ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in ONE piece
— no plane cut it — the offset sign was wrong or `parentToothPlane` sits outside the tooth's span, so
**retry the whole cut once with the opposite sign**. If it is still one piece, **raise a clear
self-diagnosing error** naming the gear, the final piece count, `span` and the sign tried
(`[PB-SELF-DIAGNOSING]`, `[PB-EMPTY-RESULT]`). Do **NOT** return an unsliced single-piece result:
S17 then drops that one piece as the apex scrap, leaving the segments empty, and the crown later
crashes with `ValueError: max() iterable argument is empty` far from the cause.

### What the proof establishes

decad cannot split a Loft, and every slab is one, so `stepSliceTooth` BUILDS the pieces at the
stations the cut planes would have left them at. The assertion checks that the scheme produces pieces
at all, that there are nine of them — eight cut planes and the piece beyond each end — that the cut
cone distances rise strictly toward the heel, that every step is one `span/6`, that the first cut
sits exactly one step from the parent tooth plane and on the apex side of it, that the parent plane
meets the root element at `|Apex→Ded|`, and that the span is positive, which is the frame guard S14
describes.

The proof works in **cone distance** throughout, because a plane perpendicular to the element gives
every point on it the same cone distance, so the slab keys stay the real ones under the
axis-perpendicular substitution the file records. **What the substitution drops is the evaluator's own
division — including the retry-with-the-opposite-sign guard, which only a real cut can miss.**

**From:** `spec/bevelgear/instructions.md` L662, L828-830;
`.claude/skills/generate-gear/PLAYBOOK.md` L175-177, L431, L753-760.

## S17 `[GO]` Order the slabs and drop the apex scrap — `stepDropApexScrap`

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepDropApexScrap, assertDropApexScrap) -->

<!-- check-compile: ignore physicalProperties centerOfMass removeFeatures -->
Sort the segments by the `distAlong` of their centroid, `body.physicalProperties.centerOfMass`. The
first — the apex-most — is the long **apex-side scrap** below the toe: remove it and keep the rest as
the working `segments`. **Drop the scrap by re-slicing the list FIRST and deleting it after** —
`segments = segments[1:]` before
`designComponent.features.removeFeatures.add(scrap)` — because a removed body cannot then be read
(`[PB-REMOVE-PIECES]`).

After dropping the scrap, `segments` must be **non-empty**, at least one cross-section. If it is
empty the slice failed in S16: raise a clear error rather than proceeding into the twist and the
crown, both of which assume at least one segment.

### What the proof establishes

`stepDropApexScrap` builds the segments that survive the drop and checks that some do; that there are
eight, the nine pieces less the scrap; that the dropped piece really was the apex-most, the first
kept segment starting exactly at the first cut; and that the scrap really was the long one, spanning
further than the shortest kept segment.

**From:** `spec/bevelgear/instructions.md` L664, L828-830;
`.claude/skills/generate-gear/PLAYBOOK.md` L431, L761-765.

## S18 `[GO]` Twist the slabs — `stepTwistSegments`

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepTwistSegments, assertTwistSegments) -->

Rotate each segment about the **shaft axis** — `axisDir` through `apex` — so the tooth follows the
trace, **centred on `R_mean` so the mid-face section stays unrotated**. That section then meshes
exactly like the straight tooth, which is what the pinion's zero mesh nudge depends on.

The total toe-to-heel shaft-axis twist comes from the **conjugate crown-gear generation law**: a
spiral bevel is generated by an imaginary flat crown gear, and the work gear's shaft rotation relates
to the developed crown-plane azimuth by the roll ratio `1 / sin γ`, the generating crown gear having
`N / sin γ` teeth. Compute it **analytically — no projection, no curve sampling**:

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`phi_crown` is the angle the cutter arc's toe and heel endpoints subtend at the apex in the flat 2-D
crown frame — exactly the `toe2d` / `heel2d` pairs from S15. ⚠️ **`gamma` is this gear's PITCH cone
angle**, `self._gamma_p` for the pinion and `self._gamma_g` for the driving gear, already computed in
S2 — **NOT `acos(coneVec · axisDir)`**, which is the root cone angle, about 14° against the pitch's
29° for a 17-tooth pinion, and yields a twist about 1.6 times too large. ⚠️ The two members of a
meshing pair **legitimately get different twists**: same cutter, same ψ, but γ differs, so
`1 / sin γ` differs — about 2.08 for a 17-tooth pinion against about 1.14 for a 31-tooth gear. That
is why equal-teeth pairs always meshed while ratio pairs failed under any method that gets the roll
ratio wrong.

Each segment's rotation is a **linear share keyed to the cone distance of its HEEL FACE**:

```
ang = -handSign * total * (R_mean - R_heelFace(seg)) / span
```

⚠️ **Key the twist on the segment's HEEL-FACE cone distance, NOT its centroid.** The S20 loft samples
each segment's heel face, so that face is what must land at the right azimuth; centroid keying leaves
the loft's mid-face section rotated by half a segment and the mid-faces overlap.

**Define a slab's heel face precisely: the face whose centroid has the GREATEST
`distAlong(face.centroid)`, searched across ALL of the slab's faces with NO surface-type filter**;
its toe-side face is the least-centroid one. ⚠️ Do **NOT** restrict this search to
`PlaneSurfaceType`, or to any surface type — a sliced slab is bounded by a mix of the two planar cut
faces and ruled side faces, and a type filter can pick the wrong face or miss the cut face, which
makes the S20 loft fail with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this same
all-faces-by-centroid rule wherever a slab end face is needed: here, in S19 and in S20.

Apply the rotation as a free move by a rotation matrix:
`matrix = adsk.core.Matrix3D.create()`, then `matrix.setToRotation(ang, axisVector, apexPoint)`,
then `moveInput = designComponent.features.moveFeatures.createInput2(bodyCollection)`,
`moveInput.defineAsFreeMove(matrix)` and `designComponent.features.moveFeatures.add(moveInput)`
(`[PB-MOVE-ROTATE]`; use `defineAsFreeMove` with a matrix, not `defineAsRotate`, which rejects a
`SketchLine` axis).

### What the proof establishes

`stepTwistSegments` builds the twisted segments and reads each one's rotation off the body against an
untwisted copy of the same slab, checking it is that slab's own linear share. It then checks the
twist is centred on `R_mean` — no segment turns less than the one nearest the mean cone distance —
that the toe-to-heel twist is `|phi_crown| / sin(pitch cone angle)` and not the root-angle reading,
and that flipping the hand mirrors every share and changes nothing else, which is what makes an
equal-teeth pair's two traces exact mirror images.

**One reading the harness does not publish** is recorded in the proof file: decad has `Body.Centroid`
and no `Face.Centroid`, so the proof computes each end face's centroid from the section polygon it
built that face from. The all-faces-no-filter part of the rule is therefore NOT exercised — a proof
that hands itself the right face cannot catch a module that filters on `PlaneSurfaceType` and picks
the wrong one.

**From:** `spec/bevelgear/instructions.md` L666-679, L828-830;
`spec/bevelgear/spiral-tooth-trace.md` L186-214;
`.claude/skills/generate-gear/PLAYBOOK.md` L791-800.

## S19 `[GO]` Crown the slabs — `stepCrownSegments`

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepCrownSegments, assertCrownSegments) -->

<!-- check-compile: ignore _CROWN_PER_RAD -->
Crown the tooth by scaling each segment **except the outermost (heel) one** down by a **monotonic**
factor — full at the heel, growing smoothly toward the toe — **about a sketch point on the ROOT edge
of its heel face**.

For each segment compute its **heel-distance fraction** `u = (R_heel - R_heelFace) / span`, with
`R_heelFace` the `distAlong` of that segment's heel face found by the S18 all-faces-by-centroid rule
but **RECOMPUTED here, AFTER the twist has moved the slabs** — do not reuse pre-twist values — and
`R_heel` and `span` from S14. `u` runs 0 at the held-full heel to 1 at the toe. **"Outermost (heel)
segment" is the one with the GREATEST post-twist heel-face `distAlong`**: sort by that and skip the
last. Then

```
factor = 1 - _CROWN_PER_RAD * (abs(total) / 2) * u
```

`total` is the full toe-to-heel twist from S18, so `abs(total)/2` is the per-end peak twist magnitude
and the maximum relief — now at the toe — keeps the magnitude the old per-end peak had, just
relocated. This makes relief grow monotonically from the full heel to the toe, so slab heights stay
strictly ordered heel to toe and the natural cone taper is never reversed. If a computed `factor`
comes out **≤ 0**, raise a self-diagnosing error naming the gear, the segment's `u` and the factor;
never scale by a non-positive factor. **`_CROWN_PER_RAD` is a tunable class constant with the value
`0.5`** — 0 disables the crown; set it to 0.5 and do not leave it unset.

⚠️ **Do NOT key the relief on `abs(ang)`, the twist magnitude.** That is symmetric about mid-face,
maximal at BOTH ends, so with the heel slab held full the slab just inside the heel becomes the
**most** relieved one and dips below both its neighbours — a notch that reverses the heel-to-toe
taper. Measured on the bug this rule comes from, the heel-adjacent slab came out at factor 0.932
while the next slab inward was 0.972, which is taller. Key on the monotonic `u`, never on `abs(ang)`.

**Three gotchas.**

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). `scaleFeatures` is the ONE exception to
   never-activate: it needs the Design occurrence as the **active** edit target, so call
   `designOccurrence.activate()` before the crown scales and restore afterwards, in a `finally`,
   with `design.activateRootComponent()`. ⚠️ Do **NOT** write `design.rootComponent.activate()` or
   `someComponent.activate()` — a `Component` has **no** `activate` method and raises
   `AttributeError`. Only `Occurrence` has it, and the root is re-activated through
   `Design.activateRootComponent`.
2. **Skip the outermost (heel) segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone trims it flush with the gear base.
3. ⚠️ **Anchor the scale on the heel face's ROOT edge, NOT its centroid — otherwise the crowned tooth
   lifts off the gear base.** `scaleFeatures` shrinks uniformly toward the base point, so a base at
   the heel face's centroid, at mid tooth-height, pulls the tooth's root edge upward by
   `(1 - factor) * (½ tooth height)`: the tooth no longer seats on the gear body's root cone, floats
   above the base, and the Combine-Join leaves a gap — clearly visible on ratio pairs such as
   Module 2 with driving 19 and pinion 13, which is the symptom that exposed this. Put the base point
   on the root instead: of the heel face's vertices (`heelFace.vertices`, each `.geometry` a world
   `Point3D`), take the **two with the smallest perpendicular distance to the shaft axis** — the line
   through `apex` along `axisDir`, the distance being
   `|(p - apex) - ((p - apex) · axisDir) * axisDir|` — those are the two **root corners**, the tip
   corners being the farthest — and place the base sketch point at their **midpoint**, mapped into
   the heel-face sketch with `sketch.modelToSketchSpace(worldPoint)`. The heel face is a planar cut,
   so that midpoint lies on it. A uniform scale about a point keeps every line and plane through that
   point invariant, so anchoring on the root keeps the root edge on the seating cone while the tip is
   relieved progressively toward the toe, which is exactly the lengthwise crown intended.

The feature itself is
`scaleInput = designComponent.features.scaleFeatures.createInput(bodyCollection, basePoint, adsk.core.ValueInput.createByReal(factor))`
then `designComponent.features.scaleFeatures.add(scaleInput)`.

### What the proof establishes

decad has no scale feature, so `stepCrownSegments` builds each crowned slab with its sections already
scaled about the base point rather than scaled after the fact — the same base point, the root edge of
the heel face at the tooth's own symmetry axis.

The assertion checks the outermost segment is held full; that every other factor is exactly
`1 - 0.5 * |total|/2 * u` and strictly positive; that the factors are monotonic from the held heel to
the toe, sorted by heel distance, so no slab dips below its neighbours; that after the crown the heel
face still carries a vertex at the base point's own radius, which is how "the root edge has not
lifted off the seating cone" is read; that the heel face's tip is relieved below the uncrowned one;
and that a centroid-anchored scale WOULD have lifted the root by a positive amount, so the choice of
base point is not academic on this geometry.

**From:** `spec/bevelgear/instructions.md` L681-693, L828-830;
`.claude/skills/generate-gear/PLAYBOOK.md` L576-581, L782-790.

## S20 `[GO]` Loft the spiral tooth — `stepLoftSpiralTooth`

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepLoftSpiralTooth, assertLoftSpiralTooth) -->

⚠️ **Re-sort the segments by their heel-face cone distance HERE, AFTER the twist and the crown — do
NOT reuse the pre-twist slice or centroid order.** The twist rotates each slab about the shaft axis,
and for high-twist unequal-ratio pairs that rotation changes the slabs' along-cone order enough to
**reorder adjacent slabs**; lofting in the stale pre-twist order assembles the cross-sections out of
sequence and the crowned tooth comes out distorted, so the two gears interfere. For equal or low-twist
pairs the two orders coincide, which is why equal-teeth gears mesh even with the stale order while
unequal ratios distort — this is the single thing that makes a ratio pair like 31/17 fail while 31/31
looks fine.

<!-- check-compile: ignore sorted slabHeelFace -->
<!-- check-step-calls: ignore sorted slabHeelFace -->
`sorted` is Python's own and `slabHeelFace` is the shorthand this step uses for the
all-faces-by-centroid rule of S18, so neither is a call the module must make by that name.

So compute `order = sorted(segment indices, key = distAlong(slabHeelFace(seg).centroid))` **now**,
and loft a **new body** through, in that order:

1. first the **toe-most segment's apex-side (toe-facing) face** — the toe segment is `order[0]`, and
   its toe face goes first so the loft pushes past the toe cone and the toe trim bites;
2. then the **heel-facing face of every segment, iterated in `order`** — each segment's
   farthest-along-the-element face by post-twist centroid, the last reaching past the heel cone.

Add each with `loftInput.loftSections.add(face)` in that order; `loftInput` comes from
`designComponent.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`
and the body from `designComponent.features.loftFeatures.add(loftInput)`. Name the resulting body
**`{gear} Spiral Tooth`**. Then remove the segment scaffolding, the loft having captured their faces.

**Flush trim and mesh phase.** Return
`cut_conical_ends(designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)` —
the same toe-then-heel two-cone trim S13 describes — so the curved tooth's ends sit flush on the gear
base. The toe and heel **mesh phasing** is handled outside this hook, by S25's mesh-rotate step; the
pinion's extra phase is 0 by default because the mid-face section is unrotated and already meshes.

### What the proof establishes

decad's Loft takes exactly two profiles, so `stepLoftSpiralTooth` builds the nine-section loft as its
consecutive pairs and lays them apart. **The cost is the single body; what is kept is the ORDER**,
which is the whole subject of this step.

The assertion checks the loft runs through the sections in strictly rising cone distance, and that the
first section is the toe-most segment's apex-side face — no other segment reaches further toward the
apex.

What this cannot reach is recorded beside it: where the twist is small the post-twist order and the
pre-twist slice order coincide, so the proof shows the order it uses is the post-twist one and never
that a module using the stale one would be caught. That distinction needs a real split, whose piece
list carries the slice order with it.

**From:** `spec/bevelgear/instructions.md` L695-697, L828-830;
`.claude/skills/generate-gear/PLAYBOOK.md` L715-719.

## S21 `[GO]` Circular-pattern the teeth — `stepCircularPattern`

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCircularPattern, assertCircularPattern) -->

Run once per gear. First create this gear's own component: a child of the **Bevel Gear** component —
the same component that owns Design, *not* the user's Parent Component — named `{gearLabel} Gear`,
so `Pinion Gear` and `Driving Gear`. The finished bodies for this gear end up there. Fusion rejects
cross-sibling sketch and project calls even when the target is activated or the entities are wrapped
in `createForAssemblyContext` proxies (`[PB-NO-CROSS-SIBLING]`), so every feature operation runs in
the Design component and the finished bodies are moved across at the end, in S26. The visible end
state is identical.

Circular-pattern the remaining tooth piece around the **shaft-axis edge** — the same in-sketch profile
edge the revolve used, never the §2 construction line:

`patternInput = designComponent.features.circularPatternFeatures.createInput(bodyCollection, shaftAxisEdge)`,
then pin all three inputs explicitly (`[PB-CIRCULAR-PATTERN]`) —
`patternInput.quantity = adsk.core.ValueInput.createByReal(teethNumber)`,
`patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')`,
`patternInput.isSymmetric = False` — then
`designComponent.features.circularPatternFeatures.add(patternInput)`. The number of copies is this
gear's Teeth Number.

Although the pitch diameter shrinks from heel toward apex, the ANGULAR spacing around the shaft axis
stays constant at `360° / N` for the entire face width: the radial taper is already produced by the
apex-to-heel loft, so the pattern just rotates that single tapered tooth into N evenly spaced copies.

### What the proof establishes

`stepCircularPattern` builds the seed tooth, reads what the pattern must preserve, and returns the
first increment — which is what retires the seed. The remaining copies are measured one document at a
time, because decad verifies every PAIR of live bodies and a real gear's teeth resolve neither as
disjoint nor as overlapping, which the gate refuses. The assertion checks every copy sits at
`2π k / N` measured one way from the seed, which a symmetric pattern or a different total angle would
not produce, and that each carries the seed's own volume, reach and height, so none is deformed by
its placement.

**THIS STEP IS SERIAL, on `proofkit3d.RunSolid`, while every other bevel step runs its cases in
parallel.** The pattern increment retires the seed tooth, so the seed cannot be measured after the
step runs, and its azimuth, radius, height and volume have to be read during the build and handed to
the assertion through package-level variables. That hand-off leaves the case, and two cases sharing
one set of readings overwrite each other. It is not a hazard that announces itself: the two gear
sides differ enough in volume that an overwrite was caught when it happened, and a pair of cases whose
seeds measured alike would have passed on each other's numbers instead. The carried readings and this
reason are recorded beside them in the proof file.

**From:** `spec/bevelgear/instructions.md` L713, L750-751, L848-872;
`.claude/skills/generate-gear/PLAYBOOK.md` L684-694, L820-825.

## S22 `[GO]` Combine-Join the teeth into the Gear Body — `stepCombineTeeth`

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCombineTeeth, assertCombineTeeth) -->

Run once per gear. Join all patterned tooth pieces with the Gear Body in a **single** Combine-Join,
the Gear Body as the target and the patterned tooth bodies as the tools:
`combineInput = designComponent.features.combineFeatures.createInput(gearBody, toolCollection)`,
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`, then
`designComponent.features.combineFeatures.add(combineInput)`.

`CircularPatternFeature.bodies` already includes the seed body plus the copies, so do not re-add the
seed, and **copy them into an `adsk.core.ObjectCollection` first** — `pattern.bodies` is a
`BRepBodies` and `createInput` rejects it (`[PB-PATTERN-BODIES]`). Loop `pattern.bodies.item(i)` into
a fresh `adsk.core.ObjectCollection.create()` and pass that.

### What the proof establishes

`stepCombineTeeth` performs NO join: both operands are Lofts, so the boolean is unavailable. It lays
the Gear Body's root band and the seated tooth apart and reads the join's two consequences off their
own geometry — a join leaves ONE lump when the tooth's root is at or below the body's root cone,
seated rather than floating, and the joined body reaches further out than the frustum when the
tooth's tip stands proud of it. Both readings are taken at the toe, the middle and the heel of the
band the join would cover. It also checks the seated tooth is still the tooth this gear's virtual
tooth number and Module draw, scaled onto the cone rather than reshaped.

⚠️ **The proof sinks the tooth's root a twentieth of the tooth height below the gear body's root
cone**, which is what makes "seated" measurable as a strict inequality rather than an equality inside
a tolerance. **The generated module seats the tooth exactly on the cone and must not sink it** — the
sink belongs to the proof alone, and it is marked as such in the proof file.

**The cost is the stitch**: the proof cannot show the evaluator making one boundary out of two.

**From:** `spec/bevelgear/instructions.md` L752-753, L812-820;
`.claude/skills/generate-gear/PLAYBOOK.md` L684-688.

## S23 `[GO]` `{gearLabel} Bore` sketch — `stepBoreSketch`

<!-- proof-run: proofkit.RunParallel(latticeCases, stepBoreSketch) -->

Run once per gear, and **skip this step entirely if Enable Bore is unchecked**. The bore diameter is
this gear's Bore Diameter if specified and non-zero, otherwise **this gear's Pitch Diameter / 4**.

Build the bore plane normal to the shaft at its start:
`planeInput = designComponent.constructionPlanes.createInput()`, then
`planeInput.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))`, then
`designComponent.constructionPlanes.add(planeInput)`. Pass the in-sketch edge, not the §2
construction line.

Create a sketch on that plane named `{gearLabel} Bore`, and sketch the bore circle centred at the
sketch origin — the plane is rooted at the shaft start, so the origin is on the axis:
`sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), radius)`,
then **fix the centre and dimension the diameter** —
`circle.centerSketchPoint.isFixed = True` and
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` set to the bore diameter
(`[PB-CIRCLE-CENTER]`: a circle created at the origin does not reuse the sketch's `originPoint`, its
centre is a free point that happens to sit there, and `addCoincident` to the origin has thrown
`VCS_SKETCH_SOLVING_FAILED` on a `setByDistanceOnPath` plane). Gate the sketch: raise if
`sketch.isFullyConstrained` is false (`[BEVEL-F-FULL-CONSTRAINT]`).

### What the proof establishes

`stepBoreSketch` draws the circle with its centre fixed and its diameter dimensioned, gates it at
DOF 0, and reads back the diameter and the one region the cut consumes. Where the two bore inputs are
left at 0 it also checks the auto value is THIS gear's own Pitch Diameter / 4 and never the pair's or
the other gear's. A case with Enable Bore unchecked is skipped as unmodelled, naming the reason,
because no Bore sketch is drawn at all then.

**From:** `spec/bevelgear/instructions.md` L100-104, L755;
`.claude/skills/generate-gear/PLAYBOOK.md` L442-448.

## S24 `[GO]` Bore cut — `stepBoreCut`

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepBoreCut, assertBoreCut) -->

Run once per gear, and skip it entirely if Enable Bore is unchecked. Extrude-cut the bore circle as a
symmetric through-cut restricted to this Gear Body:

`extrudeInput = designComponent.features.extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
then `extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(2 * coneDistance_cm), False)`
— the second argument `isFullLength=False` means the distance is the half-length **per side**, and
`2 * Cone Distance` is generously past any face width; do not pass a third taper argument
(`[PB-THROUGH-CUT]`) — then `extrudeInput.participantBodies = [gearBody]`, then
`designComponent.features.extrudeFeatures.add(extrudeInput)`.

### What the proof establishes

The TOOL is a real extrude, which a symmetric extent produces as a prism. **No cut is performed** —
the target is the frustum, whose bands are Lofts — so `stepBoreCut` lays the tool and the frustum's
root band apart and reads the cut off the tool's own geometry: its radius is the bore diameter's
half; its two ends sit exactly `2 * Cone Distance` either side of the shaft edge's start; and both
ends clear the frustum, which is what makes it a THROUGH cut. It then computes the material the cut
would remove, the frustum's own profile clipped to the bore radius and revolved, and checks it is a
real bite — positive, and short of the whole frustum.

**The cost is the pierced body**: one lump with a hole and no enclosed void is not shown.

**From:** `spec/bevelgear/instructions.md` L755, L821-827;
`.claude/skills/generate-gear/PLAYBOOK.md` L720-723.

## S25 `[GO]` Meshing rotation — `stepMeshRotation`

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepMeshRotation, assertMeshRotation) -->

<!-- check-compile: ignore rotate_body_about_edge _pinionMeshPhase _PINION_MESH_PHASE_TEETH -->
`rotate_body_about_edge` is a framework helper in `lib/geargen/solids.py`; `_pinionMeshPhase` is a
method the module defines and `_PINION_MESH_PHASE_TEETH` a class constant it declares.

<!-- check-step-calls: ignore _pinionMeshPhase -->
`_pinionMeshPhase(pinionTeeth)` is named here so its surface is fixed; it returns the pinion's extra
mesh rotation in **radians**, `_PINION_MESH_PHASE_TEETH * 2π / pinionTeeth`, and
`_PINION_MESH_PHASE_TEETH` defaults to **0**.

**Driving gear only, and do it here — in the Design component, before the body is moved out.**
Rotate the driving body by `180° / Driving Gear Teeth Number`, half a tooth pitch, about its shaft
axis: `rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)`, which takes the
rotation axis and origin from the B′→I profile edge's **world** endpoints (`[PB-MOVE-ROTATE]`). A
driving valley then sits where the pinion tooth crosses the axial plane, giving the interlocked
meshing look: both gears are patterned from a starting tooth in the axial plane, so without the offset
a driving tooth and a pinion tooth would both sit at the axial-plane crossing and visually collide.

This runs in Design before `moveToComponent` because a construction axis cannot be added in the moved-
out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation must use the edge's world
geometry while the body is still in Design. The pinion additionally gets
`_pinionMeshPhase(pinionTeeth)`, which is 0 unless a spiral pair needs it.

⚠️ **A zero angle is a no-op, not a move.** `setToRotation(0, axis, origin)` builds the identity and
Fusion refuses to move a body by it, with `RuntimeError: 3 : invalid transform` — measured on the
bevel pinion, whose mesh phase is 0 by default. `rotate_body_about_edge` absorbs that with an early
return, which is why each call site does not guard it.

### What the proof establishes

`stepMeshRotation` turns a tooth — a body that is not axisymmetric, so the rotation is readable —
and measures its azimuth against an unturned copy. It checks the driving gear turned exactly half a
tooth pitch and the pinion turned 0, that the driving figure really is `π / N`, and that the rotation
changed the body's volume by nothing, a rotation not being a reshaping. It takes the same early
return at a zero angle that the framework helper does, for the same reason: this proof's own rotation
constructor refuses a zero angle too.

**From:** `spec/bevelgear/instructions.md` L356-357, L757;
`.claude/skills/generate-gear/PLAYBOOK.md` L791-800.

## S26 `[PROSE]` Move the finished bodies into the gear components

<!-- check-compile: ignore moveToComponent -->
Once this gear's body is complete and rotated, relocate it out of Design and into the gear's own
component with `body.moveToComponent(gearOccurrence)`, which preserves world position and needs no
activation (`[PB-NO-CROSS-SIBLING]`).

No proof function realises this step: decad has no component tree, and the move preserves world
position, so there is no geometry for a proof to read. Every measurement the move would preserve is
already asserted in S21 through S25, in the frame the bodies keep.

**From:** `spec/bevelgear/instructions.md` L713;
`.claude/skills/generate-gear/PLAYBOOK.md` L820-825.

## S27 `[PROSE]` Cleanup

<!-- check-compile: ignore hide_construction_geometry -->
Call the framework helper `hide_construction_geometry(bevelComponent)` from `.solids`. It recursively
walks the Bevel Gear component tree, deduping by `entityToken`, and hides every sketch, construction
plane and construction axis with `isLightBulbOn = False` — construction planes and axes are **not**
hidden by `isVisible` (`[PB-HIDE-AFTER-USE]`, `[BEVEL-F-CLEANUP]`). Leave only the two finished gear
bodies visible. There is no sketch-only mode and no per-mode guard; bevel always builds solids.

Do **not** add a display-settling call of any kind. `commands/_gear_command.py` calls
`geargen.settle_sketch_display` once after `generate()` returns, every gear command runs through that
one call, and nothing about it belongs in a generated module (`[PB-SETTLE-DISPLAY]`).

The driving gear's meshing rotation is performed earlier, at S25, in the Design component before the
body is moved out. It is not a cleanup step.

This step changes visibility and no geometry, so no proof function realises it.

**From:** `spec/bevelgear/instructions.md` L761-765;
`spec/bevelgear/fusion.md` L161-166;
`.claude/skills/generate-gear/PLAYBOOK.md` L525-546, L650-662.
