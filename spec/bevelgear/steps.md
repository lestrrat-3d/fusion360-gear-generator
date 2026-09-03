# Bevel Gear — compiled step list

The proof for these steps is `proof/bevelgear/geometry_test.go`, `proof/bevelgear/inputs_test.go`,
`proof/bevelgear/lattice_test.go`, `proof/bevelgear/tooth_test.go`, `proof/bevelgear/solids_test.go`,
`proof/bevelgear/spiral_test.go` and the generated `proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `6cb50bdb875e150a1504b22f13126f4d672c6eca` |
| `spec/bevelgear/fusion.md` | `874624488756f04d431351de29557f279cdbd9c7` |
| `spec/bevelgear/spiral-tooth-trace.md` | `c9ec08561ced7975aa0ed9ad6a330186259c0d08` |
| `spec/spurgear/instructions.md` | `560549a350c789b6598ee49b72d3cd7b4460e214` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `aa97523b214dbc98b679e8eea73c56c5115c4e54` |

## S01 `[GO]` Read and validate every dialog input

Read all 17 inputs in one `_readInputs` pass **before anything creates an occurrence**, and validate
in the order below. Nothing here writes to the timeline; every later step reads what this one
resolves.

Read each input with the helper that matches how it was declared (`[PB-INPUT-READ]`): selections
with `get_selection`, the Enable Bore checkbox with `get_boolean`, the dropdown through
`itemById(INPUT_ID_HAND).selectedItem` (default `Right` when none), and every numeric or angle field
by evaluating its expression, `design.unitsManager.evaluateExpression(input.expression, units)`,
which always returns Fusion internal units — cm for length, **radians** for angle — whatever unit
string is passed (`[PB-EVAL-EXPRESSION]`). Convert an angle with `math.degrees(...)` before any
degree range check.

**Units.** The `mm` inputs (both base heights, both bore diameters, Face Width, Tooth Spacing) and
the `deg` inputs (Shaft Angle, Mean Spiral Angle) come back already internal — use them as-is. The
`Module` input is read with unit `''` and comes back as a raw number meaning **millimetres**, so
every length derived from Module must be `to_cm`-converted before it touches geometry: both pitch
diameters, the Cone Distance, the dedendum `1.25 * Module`, the module-length extensions and the
default Face Width. Coerce both tooth counts with `int(round(...))`.

Resolve, in this order:

1. `Module > 0`; each tooth count `>= 3`; heights, bores, Face Width and Tooth Spacing non-negative;
   Cutter Radius non-negative; Mean Spiral Angle in `[0, 60)` degrees.
2. **Maximum Shaft Angle** = `min(150, degrees(acos(-min(PPD, DPD) / max(PPD, DPD))))`. The
   cone-angle half is **exclusive** and the 150° half **inclusive**; the Shaft Angle floor is an
   inclusive 30°. Name the computed limit in the rejection message. A 31/17 pair's limit is
   `acos(-17/31) = 123.26°`, not the flat 150° earlier revisions promised; an equal pair gives
   `acos(-1) = 180°`, so the 150° cap is what binds.
3. The two pitch cone angles from the closed form: `tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`,
   `γ_g = Σ − γ_p`, and `R = (PPD / 2) / sin γ_p`.
4. **Minimum Teeth** per gear, `5.27 * cos γ` with that gear's own γ, on top of the blanket
   `teeth >= 3`. Name the computed floor.
5. **The base-height window per gear**, closed-form and needing no sketch:
   `Maximum Base Height = 0.95 * (r - 1.25 * Module * cos γ) * tan γ` and
   `Minimum Base Height = 1.05 * 1.25 * Module * sin γ`, with `r` that gear's own pitch radius. The
   **driving gear resolves first**: its fallback is `Module * Driving Gear Teeth Number / 8`, raised
   to the minimum and capped at the maximum; the pinion's fallback is the **resolved** driving
   height times `Pinion Gear Teeth Number / Driving Gear Teeth Number`, then held to the **pinion's
   own** window. A user-specified value outside either end is rejected naming the bound it broke.
6. Bore diameters: a zero means auto, `this gear's Pitch Diameter / 4`. Both are ignored when Enable
   Bore is unchecked.

Order matters: the Minimum Teeth check is the statement that the base-height window is non-empty, so
running it first means step 5 never meets a minimum above its maximum.

The Face Width is **not** resolved here — its cap needs the solved §2 geometry (see S06).

From `.base` import **only** the two input readers `get_selection` and `get_boolean`: this
generator does not subclass `base.Generator` and uses none of its `Generator`,
`GenerationContext`, `ParamNamePrefix` or `ComponentCleaner` machinery. Every import is explicit —
no `import *` anywhere in the module.

`_readInputs` returns the 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth,
pinionTeeth, shaftAngle_deg)` and stashes the rest on `self` (`self._drivingBaseHeight_cm`,
`self._pinionBaseHeight_cm`, `self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`,
`self._faceWidth_cm`, `self._toothSpacing_cm`, `self._spiralAngle_rad`, `self._hand`,
`self._cutterRadius_cm`); generate() then stashes `self._coneDistance_cm`, `self._gamma_p` and
`self._gamma_g`. There is no `GenerationContext` and no user parameter — every value is precomputed
in Python in internal cm (`[PB-PRECOMPUTED-MODE]`).

`get_selection(...)`, `get_boolean(...)`, `design.unitsManager.evaluateExpression(...)`, `math.degrees(...)`,
`to_cm(...)`, `int(...)`, `round(...)`, `math.acos(...)`, `math.atan2(...)`, `math.tan(...)`,
`math.sin(...)`, `math.cos(...)`

Proof: `stepInputBounds`, which checks each bound from both sides — a value inside it resolves, a
value outside is refused naming its computed figure — and draws the heel edge the base-height window
is really about.

<!-- proof-run: proofkit.Run(inputCases, stepInputBounds) -->

**From:** `spec/bevelgear/instructions.md` L25–L130 (Variables), L188–L245 (Units and reading the raw
numbers), L273–L295 (Generation Context); `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-INPUT-READ]`, `[PB-EVAL-EXPRESSION]`, `[PB-PRECOMPUTED-MODE]`.

## S02 `[PROSE]` Configure the command dialog

`BevelGearCommandInputsConfigurator.configure(cmd)` adds the 17 inputs below to `cmd.commandInputs`,
in the row order given. That order is the dialog's display order and is fixed: Target Plane is added
first so it wins Fusion's auto-focus, which ignores a later focus flag (`[PB-AUTOFOCUS-FIRST]`);
Center Point follows so the user flows from plane to point; the pre-selected Parent Component comes
third; then the numeric and boolean fields, with the three spiral fields last.

**Every string in this table is reproduced surface** — the ids, the labels, the unit strings, the
tooltips and the two dropdown item names are literals the module carries exactly as written here:

| # | dialog label | input id | added with | unit | default | filters, limits, tooltip |
|---|---|---|---|---|---|---|
| 1 | Target Plane | `targetPlane` | `addSelectionInput(...)` | — | — | `ConstructionPlanes`, `PlanarFaces`; limit 1; tooltip `Plane the bottom of the driving gear sits flush against` |
| 2 | Center Point | `centerPoint` | `addSelectionInput(...)` | — | — | `ConstructionPoints`, `SketchPoints`; limit 1; tooltip `Point the driving bevel gear is centered on` |
| 3 | Parent Component | `parentComponent` | `addSelectionInput(...)` | — | root component pre-selected | `Occurrences`, `RootComponents`; limit 1; tooltip `Component the gear pair is created under` |
| 4 | Module | `module` | `addValueInput(...)` | `''` | `createByReal(1)` | — |
| 5 | Shaft Angle | `shaftAngle` | `addValueInput(...)` | `deg` | `createByString('90 deg')` | — |
| 6 | Driving Gear Teeth | `drivingTeeth` | `addValueInput(...)` | `''` | `createByReal(31)` | — |
| 7 | Pinion Gear Teeth | `pinionTeeth` | `addValueInput(...)` | `''` | `createByReal(31)` | — |
| 8 | Driving Gear Base Height | `drivingBaseHeight` | `addValueInput(...)` | `mm` | `createByReal(to_cm(0))` | — |
| 9 | Pinion Gear Base Height | `pinionBaseHeight` | `addValueInput(...)` | `mm` | `createByReal(to_cm(0))` | — |
| 10 | Enable Bore | `boreEnable` | `addBoolValueInput(...)` (checkbox) | — | `True` | — |
| 11 | Driving Gear Bore Diameter | `drivingBore` | `addValueInput(...)` | `mm` | `createByReal(to_cm(0))` | — |
| 12 | Pinion Gear Bore Diameter | `pinionBore` | `addValueInput(...)` | `mm` | `createByReal(to_cm(0))` | — |
| 13 | Face Width | `faceWidth` | `addValueInput(...)` | `mm` | `createByReal(to_cm(0))` | — |
| 14 | Tooth Spacing | `toothSpacing` | `addValueInput(...)` | `mm` | `createByReal(to_cm(0))` | — |
| 15 | Mean Spiral Angle | `spiralAngle` | `addValueInput(...)` | `deg` | `createByString('35 deg')` | — |
| 16 | Hand of Spiral | `spiralHand` | `addDropDownCommandInput(...)` (text list) | — | items `Right` (selected), `Left` | — |
| 17 | Cutter Radius | `cutterRadius` | `addValueInput(...)` | `mm` | `createByReal(to_cm(0))` | — |

The ids live in 17 module-level constants, named exactly and holding exactly these strings, in row
order:

```
INPUT_ID_PLANE              = 'targetPlane'
INPUT_ID_CENTER_POINT       = 'centerPoint'
INPUT_ID_PARENT             = 'parentComponent'
INPUT_ID_MODULE             = 'module'
INPUT_ID_SHAFT_ANGLE        = 'shaftAngle'
INPUT_ID_DRIVING_TEETH      = 'drivingTeeth'
INPUT_ID_PINION_TEETH       = 'pinionTeeth'
INPUT_ID_DRIVING_BASE_HEIGHT = 'drivingBaseHeight'
INPUT_ID_PINION_BASE_HEIGHT = 'pinionBaseHeight'
INPUT_ID_BORE_ENABLE        = 'boreEnable'
INPUT_ID_DRIVING_BORE       = 'drivingBore'
INPUT_ID_PINION_BORE        = 'pinionBore'
INPUT_ID_FACE_WIDTH         = 'faceWidth'
INPUT_ID_TOOTH_SPACING      = 'toothSpacing'
INPUT_ID_SPIRAL_ANGLE       = 'spiralAngle'
INPUT_ID_HAND               = 'spiralHand'
INPUT_ID_CUTTER_RADIUS      = 'cutterRadius'

_HAND_RIGHT = 'Right'
_HAND_LEFT  = 'Left'
```

Those two hand strings and the 17 ids are the only module-level constants this generator has, apart
from the spiral tunables `_CROWN_PER_RAD = 0.5` (S25) and `_PINION_MESH_PHASE_TEETH = 0` (S18).
There are **no** live Fusion user parameters and so no `PARAM_*` strings: every value is precomputed
in Python in internal cm (`[PB-PRECOMPUTED-MODE]`).

Four things the table's cells stand for:

- **Filters and limits** (`[PB-SELECTION-DECL]`). Each selection input gets its filters through
  `addSelectionFilter(...)` and then `setSelectionLimits(1, 1)`, exactly one selection. A filter is
  the enum member — `adsk.core.SelectionCommandInput.ConstructionPlanes`,
  `adsk.core.SelectionCommandInput.PlanarFaces`, `adsk.core.SelectionCommandInput.ConstructionPoints`,
  `adsk.core.SelectionCommandInput.SketchPoints`, `adsk.core.SelectionCommandInput.Occurrences`,
  `adsk.core.SelectionCommandInput.RootComponents` — never a quoted literal
  (`[PB-SELECTION-FILTER-ENUM]`). The tooltip is the third argument of
  `addSelectionInput(id, label, tooltip)`. The Parent Component input pre-selects
  `get_design().rootComponent`.
- **Defaults are in Fusion INTERNAL units** whatever the unit string says
  (`[PB-DIALOG-DEFAULT-UNITS]`): a 0 mm field is `createByReal(to_cm(0))`, the Module default is the
  bare `createByReal(1)`, the two tooth counts are `createByReal(31)`, and the two angles are
  written as expressions, `createByString('90 deg')` and `createByString('35 deg')`, so the
  expression engine parses them. Writing `createByReal(90)` for the angle would ship a 90-radian
  default that the dialog still displays in degrees.
- **The Hand dropdown** is `addDropDownCommandInput(INPUT_ID_HAND, 'Hand of Spiral',
  adsk.core.DropDownStyles.TextListDropDownStyle)`, with `Right` added selected and `Left` added
  unselected through the returned input's `listItems.add(...)`. It is read back as
  `selectedItem.name`, defaulting to `Right` when there is none.
- **Enable Bore** is a checkbox, `addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '',
  True)` — checkbox style with an initial value of `True`.

The two spiral-only rows are hidden while ψ = 0. Hand of Spiral and Cutter Radius are relevant only
to a curved bevel, so they are hidden whenever the Mean Spiral Angle is 0 and shown when it is above
0; Mean Spiral Angle itself is the controller and is always visible, since it is how the user
reaches ψ > 0. There is no declarative show-if in the Fusion API, so it is the `isVisible` property:
a `@classmethod _updateSpiralInputVisibility(cls, inputs)` helper evaluates the `spiralAngle` input's
**`.expression`** with `unitsManager.evaluateExpression(spiral.expression, 'rad')` — internal
radians, and not the input's `.value` — and sets `inputs.itemById(INPUT_ID_HAND).isVisible` and
`inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible` to `(value > 0)`. Guard it: return early if any
of the three inputs is `None`, and wrap the evaluation in `try`/`except`, since a half-typed
expression can raise mid-edit — on failure leave both inputs shown. `configure` calls it as its
**last** step, so the initial state is right for the default ψ = 35°, and
a second class method, `handle_input_changed`, taking `(cls, args)`, calls it on **every** input
change by delegating one line to `cls._updateSpiralInputVisibility(args.inputs)`. Hiding is cosmetic
only: the input still exists and is read normally, and the ψ = 0 build ignores Hand and Cutter
Radius anyway.

`configure` and `handle_input_changed` are both bound **by name** from `commands/bevelgear/entry.py`,
alongside `BevelGearGenerator(design).generate(inputs)` and its `deleteComponent()` on failure.

`addSelectionInput(...)`, `addSelectionFilter(...)`, `setSelectionLimits(...)`, `addValueInput(...)`,
`addBoolValueInput(...)`, `addDropDownCommandInput(...)`, `listItems.add(...)`,
`ValueInput.createByReal(...)`, `ValueInput.createByString(...)`, `itemById(...)`,
`unitsManager.evaluateExpression(...)`, `isVisible`

`configure`, `handle_input_changed`, `generate` and `deleteComponent` are the members the entry
point binds by name and calls; they are this module's surface, not calls it makes. Its own helper
`_updateSpiralInputVisibility` is named here so the first two find it.

<!-- check-step-calls: ignore configure handle_input_changed generate deleteComponent
     _updateSpiralInputVisibility -->

**From:** `spec/bevelgear/instructions.md` L131–L187 (Exact input ids, conditional visibility),
L246–L272 (Architecture), L296–L310 (external bindings); `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-AUTOFOCUS-FIRST]`, `[PB-DIALOG-DEFAULT-UNITS]`, `[PB-SELECTION-DECL]`,
`[PB-SELECTION-FILTER-ENUM]`, `[PB-PRECOMPUTED-MODE]`.

## S03 `[PROSE]` Create the Bevel Gear and Design components

Create the occurrence tree directly, never through `base.Generator` (`[PB-OCCURRENCE-TREE]`): a
`Bevel Gear` component under the user's Parent Component, and a `Design` component under it. Name
each through `occurrence.component.name`. `Design` holds every sketch, construction plane and axis,
and every feature runs there; the finished bodies are moved out at the end (`[PB-NO-CROSS-SIBLING]`).
`BevelGearGenerator.__init__(self, design)` stores `self.design` and initialises
`self.bevelOccurrence = None`; this step fills it, along with `self.designOccurrence`,
`self.designComponent` and `self.bevelComponent`, which cleanup and rollback read.

**Never activate any occurrence** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`): the Anchor
Sketch sits on the user's root-owned target plane, and an activated occurrence resolves that external
plane in its own frame, collapsing the whole build onto world XY. The one exception is the spiral
crown's scale step (S25).

`parent.occurrences.addNewComponent(...)`, `adsk.core.Matrix3D.create(...)`

`deleteComponent` is the rollback method the entry point calls on failure, not a call this step
makes.

<!-- check-step-calls: ignore deleteComponent -->

**From:** `spec/bevelgear/instructions.md` L19–L23, L432–L442, L246–L272;
`spec/bevelgear/fusion.md` L172–L179; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-OCCURRENCE-TREE]`, `[PB-NEVER-ACTIVATE]`, `[PB-NO-CROSS-SIBLING]`.

## S04 `[GO]` Anchor sketch

Add a sketch named `Anchor` **directly on the user-selected target plane**, whether that selection is
a `ConstructionPlane` or a `PlanarFace` — do not re-derive or offset it (`[PB-USE-SELECTED-PLANE]`;
re-deriving collapses the gear onto XY).

Project the user's Center Point into it, and draw the Anchor Line through the projected point with
its two endpoints seeded at exactly **±0.5 cm** along the sketch-local X, a seeded length of 10 mm.
Apply **both** `addCoincident(projectedCenter, anchorLine)` — the point-on-line pin — **and**
`addMidPoint(projectedCenter, anchorLine)`. Add an aligned distance dimension on the line and do
**not** assign `.parameter.value`: it simply locks the seeded 10 mm, since the length is arbitrary.
Then pin the direction with `addHorizontal(anchorLine)`, which is sketch-local and so survives a
tilted target plane (`[PB-REFLINE-DIRECTION]`); a world-axis lock would mis-orient it.

Stash the projected-centre `SketchPoint` on `self._anchorCenterPoint`: §2 re-projects **that** point,
not the raw user selection. Gate the sketch on `isFullyConstrained` at the end of the step and raise
naming it (`[BEVEL-F-FULL-CONSTRAINT]`, `[PB-FULL-CONSTRAINT]`).

`sketches.add(...)`, `sketch.project(...)`, `sketch.sketchCurves.sketchLines.addByTwoPoints(...)`,
`adsk.core.Point3D.create(...)`, `sketch.geometricConstraints.addCoincident(...)`,
`sketch.geometricConstraints.addMidPoint(...)`, `sketch.geometricConstraints.addHorizontal(...)`,
`sketch.sketchDimensions.addDistanceDimension(...)`,
`adsk.fusion.DimensionOrientations.AlignedDimensionOrientation`, `sketch.isFullyConstrained`

`project2` is named only to be refused: the compiled API reference declares `project2(entities,
isLinked)` and no `project`, so every gate reports `project` as unverified, and this repo's settled
position is to keep `project` and keep reporting it. The two are not interchangeable — `project2`
takes and returns lists — so do not swap the name.

<!-- check-step-calls: ignore project2 -->

Proof: `stepAnchorSketch`.

<!-- proof-run: proofkit.Run(anchorCases, stepAnchorSketch) -->

**From:** `spec/bevelgear/instructions.md` L444–L448, L458–L461 (the `project` note);
`spec/bevelgear/fusion.md` L19–L33, L74–L91; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-USE-SELECTED-PLANE]`, `[PB-REFLINE-DIRECTION]`, `[PB-FULL-CONSTRAINT]`, `[PB-DRIVING-DIM]`.

## S05 `[PROSE]` Gear Profiles Plane

Create a construction plane through the Anchor Line at 90°, named `Gear Profiles Plane`:
`constructionPlanes.createInput()` then `setByAngle(anchorLine, ValueInput.createByString('90 deg'),
targetPlane)` then `add(input)`. Build it off the **original** `targetPlane` as the reference
(`[PB-USE-SELECTED-PLANE]`) — this is the second place the target-plane orientation reaches the
bodies, and substituting a different plane here also collapses the gear onto XY. Pass the sketch line
directly; never wrap it in `Path.create` first (`[PB-CONSTRUCTION-PLANES]`).

`component.constructionPlanes.createInput(...)`, `constructionPlaneInput.setByAngle(...)`,
`component.constructionPlanes.add(...)`, `ValueInput.createByString(...)`

`Path.create` is named only to forbid it: on a sketch curve it raises
`InternalValidationError : Utils::getObjectPath` whenever the owner sketch is not trivially
resolvable in a multi-component context.

<!-- check-step-calls: ignore Path.create -->

**From:** `spec/bevelgear/instructions.md` L450–L452; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-CONSTRUCTION-PLANES]`, `[PB-USE-SELECTED-PLANE]`.

## S06 `[GO]` Gear Profiles sketch — the §2 lattice

One sketch named `Gear Profiles` on the Gear Profiles Plane, holding the whole two-gear lattice.
**Every line in it is a construction line** (`isConstruction = True`) — lattice lines, the toe lines
M->N and O->P, and the short reference lines M->C, N->A, O->D, P->B, A->G, B->I, C->K′, D->L′ alike.
The solid features later consume only the per-gear Profile sketches, never a §2 curve.

**Every line in this sketch is built the same way** (`[BEVEL-F-COINCIDENT-STYLE]`): create it from
raw `Point3D` coordinates for **both** endpoints and pin each connected end with exactly one
`addCoincident(endpoint, existingPoint)`. Never pass an existing `SketchPoint` into
`addByTwoPoints` to share it — sharing without a coincident leaves the sketch under-constrained, and
sharing *and* coinciding fails the solve with `VCS_SKETCH_SOLVING_FAILED`
(`[PB-SHARE-XOR-COINCIDENT]`). This covers the short reference lines too; a regen that shared only
those came out about 14 coincidents short. Each named line is created **once** and reused, never
redrawn (`[BEVEL-F-LINE-ONCE]`), so the helper that draws a module extension returns the line.

**Every length dimension here is `AlignedDimensionOrientation`** — the figure has no axis-aligned
line in it, so a horizontal or vertical orientation would dimension a projection instead of the
length. The offset dimensions are the different call `addOffsetDimension`, which takes no
orientation. The driven lengths — Apex->A, Apex->B and the module-length extensions — carry **no**
dimension at all (`[BEVEL-F-DRIVEN-DIMS]`, `[PB-NO-OVERCONSTRAIN]`).

Build it in this order:

1. Project **the Anchor Sketch's centre `SketchPoint`** (`self._anchorCenterPoint`), not the raw user
   selection: projecting the anchor point keeps the chain inside Design.
2. From the projected centre, a construction line perpendicular to the **projected anchor line**,
   its far end the **Apex**, seeded in sketch-local 2-D at `c + perp·(R·cos γ_g + <resolved Driving
   Gear Base Height>)` where `perp = (-d.y, d.x)` for the projected anchor direction `d`
   (`[BEVEL-F-APEX-LOCAL]`, `[PB-SEED-NEAR]`). Do **not** seed it at the Driving Gear Pitch Diameter,
   which sat 11.6 mm past the solved position for the default pair, and do **not** compute the apex
   from a world round-trip. The **sign** of `perp` is the one bit read from the world: take
   `targetPlane.geometry.normal` — a `core.Plane` for both selection kinds — and grow toward it
   (`[BEVEL-F-GROW-SIDE]`). Add no length constraint on this line.
3. The **Driving Gear Shaft Axis** from the Apex, seeded at `apex - perp·(R·cos γ_g)`, which is
   `c + perp·(<resolved Driving Gear Base Height>)` — measured from the apex, not from `c`. Its start
   coincides with the Apex, and it is held with `addParallel(drivingShaftAxis, centerToApex)`. Do
   **not** use `addVertical`: the gear-profiles sketch is not world-aligned, and an absolute vertical
   mis-orients the figure on a tilted plane. Its end is **point B**; no length dimension.
4. The **Pinion Gear Shaft Axis** from the Apex, seeded by rotating the driving-shaft direction about
   the Apex by ±Shaft Angle and keeping **the candidate whose endpoint has the greater X** in this
   sketch. Form both candidates and compare — do not rotate one fixed sense and flip only on a
   negative X, which keeps the wrong one when both are positive. Its end is **point A**; seed
   `|Apex→A| = R·cos γ_p` and `|Apex→B| = R·cos γ_g`; no length dimension.
5. `addAngularDimension(drivingShaftAxis, pinionShaftAxis, textPoint)` set to the Shaft Angle, with
   the text point **inside the Σ wedge** so it measures Σ and not `180 − Σ` (`[PB-ANGULAR-DIM]`):
   the apex plus the unit bisector of `pinionDir` and `drivingDir`, scaled by PPD/4.
6. From **A**, the perpendicular drop whose far end becomes **Apex 2**: perpendicular to the *Pinion
   Gear Shaft Axis*, aimed at the interior wedge by the sign of its dot with the **A→B** direction,
   with an aligned dimension of `Pinion Gear Pitch Diameter / 2`. From **B**, the twin drop
   perpendicular to the *Driving Gear Shaft Axis*, aimed by the sign of its dot with the **B→A**
   direction, dimensioned `Driving Gear Pitch Diameter / 2`. ⚠️ Do **not** choose the B-side sense
   against a "toward the anchor line" reference: the Driving Gear Shaft Axis is itself parallel to
   that direction, so the test is degenerate and picks an arbitrary side; the two drops then close on
   opposite sides and the solver flips the whole frame to the mirror solution, with C collapsing onto
   D. Close them with `addCoincident` at Apex 2. Throughout this step, "A->Apex2" and "B->Apex2"
   always name these two **drop lines**, never the Apex->A / Apex->B shaft axes.
7. The **Pitch Line** Apex->Apex 2, coincident at both ends.
8. From Apex 2, the two dedendum lines, each perpendicular to the **Pitch Line** with an aligned
   dimension of `Module * 1.25`: the one toward the anchor line is the **Driving Gear Dedendum**
   ending at **D**, the one away is the **Pinion Gear Dedendum** ending at **C**.
9. The two **Root Axes**, Apex->C and Apex->D, coincident at both ends.
10. From A, a line collinear with **Apex->A** extending one module (seeded, undimensioned) to **E**;
    then C->E, coincident at both ends, with `addPerpendicular(lineAE, lineCE)`. The driving twin
    gives **F** from B along **Apex->B**, then D->F perpendicular to B->F.
11. From E, a line collinear with **line A->E** — the collinear names A->E, never the Apex->A shaft
    axis further up the chain, even though both describe the same infinite line
    (`[BEVEL-F-COLLINEAR-CHAIN]`; naming the axis raises `VCS_SKETCH_OVER_CONSTRAINTS`) — one module
    long, ending at **G**. From C, a line one module long ending at **H**, collinear with **line
    Apex2->C**. Connect **G->H**, coincident at both ends, and constrain `addPerpendicular(lineEG,
    lineGH)`. The driving twins are F->I collinear with **B->F**, D->J collinear with **Apex2->D**,
    the line **I->J**, and `addPerpendicular(lineFI, lineIJ)`.
12. `addOffsetDimension(B->Apex2 drop, J->I, textPoint).parameter.value` = the **resolved** Driving
    Gear Base Height, and `addOffsetDimension(A->Apex2 drop, G->H, textPoint).parameter.value` = the
    **resolved** Pinion Gear Base Height (S01 resolved both). J->I and G->H are already parallel to
    their drops by construction, so add **no** `addParallel` (`[PB-OFFSET-DIM]`).
13. The reference line **A->G**, then `addCoincident(pointI, projectedCenter)` — the closure that
    fixes the apex's undimensioned height.
14. **K**: a line from G along Apex->A, its far end pinned by **two** point-on-line coincidents,
    `addCoincident(K, Apex->A)` and `addCoincident(K, the Pinion Dedendum line Apex2->C extended)`.
    Do not use `addCollinear` here: G and C are already fixed, so a collinear over-constrains. Then
    the reference line C->K. **L** is the driving twin from I along Apex->B, pinned to Apex->B and to
    the Driving Dedendum Apex2->D, then D->L.
15. **K′ and L′.** When Tooth Spacing is 0, set K′ ≡ K and reuse the existing C->K line — build
    nothing (a zero-length dimensioned line is degenerate, and one segment gets one line). When it is
    positive, draw a line starting at K with its far end seeded on the far side of K from C along the
    dedendum direction, pin the far end to the dedendum line the same way K is pinned, add a length
    dimension equal to the Tooth Spacing, and draw the tooth-centre reference line **C->K′**. The
    driving side substitutes L, D and the Driving Dedendum, giving **D->L′**. Build both here, inside
    this sketch, before the end-of-step gate.
16. **Resolve the Maximum Face Width now**, from the **solved** `.geometry` of A, B, C, D, H and J
    (`[PB-SOLVED-GEOMETRY]`) — never the seeds, which diverge markedly for asymmetric tooth counts
    and leave the bound too loose on the binding side. It is `0.95 *` the smaller of the
    perpendicular distance from A to the line through C and H and the perpendicular distance from B
    to the line through D and J. Cap the auto Face Width `Cone Distance / 6` to it, and reject a user
    value above it naming the maximum. Without this the toe crosses the shaft axis and the S12
    revolve fails with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`). Stash the result as
    `self._faceWidthResolved_cm`.
17. **M->N**, seeded near its solved position (`[PB-SEED-NEAR]`): M at roughly the midpoint of
    Apex->C, N slid from that seed along the C->H direction far enough to roughly reach line
    A->Apex2 — not `Face Width` away from C and H, which starts N near H and fails to converge. Then
    exactly four constraints: `addCoincident(M, Pinion Root Axis)`, `addCoincident(N, line
    A->Apex2)` — the **drop**, not the Apex->A shaft axis; pinning N to the axis puts it on the axis
    of revolution and the later conical split fails with `ASM_API_FAILED` — `addParallel(M->N, C->H)`
    and `addOffsetDimension(C->H, M->N, textPoint).parameter.value` = the resolved Face Width, with
    the text point in the gap on the Apex side, `(M_seed + C)/2`. Then the reference lines M->C and
    N->A. The driving mirror is **O->P** with `addCoincident(O, Driving Root Axis)`,
    `addCoincident(P, line B->Apex2)`, `addParallel(O->P, D->J)`,
    `addOffsetDimension(D->J, O->P, textPoint)` at `(O_seed + D)/2`, then O->D, P->B and B->I.
18. Gate the sketch on `isFullyConstrained` and raise naming it.

`sketch.project(...)`, `sketch.sketchCurves.sketchLines.addByTwoPoints(...)`, `adsk.core.Point3D.create(...)`,
`sketch.geometricConstraints.addCoincident(...)`, `sketch.geometricConstraints.addPerpendicular(...)`,
`sketch.geometricConstraints.addParallel(...)`, `sketch.geometricConstraints.addCollinear(...)`,
`sketch.sketchDimensions.addDistanceDimension(...)`, `sketch.sketchDimensions.addAngularDimension(...)`,
`sketch.sketchDimensions.addOffsetDimension(...)`,
`adsk.fusion.DimensionOrientations.AlignedDimensionOrientation`, `targetPlane.geometry.normal`,
`sketch.isFullyConstrained`

`addVertical` is named only to forbid it on the driving shaft axis.

<!-- check-step-calls: ignore addVertical -->

Proof: `stepGearProfiles`, which builds the whole lattice in the sketch engine and checks the solved
figure against the closed form — the driven along-shaft lengths, the closure of point I on the
projected centre, both cone angles, the dedendum corners, the heel edge against the base-height
window, K and K′, and that no hexagon vertex crosses its own shaft axis. Three constraint-arity
differences between Fusion and the bench, and the reason the drops are written as signed angles
there, are recorded in the proof file next to that function.

<!-- proof-run: proofkit.Run(latticeCases, stepGearProfiles) -->

**From:** `spec/bevelgear/instructions.md` L450–L546, L116–L123 (Maximum Face Width), L124–L127
(Tooth Spacing); `spec/bevelgear/fusion.md` L74–L141 (§2 lattice construction), L142–L171
(orientation); `.claude/skills/generate-gear/PLAYBOOK.md` `[PB-SHARE-XOR-COINCIDENT]`,
`[PB-SEED-NEAR]`, `[PB-SOLVED-GEOMETRY]`, `[PB-OFFSET-DIM]`, `[PB-ANGULAR-DIM]`,
`[PB-NO-OVERCONSTRAIN]`, `[PB-DIM-VALUE-SEMANTICS]`, `[PB-REVOLVE]`, `[PB-API-SPELLING]`,
`[PB-FULL-CONSTRAINT]`.

## S07 `[PROSE]` `{gearLabel} Plane` — the tooth plane

Once per gear, pinion first. **`{gearLabel}` is the string `Pinion` or `Driving`**, so this plane is
named `Pinion Plane` or `Driving Plane`, and the same substitution gives every other name below:
`Pinion Tooth` / `Driving Tooth`, `Pinion Tooth Axis` / `Driving Tooth Axis`, `Pinion Gear` /
`Driving Gear`, `Pinion Profile` / `Driving Profile`, `Pinion Bore` / `Driving Bore`, and the
spiral's `Pinion Cone Element`, `Pinion Trace Plane`, `Pinion 2D Tooth Trace` and
`Pinion Spiral Tooth` with their driving twins.

Create a construction plane that includes the tooth-centre reference line — **C->K′** for the
pinion, **D->L′** for the driving gear — perpendicular to the Gear Profiles sketch plane, named
`{gearLabel} Plane`. Use the framework helper `plane_by_angle(designComponent,
<tooth-centre reference line>, gearProfilesPlane, 90)`, which passes the sketch line **directly** to
`setByAngle`; never wrap it in `Path.create` (`[PB-CONSTRUCTION-PLANES]`).

`plane_by_angle(...)`

**From:** `spec/bevelgear/instructions.md` L548–L560; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-CONSTRUCTION-PLANES]`, "Shared geargen helper library".

## S08 `[GO]` `{gearLabel} Tooth` sketch — the virtual spur tooth

Once per gear. Compute this gear's virtual (back-cone, Tredgold) tooth number from the closed form,
never by measuring Apex2->K′: `virtualPitchRadius_mm = (pitchDia_cm * 10 / 2) / cos(γ)` and
`virtualTeeth = floor(2 * virtualPitchRadius_mm / Module)` as an int. The `* 10` is the cm→mm
conversion the stashed pitch diameters need against the raw-mm Module; skipping it makes the count
about ten times too small.

Add a sketch named `{gearLabel} Tooth` on the `{gearLabel} Plane`, centred on the tooth-centre point
**K′ / L′**, and let the borrowed spur generator draw into it:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))
```

The 180° rotation is delivered **through the `draw()` angle**, never by rotating the sketch
afterwards. Import `VirtualSpurProxy` from `.spurproxy` and
`SpurGearInvoluteToothDesignGenerator` from `.spurgear`; define no local proxy or value wrapper.

The borrowed drawer's surface is `(sketch, parent, angle=0)` for the constructor and
`draw(anchorPoint, angle=0)`, which runs its circles, then the tooth at that angle, then the anchor
projection, reading every parameter through the proxy's `getParameter` lookup and taking the `.value`
off what comes back. `VirtualSpurProxy(module_mm, virtualTeeth)` serves those keys in internal cm
from two more defaults bevel does not override and does not expose as dialog inputs: a **pressure
angle of 20°** and **`InvoluteSteps` = 15**.

After `draw()` returns, read `proxy._lastToothEmbedded` back and thread it out with the sketch and
plane: it is the deterministic selector for the tooth loop's line count in S13,
`wantLines = 0 if embedded else 2`. Do **not** hard-gate this sketch — `futil.log` when
`not toothSketch.isFullyConstrained` and never raise. The exemption is for the four along-path circle
labels the drawer writes, which `isFullyConstrained` counts and nothing pins; it is not licence for
loose geometry (`[BEVEL-F-FULL-CONSTRAINT]`).

`VirtualSpurProxy`, `SpurGearInvoluteToothDesignGenerator`, `drawer.draw(...)`, `sketches.add(...)`,
`math.radians(...)`, `math.floor(...)`, `math.cos(...)`, `futil.log(...)`

Proof: `stepVirtualSpurTooth`, which checks the virtual count and the four circle radii it derives,
the 180° placement, the curve-count key on both sides of the `embedded` branch, and that both arc
centres close on the tooth centre — the stranded centre that gave a 0.5743 mm tooth-top arc where
22.5 mm was meant.

<!-- proof-run: proofkit.Run(toothCases, stepVirtualSpurTooth) -->

**From:** `spec/bevelgear/instructions.md` L548–L562, L402–L431 (Dependencies), L330–L350
(tooth-profile selection); `spec/bevelgear/fusion.md` L19–L72 (the gate and its exemptions);
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-SKETCH-TEXT]`, `[PB-SHARE-XOR-COINCIDENT]`,
`[PB-PRECOMPUTED-MODE]`.

## S09 `[PROSE]` `{gearLabel} Tooth Axis`

Once per gear. Create a construction axis named `{gearLabel} Tooth Axis` through the tooth-centre
point, normal to the plane the tooth was drawn on, as the intersection of two planes
(`[PB-CONSTRUCTION-AXES]`): the **Gear Profiles plane** and a helper plane built
`setByDistanceOnPath(<tooth-centre reference line>, 1.0)`, which is perpendicular to that line at its
far end, the tooth centre. Pass the sketch line directly to `setByDistanceOnPath`.

Creating this axis in the never-activated Design component is proven to work — keep it.

`component.constructionPlanes.createInput(...)`, `constructionPlaneInput.setByDistanceOnPath(...)`,
`component.constructionPlanes.add(...)`, `component.constructionAxes.createInput(...)`,
`constructionAxisInput.setByTwoPlanes(...)`, `component.constructionAxes.add(...)`

`setByPerpendicularAtPoint` is named only to say why it is not used: it needs a `BRepFace` this step
does not have.

<!-- check-step-calls: ignore setByPerpendicularAtPoint -->

**From:** `spec/bevelgear/instructions.md` L563–L564; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-CONSTRUCTION-AXES]`, `[PB-CONSTRUCTION-PLANES]`, `[PB-CONSTRUCTION-NEEDS-ACTIVE]`.

## S10 `[PROSE]` Create the `{gearLabel} Gear` component

Once per gear. Create a component named `Pinion Gear` / `Driving Gear` as a child of the **Bevel
Gear** component — the same component that owns Design, not the user's Parent Component — so the pair
nests inside Bevel Gear. The finished bodies land here at the end; every feature until then runs in
Design, because Fusion rejects cross-sibling sketch and project calls even with activation or
assembly-context proxies (`[PB-NO-CROSS-SIBLING]`).

`parent.occurrences.addNewComponent(...)`, `adsk.core.Matrix3D.create(...)`

**From:** `spec/bevelgear/instructions.md` L659–L674; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-OCCURRENCE-TREE]`, `[PB-NO-CROSS-SIBLING]`.

## S11 `[GO]` `{gearLabel} Profile` sketch — the hexagon

Once per gear: a **fresh sketch on the axial (Gear Profiles) plane**, named `Pinion Profile` /
`Driving Profile`, holding exactly one hexagon loop. Do not draw both gears' hexagons in one sketch —
that leaves two identically-shaped loops to disambiguate.

Build it by the recreate-share-fix recipe (`[PB-PROJECT-NOT-FIXED]`): recreate the six §2 vertices as
**new** sketch points at their exact world-mapped positions with
`sketch.sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))` — valid because §2 is fully
constrained by now — draw the closed hexagon as six `SketchLine`s **sharing** those points in draw
order, and only **then** fix the lines' endpoints. Order matters: fixing a bare point before it is
consumed as a line endpoint does not leave the sketch fully constrained.

Draw order is `A -> G -> H -> C -> M -> N -> A` for the pinion and `B -> I -> J -> D -> O -> P -> B`
for the driving gear. **The loop's first edge — A->G / B->I — is this gear's shaft axis** for the
revolve, the pattern, the bore plane and the meshing rotation, so it must carry a trustworthy
`worldGeometry`, which fixed endpoints are what give it (`[PB-WORLDGEO-CONSTRAINED]`); a free edge
resolves against a default frame and silently moves the body onto world XY. Use this in-sketch edge
everywhere below, never the §2 `Apex->A` / `Apex->B` construction line, which lives in another
sketch.

Gate the sketch on `isFullyConstrained` and raise naming it.

`sketch.sketchPoints.add(...)`, `sketch.modelToSketchSpace(...)`, `sketchPoint.worldGeometry`,
`sketch.sketchCurves.sketchLines.addByTwoPoints(...)`, `sketchPoint.isFixed`, `sketch.isFullyConstrained`,
`sketches.add(...)`

Proof: `stepProfileHexagon`, which checks the single loop and its six line edges, that the first edge
lies on the §2 shaft line and spans the resolved base height, that the toe and heel edges are the
ones the trims and the spiral hand-off name, and that no vertex crosses the axis of revolution.

<!-- proof-run: proofkit.Run(profileCases, stepProfileHexagon) -->

**From:** `spec/bevelgear/instructions.md` L659–L679; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-PROJECT-NOT-FIXED]`, `[PB-WORLDGEO-CONSTRAINED]`, `[PB-SPACE-METHODS]`, `[PB-SINGLE-PROFILE]`,
`[PB-FULL-CONSTRAINT]`.

## S12 `[GO]` Revolve the hexagon into the Gear Body

Once per gear. The Profile sketch holds exactly one closed loop, so take `sketch.profiles.item(0)`
directly and do not filter (`[PB-SINGLE-PROFILE]`). Revolve it a full turn about the **hexagon's
first edge**: `revolveFeatures.createInput(profile, shaftAxisEdge, FeatureOperations.NewBodyFeatureOperation)`,
`setAngleExtent(False, ValueInput.createByString('360 deg'))`, `add(input)`. The result is the Gear
Body, the frustum.

The profile must not cross its axis of revolution or Fusion aborts with `ASM_WIRE_X_AXIS`
(`[PB-REVOLVE]`); that is what the Maximum Face Width cap in S06 is for. The revolved body carries
the conical faces the toe and heel edges sweep, and S14 reuses them as cutting tools.

`component.features.revolveFeatures.createInput(...)`, `revolveFeatureInput.setAngleExtent(...)`,
`component.features.revolveFeatures.add(...)`, `sketch.profiles.item(...)`, `ValueInput.createByString(...)`,
`adsk.fusion.FeatureOperations.NewBodyFeatureOperation`

Proof: `stepGearBody` with `assertGearBody`. The harness cannot gate a revolve at all — the volume
decad publishes for a full turn carries a bound of twice the volume, so no gate accepts the
document — so the proof builds the band of the blank the teeth seat on, as the loft between the toe
and heel sections, and checks its wall is the cone through the apex, its span, and its volume against
the chord-polygon frustum. What that leaves unreached, and where the axis-crossing check lives
instead, is recorded next to the function.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepGearBody, assertGearBody) -->

**From:** `spec/bevelgear/instructions.md` L679–L680, L116–L123; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-REVOLVE]`, `[PB-SINGLE-PROFILE]`.

## S13 `[GO]` Loft the Apex point to the tooth profile

Once per gear. Loft from the **§2 Apex sketch point** — `centerToApex.endSketchPoint`, the degenerate
point section — to this gear's `{gearLabel} Tooth` profile, giving the Tooth Body:
`loftFeatures.createInput(FeatureOperations.NewBodyFeatureOperation)`, then
`loftInput.loftSections.add(apexSketchPoint)` and `loftInput.loftSections.add(toothProfile)` in that
order, then `add(input)` (`[PB-LOFT]`). Use the §2 Apex **sketch** point directly; do not create a
construction point for it, since the Design component is never active
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`).

Select the tooth profile with `find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2,
lines=wantLines)` where `wantLines = 0 if embedded else 2`, taking `embedded` from the flag S08 read
back off the proxy. Do **not** accept "0 **or** 2 lines": for a given gear only one of those is the
real tooth, an unrelated loop between the drawer's circles can carry the same two NURBS and two arcs
with the other line count, and lofting that impostor fails with
`RuntimeError ... ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`.

`component.features.loftFeatures.createInput(...)`, `loftInput.loftSections.add(...)`,
`component.features.loftFeatures.add(...)`, `find_profile_by_curve_counts(...)`,
`adsk.fusion.FeatureOperations.NewBodyFeatureOperation`

Proof: `stepToothLoft` with `assertToothLoft`, which checks the cone taper the loft produces. decad
has no point section and pairs loft segments by kind, so the proof chords the tooth and starts the
loft a short way out from the apex; the substitution and its cost are stated at the function.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepToothLoft, assertToothLoft) -->

**From:** `spec/bevelgear/instructions.md` L681–L682, L330–L350 (tooth-profile selection);
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-LOFT]`, `[PB-PROFILE-MATCH]`,
`[PB-CONSTRUCTION-NEEDS-ACTIVE]`.

## S14 `[GO]` Conical end cuts — trim the tooth flush

Once per gear, and the tooth-body hook `_transformToothBody` runs here. Its first line is the gate
`if self._spiralAngle_rad <= 0: return cut_conical_ends(...)`; everything in S19–S26 is the ψ > 0
branch that replaces this trim, and S27 is where the curved tooth comes back to it.

Trim the Tooth Body to a flush band with the framework helper — do not re-implement the cut
machinery:

```python
cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)
```

**Two distinct bodies:** the cutting **tools** are `ConeSurfaceType` faces of the **Gear Body**, the
revolved frustum, because the lofted Tooth Body has no cone faces of its own; the **target** being
split is the Tooth Body.

Caller obligations, which stay in the generator: `toeMid` is the toe edge's world midpoint,
`(M_world + N_world)/2` for the pinion and `(O_world + P_world)/2` for the driving gear; `heelMid` is
the heel edge's, `(C_world + H_world)/2` / `(D_world + J_world)/2`; `apexWorld` is the §2 Apex sketch
point's world geometry; `gearBody` is the revolved frustum. The helper cuts the **toe first**, finds
its cone face best-first by the toe edge's world **midpoint** (`[PB-FACE-BY-MIDPOINT]` — endpoints
sit near the apex singularity), keeps the first face that actually splits, selects the keeper by
dropping apex-containing pieces and keeping the largest (`[PB-REMOVE-PIECES]`), then cuts the **heel
on the keeper alone**. The toe cut must split and its failure propagates; only the heel cut is
lenient, and only through the typed `solids.NonIntersectError`, which is raised when the heel cone
does not reach the keeper at all and is caught so the keeper is returned whole.

`cut_conical_ends(...)`, `sketchPoint.worldGeometry`

`apply_conical_cut`, `select_keeper`, `find_cone_faces_by_midpoint` and `surface_distance` are the
helper's own internals, named here to describe the pinned behaviour; the module calls
`cut_conical_ends` and none of them directly.

<!-- check-step-calls: ignore apply_conical_cut select_keeper find_cone_faces_by_midpoint
     surface_distance NonIntersectError -->

Proof: `stepConicalTrim` with `assertConicalTrim`. decad refuses a boolean whose operand is a loft,
so the proof builds the band the two cuts leave — every vertex on one of the two cut sections, none
between or beyond them, and the taper kept. The cost, that the real cut surfaces are cones where the
stand-in's are flat, is stated at the function.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepConicalTrim, assertConicalTrim) -->

**From:** `spec/bevelgear/instructions.md` L683–L710, L296–L330 (the `_transformToothBody` hook);
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-SPLIT-BODY]`, `[PB-FACE-BY-MIDPOINT]`,
`[PB-REMOVE-PIECES]`, `[PB-SELF-DIAGNOSING]`, "Shared geargen helper library".

## S15 `[GO]` Circular-pattern the tooth

Once per gear. Circular-pattern the trimmed tooth piece around the **shaft-axis edge** — the profile
sketch's first edge, not the §2 construction line: `circularPatternFeatures.createInput(bodies,
shaftAxisEdge)` with `bodies` an `ObjectCollection` holding the tooth, then pin all three inputs
(`[PB-CIRCULAR-PATTERN]`): `quantity = ValueInput.createByReal(<this gear's Teeth Number>)`,
`totalAngle = ValueInput.createByString('360 deg')`, `isSymmetric = False`, then `add(input)`.

The angular spacing stays `360° / N` for the whole face width even though the pitch diameter shrinks
toward the apex — the radial taper is already in the loft, so the pattern only rotates one tapered
tooth into N copies. The result includes the seed body plus its copies (`[PB-PATTERN-BODIES]`).

`component.features.circularPatternFeatures.createInput(...)`, `component.features.circularPatternFeatures.add(...)`,
`adsk.core.ObjectCollection.create(...)`, `ValueInput.createByReal(...)`, `ValueInput.createByString(...)`

Proof: `stepToothPattern` with `assertToothPattern`, which checks the copy count, that each copy is
rigid, that they sit at exact shares of the full circle, and that the tooth fits inside its angular
pitch so neighbours do not touch. Each copy is gated in its own document, because decad will not
decide a pair of loft bodies; that is stated at the function.

<!-- proof-run: proofkit3d.RunSolid(patternCases, stepToothPattern, assertToothPattern) -->

**From:** `spec/bevelgear/instructions.md` L711–L712; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-CIRCULAR-PATTERN]`, `[PB-PATTERN-BODIES]`.

## S16 `[GO]` Combine the teeth into the Gear Body

Once per gear. One Combine-Join with the Gear Body as the target and the patterned tooth bodies as
the tools: copy `pattern.bodies` item by item into a fresh `adsk.core.ObjectCollection` first, since
`combineFeatures.createInput(targetBody, toolBodies)` rejects the `BRepBodies` collection the pattern
returns (`[PB-PATTERN-BODIES]`). Do not re-add the seed body — the pattern's collection already holds
it.

`component.features.combineFeatures.createInput(...)`, `component.features.combineFeatures.add(...)`,
`adsk.core.ObjectCollection.create(...)`, `pattern.bodies.item(...)`

Proof: `stepCombineSeat` with `assertCombineSeat`. The Join itself cannot run in the harness — both
operands are lofts — so the proof builds the two bodies apart and asserts what the Join depends on:
the tooth's root reaches the blank's root cone rather than floating above it, and sits inside it by
no more than the rounding the virtual tooth count's `floor()` introduces.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCombineSeat, assertCombineSeat) -->

**From:** `spec/bevelgear/instructions.md` L713–L714; `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-PATTERN-BODIES]`.

## S17 `[GO]` Bore

Once per gear, and only when Enable Bore is checked — skip the whole step when it is not.

Build the bore plane normal to the shaft at its start with
`setByDistanceOnPath(shaftAxisEdge, ValueInput.createByReal(0.0))`, passing the in-sketch edge, not
the §2 construction line. In a sketch named `{gearLabel} Bore`, draw the bore circle centred on the
sketch origin — the plane is rooted at the shaft start, so the origin is on the axis — then **fix the
circle's centre** with `circle.centerSketchPoint.isFixed = True` and add a diameter dimension set to
the bore diameter (`[PB-CIRCLE-CENTER]`; do not coincident the centre to the sketch origin, which has
thrown `VCS_SKETCH_SOLVING_FAILED` on a `setByDistanceOnPath` plane). The diameter is this gear's
Bore Diameter when non-zero, otherwise `this gear's Pitch Diameter / 4`.

Extrude-cut it as a symmetric through cut restricted to this Gear Body (`[PB-THROUGH-CUT]`):
`extrudeFeatures.createInput(profile, FeatureOperations.CutFeatureOperation)`,
`setSymmetricExtent(ValueInput.createByReal(2 * coneDistance_cm), False)` — the second argument
`isFullLength=False` makes that a half-length per side, generously past any face width — and
`participantBodies = [gearBody]`, then `add(input)`. Gate the sketch on `isFullyConstrained`.

`component.constructionPlanes.createInput(...)`, `constructionPlaneInput.setByDistanceOnPath(...)`,
`component.constructionPlanes.add(...)`, `sketches.add(...)`,
`sketch.sketchCurves.sketchCircles.addByCenterRadius(...)`, `circle.centerSketchPoint.isFixed`,
`sketch.sketchDimensions.addDiameterDimension(...)`, `component.features.extrudeFeatures.createInput(...)`,
`extrudeInput.setSymmetricExtent(...)`, `component.features.extrudeFeatures.add(...)`,
`adsk.fusion.FeatureOperations.CutFeatureOperation`, `sketch.isFullyConstrained`

Proof: `stepBoreCut` with `assertBoreCut`, which checks the auto diameter, an explicit one, and the
disabled branch, and that the cut pierces rather than pockets. The blank is a chorded prism there
because decad admits a boolean only on prisms, cups and faceted bodies; that is stated at the
function.

<!-- proof-run: proofkit3d.RunSolid(boreCases, stepBoreCut, assertBoreCut) -->

**From:** `spec/bevelgear/instructions.md` L715–L716, L100–L105 (bore inputs);
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-CIRCLE-CENTER]`, `[PB-THROUGH-CUT]`,
`[PB-CONSTRUCTION-PLANES]`.

## S18 `[GO]` Meshing rotation

Once per gear, **in the Design component, before the body is moved out**, because a construction axis
cannot be added in the moved-out gear component and the rotation has to read the edge's world
geometry while it is still here.

Rotate the driving body by `180° / Driving Gear Teeth Number`, half a tooth pitch, about its own
shaft axis with `rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)` — the
framework helper, which takes the axis and origin from the B->I profile edge's **world** endpoints
and applies a free move (`[PB-MOVE-ROTATE]`). Both gears are patterned from a starting tooth in the
axial plane, so without this offset a driving tooth and a pinion tooth would both sit at the
axial-plane crossing and visually collide.

The pinion gets `_pinionMeshPhase(pinionTeeth)`, which returns `_PINION_MESH_PHASE_TEETH * 2π /
pinionTeeth` in radians. That class constant is **`_PINION_MESH_PHASE_TEETH = 0`**, so the pinion's
phase is 0 for both branches: the spiral's mid-face section is unrotated and already meshes.

Then `body.moveToComponent(gearOccurrence)` relocates the finished body into this gear's component;
it preserves world position and needs no activation.

`rotate_body_about_edge(...)`, `_pinionMeshPhase(...)`, `body.moveToComponent(...)`

`defineAsRotate` is named only to forbid it: it rejects a `SketchLine` axis, which is why the helper
uses a free move with a `Matrix3D`.

<!-- check-step-calls: ignore defineAsRotate -->

Proof: `stepMeshRotation` with `assertMeshRotation`, which checks the half-pitch phase for the
driving gear, none for the pinion, and that the body turns by exactly that about its own axis.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepMeshRotation, assertMeshRotation) -->

**From:** `spec/bevelgear/instructions.md` L717–L719, L296–L330 (`_pinionMeshPhase`);
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-MOVE-ROTATE]`, `[PB-CONSTRUCTION-NEEDS-ACTIVE]`,
`[PB-NO-CROSS-SIBLING]`.

## S19 `[GO]` `{gear} Cone Element` sketch and the spiral frame

Everything from here to S27 is the **ψ > 0 branch** of `_transformToothBody`, run once per gear
inside `_createGearBody` on the freshly lofted uncut apex→heel tooth, before pattern and combine. At
ψ = 0 the hook has already returned `cut_conical_ends` in S14 and none of this runs.

`_createGearBody` hands the hook four world points, positionally, in the order `toeMid, heelMid,
toeConeWorld, heelConeWorld`, and mislabelling them silently inverts the spiral:

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

`toeMid` is the **midpoint of the toe edge**, `½(M+N)` / `½(O+P)`, and `heelMid` the **midpoint of
the heel edge**, `½(C+H)` / `½(D+J)` — two different edges. Do not pass the two endpoints of one edge
as the pair: M and N both sit at the toe, so the span collapses and the spiral inverts.
`heelConeWorld` is the **dedendum corner C / D**, which lies on the root cone element Apex→C /
Apex→D — never H or J, which sit a module beyond it on the Apex2→C / Apex2→D dedendum line and skew
the element away from the root cone.

Build the frame from them: `axisDir` from the two world endpoints of the shaft-axis edge (A→G /
B→I); `coneVec`, the unit vector from `apexWorld` to `heelConeWorld`, which is the root cone
element; `v = axisDir × coneVec` normalized, the circumferential direction; `tpNormal = coneVec ×
v`; and the cone distance of a point, `(p - apexWorld) · coneVec`, written distAlong below.
**Before building `coneVec`, run the swap guard:** if
`apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid` with `heelMid` **and**
`toeConeWorld` with `heelConeWorld`. A negative span flips the cutter-arc direction, the slice
direction and the per-segment twist at once, and the gear comes out wrong with no error. Then
R_toe is the distAlong of `toeMid`, R_heel the distAlong of `heelMid`,
`R_mean = ½(R_toe + R_heel)` and `span = R_heel − R_toe`, now positive.

The sketch this step writes is `{gear} Cone Element`, on the **axial (Gear Profiles) plane**, holding
one construction line from the apex to `apex + R_heel · coneVec`. It is the line the Trace Plane is
rotated about in S20. Its points are passed to the sketch call as they are, with **no
`modelToSketchSpace` conversion** — see the coordinates note in S21, which governs this sketch as
well.

`sketches.add(...)`, `sketch.sketchCurves.sketchLines.addByTwoPoints(...)`, `adsk.core.Point3D.create(...)`,
`sketchLine.worldGeometry`, `adsk.core.Point3D.distanceTo(...)`

Proof: `stepConeElement`, which checks that `coneVec` really is the root element and neither
Apex→Apex2 nor the shaft axis, that the circumferential direction is across both, that the hand-off
points are the ones the table names, and that the swap guard restores the same frame from swapped
arguments.

<!-- proof-run: proofkit.Run(spiralCases, stepConeElement) -->

**From:** `spec/bevelgear/instructions.md` L565–L597 (§3a intro, hand-off, step A), L296–L330
(the hook's signature); `spec/bevelgear/spiral-tooth-trace.md` L30–L79 (the drawing plane),
L66–L79 (the three cone-distance marks); `.claude/skills/generate-gear/PLAYBOOK.md`
`[PB-WORLD-FRAME]`.

## S20 `[PROSE]` `{gear} Trace Plane`

Rotate the axial plane 90° about the cone-element line to get the tangent plane:
`plane_by_angle(designComponent, coneElementLine, axialPlane, 90)`, named `{gear} Trace Plane`. The
sketch line goes to `setByAngle` directly (`[PB-CONSTRUCTION-PLANES]`).

`plane_by_angle(...)`

**From:** `spec/bevelgear/instructions.md` L609–L611; `spec/bevelgear/spiral-tooth-trace.md` L30–L64;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-CONSTRUCTION-PLANES]`.

## S21 `[GO]` `{gear} 2D Tooth Trace` sketch — the cutter arc

Work in the tangent-plane 2-D frame with the apex at the origin, `x = coneVec` (so a point's x is its
cone distance) and `y = v`. The cutter radius `r_c` is the Cutter Radius when non-zero, otherwise
`R_mean`. The hand sign is `+1` for `Right` and `−1` for `Left`, **negated for the pinion**, since
the pair meshes with opposite hands. The cutter-circle centre is

```
Cx = R_mean − r_c · sin ψ
Cy = handSign · r_c · cos ψ
```

⚠️ The hand sign goes on the **cos / Cy** term, never on the sin / Cx term. Opposite hands mirror the
centre across the cone element, which flips `Cy`; putting the sign on `Cx` mirrors about
`x = R_mean` instead, a different curve that gives the two gears unequal twist where equal teeth must
give exact mirror images.

The arc's ends are taken a hair past the face so the kept arc reaches past the end trims:
`toe2d = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)` and
`heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)` with `R_lo = R_toe − 0.06·span` and
`R_hi = R_heel + 0.06·span`. The helper intersects the apex circle of that radius with the cutter
circle and keeps the solution nearest `(R_mean, 0)` — the branch the mean point sits on.

In a sketch named `{gear} 2D Tooth Trace` on the Trace Plane, with
`tanW(px, py) = combine_point(apex, px, coneVec, py, v)` mapping 2-D coordinates to world, draw:

- the **cutter circle**, centred at `tanW(Cx, Cy)` with radius `r_c`, `isConstruction`, its centre
  pinned by `centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`) and a diameter dimension of
  `2·r_c` whose text point is on the curve, `tanW(Cx + r_c, Cy)` (`[PB-RADIAL-DIM]` — a text point at
  the centre is rejected);
- the **trace arc**, a three-point arc through `tanW(toe2d)`, `tanW(R_mean, 0)` and `tanW(heel2d)`,
  with its centre made **coincident to the cutter circle's centre** and a radius dimension of `r_c`,
  so it is the genuine cutter circle and not a look-alike. Its text point is the mean point
  `tanW(R_mean, 0)`. The coincident is required and is not the redundant case of
  `[PB-SHARE-XOR-COINCIDENT]`: `addByThreePoints` and `addByCenterStartEnd` **copy** the centre
  rather than sharing it, so without it the centre is a free point that stays behind and deforms the
  arc.

**Coordinates — this governs the `{gear} Cone Element` sketch of S19 as well.** The world `Point3D`s
from `tanW(...)`, and the raw apex and cone-end points of the cone-element line, are passed
**directly** into `addByTwoPoints`, `addByCenterRadius` and `addByThreePoints`, where they are
consumed as sketch-space input, with **no `modelToSketchSpace` conversion**, even though the points
really are model-space coordinates. This is deliberate and harmless only because nothing downstream
consumes either sketch: the twist is computed analytically in S24, and the trace sketch exists only
so the cutter arc is inspectable before cleanup hides it. The cone-element line is consumed, by
`plane_by_angle`, so an unconverted line does place the Trace Plane somewhere other than the true
tangent plane — and that still reaches no feature, because the only thing built on the Trace Plane is
this inspection-only sketch. **If any later revision makes a feature consume the trace sketch or the
Trace Plane, both sketches need `modelToSketchSpace` on every point.**

This sketch is **deliberately left with free degrees of freedom** — the arc's ends are pinned by the
three-point construction, not by endpoint dimensions, which would over-constrain the solve against
the cone-element plane — and it is **exempt** from the full-constraint gate along with the other
spiral auxiliaries. Do not gate it.

There is **no 3-D projection**: no `projectToSurface`, no root-cone face search and no 3-D trace
sketch. For unequal-ratio pairs the projected arc wraps around the cone and comes back as multiple
disjoint fragments, so the measured azimuth collapses and the pinion comes out grossly
under-twisted; the analytic law in S24 is exact and cannot wrap.

`circle_intersect_nearest(...)`, `combine_point(...)`, `sketches.add(...)`,
`sketch.sketchCurves.sketchCircles.addByCenterRadius(...)`, `sketch.sketchCurves.sketchArcs.addByThreePoints(...)`,
`circle.centerSketchPoint.isFixed`, `sketch.sketchDimensions.addDiameterDimension(...)`,
`sketch.sketchDimensions.addRadialDimension(...)`, `sketch.geometricConstraints.addCoincident(...)`,
`math.sin(...)`, `math.cos(...)`

`projectToSurface` is named only to say it is not called here, and the `modelToSketchSpace`
conversion is named to say this sketch does without it — S11 and S25 do call it.

<!-- check-step-calls: ignore projectToSurface -->

Proof: `stepSpiralTrace`, which checks the invariants the trace derivation lists — the ends on their
apex circles, one cutter radius everywhere, the centre gap closed, the arc through the mean point,
the spiral angle realised **at** the mean point, and the two hands as mirror images — plus that the
twist law uses the pitch cone angle and not the root one. The proof pins the arc's ends where the
spec leaves them free, because proofkit waives nothing; the cost is stated at the function.

<!-- proof-run: proofkit.Run(spiralCases, stepSpiralTrace) -->

**From:** `spec/bevelgear/instructions.md` L598–L621 (steps B, C, D);
`spec/bevelgear/spiral-tooth-trace.md` L80–L185 (apex circles, the hand, the centre, the ends, the
arc), L218–L239 (invariants); `spec/bevelgear/fusion.md` L60–L72 (the auxiliary-sketch exemption);
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-CIRCLE-CENTER]`, `[PB-RADIAL-DIM]`,
`[PB-SHARE-XOR-COINCIDENT]`.

## S22 `[GO]` Slice the straight tooth into cross-section slabs

Split the uncut apex→heel tooth body into cross-section slabs with planes **perpendicular to the cone
element**, on a **fixed** scheme of eight planes — the count is not user-configurable. The first cut
plane is the **parent transverse tooth plane** (`parentToothPlane`, the `{gearLabel} Plane` of S07,
passed into the hook) offset toward the apex by `span/6`, and the rest step on in `span/6`
increments: `offsets = [sign·(k+1)·span/6 for k in 0…7]`. The **sign is chosen per gear** so the
offsets move apex-ward — the parent plane's normal points opposite ways for the two gears — by
testing `(apex − planeOrigin) · normal`.

Split with the framework helper `slice_body_by_offset_planes(designComponent, toothBody,
parentToothPlane, offsets)`, which splits piece by piece and keeps a piece whole when a plane misses
it.

⚠️ **The slice must actually split the tooth.** If the body is still in one piece after the loop, the
sign was wrong or the parent plane sits outside the tooth's span: retry the whole cut once with the
opposite sign, and if it is still one piece **raise** a self-diagnosing error naming the gear, the
final piece count, the span and the sign tried (`[PB-EMPTY-RESULT]`, `[PB-SELF-DIAGNOSING]`). Never
return an unsliced result: S23 then drops that one piece as the apex scrap, leaving no segments, and
the crown dies later with `ValueError: max() iterable argument is empty`, far from the cause.

`slice_body_by_offset_planes(...)`, `constructionPlane.geometry.normal`

Proof: `stepSpiralSlices` with `assertSpiralSlices`, which checks the eight offsets are the span/6
shares, that they step toward the apex, and that the cuts fall across the face rather than leaving it
whole. decad has no split and refuses booleans on lofts, so each slab is built as the loft between
consecutive cut sections and gated in its own document; that is stated at the function.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepSpiralSlices, assertSpiralSlices) -->

**From:** `spec/bevelgear/instructions.md` L622–L623 (step E);
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-EMPTY-RESULT]`, `[PB-SELF-DIAGNOSING]`,
`[PB-SPLIT-BODY]`, "Shared geargen helper library".

## S23 `[GO]` Order the segments and drop the apex scrap

Sort the pieces by the `distAlong` of their centroid, `body.physicalProperties.centerOfMass`. The
first, apex-most piece is the long **apex-side scrap** below the toe: re-slice the list first,
`segments = segments[1:]`, and only **then** delete the scrap with
`component.features.removeFeatures.add(scrap)` — a timeline-visible removal, not a bare `deleteMe`
(`[PB-REMOVE-PIECES]`).

After the drop, `segments` must hold at least one cross-section. If it is empty the slice failed in
S22: **raise** a clear error rather than proceeding into the twist and the crown, which both assume a
non-empty list (`[PB-EMPTY-RESULT]`).

`body.physicalProperties.centerOfMass`, `component.features.removeFeatures.add(...)`

`deleteMe` is named only to forbid it as the way to drop the scrap.

<!-- check-step-calls: ignore deleteMe -->

Proof: `stepSpiralScrap` with `assertSpiralScrap`, which checks that sorting by centroid cone
distance orders the slabs strictly, and that the piece kept after the drop lies beyond the first cut.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepSpiralScrap, assertSpiralScrap) -->

**From:** `spec/bevelgear/instructions.md` L624–L625 (step F);
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-REMOVE-PIECES]`, `[PB-EMPTY-RESULT]`.

## S24 `[GO]` Twist each segment about the shaft axis

Rotate each segment about the **shaft axis** — `axisDir` through `apexWorld` — so the tooth follows
the trace, centred on `R_mean` so the mid-face section stays unrotated. That section then meshes
exactly like the straight tooth, which is why the pinion's mesh phase can be zero.

The total toe→heel twist comes from the conjugate crown-gear generation law, computed
**analytically** — no projection, no curve sampling:

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`gamma` is **this gear's pitch cone angle**, `self._gamma_p` or `self._gamma_g` from §2, forwarded
into the hook. ⚠️ It is **not** `acos(coneVec · axisDir)`, which measures the root cone — about 14°
against a pitch 29° for a 17-tooth pinion — and inflates the twist by roughly 1.6×. The two members
of a meshing pair legitimately get **different** twists: same cutter and same ψ, but different γ, so
different `1/sin γ`. That is why equal-teeth pairs meshed under methods that got this factor wrong
while ratio pairs failed.

Each segment's share is linear and keyed on the cone distance of its **heel face**, the exact section
the S26 loft samples:

```
ang = −handSign · total · (R_mean − R_heelFace(seg)) / span
```

**Find a slab's heel face as the face whose centroid has the greatest distAlong of `face.centroid`,
searched across ALL of the slab's faces with no surface-type filter** (its toe face is the least).
Do not restrict the search to `PlaneSurfaceType` or any other type: a sliced slab is bounded by a mix
of planar cut faces and ruled side faces, and a type filter can pick the wrong face or miss the cut
face, which makes the S26 loft fail with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. The same
all-faces-by-centroid rule is used everywhere a slab end face is needed — here, in S25 and in S26.
⚠️ Key the twist on the heel face, not on the segment's centroid: the loft samples that face, and
centroid-keying leaves the mid-face section rotated by half a segment and the tooth overlapping
itself there.

Apply the rotation as a free move by a `Matrix3D.setToRotation(ang, axisDir, apexWorld)`
(`[PB-MOVE-ROTATE]`).

`adsk.core.Matrix3D.setToRotation(...)`, `component.features.moveFeatures.createInput2(...)`,
`moveFeatureInput.defineAsFreeMove(...)`, `component.features.moveFeatures.add(...)`, `face.centroid`,
`body.faces`, `math.atan2(...)`, `math.sin(...)`, `adsk.core.ObjectCollection.create(...)`

Proof: `stepSpiralTwist` with `assertSpiralTwist`, which checks the mid-face section is unrotated,
that the two ends turn opposite ways by half the total each, that the hand reverses every rotation,
and that a pair with different pitch cone angles gets different twists while an equal pair gets
equal ones.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepSpiralTwist, assertSpiralTwist) -->

**From:** `spec/bevelgear/instructions.md` L626–L640 (step G);
`spec/bevelgear/spiral-tooth-trace.md` L186–L217 (the analytic twist and `1/sin γ`);
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-MOVE-ROTATE]`, `[PB-WORLD-FRAME]`.

## S25 `[GO]` Lengthwise crown

Scale every segment **except the outermost (heel) one** down by a monotonic factor — full at the
heel, growing toward the toe. For each segment take its heel-distance fraction
`u = (R_heel − R_heelFace) / span`, with `R_heelFace` found by the S24 all-faces-by-centroid rule but
**recomputed here, after the twist has moved the slabs** (do not reuse pre-twist values), so `u` runs
0 at the held-full heel to 1 at the toe. The outermost segment is the one with the **greatest
post-twist heel-face `distAlong`**: sort by that and skip the last. Then

```
factor = 1 − _CROWN_PER_RAD · (|total| / 2) · u
```

with `_CROWN_PER_RAD` a tunable class constant, default **0.5** (0 disables the crown) — set it to
0.5, do not leave it unset. If a computed factor comes out at or below zero, **raise** a
self-diagnosing error naming the gear, the segment's `u` and the factor; never scale by a
non-positive factor. ⚠️ Do **not** key the relief on `|ang|`, the twist magnitude: that is symmetric
about the mid-face and maximal at both ends, so with the heel slab held full the slab just inside it
becomes the most-relieved one and dips below both neighbours, reversing the heel→toe taper (measured:
0.932 against a taller 0.972 next inward).

Three things about the scale itself:

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). `scaleFeatures` is the one exception to never
   activating: call **`designOccurrence.activate()`** before the crown scales and restore afterwards,
   in a `finally`, with **`design.activateRootComponent()`**.
2. **Skip the outermost segment.** Its heel face is the loft's heel end and must stay full so the
   heel cone trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, not its centroid.** `scaleFeatures` shrinks
   uniformly toward the base point, so a centroid base pulls the tooth's root edge up by
   `(1 − factor) · ½ tooth height`: the tooth stops seating on the root cone, floats above the base,
   and the Combine-Join leaves a visible gap on ratio pairs. Of the heel face's vertices
   (`heelFace.vertices`, each `.geometry` a world `Point3D`), take the **two with the smallest
   perpendicular distance to the shaft axis** — the two root corners, since the tip corners are
   farthest from the axis — and place the base sketch point at their **midpoint**, mapped into the
   heel-face sketch with `modelToSketchSpace`. A uniform scale keeps every line through its base
   point fixed, so a root anchor keeps the root edge on the seating cone while the tip is relieved.

`component.features.scaleFeatures.createInput(...)`, `component.features.scaleFeatures.add(...)`,
`designOccurrence.activate(...)`, `design.activateRootComponent(...)`, `sketch.sketchPoints.add(...)`,
`sketch.modelToSketchSpace(...)`, `face.vertices`, `vertex.geometry`, `face.centroid`,
`adsk.core.ObjectCollection.create(...)`, `ValueInput.createByReal(...)`

A `Component` has no `.activate()` — `design.rootComponent.activate` and `someComponent.activate` are
named only to forbid them; only an `Occurrence` has `activate`, and the root is restored through
`Design.activateRootComponent`.

<!-- check-step-calls: ignore rootComponent.activate someComponent.activate -->

Proof: `stepSpiralCrown`, which checks the factor law and its monotonic key, that the heel slab is
held full, that a factor at or below zero is refused, that scaling about the root-edge midpoint
leaves the root edge exactly where it was while the tip comes down, and that the centroid anchor
lifts the root by half the height lost. decad's transforms are isometries, so a scale is
unrepresentable there and the crown is proved on the section a slab end face is; that is stated at
the function.

<!-- proof-run: proofkit.Run(spiralCases, stepSpiralCrown) -->

**From:** `spec/bevelgear/instructions.md` L641–L654 (step H);
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-CONSTRUCTION-NEEDS-ACTIVE]`, `[PB-SPACE-METHODS]`,
`[PB-SELF-DIAGNOSING]`; `spec/bevelgear/fusion.md` L172–L179 (the sole never-activate exception).

## S26 `[GO]` Loft the crowned segments into the curved tooth

⚠️ **Re-sort the segments by their heel-face cone distance here, after the twist and the crown** —
do not reuse the pre-twist slice order from S23. The twist rotates each slab about the shaft axis,
and for a high-twist unequal-ratio pair that rotation changes the slabs' along-cone order enough to
swap neighbours; lofting in the stale order assembles the cross-sections out of sequence and the
crowned tooth comes out distorted, which is the single thing that makes a 31/17 pair fail while
31/31 looks fine. So sort the segment indices **now**, keyed on the distAlong of each segment's
heel-face `face.centroid`, with the same all-faces-by-centroid rule as S24.

Loft a `NewBodyFeatureOperation` through, in that order: first the **toe-most segment's apex-side
(toe-facing) face** — `order[0]`'s least-centroid face, added first so the loft pushes past the toe
cone and the toe trim bites — then the **heel-facing face of every segment, iterated in `order`**,
the last of which reaches past the heel cone. Name the resulting body `{gear} Spiral Tooth`. Then
remove the segment scaffolding; the loft has captured their faces.

`component.features.loftFeatures.createInput(...)`, `loftInput.loftSections.add(...)`,
`component.features.loftFeatures.add(...)`, `component.features.removeFeatures.add(...)`, `face.centroid`,
`body.faces`, `adsk.fusion.FeatureOperations.NewBodyFeatureOperation`

Proof: `stepSpiralLoft` with `assertSpiralLoft`, which checks the order is strictly toe to heel, that
the twist across the chain is the full toe→heel share, and that the chain spans the face. decad lofts
one pair of sections at a time, so the chain is built pairwise and each link gated on its own; that
is stated at the function.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepSpiralLoft, assertSpiralLoft) -->

**From:** `spec/bevelgear/instructions.md` L655–L656 (step I);
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-LOFT]`, `[PB-EMPTY-RESULT]`.

## S27 `[PROSE]` Flush-trim the curved tooth

The hook ends by returning `cut_conical_ends(designComponent, curvedTooth, gearBody, toeMid, heelMid,
apexWorld, gearLabel)` — the same toe-then-heel two-cone trim S14 describes and proves, applied to
the curved tooth so its ends sit flush on the gear base. The caller obligations are S14's,
unchanged.

The toe and heel **mesh phasing** is not done here: it belongs to `_createGearBody`'s mesh-rotate
step (S18), and the pinion's extra phase is 0 by default because the mid-face section is unrotated
and already meshes.

`cut_conical_ends(...)`

**From:** `spec/bevelgear/instructions.md` L657–L658 (step J), L683–L710.

## S28 `[PROSE]` Cleanup

Call `hide_construction_geometry(bevelComponent)` — the framework helper, do not re-implement the
walk. It walks the Bevel Gear component tree recursively, dedupes by `entityToken`, and sets
`isLightBulbOn = False` on every sketch, construction plane and construction axis; construction
planes and axes are **not** hidden by `isVisible` (`[PB-HIDE-AFTER-USE]`, `[BEVEL-F-CLEANUP]`).
Only the two finished gear bodies stay visible. Bevel always builds solids, so there is no
sketch-only mode and no per-mode guard.

`hide_construction_geometry(...)`

**From:** `spec/bevelgear/instructions.md` L721–L724; `spec/bevelgear/fusion.md` L180–L185;
`.claude/skills/generate-gear/PLAYBOOK.md` `[PB-HIDE-AFTER-USE]`, `[PB-TREE-CLEANUP]`.
