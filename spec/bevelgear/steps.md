The proof for this gear is `proof/bevelgear/geometry_test.go`, `proof/bevelgear/sketches_test.go`, `proof/bevelgear/solids_test.go`, `proof/bevelgear/spiral_test.go` and `proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `34b7d96324989b2b62d1f057eccbbe3ced9642f3` |
| `spec/bevelgear/fusion.md` | `754729fc9a33ad9dbf93816e0430e4b94d82e05b` |
| `spec/bevelgear/spiral-tooth-trace.md` | `c9ec08561ced7975aa0ed9ad6a330186259c0d08` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `abb1123b5910f10c5c96c1ad936a38691ea3e7fb` |

## How to read this step list

Each step below is one entry in the Fusion timeline, in build order. A `[GO]` step names the proof
function that realises it and carries a `proof-run` annotation; a `[PROSE]` step has no geometry the
bench can hold.

**Units.** Every length this module computes is in Fusion's internal **centimetres**. `Module` is the
one exception: it is read with unit `''`, so it comes back as a raw number meaning **millimetres**.
Every length derived from `Module` must therefore be `to_cm`-converted before it touches geometry —
the pitch diameters, the Cone Distance, the dedendum `1.25 * Module`, the module-length construction
extensions, and the default Face Width. The `'mm'` and `'deg'` inputs come back already in internal
units (cm / radians) and must **not** be `to_cm`'d again. Mixing a raw-mm Module-derived length with
an already-cm `'mm'` input makes the gear come out about ten times off and the Face-Width bound
meaningless.

**Symbols used throughout.** `Σ` Shaft Angle; `m` Module (raw mm); `N_g` Driving Gear Teeth Number;
`N_p` Pinion Gear Teeth Number; `DPD = m · N_g` Driving Gear Pitch Diameter; `PPD = m · N_p` Pinion
Gear Pitch Diameter; `γ_p` pinion pitch cone angle; `γ_g` driving pitch cone angle; `R` **Pitch Cone
Distance** (never the Cone Distance); `Cone Distance = sqrt(DPD² + PPD²)`; `ψ` Mean Spiral Angle.

**Names that are ours, not Autodesk's.** The framework helpers, the generator's own private methods,
Python builtins and quoted Fusion error text appear in code spans below so the emit stage reproduces
them exactly, but they are not `adsk.*` calls:

<!-- check-compile: ignore cut_conical_ends apply_conical_cut select_keeper find_cone_faces_by_midpoint surface_distance slice_body_by_offset_planes rotate_body_about_edge plane_by_angle combine_point circle_intersect_nearest hide_construction_geometry find_profile_by_curve_counts VirtualSpurProxy SpurGearInvoluteToothDesignGenerator NonIntersectError -->
<!-- check-compile: ignore get_selection get_boolean get_value to_cm to_mm get_design futil log settle_sketch_display -->
<!-- check-compile: ignore configure handle_input_changed _updateSpiralInputVisibility generate deleteComponent _readInputs _buildAnchorSketch _buildGearProfiles _buildVirtualSpurProfile _createGearBody _transformToothBody _cutBore _pinionMeshPhase _hideConstructionGeometry -->
<!-- check-compile: ignore _lastToothEmbedded _HAND_RIGHT _HAND_LEFT _CROWN_PER_RAD _PINION_MESH_PHASE_TEETH INPUT_ID_PLANE INPUT_ID_CENTER_POINT INPUT_ID_PARENT INPUT_ID_MODULE INPUT_ID_SHAFT_ANGLE INPUT_ID_DRIVING_TEETH INPUT_ID_PINION_TEETH INPUT_ID_DRIVING_BASE_HEIGHT INPUT_ID_PINION_BASE_HEIGHT INPUT_ID_BORE_ENABLE INPUT_ID_DRIVING_BORE INPUT_ID_PINION_BORE INPUT_ID_FACE_WIDTH INPUT_ID_TOOTH_SPACING INPUT_ID_SPIRAL_ANGLE INPUT_ID_HAND INPUT_ID_CUTTER_RADIUS INPUT_ID_TOE_EXTENSION INPUT_ID_DRIVING_TOE_RADIUS INPUT_ID_PINION_TOE_RADIUS -->
<!-- check-compile: ignore math radians degrees floor sqrt sin cos tan atan atan2 acos hypot int round min max abs raise draw drawCircles drawTooth getParameter -->
**Names this step list mentions but the module must NOT call.** Each appears in a code span only so
the emit stage can see which call is being ruled out and why: `project2` is named to say it is not a
substitute for `project`; `addVertical` would force a §2 line to the sketch's world-vertical, which is
wrong on a tilted target plane; `setByPerpendicularAtPoint` is the construction-axis constructor this
build cannot use, because it would need a `BRepFace` that does not exist here; `defineAsRotate` is the
move variant that rejects a `SketchLine` axis; `projectToSurface` is the removed 3-D trace projection
that wraps around the cone for ratio pairs; `getParameterAtPoint` and `transformBy` are named only in
the explanations of why a face is found by midpoint and why `modelToSketchSpace` is called directly.

<!-- check-step-calls: ignore project2 addVertical setByPerpendicularAtPoint defineAsRotate projectToSurface getParameterAtPoint transformBy -->

<!-- check-compile: ignore ASM_WIRE_X_AXIS ASM_API_FAILED ASM_RBI_INTERNAL LOFT_NO_TOOLBODY ASM_NOT_ALL_SECTIONS_MEET VCS_SKETCH_OVER_CONSTRAINTS VCS_SKETCH_SOLVING_FAILED SPLIT_TARGET_TOOL_NOT_INTERSECT -->

## S01 `[PROSE]` Configure the command dialog

`BevelGearCommandInputsConfigurator.configure(cls, cmd)` adds **20** inputs to `cmd.commandInputs`,
in exactly the order of the table below. Target Plane goes first so it wins Fusion's auto-focus
(`[PB-AUTOFOCUS-FIRST]` — Fusion focuses the FIRST `addSelectionInput` and ignores a later
`hasFocus`), then Center Point, then the pre-selected Parent Component, then the numeric and boolean
fields.

Module-level constants hold the ids, in row order, named exactly `INPUT_ID_PLANE`,
`INPUT_ID_CENTER_POINT`, `INPUT_ID_PARENT`, `INPUT_ID_MODULE`, `INPUT_ID_SHAFT_ANGLE`,
`INPUT_ID_DRIVING_TEETH`, `INPUT_ID_PINION_TEETH`, `INPUT_ID_DRIVING_BASE_HEIGHT`,
`INPUT_ID_PINION_BASE_HEIGHT`, `INPUT_ID_BORE_ENABLE`, `INPUT_ID_DRIVING_BORE`,
`INPUT_ID_PINION_BORE`, `INPUT_ID_FACE_WIDTH`, `INPUT_ID_TOOTH_SPACING`, `INPUT_ID_SPIRAL_ANGLE`,
`INPUT_ID_HAND`, `INPUT_ID_CUTTER_RADIUS`, `INPUT_ID_TOE_EXTENSION`, `INPUT_ID_DRIVING_TOE_RADIUS`,
`INPUT_ID_PINION_TOE_RADIUS`. Two more module constants carry the dropdown's item strings:
`_HAND_RIGHT = 'Right'` and `_HAND_LEFT = 'Left'`. There are **no** `PARAM_*` strings: bevel
registers **no** live Fusion user parameters at all (`[PB-PRECOMPUTED-MODE]`).

**The dialog input table. Every label, id, unit string and default below is contract surface; copy
each one character for character and tidy none of them — row 18's label carries a unit marker its
input's own unit string cannot show.**

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

**Calls.** Each selection input is `inputs.addSelectionInput(<id>, <label>, <tooltip>)`, then one
`selectionInput.addSelectionFilter(...)` per filter written as the named constant and never a quoted
literal (`[PB-SELECTION-FILTER-ENUM]`) — `adsk.core.SelectionCommandInput.ConstructionPlanes`,
`adsk.core.SelectionCommandInput.PlanarFaces`, `adsk.core.SelectionCommandInput.ConstructionPoints`,
`adsk.core.SelectionCommandInput.SketchPoints`, `adsk.core.SelectionCommandInput.Occurrences`,
`adsk.core.SelectionCommandInput.RootComponents` — then
`selectionInput.setSelectionLimits(1, 1)` (`[PB-SELECTION-DECL]`). The Parent input pre-selects the
root component with `parentInput.addSelection(get_design().rootComponent)`.

Numeric rows are `inputs.addValueInput(<id>, <label>, <unit>, <ValueInput>)` with the default built by
`adsk.core.ValueInput.createByReal(...)` or `adsk.core.ValueInput.createByString('90 deg')` /
`adsk.core.ValueInput.createByString('35 deg')`. **A `createByReal` default is in INTERNAL units
regardless of the unit string** (`[PB-DIALOG-DEFAULT-UNITS]`), which is why every `mm` default is
written `createByReal(to_cm(0))` and the two angles are given as expression strings. Row 18 is a
plain unitless percentage and needs no `to_cm`. The checkbox is
`inputs.addBoolValueInput('boreEnable', 'Enable Bore', True, '', True)`. The dropdown is
`inputs.addDropDownCommandInput('spiralHand', 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)`
followed by `handInput.listItems.add(_HAND_RIGHT, True)` and `handInput.listItems.add(_HAND_LEFT, False)`.

**Conditional visibility — the spiral-only inputs show only when ψ > 0.** `spiralHand` and
`cutterRadius` are hidden whenever ψ = 0 and shown when ψ > 0. `spiralAngle` is the controller and is
**always visible**. There is no declarative show-if in the Fusion API, so this is done with
`commandInput.isVisible`:

- a `@classmethod _updateSpiralInputVisibility(cls, inputs)` helper reads the `spiralAngle` input's
  **`.expression`** through `unitsManager.evaluateExpression(spiral.expression, 'rad')` — internal
  **radians**, and NOT the input's `.value` — and sets
  `inputs.itemById(INPUT_ID_HAND).isVisible` and `inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible`
  to `(value > 0)`. **Guard it:** return early if any of the three inputs is `None`, and wrap the
  expression evaluation in `try`/`except`, because a half-typed expression can raise mid-edit; on
  failure leave both inputs **shown**;
- `configure()` calls `cls._updateSpiralInputVisibility(inputs)` as its **last** step, so the initial
  state is right (default ψ = 35° → both shown);
- `@classmethod handle_input_changed(cls, args)` calls
  `cls._updateSpiralInputVisibility(args.inputs)` on **every** input change, and is bound by name from
  `commands/bevelgear/entry.py`.

`isVisible` only hides the dialog row. The input still exists, `_readInputs` reads it normally, and
the ψ = 0 build ignores Hand and Cutter anyway, so hiding is cosmetic and cannot affect generation.

**`configure` and `handle_input_changed` are entry points this module DEFINES, not calls it makes.**
`commands/bevelgear/entry.py` binds both by name, and Fusion's own command lifecycle is what invokes
`configure(args.command)` when the dialog opens and `handle_input_changed(args)` on every
`inputChanged` event. Nothing inside this module calls either one, so naming them above pins the
surface the entry point binds to rather than asking for a call.

<!-- check-step-calls: ignore configure handle_input_changed -->

**From:** `spec/bevelgear/instructions.md` L27–34 L100–104 L138 L140–144 L145–229, `.claude/skills/generate-gear/PLAYBOOK.md` L53–60 L128–143 L346–348 L548–559 L843–852

## S02 `[PROSE]` Read and validate the inputs

`generate(inputs)` calls `_readInputs(inputs)` **first**, before anything creates an occurrence, and
before any geometry. It returns the 7-tuple `(parentComponent, targetPlane, centerPoint, module,
drivingTeeth, pinionTeeth, shaftAngle_deg)` and stashes the rest on `self`:
`self._drivingBaseHeight_cm`, `self._pinionBaseHeight_cm`, `self._boreEnable`,
`self._drivingBore_cm`, `self._pinionBore_cm`, `self._faceWidth_cm`, `self._toothSpacing_cm`,
`self._spiralAngle_rad`, `self._hand`, `self._cutterRadius_cm`.

**How each input is read** (`[PB-INPUT-READ]`). Selections use `get_selection(inputs, <id>)`. The
checkbox uses `get_boolean(inputs, 'boreEnable')` — never `get_value`, which reads `.expression` and
raises `AttributeError` on a `BoolValueCommandInput`. The dropdown is read as
`inputs.itemById(INPUT_ID_HAND).selectedItem` and then `.name`, defaulting to `_HAND_RIGHT` when
nothing is selected. Every numeric and angle input is read by evaluating its expression with
`design.unitsManager.evaluateExpression(input.expression, <units>)` using `''` / `'mm'` / `'deg'` as
the table says; the value comes back in Fusion internal units (cm, radians) **whatever** the unit
string, so a `deg` field arrives in radians and must go through `math.degrees(...)` before any
degree-range check (`[PB-EVAL-EXPRESSION]`). Both teeth inputs are coerced with `int(round(...))`
before validation.

**Range checks, in this order.**

1. `module > 0`; both teeth `>= 3`; non-negative base heights, bore diameters, face width, tooth
   spacing, cutter radius and Toe Radii; Toe Extension in `[0, 100]`; ψ in `[0, 60)` degrees.
2. **Shaft Angle**: at least `30°` and **below the Maximum Shaft Angle**. It depends on both tooth
   counts, so check it once both are read and coerced, and name the computed limit in the message.

   `Maximum Shaft Angle = min(degrees(acos(-min(DPD, PPD) / max(DPD, PPD))), 150)`.

   The cone-angle half is **exclusive** and the 150° half **inclusive**. A pitch cone angle reaching
   90° turns that gear's cone inside out: `R · cos γ` — the along-shaft seed length for Apex→A and
   Apex→B, and the denominator of the back-cone virtual pitch radius — passes through zero and
   changes sign, so the seed points backwards and the virtual radius is unbounded. A 31/17 pair gives
   `acos(-17/31) = 123.26°`; equal tooth counts give `acos(-1) = 180°`, which is no constraint, hence
   the 150° practical cap.

   ⚠ **30° is the documented floor but is not known to be reachable.** Of three independently written
   §2 lattices two refuse the default pair there on conditioning and first clear at 35°. That split is
   a property of the construction, so the range stated here is the geometric one and the reachable
   floor is the proof's business. **Never write a Shaft Angle bound from a conditioning measurement.**

Then compute `γ_p` and `γ_g` from the closed form (see S07), and in this order:

3. **Minimum Teeth**, per gear, against that gear's own `γ`, on top of the blanket `teeth >= 3`:
   `teeth >= 5.27 * cos γ`. The constant is `2 * (1.05 * 1.25 / 0.95 + 1.25) = 5.2632`, **rounded UP
   to 5.27** so the published floor stays at or above the exact crossing; do not round it down and do
   not substitute the exact value without re-running the low-tooth-count cases. Name the computed
   floor in the message. At Σ = 90° the floor is 3.72, i.e. **4 teeth** — measured, an equal 4-tooth
   pair solves and a 3-tooth pair still fails on the heel edge.
4. **Base heights**, per gear, with `r` that gear's pitch radius and `γ` its own cone angle:

       Minimum Base Height = 1.05 * 1.25 * m * sin γ
       Maximum Base Height = 0.95 * (r - 1.25 * m * cos γ) * tan γ

   Both are closed-form and need no solved sketch geometry, so resolve them here. Apply each in both
   directions: raise a fallback below the minimum, cap a fallback above the maximum, and **reject** a
   user value outside either end with a message stating the bound it broke. The driving fallback is
   `m * N_g / 8`; the pinion fallback is the **resolved** driving height `* (N_p / N_g)` — resolved
   meaning after the driving fallback and the driving cap, never the raw driving input — and then the
   **pinion's own** bounds are applied to that result, because the two gears have different cone
   angles whenever the tooth counts differ.

   Read the origin carefully. The base height is the offset between the A→Apex2 drop and G→H (resp.
   B→Apex2 and I→J), so it is measured from **Apex 2's plane**, not from the dedendum point. Walking
   out along the dedendum line from Apex 2 the perpendicular distance to the shaft axis falls at
   `cos γ` per unit, so H reaches the axis at `r * tan γ`; the bound above sits `1.25 * m * sin γ`
   below that crossing and is **deliberately conservative, not exact**. Past the true crossing the
   hexagonal frustum profile has crossed its own axis of revolution and the revolve fails with
   `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`). Worked case, Module 1 / 31 / 31 / Σ = 30°: each `γ` is 15°, the
   bound is `0.95 * (15.5 - 1.25*cos 15°) * tan 15° = 3.638 mm`, the true crossing is
   `15.5 * tan 15° = 4.153 mm`, and the driving fallback resolves to `3.875 mm`, so the default is
   capped to 3.638 mm but would not have folded uncapped.

   Order matters between 3 and 4: the Minimum Teeth check is exactly the statement that the base
   height window is non-empty, so running it first means 4 never has to describe an empty window.

The remaining resolutions (Maximum Face Width, Face Width, the Toe Radii, Root Length) need solved §2
geometry and belong to S07.

⚠ **A configuration can satisfy every bound here and still be refused as near-singular.** That limit
belongs to the particular §2 lattice, not to this spec. Treat a near-singular report as a real refusal
of that construction, never as a tolerance to loosen.

**Bore diameters.** Only consulted when Enable Bore is checked. A value of `0` means auto-calculate:
use `this gear's Pitch Diameter / 4`.

**`generate` is an entry point this module DEFINES, not a call it makes.** The shared
`commands/_gear_command.py` is what constructs the generator on the active design and runs its
`generate` on the dialog's inputs, inside its own try/except. Naming it above says where the
read-and-validate pass sits within that method, not that this module calls it.

<!-- check-step-calls: ignore generate -->

**From:** `spec/bevelgear/instructions.md` L35–52 L54–64 L66–104 L106–115 L231–265 L411–422, `.claude/skills/generate-gear/PLAYBOOK.md` L103–118 L196–218 L854–858

## S03 `[PROSE]` Create the Bevel Gear component

Create the Bevel Gear component as a child of the user's Parent Component with
`parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`, and name it `Bevel Gear` through
`occurrence.component.name` (`[PB-OCCURRENCE-TREE]`). Keep the occurrence on `self.bevelOccurrence`
for cleanup, and the component on `self.bevelComponent`.

**Never call `occurrence.activate()`** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`). The
Anchor Sketch is created on the user's **external**, root-owned target plane, and an activated
occurrence resolves that external plane in its own local frame, collapsing the whole build onto world
XY regardless of the real tilt. The sole exception in this module is the spiral crown's
`scaleFeatures` step (S23), which activates the Design occurrence and restores the root in a
`finally`.

Bevel uses a **standalone generator**: `BevelGearGenerator` does **not** subclass `base.Generator`,
carries **no** `GenerationContext`, and registers no user parameters. From `base.py` it imports only
`get_selection` and `get_boolean`. **`deleteComponent()` calls `deleteMe()` on the top occurrence** —
write it as `self.bevelOccurrence.deleteMe()` — and the entry point calls `deleteComponent` on any
exception.

**Both of those names are exempted from the step-call check, and only one of them is exempt because
it is not a requirement.**

`deleteComponent` is an entry point this module DEFINES rather than a call it makes: the command's
try/except is what invokes it.

**`deleteMe` IS a call this module genuinely makes** — the sentence above is a requirement and stays
one — and it is exempt only because the check cannot see it. That check walks the module from a fixed
set of roots: module-level code, any function named `configure` or `generate`, and the methods of any
class whose base the module does not define. Bevel's two classes have **no bases at all**, because it
is a standalone generator that does not subclass `base.Generator`, so that last root yields nothing
here and the walk starts at `configure` and `generate` alone. `deleteMe`'s only caller is
`deleteComponent`, which no code in the module calls, so every call inside `deleteComponent` is
out of reach by construction and the check reports `deleteMe` as a textual match that is not a
reachable executable call — however the emit stage writes it, and it is already written as
`self.bevelOccurrence.deleteMe()`. The two names cannot be classified apart: exempting the caller
puts its whole body beyond the walk. **The gate cannot enforce this one, so a reader has to.**

<!-- check-step-calls: ignore deleteComponent deleteMe -->

**From:** `spec/bevelgear/instructions.md` L19–23 L267–293 L294–316 L455–458, `.claude/skills/generate-gear/PLAYBOOK.md` L804–828, `spec/bevelgear/fusion.md` L153–160

## S04 `[PROSE]` Create the Design component

Create the Design component as a child of the **Bevel Gear** component, again with
`parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`, and name it `Design`. Hold the
occurrence on `self.designOccurrence` and the component on `self.designComponent`.

Every sketch, construction plane, construction axis and feature operation in this build runs in this
one component. Fusion rejects cross-sibling sketch and `project` references even when the target is
activated or the entities are wrapped in `createForAssemblyContext` proxies (`[PB-NO-CROSS-SIBLING]`),
so the finished bodies are relocated into the per-gear components at the very end with
`body.moveToComponent(...)` (S32).

**From:** `spec/bevelgear/instructions.md` L459–464 L699–713, `.claude/skills/generate-gear/PLAYBOOK.md` L820–825

## S05 `[GO]` The Anchor sketch

Proof function `stepAnchorSketch`.

<!-- proof-run: proofkit.RunParallel(anchorCases, stepAnchorSketch) -->

Start the Anchor sketch **directly on the user-selected target plane** with
`designComponent.sketches.add(targetPlane)`, whether the selection is a `ConstructionPlane` or a
`PlanarFace`; do not re-derive or offset it (`[PB-USE-SELECTED-PLANE]` — re-deriving collapses the
gear onto XY). Name it `Anchor`.

Mark the centre by projecting the user-specified centre point into the sketch: `sketch.project(centerPoint)`.

⚠ **Write the call as `sketch.project(entity)`, and do not substitute `project2`.** The compiled
Fusion API reference declares `project2(entities, isLinked)` and no `project`, so every gate in this
repo reports the call as unverified; that report is expected and is not a defect to fix here.
`project` is what the shipped add-ins call, it sits on `fusion_api.py`'s `UNVERIFIED_CALLS`, which is
reported rather than blocking and is explicitly not waived. The two are not interchangeable in any
case: `project2` takes a list and returns a list, so swapping the name alone would be wrong.

Draw the Anchor Line through the projected centre with
`sketch.sketchCurves.sketchLines.addByTwoPoints(...)`, seeding its two endpoints at **exactly ±0.5 cm
from the projected centre** along the sketch-local X, so the seeded length is 10 mm. Then:

- `sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)` — the intersection, pinning
  the centre onto the line;
- `sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)` — the centre bisects the
  line. **Use both, not the midpoint alone;**
- an aligned distance dimension,
  `sketch.sketchDimensions.addDistanceDimension(start, end, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`,
  **without assigning `.parameter.value`** — the dimension simply locks the length at the seeded
  10 mm. The value is arbitrary; nothing downstream reads it;
- `sketch.geometricConstraints.addHorizontal(anchorLine)` to pin the direction sketch-locally
  (`[PB-REFLINE-DIRECTION]`). A world-axis lock would mis-orient the line on a tilted target plane.

The line's absolute direction is arbitrary — §2 derives every direction *relative* to the projected
anchor line — but it must not be a free degree of freedom. With midpoint, length and Horizontal the
line has zero DOF.

**Stash the projected-centre `SketchPoint`** on `self._anchorCenterPoint` so §2 re-projects *this*
anchor-sketch point rather than the raw user-selected centre.

Gate the sketch at the end of the step: `if not sketch.isFullyConstrained: raise ...` naming the
sketch (`[BEVEL-F-FULL-CONSTRAINT]`, `[PB-FULL-CONSTRAINT]`). A free DOF here is a generation defect,
not a warning.

**What the proof adds, and what it costs.** The bench's distance target is signed where Fusion's is a
magnitude whose direction comes from the seed (`[PB-DIM-VALUE-SEMANTICS]`), so the proof writes the
sign the seed carries; without it the line satisfies every constraint end-for-end as well and the
gate reports two configurations. Fusion's coincident-plus-midpoint pair carries one dependent row on
the bench, so the proof keeps the midpoint, which is the stronger of the two, and says so at the call.

**From:** `spec/bevelgear/instructions.md` L465–469 L479–481 L389–397, `spec/bevelgear/fusion.md` L19–30, `.claude/skills/generate-gear/PLAYBOOK.md` L432–441 L449–463 L626–627 L829–839

## S06 `[GO]` The Gear Profiles plane

Proof function `stepGearProfilesPlane`.

<!-- proof-run: proofkit.RunParallel(gearProfilesPlaneCases, stepGearProfilesPlane) -->

Create a construction plane that includes the Anchor Line, set at 90° so it stands perpendicular to
the anchor line's own plane:

```
planeInput = designComponent.constructionPlanes.createInput()
planeInput.setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
```

Pass the `SketchLine` **directly** to `setByAngle`; never wrap it in `adsk.fusion.Path.create` first
(`[PB-CONSTRUCTION-PLANES]`). **Build it off the ORIGINAL `targetPlane`** as the reference — do not
re-derive or offset it (`[PB-USE-SELECTED-PLANE]`). This is the other place the target-plane
orientation reaches the bodies; substituting a different plane here also collapses the gear onto XY.

Name it `Gear Profiles Plane` and stash it on `self._gearProfilesPlane`.

Why this plane is what makes the sketch-local placement safe (`[BEVEL-F-APEX-LOCAL]`): it is
perpendicular to the target plane and contains the anchor line, so **inside its sketch the direction
perpendicular to the projected anchor line IS the target-plane normal**, and "up toward the Apex" is
simply that in-plane perpendicular. The **sign** of that perpendicular — which side the gear grows —
is chosen by the target-plane normal as a one-bit direction (`[BEVEL-F-GROW-SIDE]`), read as
`targetPlane.geometry.normal` for **both** selection kinds: a `BRepFace`'s `geometry` and a
`ConstructionPlane`'s `geometry` are each a `core.Plane` carrying `.normal`. A sketch-local rule like
`perp.y >= 0` is deterministic but not tied to a physical side, so the gear would grow inconsistently.
That one comparison is the single permitted world use in §2 — a direction, never a position
round-trip.

**What the proof substitutes, and what it costs.** A Fusion construction plane has no bench
counterpart, so the proof builds the same two planes in the sketch engine's World, over target planes
tilted flat, 30°, 90° and past vertical, and reads their frames back: the plane contains the anchor
line, stands square to the target plane, and its in-plane perpendicular is the target normal. What is
not exercised is `setByAngle` itself; what is proved is the geometry that call has to produce.

**From:** `spec/bevelgear/instructions.md` L471–473 L483, `spec/bevelgear/fusion.md` L117–151, `.claude/skills/generate-gear/PLAYBOOK.md` L766–777 L829–839

## S07 `[GO]` The Gear Profiles sketch — the §2 lattice

Proof function `stepGearProfiles`.

<!-- proof-run: proofkit.RunParallel(gearProfilesCases, stepGearProfiles) -->

Create a sketch on the Gear Profiles plane with `designComponent.sketches.add(gearProfilesPlane)`,
name it `Gear Profiles`, and stash it on `self._gpSketch`. This one step draws the whole lattice; it
is one sketch and therefore one timeline entry, however much geometry goes into it.

### The rules that govern every line in this sketch

- **Every line drawn here is a construction line**: `line.isConstruction = True`. That covers the
  lattice lines, the toe lines M→N / O→P, and the short reference and connector lines
  (M→C, N→A′, O→D, P→B′, A′→G, B′→I, C→K/K′, D→L/L′) alike. The solid features later consume only the
  per-gear Profile sketches, never a §2 curve.
- **Coincident style, never sharing** (`[BEVEL-F-COINCIDENT-STYLE]`, a stricter delta to
  `[PB-SHARE-XOR-COINCIDENT]`). When a §2 line must start at or connect to an existing point, create
  the line from raw `adsk.core.Point3D.create(x, y, 0)` coordinates and pin the connecting endpoint
  with exactly one `sketch.geometricConstraints.addCoincident(line.startSketchPoint, existingPoint)`.
  Never pass an existing `SketchPoint` into `addByTwoPoints` to share it. Both directions are
  load-bearing: sharing without a coincident leaves the sketch **under**-constrained and the gate
  fails on "Gear Profiles"; sharing **and** coinciding is redundant and the solve fails outright with
  `RuntimeError … VCS_SKETCH_SOLVING_FAILED - failed to create offset`. ⚠ **This covers the short
  reference and connector lines too, the ones whose BOTH endpoints already exist.** A regen that
  shared only those came out about 14 coincidents short. No §2 line is exempt.
- **One segment, one line** (`[BEVEL-F-LINE-ONCE]`). Each named §2 line is created once and the
  reference is kept; a helper that creates a module-extension must RETURN the line. Drawing a second
  line between the same two points to obtain a reference over-determines the coupled net and the
  solve fails with `RuntimeError … VCS_SKETCH_OVER_CONSTRAINTS`. A duplicate carrying only per-end
  coincidents has been observed to solve and even pass the gate, so do not rely on the solver to catch
  one for you.
- **The driven lengths are NOT dimensioned** (`[BEVEL-F-DRIVEN-DIMS]`, `[PB-NO-OVERCONSTRAIN]`). The
  along-shaft lengths Apex→A and Apex→B and the module-length extensions are driven by the closing and
  collinear constraints. Every "do NOT add a dimensional constraint" below is as load-bearing as the
  dimensions that ARE added.
- **Every length dimension in this sketch is `adsk.fusion.DimensionOrientations.AlignedDimensionOrientation`.**
  `sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, orientation, textPoint)` takes an
  orientation, and this figure has no axis-aligned line in it: the shaft axes sit at Σ to each other,
  the whole lattice tilts with the target plane, and the sketch is not world-aligned.
  `HorizontalDimensionOrientation` or `VerticalDimensionOrientation` would each dimension the line's
  *projection* onto a sketch axis. Wherever a step below says "a dimensional constraint with
  length = X" it means an aligned distance dimension of that value, set through
  `dimension.parameter.value = <number in cm>` (`[PB-NUMERIC-SNAPSHOT]`). The offset dimensions are a
  different call, `sketch.sketchDimensions.addOffsetDimension(lineA, lineB, textPoint)`, which takes
  no orientation.
- **A collinear names the line the new line's start point actually sits ON**, never a farther line up
  the same chain (`[BEVEL-F-COLLINEAR-CHAIN]`, `[PB-COLLINEAR-CHAIN]`). A→E is collinear with
  Apex→A, and E→G with **A→E**; B→F with Apex→B, then F→I with **B→F**; C→H names the Pinion
  Dedendum Apex2→C, and D→J the Driving Dedendum Apex2→D. K and L have both ends already fixed, so
  they take two point-on-line `addCoincident` calls and **no collinear at all**. Measured on this
  lattice, `addCollinear(E→G, Apex→A)` raised
  `RuntimeError: 3 : failed to create offset: VCS_SKETCH_OVER_CONSTRAINTS`.

### The closed form the lattice is seeded from

```
tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)
γ_g     = Σ − γ_p
R       = (PPD / 2) / sin γ_p          # the Pitch Cone Distance, NOT the Cone Distance
|Apex→A| = R · cos γ_p
|Apex→B| = R · cos γ_g
```

Both cosines are positive for every Shaft Angle the range check admits, which is what the Maximum
Shaft Angle is there to guarantee. Seeding A and B merely by a pitch diameter is wrong for Σ ≠ 90° and
sends the solver to the wrong branch. `γ_p` and `γ_g` are stashed on `self._gamma_p` /
`self._gamma_g` and reused in §3 and §3a; `self._coneDistance_cm` holds the Cone Distance.

**"Cone Distance" and "Pitch Cone Distance" are two different lengths and both are used.** The Cone
Distance is `sqrt(DPD² + PPD²)`, the diagonal of the two pitch diameters, and depends on the tooth
counts only. `R` is the real apex-to-heel length along the pitch cone. They coincide as
`Cone Distance = 2R` **exactly when Σ = 90°**, for any tooth counts, and diverge everywhere else: an
equal 31/31 pair at Σ = 30° has `Cone Distance = 43.84 mm` against `R = 59.89 mm`, and at 140°
`R = 16.49 mm`.

### The build, in order

**Project the anchor geometry.** In this sketch, project **the Anchor sketch's centre `SketchPoint`**
(the one stashed in S05) with `sketch.project(self._anchorCenterPoint)` — NOT the raw user-selected
centre point. Both happen to be coincident, but projecting the anchor-sketch point keeps the chain
inside the Design component; projecting the raw external point is a cross-component reference and can
resolve inconsistently. Project the anchor line the same way. Let `c` be the projected centre and `d`
the projected anchor line's 2-D unit direction; `perp = (-d.y, d.x)`, with its **sign** taken from the
target-plane normal per S06.

**The centre→apex line.** From the projected centre draw a construction line perpendicular to the
projected anchor line **in this sketch's own 2-D frame**, with
`sketch.geometricConstraints.addPerpendicular(centerToApex, projectedAnchorLine)`. Its far end is the
**Apex**, placed in sketch-local coordinates at

    Apex = c + perp · (R · cos γ_g + <resolved Driving Gear Base Height>)

**This seed formula is not a heuristic and must be carried in exactly this form.** The constraint net
closes this line at `R · cos γ_g` above point I plus the resolved driving base height. Seeding it at
`Driving Gear Pitch Diameter`, which an earlier revision said, puts the seed 11.6 mm past where the
solve lands it for the default 31/31 pair at Σ = 90° (31 mm seeded against 19.375 mm solved). Fusion
converges from the far seed so that was latent rather than broken, but a seed that disagrees with its
own closure by that margin is a seed waiting to pick the wrong branch (`[PB-SEED-NEAR]`). The apex
**position** is sketch-local; do **not** compute it from a world-coordinate round-trip
(`[BEVEL-F-APEX-LOCAL]`). Pass raw `Point3D` coordinates for BOTH endpoints to `addByTwoPoints` and
pin the start with exactly one `addCoincident(centerToApex.startSketchPoint, projectedCenter)`. **Do
NOT add a length constraint on this line.**

**The Driving Gear Shaft Axis, Apex→B.** A construction line from the apex pointing back toward the
anchor line, i.e. in the `-perp` direction. **Seed its far end at `apex - perp · (R · cos γ_g)`, which
is `c + perp · (<resolved Driving Gear Base Height>)`** — measure from the apex, not from `c`. Earlier
revisions said `c - perp · (some length)`, which puts B on the far side of the projected centre from
the apex, the wrong side of the figure entirely; the closure at Apex 2 drives `|Apex→B|` to
`R · cos γ_g`, so B solves to exactly one base height above `c`. Apply
`sketch.geometricConstraints.addParallel(drivingShaftAxis, centerToApex)`. **Do NOT use
`addVertical`** — it forces the line to the sketch's world-vertical, which is wrong on a tilted target
plane and over-constrains the figure. Coincident the start with the apex. Do **not** dimension the
length.

**The Pinion Gear Shaft Axis, Apex→A.** The driving-shaft direction rotated about the apex by Σ.
Rotating has two senses and they place A on opposite sides; choosing wrong mirrors the whole gear onto
the wrong side of the target plane. **Select the sense this way: form both candidate A positions — the
driving-shaft direction rotated about the apex by +Σ and by −Σ — and keep the candidate whose endpoint
has the greater X coordinate in this sketch.** Compare the two candidates' X and take the larger; do
**not** rotate one fixed sense and flip it only when its X comes out negative, because when *both*
candidates have a positive X that shortcut keeps the wrong one. Call the chosen unit Apex→A direction
`pinionDir`, and the unit Apex→B direction `drivingDir`.

Apply an angular dimension between this line and the Driving Gear Shaft Axis equal to Σ:
`sketch.sketchDimensions.addAngularDimension(drivingShaftAxis, pinionShaftAxis, textPoint)` and then
`dimension.parameter.value = <Σ in radians>`. **Place its text point inside the Σ wedge so it measures
Σ and not its supplement 180 − Σ** (`[PB-ANGULAR-DIM]`) — on the interior bisector of the two shaft
directions, `apex + normalize(pinionDir + drivingDir) · (PPD / 4)`. The angular dimension fixes the
angle *magnitude* only; the pinion's side is held by the seed above together with the Apex 2 closure
below. Coincident the start with the apex; do **not** dimension the length.

**The two perpendicular drops to Apex 2.** From A, a construction line perpendicular to the Pinion
Gear Shaft Axis, drawn toward the side where Apex 2 will lie. ⚠ **Apex 2 sits in the interior wedge
*between* the two shaft axes, so this drop must point toward the OTHER (Driving) shaft axis / point B,
NOT "toward the anchor line".** Pick the perpendicular sense by the sign of its dot product with the
A→B direction. Apply `addPerpendicular` against the Pinion Gear Shaft Axis, and an aligned distance
dimension of `PPD / 2` — the pinion's pitch radius at the heel, which is the perpendicular distance
from Apex 2 to the Pinion Gear Shaft Axis for any Σ. Coincident the start with A.

From B, the twin. ⚠ **This drop must point toward the OTHER (Pinion) shaft axis / point A** — pick the
sense by the sign of its dot with the B→A direction. **Do NOT choose this sense by a "toward the anchor
line" reference (the `-perp` / centre→apex grow direction): the Driving Gear Shaft Axis is itself
parallel to that direction, so the perpendicular's dot with it is ≈ 0 — a degenerate test that
silently selects an arbitrary, usually wrong, side.** This is the critical failure: if the B drop seeds
Apex 2 on the wrong side of the driving shaft while the A drop seeds it on the correct side, the
coincidence that closes the two at Apex 2 makes the solver **flip the entire frame to the mirror
solution** — A jumps to the opposite side, the pinion dedendum C collapses onto the driving dedendum
D, the toe ends up *outside* the heel, the revolved frustum is degenerate and the conical end-cut
finds no cone face at the toe midpoint (`face dist = inf`). Both drops must aim at the *same*
interior-wedge point. Apply `addPerpendicular` against the Driving Gear Shaft Axis and an aligned
distance dimension of `DPD / 2`. Coincident the start with B.

Close them: `sketch.geometricConstraints.addCoincident(aDrop.endSketchPoint, bDrop.endSketchPoint)`.
That point is **Apex 2**. At Σ = 90° the four points Apex, A, Apex 2, B form a rectangle; for other
shaft angles the figure is a non-rectangular quadrilateral and the lengths of Apex→A and Apex→B adjust
so the two drops coincide.

Note that this quadrilateral deliberately lies well above the anchor line: the apex's offset of
`R · cos γ_g` plus the resolved driving base height keeps the whole figure above that line across the
supported Shaft Angle range.

**The Pitch Line.** A construction line from Apex to Apex 2, each end coincident to its point.

**The two dedendum lines.** From Apex 2, a construction line to either side, each with an aligned
distance dimension of `1.25 * m` and `addPerpendicular` against the Pitch Line. The one drawn
**towards** the anchor line is the **Driving Gear Dedendum**, ending at **point D**; the one drawn
**away** is the **Pinion Gear Dedendum**, ending at **point C**.

**The two root axes.** A construction line from Apex to D (the Driving Root Axis) and one from Apex to
C (the Pinion Root Axis), both ends coincident.

**The module-length extensions.** From A, a construction line collinear with Apex→A extending for
length equal to `m` (seed only — **no** dimensional constraint); `addCollinear` against Apex→A, and
coincident the end of Apex→A with the start of the new line. Its end is **point E**. Then a line from
C to E, each end coincident, with `addPerpendicular` between **A→E** and **C→E** — that perpendicular
is what actually fixes E, at the foot of the perpendicular from C onto the pinion axis. From B, the
twin: extension to **point F**, then D→F, with `addPerpendicular` between B→F and D→F.

**The base-height chains.** From E, a construction line collinear with **line A→E** — the collinear
names A→E, **never the Apex→A shaft axis further up the chain**, even though both describe the same
infinite line. Length equal to `m` (seed only, no dimension). Its end is **point G**. From C, a line
of length `m` (seed only, no dimension) whose collinear names **line Apex2→C**; its end is **point H**.
Connect G and H with a line, both ends coincident.

**Constrain line E→G and H→G with a perpendicular constraint.** ⚠ **This perpendicular is required in
Fusion and must be omitted in the proof harness, and the reason is a difference between the two
engines rather than a choice.** `addOffsetDimension` in Fusion is a distance dimension whose
documentation requires the second entity to be a line parallel to the first, and it controls only the
perpendicular distance — so the parallelism has to exist before the offset can be applied at all, and
this perpendicular is what supplies it: E→G runs along the pinion shaft, so making H→G perpendicular
to it makes H→G parallel to the A→Apex2 drop. Perpendicular plus offset is two equations for two
freedoms and nothing is redundant. The proof harness's offset emits **two** residual rows, holding
both endpoints of the target line at the same signed perpendicular distance, so it carries the
parallelism itself; adding this perpendicular there is a third row over the same two freedoms and the
lattice comes back overconstrained at DOF 0 with 2 redundant constraints, the engine naming the two
base-height offsets. Leave it out of the proof and say so; never weaken the gate
(`[PB-NO-OVERCONSTRAIN]`).

From F, the driving twin: a line collinear with **line B→F**, length `m` (no dimension), ending at
**point I**. From D, a line of length `m` (no dimension) collinear with **line Apex2→D**, ending at
**point J**. Connect I and J, both ends coincident, and **constrain line F→I and J→I with a
perpendicular constraint** — required in Fusion, omitted in the proof, for the reason just given.

**The two base-height offsets.** An offset dimension between the **B→Apex2 perpendicular drop line**
(the DPD/2 drop — **not** the Apex→B shaft axis) and **J→I**:
`sketch.sketchDimensions.addOffsetDimension(bDrop, lineJI, textPoint)`, then
`dimension.parameter.value = <resolved Driving Gear Base Height, cm>`. J→I is **already parallel** to
the drop by construction (J→I ⊥ F→I, which runs along the driving shaft), so add **no** extra
`addParallel` (`[PB-OFFSET-DIM]`). The value is the driving base height **after** its Maximum and
Minimum Base Height have been applied, because the offset set here is what drives the heel edge D→J
toward the shaft axis.

Then the pinion twin: an offset dimension between the **A→Apex2 drop** and **G→H**, already parallel
by construction, value `<resolved Pinion Gear Base Height, cm>`.

**Naming convention used throughout this spec.** "A->Apex2" always means the PPD/2 perpendicular drop
line from A, never the Apex→A shaft axis; the two share point A but are different lines. The same
holds for "B->Apex2" against the Apex→B shaft axis.

**Close the figure: constrain point I with the projected centre point** —
`addCoincident(pointI, projectedCenter)`. This is what fixes the apex's height, and it is why the apex
seed above must be the closure's own value.

**The shaft-axis edge's first vertex.** Draw a line from **A′** to **G**, the hexagon's shaft-axis
edge, endpoints constrained appropriately. It starts at the front face's foot A′, not at A; the two
coincide at Toe Extension 0. (A′ is built below with the toe line.)

**The tooth-centre points K and L.** Draw a construction line away from the Apex starting from point
G, extending along Apex→A; call its end **K**. Then **pin K with two point-on-line coincident
constraints** — `addCoincident(K, line Apex→A)` and `addCoincident(K, the Pinion Dedendum line Apex2→C
extended)` — rather than `addCollinear` on the connecting lines. By the time K is added G and C are
already fixed, so an `addCollinear` here over-constrains the sketch and Fusion errors; the two
point-on-line coincidents locate K exactly, at the intersection of the two lines, without
over-constraining. Draw a construction line from C to K for reference. Build **L** identically from
point I along Apex→B, pinned to Apex→B and to the Driving Dedendum line Apex2→D extended, with a
reference line from D to L.

**Tooth-centre point K′ (the Tooth Spacing offset).** The §3 spur tooth is centred not at K but at
**K′**, obtained by shifting K outward along the dedendum line by **Tooth Spacing**, *away from the
lower corner C*. **When Tooth Spacing is 0 (the default) do NOT build anything here — set K′ ≡ K and
reuse the C→K reference line**; a zero-length dimensioned line would be degenerate, and one segment
gets one line (`[BEVEL-F-LINE-ONCE]`). When Tooth Spacing > 0: draw a construction line starting at K
with its far end seeded on the *far side of K from C* along the dedendum direction, pin that far end
the same way K is pinned to its line — `addCoincident(start, K)` and `addCoincident(K′, the Pinion
Dedendum line Apex2→C extended)` — then add an aligned **length dimension on this line = Tooth
Spacing**. Do **not** use `addCollinear`, for the same over-constraint reason as K. Finally draw the
tooth-centre reference line **C→K′** for §3 to use in place of C→K. Build **L′** exactly the same way,
substituting L for K, D for C and the Driving Dedendum line Apex2→D, with reference line **D→L′**.
Build both here, inside this sketch, before its end-of-step full-constraint gate, so the gate covers
them. Only the tooth's centre moves; the virtual tooth number and the drawn tooth size are unchanged.

**Resolve the Maximum Face Width.** At this point A, B, C, D, H, J all exist **and are solved**, so
resolve it now and apply it before Face Width is used below. It is `0.95 *` the smaller of the
perpendicular distance from **A** to the line through **C** and **H** (the Pinion Gear Dedendum line,
Apex2→C extended) and the perpendicular distance from **B** to the line through **D** and **J** (the
Driving Gear Dedendum line). **Compute both distances from the points' SOLVED sketch geometry —
`pointA.geometry`, `pointB.geometry`, `pointC.geometry`, `pointD.geometry`, `pointH.geometry`,
`pointJ.geometry` — NOT from the pre-solve seed coordinates** (`[PB-SOLVED-GEOMETRY]`). The constraint
network has located all six by now, so `.geometry` is exact; seeds diverge substantially for
asymmetric tooth counts or non-90° shaft angles, making a seed-based bound too loose on the binding
side, and the toe still crosses the axis. Cap the auto default to it and **reject** a user Face Width
that exceeds it, naming the maximum. Take the **minimum** of the two sides: the pinion is normally the
binding one, but written with the pinion's diameter the bound is wrong whenever the driving gear
carries the smaller tooth count — on a Driving 17 / Pinion 31 pair at Module 1 the real bound is
3.883 mm and the pinion form gives 13.591, so the naive `Cone Distance / 6` default exceeds it and the
gear fails to generate for any gear ratio above roughly √2. At Σ = 90° this limit equals
`0.95 * min(DPD, PPD)² / (2 * Cone Distance)`, using the **smaller** pitch diameter and never the
pinion's by name.

**Resolve Face Width.** `Face Width = user value if specified, else min(Cone Distance / 6, Maximum
Face Width)`. In **every** case it is bounded by the Maximum Face Width. Stash the result on
`self._faceWidthResolved_cm`. The default `Cone Distance / 6` is `R / 3`, the conventional face-width
limit, **only at Σ = 90°**; below 90° it is conservative, above 90° it exceeds `R / 3` and the cap is
what holds it. That is deliberate — it keeps the default independent of Σ — and it is the cap, not the
default, that guarantees a buildable profile.

**Resolve the Toe Radii and the Root Length.** Per gear, with `r` that gear's pitch radius:

```
Toe Radius (auto, when the input is 0) = r − Face Width / sin γ
Toe Radius Ceiling                     = (r − 1.25·m·cos γ) · (1 − Face Width / R)
|Apex→Ded|                             = sqrt(R² + (1.25·m)²)
γ_root                                 = γ − atan(1.25·m / R)
Toe Limit                              = |Apex→Ded| − Toe Radius / sin γ_root
Root Length at Toe Extension 0         = Face Width · |Apex→Ded| / R
Root Length                            = RootLength0 + (Toe Extension / 100) · 0.99 · (min(Toe Limit_p, Toe Limit_g) − RootLength0)
```

A user Toe Radius must be **strictly below** that gear's Toe Radius Ceiling; reject it naming the
ceiling. The Toe Radius is the perpendicular distance from the shaft axis at which the **inner toe
corner** N (resp. P) rides, and with it the radius of the flat front face the revolve produces; the
auto value is what makes Toe Extension 0 today's profile exactly. **Toe Extension 100 stops at 0.99 of
the way from the Toe Extension 0 root length to the smaller of the two gears' Toe Limits, not at the
Toe Limit itself.** The smaller limit wins because the pair shares one root length. The 0.99 is there
because AT the limit the toe face has zero length, so the revolved body carries **no cone at its toe
end** and the conical end-cut in S16 — whose toe cut must split or the build fails — has no
`ConeSurfaceType` face to find. The last percent is worth well under a tenth of a millimetre of root
length on every case in the proof's table. **Do not drop this factor when regenerating.**

**A defaulted Toe Radius can leave no room at all, and that is a real configuration rather than a
defect.** On a driving gear with a large pitch cone angle the inner toe corner already sits at a
LARGER radius than the outer one, so X falls behind the toe corner and the Toe Limit comes out below
the Toe Extension 0 root length. Measured over gear ratio against Shaft Angle it is a diagonal band
that crosses 90° for every ratio from about 2.75 up, and Module does not move its boundary. **Reject a
Toe Extension above 0 on such a pair**, naming the gear and the Toe Radius Ceiling it needs to come
below. Toe Extension 0 still resolves, so the gear stays buildable exactly as before. Do **not**
silently substitute a smaller Toe Radius: that would change the toe end of a gear whose inputs asked
for no change.

**The toe line M→N.** **Seed BOTH ends at their closed-form solved positions, not near them**
(`[PB-SEED-NEAR]`). Seed M on `Apex→C` at the fraction `1 − <Root Length> / |Apex→C|` from the Apex.
Then seed N by sliding from that M seed along the `C→H` direction by exactly

    (<M seed's perpendicular distance from the Pinion Gear Shaft Axis> − <Pinion Gear Toe Radius>) / cos γ_p

⚠ **A seed that merely lands somewhere plausible is not enough here, and a wrong one builds the wrong
gear rather than failing to converge.** N's position is fixed by the toe line together with a LENGTH
dimension on the front face, and a length is unsigned: the toe line meets the Toe Radius on BOTH sides
of the shaft axis, so the solver takes whichever side the seed starts on. Seeded below the axis it
converges happily onto the mirror, N comes out on the far side, and the revolved hexagon crosses its
own axis of revolution — Fusion then aborts the revolve with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`) at
S14, pointing at the revolve rather than at the seed that caused it.

Two earlier seeding rules are known to do exactly that, so **do not reinstate either**: sliding from
the M seed by the **Root Length**, and sliding by the **distance from the M seed to A**. Both were
written for the scheme that pinned N to the `A→Apex2` drop. Measured on the shipped default pair,
Module 1 with 31/31 teeth at Σ = 90° and a Toe Extension of 50%, the Root Length slide puts the N seed
at a perpendicular distance of **−0.27 mm** from the shaft axis — past it — against a solved N at
**+5.17 mm**, and Fusion refuses the revolve. The slide above puts it at 5.17 mm exactly. Do NOT seed
M/N just `Face Width` away from C/H either; that starts N near H, far from its constraint target.

Then apply **exactly these three constraints** — all are required, and the front face below is what
holds N off the shaft axis, which is what the pre-Toe-Radius scheme used the A→Apex2 pin for:

- `sketch.geometricConstraints.addCoincident(M, pinionRootAxis)` — M lies on the Apex→C root axis;
- `sketch.geometricConstraints.addParallel(lineMN, lineCH)` — the toe line is parallel to C→H;
- `sketch.sketchDimensions.addOffsetDimension(lineCH, lineMN, textPoint)` with
  `dimension.parameter.value = <Root Length · R / |Apex→C|>` — the Root Length re-measured
  perpendicular to the pitch line, because an offset dimension controls a perpendicular distance. At
  Toe Extension 0 this value is exactly the resolved Face Width, which is what this dimension has
  always been. Place the `textPoint` in the gap between C→H and M→N on the Apex side, e.g. the
  midpoint of the M seed and point C, `(M_seed + C) / 2` (`[PB-OFFSET-DIM]`). The toe's side relative
  to the heel (`toe→Apex < heel→Apex`) follows from the §2 frame being built correctly, in particular
  from the Apex 2 drops aiming at the interior wedge; it is **not** controlled by this text point.

Let the beginning of the line be **M** and the end be **N**. Draw a line from M to C.

**The front face A′→N, which is what holds N.** ⚠ **N is NOT pinned to line A→Apex2.** It rides the
**Pinion Gear Toe Radius** instead, and the line that holds it there is the gear's front face:

- draw a line from N to a new point **A′**, seeding A′ at N's station on the shaft axis;
- `addCoincident(A′, line Apex→A)` — A′ lies on the **Apex→A shaft axis**. A′ is the only toe-end
  point that touches that axis, and it is a *foot*, not a corner;
- `addPerpendicular(lineNAprime, line Apex→A)` — the front face stands square to the shaft, so the
  revolve sweeps it into a flat annulus;
- an aligned distance dimension on the whole line N→A′ with
  `dimension.parameter.value = <resolved Pinion Gear Toe Radius>`.

⚠ **Pinning N itself to the Apex→A shaft axis remains forbidden** — that would put N *on the axis of
revolution*, and the later conical split fails with `ASM_API_FAILED` for asymmetric tooth counts even
though the symmetric 45° case happens to survive. A′ sits on the axis; N never does, because the Toe
Radius is strictly positive. Those three rows plus the offset above and the coincidence of M on the
root axis fully constrain M, N and A′ — six freedoms, six constraints.

**A′ replaces A as the hexagon's first vertex.** At Toe Extension 0 with a defaulted Toe Radius the
two coincide exactly, so nothing moves; a positive Toe Extension walks A′ along the shaft axis toward
the Apex and the shaft edge grows by that much.

**The driving side.** Build **O→P** as the mirror of M→N: seed O on `Apex→D` at the fraction
`1 − <Root Length> / |Apex→D|`, then P slid from that O seed along `D→J` by
`(<O seed's perpendicular distance from the Driving Gear Shaft Axis> − <Driving Gear Toe Radius>) / cos γ_g`.
The ⚠ above applies unchanged. Apply `addCoincident(O, drivingRootAxis)`,
`addParallel(lineOP, lineDJ)`, and `addOffsetDimension(lineDJ, lineOP, textPoint)` with the same
perpendicular-form value, its text point in the gap on the Apex side of D→J, e.g. `(O_seed + D) / 2`.
Draw a line from O to D. Build the driving front face **B′→P** exactly as A′→N, substituting B for A,
P for N and the **Driving Gear Toe Radius**: the line P→B′, `addCoincident(B′, line Apex→B)`,
`addPerpendicular(linePBprime, line Apex→B)` and a length dimension on P→B′. Draw a line from B′ to I.

**Gate the sketch** at the end of the step with `sketch.isFullyConstrained`, raising and naming it
(`[BEVEL-F-FULL-CONSTRAINT]`).

**What the proof substitutes, and what it costs.** Three of Fusion's calls carry a different number of
independent rows on the bench, and each substitution is named at its own call site: the G→H and J→I
perpendiculars are left out (this step says why in full); the toe lines' `addParallel` is left out,
because the bench's offset already holds both endpoints and so carries the parallelism; and a
point-to-point coincidence whose second row the net already implies — the I-on-centre closure — is
written as the single independent row. Several of Fusion's unsigned `addPerpendicular` and
`addParallel` constraints take their side from the seed, which the bench's probe reports as a second
discrete configuration, so the proof writes the signed angle that carries the seed's own bit. The cost
is real and stated in the proof: a module that seeds one of those the wrong way round still builds the
mirrored figure, and this stage cannot see that. **The proof also cannot catch a wrong toe-line seed**
— it seeds M and N at the closed form, which IS the rule, so what it proves is that the constraints
solve from a correct seed and never that the module's seed is correct.

**A refusal the case table records rather than avoids.** Measured on this lattice, Σ = 30° with the
default 31/31 pair reads a conditioning of `2.83e-05` against the sketch engine's `4e-05` floor and is
refused; it first clears at 35° (`4.19e-05`), and passes 142° (`9.73e-05`) and 150° (`4.05e-05`). The
case stays in the table as a declared refusal rather than narrowing the Shaft Angle range this spec
states.

**From:** `spec/bevelgear/instructions.md` L106–137 L389–410 L471–587, `spec/bevelgear/fusion.md` L69–115 L117–151, `.claude/skills/generate-gear/PLAYBOOK.md` L432–484 L582–613 L626–642 L708–714

## S08 `[PROSE]` Create the `{gearLabel} Gear` component

Run this and every step from here to S32 **once per gear — pinion first, then driving** — with these
substitutions:

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A′ → G → H → C → M → N → A′ | B′ → I → J → D → O → P → B′ |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A′→G | B′→I |
| toe cut edge | M→N | O→P |
| heel cut edge | C→H | D→J |
| tooth centre / reference line | K′, C→K′ | L′, D→L′ |
| pitch cone angle | `γ_p` = `self._gamma_p` | `γ_g` = `self._gamma_g` |
| teeth / bore / pitch-diameter inputs | Pinion Gear … | Driving Gear … |
| §2 shaft construction line (**NOT** usable as the axis) | Apex→A | Apex→B |
| `gearLabel` | `Pinion` | `Driving` |

Create a new component as a child of the **Bevel Gear** component — the same component that owns
Design, *not* the user's Parent Component; this intentionally overrides the looser "child of Parent
Component" phrasing so the pair nests cleanly inside Bevel Gear — named `{gearLabel} Gear`, i.e.
`Pinion Gear` and `Driving Gear`. Use `bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`
and set `occurrence.component.name`. The finished bodies for this gear end up here, moved in at S32.

**The per-gear geometric anchors travel in a plain per-gear dict** (`pinionCtx` / `drivingCtx`) built
in `_buildGearProfiles` and passed to `_buildVirtualSpurProfile` and `_createGearBody`, which write
back into it. There is **no `GenerationContext` class and no `base.Generator` context machinery** —
per-gear plain-dict carriers plus `self` attributes ARE the intended structure. The dict carries this
gear's **label**, **teeth**, **pitch diameter**, **γ**, **tooth-centre point** (K′/L′) and
**tooth-centre reference line** (C→K′ / D→L′), the **hexagon vertices** in draw order, the
**shaft-edge point pair** (A′, G / B′, I), the **toe edge** (M→N / O→P) and **heel edge** (C→H / D→J),
the **toe and heel cone points** (M / O and C / D), the **root axis** (Apex→C / Apex→D), the **bore
diameter**, and the **mesh angle**; `_buildVirtualSpurProfile` writes back the **tooth sketch**, the
**tooth plane**, the **`embedded` flag** and the **virtual tooth count**. Shared anchors are
self-stashed: `self._gearProfilesPlane`, `self._apexSketchPoint`, `self._gpSketch`, `self._apex2d`,
and the §1 `self._anchorCenterPoint`.

**From:** `spec/bevelgear/instructions.md` L294–316 L699–713

## S09 `[GO]` The `{gearLabel} Plane`

Proof function `stepToothPlane`.

<!-- proof-run: proofkit.RunParallel(toothPlaneCases, stepToothPlane) -->

Create a construction plane that includes the tooth-centre reference line **C→K′** (pinion) / **D→L′**
(driving), made perpendicular to the Gear Profiles sketch plane, and name it `{gearLabel} Plane`. Use
the framework helper `plane_by_angle(designComponent, toothCentreLine, gearProfilesPlane, 90)` from
`.solids` rather than re-implementing it; it wraps
`constructionPlanes.createInput()` + `planeInput.setByAngle(line, ValueInput, refPlane)` +
`constructionPlanes.add(planeInput)`.

**Pass the relevant sketch line DIRECTLY to `setByAngle`; never wrap it in `adsk.fusion.Path.create`
first** (`[PB-CONSTRUCTION-PLANES]`) — `Path.create` on a sketch curve raises
`RuntimeError … InternalValidationError` whenever the curve's owner sketch is not trivially resolvable
in a multi-component context, which is exactly this build.

The reference line runs along the back-cone direction, which is what puts the tooth on the back cone
at all — the Tredgold construction. Stash the plane in this gear's dict.

**What the proof substitutes, and what it costs.** As with S06, the frame is built in the sketch
engine's World and read back: the plane contains the tooth-centre reference line, stands square to the
Gear Profiles plane, and passes through the tooth centre. Fusion's own `setByAngle` is not exercised.

**From:** `spec/bevelgear/instructions.md` L588–591 L599, `.claude/skills/generate-gear/PLAYBOOK.md` L181 L766–777

## S10 `[GO]` The `{gearLabel} Tooth` sketch

Proof function `stepToothProfile`.

<!-- proof-run: proofkit.RunParallel(toothProfileCases, stepToothProfile) -->

Create a sketch on the `{gearLabel} Plane`, named `{gearLabel} Tooth`, and draw a spur gear tooth
profile on it, centred on the tooth-centre point K′ / L′, with this gear's Module and **virtual tooth
number**.

**The virtual (back-cone / Tredgold) tooth number comes from the closed form, NOT from measuring
Apex2→K′/L′:**

```
virtualPitchRadius = (this gear's Pitch Diameter / 2) / cos γ
virtualTeeth       = floor(2 · virtualPitchRadius / Module)        # an int
```

**Units — pin the cm→mm conversion.** The stashed pitch diameters are internal **cm** while Module is
the raw **mm** value, so compute
`virtualPitchRadius_mm = (pitchDia_cm * 10 / 2) / math.cos(gamma)` — the `* 10` converts cm to mm —
and then `virtualTeeth = int(math.floor(2 * virtualPitchRadius_mm / module))`. Skipping the ×10 makes
the virtual tooth count about ten times off.

**Draw it with the borrowed spur tooth generator.** Bevel imports
`from .spurgear import SpurGearInvoluteToothDesignGenerator` and uses it only here, once per gear:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))   # the 180° tooth rotation IS the draw() angle
```

The borrowed generator's surface: constructor `(sketch, parent, angle=0)`; `draw(anchorPoint,
angle=0)` runs `drawCircles()` → `drawTooth(angle)` → the anchor projection; it reads parameters via
`parent.getParameter(name).value`. That `parent` is the framework's **`VirtualSpurProxy`**, imported
from `.spurproxy` — **do NOT define a local copy**. It precomputes, in internal cm, exactly the keys
the spur drawer reads, and its defaults match bevel: pressure angle **20°**, which is not a bevel
dialog input, and `InvoluteSteps` **15**. Construct it as
`VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)` with the **raw-mm** module.

The **180° rotation is delivered through the `draw()` angle argument** — the spur generator rotates
the whole tooth by `angle` — and *not* by a post-hoc Move or sketch rotation. This relies on spur's
radial flank-to-root pinning so the connecting lines rotate with the tooth. The tooth plane's own +X
runs from the dedendum corner toward the tooth centre, so a tooth drawn at 180° faces back at the
corner.

**`getParameter` is a call the BORROWED generator makes on an object this module merely supplies.**
`SpurGearInvoluteToothDesignGenerator` reads its parameters through `parent.getParameter(name).value`,
and the `parent` is the framework's `VirtualSpurProxy`. Naming that call above pins the interface the
proxy has to satisfy; bevel itself never makes it.

<!-- check-step-calls: ignore getParameter -->

**The proxy carries `_lastToothEmbedded`, an OUTPUT the spur generator writes, and bevel MUST read it
back.** During `draw()` the spur generator decides whether the tooth is *embedded* — tip, root and
flanks meeting with no connecting lines — and records it with `self.parent._lastToothEmbedded = <bool>`;
the framework proxy pre-initialises the slot to absorb that write. **After `drawer.draw(...)` returns,
read `proxy._lastToothEmbedded` and thread it into this gear's dict alongside the tooth sketch and
plane.** This flag is not optional bookkeeping: it is the deterministic selector for the tooth loop's
line count at S15, and skipping it grabs an unrelated loop and the apex→tooth loft dies with
`LOFT_NO_TOOLBODY`.

**After `draw()` returns do NOT hard-gate this sketch: `futil.log` if `not toothSketch.isFullyConstrained`,
never raise.** The two tooth-profile sketches are exempt from the full-constraint gate, and **only
because they are labelled**: `drawCircles` labels each of the four circles with along-path sketch text
and sketch text holds a DOF (`[PB-TEXT-HOLDS-DOF]`), so a tooth sketch whose geometry is completely
determined still reads `False` purely because it is labelled. ⚠ **This exemption covers the labels and
nothing else; never read it as licence for loose geometry.** Bevel's own four sketches carry no text,
which is why they gate normally. The reading is also not stable between runs
(`[PB-SETTLE-DISPLAY]`), which is the other reason to log rather than raise.

**What the proof substitutes, and what it costs.** The involute flanks, ribs, spine and tooth-top arc
are spur's geometry and are proved in `proof/spurgear/sketches_test.go`. What BEVEL supplies is the
virtual tooth number, the module, the tooth centre and the 180° draw angle, so the proof builds the
four Tredgold circles about the tooth centre plus the spine at 180° and checks those, together with
the Tredgold invariant that the dedendum corner sits one dedendum inside the virtual pitch radius and
the drawn root circle a little further in again because the virtual tooth number is floored. A wrong
involute would not be seen here; a wrong virtual tooth count, a wrong centre or a tooth facing the
wrong way would.

**From:** `spec/bevelgear/instructions.md` L374–384 L423–452 L588–601, `spec/bevelgear/fusion.md` L31–58, `.claude/skills/generate-gear/PLAYBOOK.md` L151–156 L188–194 L508–523 L663–683

## S11 `[GO]` The tooth-axis helper plane

Proof function `stepToothAxisHelperPlane`.

<!-- proof-run: proofkit.RunParallel(toothAxisHelperPlaneCases, stepToothAxisHelperPlane) -->

Create the helper plane the tooth axis is one half of:

```
helperInput = designComponent.constructionPlanes.createInput()
helperInput.setByDistanceOnPath(<tooth-centre reference line>, adsk.core.ValueInput.createByReal(1.0))
helperPlane = designComponent.constructionPlanes.add(helperInput)
```

`setByDistanceOnPath` takes a fractional distance from 0 to 1 along the path, so `1.0` puts the plane
perpendicular to that line **at its far end, the tooth-centre point**. Pass the `SketchLine` directly;
never wrap it in `Path.create` (`[PB-CONSTRUCTION-PLANES]`).

**What the proof substitutes, and what it costs.** The frame is built in the sketch engine's World and
read back — normal to the tooth-centre reference line and through the tooth centre. Fusion's
`setByDistanceOnPath` is not itself exercised.

**From:** `spec/bevelgear/instructions.md` L603, `.claude/skills/generate-gear/PLAYBOOK.md` L766–777

## S12 `[GO]` The `{gearLabel} Tooth Axis`

Proof function `stepToothAxis`.

<!-- proof-run: proofkit.RunParallel(toothAxisCases, stepToothAxis) -->

Create a construction axis through the tooth-centre point, normal to the plane the tooth profile was
drawn on, named `{gearLabel} Tooth Axis`:

```
axisInput = designComponent.constructionAxes.createInput()
axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
toothAxis = designComponent.constructionAxes.add(axisInput)
```

The two planes are the **Gear Profiles plane** and the **helper plane** from S11; their intersection
is the line through the tooth centre normal to the tooth plane (`[PB-CONSTRUCTION-AXES]`).
`setByPerpendicularAtPoint` would need a `BRepFace` this build does not have.

Creating this axis in the never-activated Design component is proven to work —
`constructionAxes.add` via `setByTwoPlanes` does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here. Keep
the axis.

**What the proof substitutes, and what it costs.** The intersection is computed from the two planes'
frames and checked: it lies in the Gear Profiles plane, stands normal to the tooth plane, and is
perpendicular to the tooth-centre reference line. Fusion's `setByTwoPlanes` is not itself exercised.

**From:** `spec/bevelgear/instructions.md` L603, `.claude/skills/generate-gear/PLAYBOOK.md` L778–781 L782–790

## S13 `[GO]` The `{gearLabel} Profile` sketch

Proof function `stepGearProfileSketch`.

<!-- proof-run: proofkit.RunParallel(gearProfileSketchCases, stepGearProfileSketch) -->

Open a **fresh sketch on the axial (Gear Profiles) plane**, named per the S08 table —
**one profile sketch per gear**, so `sketch.profiles` holds exactly this one hexagon loop. Do not draw
both gears' hexagons in the shared Gear Profiles sketch; that would leave two identically-shaped loops
to disambiguate.

Build the hexagon on fixed vertices per the `[PB-PROJECT-NOT-FIXED]` **recreate-share-fix** recipe, in
that order:

1. recreate the six §2 vertices as new points at their exact world-mapped positions —
   `sketch.sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))` for each — which is valid
   because §2 is fully constrained by now, so every `worldGeometry` is defined
   (`[PB-WORLDGEO-CONSTRAINED]`). `modelToSketchSpace` is a point-transforming **method**, not a
   matrix: call it directly and never pass it to `Point3D.transformBy` (`[PB-SPACE-METHODS]`);
2. draw the closed hexagon in the table's draw order as six `sketch.sketchCurves.sketchLines.addByTwoPoints(...)`
   **sharing** those points;
3. **then** fix the lines and their endpoints —
   `for e in lines: e.startSketchPoint.isFixed = True; e.endSketchPoint.isFixed = True` — **after** the
   lines exist, not before. Setting `isFixed` on a bare point before it is consumed as a line endpoint
   does NOT leave the sketch fully constrained.

`sketch.project` is **not** used here: a projected point is brought in associatively and still carries
free DOF, so a sketch hanging off projections reports under-constrained even though every point looks
right (`[PB-PROJECT-NOT-FIXED]`).

**The hexagon's FIRST edge is the gear's shaft axis** for the revolve, the pattern, the bore plane and
the meshing rotation, so it must be fixed well enough to carry a trustworthy world position: fixed
endpoints give that edge a well-defined `worldGeometry`, while a free edge resolves against a
default/world-XY frame and silently moves the body onto world XY — observed on the driving gear, where
the pinion looked fine only because it never read the edge's `worldGeometry`.

**The shaft axis used by every body operation below is this profile sketch's first edge, NOT the §2
`Apex->A` / `Apex->B` construction line.** The edge is collinear with the shaft axis but lives in the
*same* sketch as the profile, which is what Fusion's revolve, pattern and path builders accept;
reusing the §2 construction line, which lives in a different sketch, fails or misbuilds.

Gate the sketch with `sketch.isFullyConstrained` (`[BEVEL-F-FULL-CONSTRAINT]`).

**What the proof checks.** The six vertices land on their §2 solved positions, the first edge's two
endpoints are the only ones on the axis — nothing else may touch it or the revolve aborts with
`ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`) — and the sketch holds exactly one valid, non-self-intersecting
profile.

**From:** `spec/bevelgear/instructions.md` L536 L699–719, `.claude/skills/generate-gear/PLAYBOOK.md` L449–463 L485–491 L576–581 L589–596 L708–714

## S14 `[GO]` Revolve the Gear Body

Proof function `stepRevolveGearBody`.

<!-- proof-run: proofkit3d.RunSolidParallel(revolveCases, stepRevolveGearBody, assertRevolveGearBody) -->

This sketch holds exactly one hexagon loop, so take its single profile directly —
`profile = sketch.profiles.item(0)` (`[PB-SINGLE-PROFILE]`); resist inventing a search that filters by
`profileLoops` or curve type, which has spuriously rejected a valid all-line loop and made the revolve
fail with "could not find profile". Revolve it around the **shaft-axis edge**:

```
revolveInput = designComponent.features.revolveFeatures.createInput(
    profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
gearBody = designComponent.features.revolveFeatures.add(revolveInput).bodies.item(0)
```

The result is the **Gear Body**, the frustum. Because the toe edge is one edge of the revolved
profile, the body already carries the conical face that edge sweeps around the axis — that face is
reused as the cutting tool at S16, and likewise the heel edge's cone.

**Hard failure to design around** (`[PB-REVOLVE]`): the profile must NOT cross the axis of revolution.
If it does, Fusion aborts with `RuntimeError … ASM_WIRE_X_AXIS … the profile crosses the axis of
revolution`. The Maximum Face Width cap and the strictly-positive Toe Radius are what keep it on one
side; reproduce them exactly.

**What the proof substitutes, and what it costs — THE UNION.** decad publishes a revolved body's
volume with a proven bound equal to the volume itself, so a revolved body is Suspect at any tolerance
and cannot pass the harness gate at all. The proof therefore builds a **polygonal sweep**: the three
bands the frustum's profile edges sweep — the **root** cone out to the dedendum corner (M→C), the
**heel** cone out to the heel end (C→H), and the **toe-dish plug** that hollows the front face (N→M) —
laid apart and never joined. It asserts the frustum as their **SIGNED SUM** against Pappus on the §2
hexagon (`root + heel − toe plug`), band by band against its own stations and ring radii, and cone
half-angle by cone half-angle: the heel band and the toe plug come out **parallel**, on the back-cone
family at `90° − γ`, and the root band at the **dedendum angle** `atan(1.25·m / R)` to them. The cost
is the union: the proof does not show the three bands closing into one watertight solid, only that
each is separately watertight and that together they have the right volume, stations and angles. Each
ring is drawn as an explicit regular polygon rather than a circle, because a lofted polygon is a
polyhedron whose volume decad proves exactly; the polygon's known area factor is carried in every
volume assertion.

**The solid tables run at Module 4 to 8.** Do not put Module 1 in a solid case table: decad's mesh
bound has an absolute floor, so a figure small enough brings every measurement inside it and the gate
reports Suspect on geometry that is in fact correct. Module is a pure scale on this figure. The sketch
tables are unaffected and stay at the dialog's own default.

**From:** `spec/bevelgear/instructions.md` L715–721 L767–801 L832–838, `.claude/skills/generate-gear/PLAYBOOK.md` L485–491 L708–714

## S15 `[GO]` Loft the Tooth Body

Proof function `stepLoftToothBody`.

<!-- proof-run: proofkit3d.RunSolidParallel(toothLoftCases, stepLoftToothBody, assertLoftToothBody) -->

Loft the **§2 Apex sketch point** — `centerToApex.endSketchPoint` from the Gear Profiles sketch, the
degenerate point-section — to this gear's `{gearLabel} Tooth` profile:

```
loftInput = designComponent.features.loftFeatures.createInput(
    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
loftInput.loftSections.add(apexSketchPoint)
loftInput.loftSections.add(toothProfile)
toothBody = designComponent.features.loftFeatures.add(loftInput).bodies.item(0)
```

`loftSections.add(...)` order is the loft order, and a section may be a single `SketchPoint` for a
degenerate end (`[PB-LOFT]`). **Use the §2 Apex SKETCH point directly — do NOT create a
`ConstructionPoint` for it** (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`: construction geometry needs an active
component and the Design component is never active, while sketch geometry does not).

**Select the tooth cross-section loop** with the framework helper
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)` from `.utilities`, where
the line count is **DETERMINED BY the `embedded` flag, NOT guessed and NOT accepted-either**:

    wantLines = 0 if embedded else 2

⚠ **Do NOT accept "0 **or** 2 lines".** For a given gear only ONE of those is the real tooth; an
**unrelated** loop — an inter-tooth or annular region between the `drawCircles` circles — can also
have 2 NURBS + 2 arcs but the *other* line count, and selecting it makes this loft fail with
`RuntimeError … ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`, because the impostor loop cannot form a loft
tool body. `embedded` ⇒ tip, root and flanks meet with no connecting lines (4 curves); non-embedded ⇒
2 connecting lines (6 curves). The flag is the one read back from `proxy._lastToothEmbedded` at S10.

**What the proof substitutes, and what it costs.** decad's `Loft` takes two profiles, so the proof
substitutes a **shrunken section** for the degenerate apex point — the same outline scaled about the
apex — and asserts the taper the loft has to produce: the volume is a third of the section's area
times the apex's perpendicular distance to the section plane, less the nose, and the root and tip both
ride straight cones through the apex. The tooth plane is the real back-cone plane, tilted by γ out of
the axis-perpendicular, which is what puts those surfaces on the true cones; a tooth corner sits a
little inside the tip cone, exactly as the drawn tooth does in Fusion. The flanks are the real
involute at the proxy's own 15 samples, with two chords standing in for the tooth-top arc and the root
arc. The cost is the true point-section, which is not built.

**From:** `spec/bevelgear/instructions.md` L374–384 L721 L802–803, `.claude/skills/generate-gear/PLAYBOOK.md` L152–154 L715–719 L782–790

## S16 `[GO]` Trim the Tooth Body — the conical end cuts

Proof function `stepConicalEndCuts`.

<!-- proof-run: proofkit3d.RunSolidParallel(conicalCutCases, stepConicalEndCuts, assertConicalEndCuts) -->

Trim the Tooth Body to a flush band with the framework helper — **do NOT re-implement the cut
machinery**:

```python
keeper = cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)
```

**Two distinct bodies are involved — do not conflate them.** The cutting TOOLS are `ConeSurfaceType`
faces of the **Gear Body**, the revolved-hexagon frustum; the lofted Tooth Body has no cone faces, so
searching *it* finds none. The TARGET being split is the **Tooth Body**.

The helper implements the pinned cut behaviour: the **toe cut first**, its cone face identified by the
toe edge's world **MIDPOINT** best-first across the frustum's cone faces (`[PB-FACE-BY-MIDPOINT]` —
endpoints sit near the apex singularity where `getParameterAtPoint` returns no result), each candidate
tried as the actual split tool with `isSplittingToolExtended=True` and the first that splits into more
than one piece kept; **keeper selection after each cut**, dropping apex-containing pieces and keeping
the largest (`[PB-REMOVE-PIECES]`); then the **heel cut on the keeper alone** — removing the apex tip
first is what makes it deterministically two split features for every gear ratio. A heel cone that does
not intersect the keeper at all, common on ratio pairs such as Module 1 / driving 31 / pinion 43 where
the heel cone never overshoots the tooth, is raised by the helper as the typed
`solids.NonIntersectError` and caught, and the keeper is returned whole. Every failure is
self-diagnosing with the per-face distance and error history (`[PB-SELF-DIAGNOSING]`), and each cut's
outcome is logged with `force_console=True`.

**Caller obligations, which stay in the generator.** Pass `toeMid` = the toe edge's world **midpoint**,
`(M_world + N_world)/2` on the pinion and `(O_world + P_world)/2` on the driving gear; `heelMid` = the
heel edge's world midpoint, `(C_world + H_world)/2` / `(D_world + J_world)/2`; `apexWorld` = the §2
Apex sketch point's `worldGeometry`; `gearBody` = the revolved frustum, the cone-face source. **The toe
cut must split** — its failure propagates and crashes the build, which is correct, since an uncut tooth
is unusable. Only the heel cut is lenient, and only via the typed `NonIntersectError`.

For ψ = 0 this is the whole tooth-body step and the tooth is the straight one. For ψ > 0 it is what
step J of the spiral build returns instead (S25), on the curved tooth.

**What the proof substitutes, and what it costs — THE SPLIT.** Neither cut is performed. Both operands
are Lofts — the tooth and each cone alike — so the split is unavailable. The proof builds the tooth,
the two cones and the gear body's own root cone, lays them apart, reads each cone's apex and
half-angle off the cone and each of the tooth's two surfaces off the tooth, solves the stations where
they cross from those readings, and checks them against the flush band. The proof does not show the
evaluator dividing the tooth, selecting the keeper, or leaving a watertight body. What it does show is
that each cut lands where the flush band requires and that the two ends land on **different** surfaces
of the tooth — a plane would cross the tooth's tip and root at one station, and the difference is the
observable signature of a conical cut face.

**From:** `spec/bevelgear/instructions.md` L359–368 L723–749 L804–811, `.claude/skills/generate-gear/PLAYBOOK.md` L160–173 L724–752 L753–765

## S17 `[GO]` Spiral: the `{gear} Cone Element` sketch

Proof function `stepSpiralConeElement`.

<!-- proof-run: proofkit.RunParallel(coneElementCases, stepSpiralConeElement) -->

**Steps S17 through S25 run only when ψ > 0.** The tooth-body hook's first line is the gate
`if self._spiralAngle_rad <= 0: return cut_conical_ends(...)` — at ψ = 0 the straight tooth's two
conical trims (S16) are the whole of the step and the behaviour is byte-for-byte the prior one. The
hook is `_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane, gearLabel,
teethNumber, gamma)`, called once per gear from `_createGearBody` on the freshly lofted uncut
apex→heel tooth, before pattern, combine and bore. `gamma` is this gear's pitch cone half-angle,
`self._gamma_p` on the pinion and `self._gamma_g` on the driving gear, forwarded to the twist law.

### The caller hand-off — PIN IT EXACTLY

`_createGearBody` builds the four toe/heel world points and passes them **positionally in the order
`toeMid, heelMid, toeConeWorld, heelConeWorld`**. Mislabelling them silently inverts the spiral, and
this is the single biggest spiral-regen hazard:

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

- `toeMid` = world **midpoint of the TOE edge** — ½(M+N) pinion, ½(O+P) driving;
- `heelMid` = world **midpoint of the HEEL edge** — ½(C+H) pinion, ½(D+J) driving;
- `toeConeWorld` = the toe edge's **inner endpoint**, M / O. It lies on the **root cone element**
  Apex→C / Apex→D at the toe end — M is pinned onto Apex→C in §2, O onto Apex→D;
- `heelConeWorld` = the **dedendum corner**, C / D, the **outer** end of that **same** root cone
  element, so `coneVec = normalize(heelConeWorld − apex)` runs along Apex→C / Apex→D pointing outward
  and the dedendum corner's own cone distance is greater than the toe point's.

⚠ **Two scrambles to avoid, both of which a fresh regen has made.** Do **NOT** pass the two endpoints
of a *single* edge as `toeMid`/`heelMid` — M and N both sit at the **toe**, so
the span — the heel midpoint's cone distance less the toe midpoint's — collapses to about zero or goes negative and the
spiral inverts; they are the midpoints of two **different** edges. And `heelConeWorld` is the dedendum
corner **C/D, never H/J**: H and J lie on the Apex2→C / Apex2→D dedendum line, one Module beyond C/D
and **off** the root cone element, so using them skews `coneVec` away from Apex→C / Apex→D.

### A. Gate and frame

Build a world frame from the geometry already constructed for this gear:

- `axisDir` = the **shaft axis** direction, from the two **world** endpoints of `shaftAxisEdge` — the
  in-sketch profile edge A′→G / B′→I — normalized;
- `coneVec` = `normalize(heelConeWorld − apexWorld)`, the dedendum (root) cone element;
- `v` = `axisDir × coneVec`, normalized — the **circumferential** direction, the sideways sense the
  tooth is displaced from the radial element;
- `tpNormal` = `coneVec × v`, normalized — the **tangent-plane normal**;
- a point's **cone distance** `distAlong` is `(p − apex) · coneVec`.

⚠ **The heel MUST be the OUTER end** so `coneVec` points outward and `span > 0`. Before building
`coneVec`, check the passed midpoints and **fix a swapped toe/heel**: if
`apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid ↔ heelMid` **and**
`toeConeWorld ↔ heelConeWorld`, then build `coneVec` from the new `heelConeWorld`. A negative `span`
**silently inverts the entire spiral frame** — it flips the cutter-arc direction, the slice direction
(so the first cut misses) and the per-segment twist — and the gear comes out completely wrong with no
error. The inversion can also originate upstream in §2 or §3 mislabelling the toe versus heel edges;
this guard catches it at the frame.

From `toeMid` and `heelMid`, **after** the swap guard, take
`R_toe` and `R_heel` as those two midpoints' cone distances, `R_mean = ½(R_toe + R_heel)` and
`span = R_heel − R_toe`, the face width, now positive. These are the only quantities the rest of the
build needs.

### This step

Draw a **cone-element construction line** Apex → (Apex + `R_heel` · `coneVec`) in a sketch on the
**axial / Gear Profiles plane**, using `sketch.sketchCurves.sketchLines.addByTwoPoints(...)`, and name
the sketch `{gear} Cone Element`.

**Coordinates — this rule governs the `{gear} Cone Element` sketch as well as the trace sketch of S19,
and it is the only place either is told what frame its points are in.** The raw `apex` and cone-end
points are passed **directly** into the sketch call, where they are consumed as **sketch-space** input
— **no `modelToSketchSpace` conversion is applied**, even though `adsk.fusion.Sketch` offers exactly
that call and the points really are model-space coordinates. This is deliberate and it is worth
stating why it is harmless, because the reasoning is not the obvious one. The trace sketch is
construction and reference only: no downstream feature ever consumes it, since the twist is computed
analytically in §3a step G and the sketch exists only so the genuine cutter arc is inspectable before
cleanup hides it. The cone-element line is the one that needs the extra sentence, because it *is*
consumed — by `plane_by_angle(comp, coneElementLine, axialPlane, 90)`, which rotates about it to make
the Trace Plane. So an unconverted cone-element line does place that plane somewhere other than the
true tangent plane. That still reaches no feature, because the only thing built on the Trace Plane is
the inspection-only trace sketch and the whole chain ends there. **If a later revision ever makes any
feature consume the trace sketch or the Trace Plane, this shortcut stops being safe and both sketches
need `modelToSketchSpace` on every point.**

This sketch and the Trace Plane and trace sketch below are the **spiral build's transient auxiliary
sketches**, which are **exempt** from the full-constraint gate (`[BEVEL-F-FULL-CONSTRAINT]`). Do not
gate them.

**What the proof checks.** That the cone element runs along the **root** cone element Apex→C / Apex→D
— never along Apex→Apex2 and never along the shaft axis, which are the two ways the whole frame gets
skewed — that it reaches `R_heel`, that `span` is positive, and that a deliberately swapped hand-off
fires the guard.

**From:** `spec/bevelgear/instructions.md` L339–357 L605–636 L649 L654–658, `spec/bevelgear/spiral-tooth-trace.md` L30–76 L240–254, `spec/bevelgear/fusion.md` L59–67

## S18 `[GO]` Spiral: the `{gear} Trace Plane`

Proof function `stepSpiralTracePlane`.

<!-- proof-run: proofkit.RunParallel(tracePlaneCases, stepSpiralTracePlane) -->

Build the tangent plane with the framework's `plane_by_angle`: the **axial plane rotated 90° about the
cone-element line** —

```python
tracePlane = plane_by_angle(designComponent, coneElementLine, axialPlane, 90)
```

— and name it `{gear} Trace Plane`. Its in-plane axes are **x = `coneVec`**, so a point's x is its cone
distance, and **y = `v`**, circumferential. The origin is the apex.

This is the plane the cutter cuts in: in the textbook construction the flat plane is the plane of the
**generating crown gear**, tangent to the pitch cone along the cone element, with its centre at the
apex, and the face-mill cutter sweeps its circular arc in this plane. The caveat is that this
implementation lays the trace on the **root** cone rather than the canonical pitch cone; the two
tangent planes differ only by the small dedendum angle, so the arc's shape is essentially identical,
but ψ then ends up measured on the root cone.

Exempt from the full-constraint gate, as S17 says.

**What the proof substitutes, and what it costs.** The frame is built in the sketch engine's World and
read back: it contains the cone element, stands square to the axial plane, and its y axis is
perpendicular to both the element and the shaft axis. `plane_by_angle` and Fusion's `setByAngle` are
not themselves exercised.

**From:** `spec/bevelgear/instructions.md` L649, `spec/bevelgear/spiral-tooth-trace.md` L30–63, `.claude/skills/generate-gear/PLAYBOOK.md` L181

## S19 `[GO]` Spiral: the `{gear} 2D Tooth Trace` sketch

Proof function `stepSpiralTrace`.

<!-- proof-run: proofkit.RunParallel(traceSketchCases, stepSpiralTrace) -->

Add a sketch on the Trace Plane named **`{gear} 2D Tooth Trace`**.

### B. Cutter-arc geometry

Work in the tangent-plane 2-D frame with origin at the apex, x = `coneVec` and y = `v`. The cutter
radius is `r_c` = **Cutter Radius** if non-zero, **else `R_mean`** — 0 means auto. The hand sign is
`handSign = +1` for `Right` else `−1`, then **negated for the pinion**, because the pair meshes with
opposite hands. The cutter-circle centre is

```
Cx = R_mean − r_c · sin ψ
Cy = handSign · r_c · cos ψ
```

⚠ **The hand sign goes on the `cos` / `Cy` term, NOT the `sin` / `Cx` term.** This was a real bug.
Opposite hand mirrors the cutter centre **across the cone element (y = 0)**, which flips `Cy`. Putting
`handSign` on `Cx` mirrors about `x = R_mean` instead — a *different* curve that gives the two gears
**unequal twist**; for equal teeth the driving and pinion traces must come out as exact mirror images.
That is the entire dependence on ψ, the hand and `r_c`.

The trace's toe and heel arc endpoints are circle∩circle intersections taken a hair **past** the face,
so the kept arc reaches cleanly past the end-trims:

```
R_lo   = R_toe  − 0.06 · span
R_hi   = R_heel + 0.06 · span
toe2d  = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)
heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)
```

`circle_intersect_nearest` is the framework helper from `.solids`: it intersects the apex circle of
radius R with the cutter circle and keeps the solution nearest `(R_mean, 0)`, the branch the mean point
sits on. Keeping the far branch gives a kinked or back-bent trace.

### C. The sketch itself

Map 2-D coordinates to world with the framework's `combine_point`:
`tanW(px, py) = combine_point(apex, px, coneVec, py, v)`. Then draw, passing those world `Point3D`s
**directly** into the sketch calls as sketch-space input, per the coordinates rule in S17:

- the **cutter circle** — centre at `tanW(Cx, Cy)`, radius `r_c`, drawn with
  `sketch.sketchCurves.sketchCircles.addByCenterRadius(...)`, marked `isConstruction = True`, with its
  centre pinned via `circle.centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]` — a circle's centre
  is a free point even when created at the origin, and `addCoincident` to `sketch.originPoint` throws
  `VCS_SKETCH_SOLVING_FAILED`) and a diameter dimension
  `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` set to `2 · r_c`;
- the **trace arc** — a three-point arc through `tanW(toe2d)`, `tanW(R_mean, 0)` (the mean point on the
  cone element) and `tanW(heel2d)`, drawn with
  `sketch.sketchCurves.sketchArcs.addByThreePoints(startPoint, point, endPoint)`, with its **centre
  coincident to the cutter circle's centre** —
  `sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, circle.centerSketchPoint)`, which
  is the one place in `[PB-SHARE-XOR-COINCIDENT]` where passing a point and also coinciding to it is
  correct rather than redundant — and a **radius dimension**
  `sketch.sketchDimensions.addRadialDimension(arc, textPoint)` set to `r_c`, so it is the genuine
  cutter circle and not a look-alike spline.

⚠ **Text points per `[PB-RADIAL-DIM]`: off-centre, on or near the curve.** A radial or diameter
dimension rejects a `textPoint` at the curve's centre with `RuntimeError: 3 : … 一部の入力引数が無効です`.
Use the mean point `tanW(R_mean, 0)` for the trace arc's radius dimension, and a point on the cutter
circle such as `tanW(Cx + r_c, Cy)` for its diameter dimension.

This sketch is **deliberately left with free DOF** — the arc's endpoints are pinned by the three-point
construction, not by endpoint dimensions, because dimensioning them over-constrains the solve against
the cone-element plane — and is **exempt** from the full-constraint gate. Do not gate it.

### D. No 3-D projection

The 2-D cutter-arc sketch is the only trace geometry needed: the spiral twist is computed
**analytically** in step G, so there is **no `projectToSurface`, no root-cone-face search and no 3-D
trace sketch**. Earlier versions projected the 2-D arc onto the root cone along `tpNormal` and measured
the trace azimuth there. That projection is fragile: for unequal-ratio pairs the arc wraps around the
cone and `projectToSurface` returns it as **multiple disjoint fragments**, so the measured azimuth
collapses to a fraction of the true sweep, the pinion comes out grossly under-twisted and the pair
interferes. **Do not reintroduce it.**

### G (the part this step fixes): the twist the trace determines

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`phi_crown` is the angle the cutter arc's **toe and heel endpoints subtend at the apex** in the flat
2-D crown frame. `total` is the toe→heel shaft-axis twist **magnitude**, from the **conjugate
crown-gear generation law**: a spiral bevel is generated by an imaginary flat crown gear, and the work
gear's shaft rotation relates to the developed crown-plane azimuth by the **roll ratio `1 / sin γ`**,
the generating crown gear having `N / sin γ` teeth. ⚠ **Use the PITCH cone angle γ from §2 — NOT
`acos(coneVec · axisDir)`**, which is the *root* cone angle, about 14° against a pitch 29° for a
17-tooth pinion, and yields a twist about 1.6× too large. ⚠ **The two members of a meshing pair
legitimately get different twists**: same cutter, same ψ, but γ differs, so `1 / sin γ` differs —
about 2.08× for a 17-tooth pinion against about 1.14× for a 31-tooth gear, a ratio near 1.83. That is
*why* equal-teeth pairs always meshed while ratio pairs failed under any method that gets `1 / sin γ`
wrong. `handSign` sets the direction; `total` is the magnitude.

**What the proof checks, and what it substitutes.** Every invariant `spiral-tooth-trace.md` §9 lists:
the toe and heel ends sit on their apex-centred circles, the arc's radius is `r_c` and its centre is
`r_c` from the mean point, the mean spiral angle is realised **at** the mean point, flipping the hand
mirrors the centre across the cone element and changes nothing else — which is the check that catches
the hand sign being put on the `sin`/`Cx` term — and at ψ = 0 the centre stands straight north of the
mean point so the arc is tangent to the element there. The substitution: Fusion leaves this sketch with
free DOF and exempts it, while proofkit's gate is not waivable, so the proof pins the two endpoints at
their own circle∩circle positions and pins the better-conditioned coordinate of the centre, then reads
the centre coincidence and the radius back rather than constraining them. What that does not exercise
is Fusion's three-point arc. ψ = 0 does **not** leave zero twist — the arc still has finite radius and
its ends still subtend an angle — because the straight tooth comes from the hook's own gate, not from
the arc degenerating.

**From:** `spec/bevelgear/instructions.md` L638–660 L666–673, `spec/bevelgear/spiral-tooth-trace.md` L90–182 L185–236, `spec/bevelgear/fusion.md` L59–67, `.claude/skills/generate-gear/PLAYBOOK.md` L442–448 L500–507 L605–625 L643–647

## S20 `[GO]` Spiral: slice the straight tooth into slabs

Proof function `stepSpiralSlice`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralSliceCases, stepSpiralSlice, assertSpiralSlice) -->

### E. Slice the straight tooth

Split the uncut apex→heel `toothBody` into cross-section slabs by planes **perpendicular to the cone
element**, spanning a touch past toe and heel, via a **fixed** slice scheme of **8 planes — the count
is not user-configurable**.

The first cut plane is the **parent transverse tooth plane** — `parentToothPlane`, the virtual-spur
tooth-profile plane `{gearLabel} Plane` from S09, passed into the hook — offset toward the apex by
`span / 6`. The offset **sign is chosen per gear** so that it moves toward the apex: the parent
plane's normal points opposite ways for the two gears, so pick `sign` such that `sign · normal` points
apex-ward, i.e. test `(apex − planeOrigin) · normal`. Then a sequence of 8 planes stepped further
toward the apex in `span / 6` increments:

    offsets = [sign · (k + 1) · span / 6  for k in 0…7]      # k = 0 is the first cut plane

Split with the framework helper — it splits piece-by-piece and keeps a piece whole when a plane misses
it:

```python
pieces = slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)
```

⚠ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in **one
piece** — no plane cut it — the offset sign was wrong or `parentToothPlane` sits outside the tooth's
span: **retry the whole cut once with the opposite sign**. If it is *still* one piece, **`raise` a
clear self-diagnosing error** naming the gear, the final piece count, `span` and the sign tried
(`[PB-SELF-DIAGNOSING]`, `[PB-EMPTY-RESULT]`). Do **NOT** return an unsliced single-piece result: step
F then drops that one piece as the apex scrap, leaving `segments` **empty**, and the crown later
crashes with `ValueError: max() iterable argument is empty` far from the cause.

The result is the set of cross-section segments.

**What the proof substitutes, and what it costs — THE SPLIT.** decad has no split at this revision:
both operands would be Lofts. Because the uncut tooth is a cone over its heel section, a plane
**parallel** to the parent tooth plane cuts it in that same section scaled by its share of the apex
distance, so the proof builds the nine pieces the eight planes leave directly, each a loft between two
consecutive cut sections, laid apart along the shaft-frame's +X about its own parallel axis. It asserts
the first plane sits `span/6` from the parent plane, the planes step by `span/6`, eight planes leave
nine pieces, the last plane is still short of the apex, and the pieces' volumes sum to the tooth
between the nose and the parent plane. The proof does not show the evaluator dividing one body.

**From:** `spec/bevelgear/instructions.md` L662, `.claude/skills/generate-gear/PLAYBOOK.md` L176–177 L431 L753–765 L724–730

## S21 `[GO]` Spiral: order the segments and drop the apex scrap

Proof function `stepSpiralScrap`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralScrapCases, stepSpiralScrap, assertSpiralScrap) -->

### F. Order and drop scrap

Sort the segments by `distAlong` of their centroid, `body.physicalProperties.centerOfMass`. The first
— the apex-most — is the long **apex-side scrap** below the toe: **remove it**, and keep the rest as
the working `segments`.

**Drop the scrap by re-slicing the list FIRST and only then deleting the body** —
`segments = segments[1:]` before
`designComponent.features.removeFeatures.add(scrap)` — so the kept list never holds the piece that is
about to go. Use `removeFeatures.add`, which is timeline-visible, and not a bare `deleteMe()`
(`[PB-REMOVE-PIECES]`).

After dropping the scrap, **`segments` must be non-empty**, at least one cross-section. If it is empty
the slice failed at E: `raise` a clear error rather than proceeding into the twist (G) and the crown
(H), which both assume at least one segment (`[PB-EMPTY-RESULT]`).

**What the proof checks.** That eight segments remain, that the dropped piece really is the apex-most
and is the long one, and that every kept segment has volume. The removal itself is not performed —
decad has no remove — so the proof simply does not build the scrap in this step.

**From:** `spec/bevelgear/instructions.md` L664, `.claude/skills/generate-gear/PLAYBOOK.md` L431 L761–765

## S22 `[GO]` Spiral: the twist

Proof function `stepSpiralTwist`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralTwistCases, stepSpiralTwist, assertSpiralTwist) -->

### G. Twist (the spiral)

Rotate each segment about the **shaft axis** — `axisDir` through `apex` — so the tooth follows the
trace, **centred on `R_mean` so the mid-face section stays unrotated**. That section then meshes
exactly like the straight tooth, which is critical: the pinion's zero mesh nudge depends on it. The
total toe→heel twist `total` is the one S19 computed from `phi_crown` and the roll ratio `1 / sin γ`.

Each segment's rotation angle is a **linear share** keyed to the **cone distance of its HEEL FACE** —
the segment's farthest-along-the-element face, which is the exact section the later loft samples:

```
ang = −handSign · total · (R_mean − R_heelFace(seg)) / span
```

**Define a slab's heel face precisely: the face whose centroid has the GREATEST cone distance
`distAlong`, searched across ALL of the slab's faces with NO surface-type filter** — its
toe / apex-side face is the LEAST-centroid one. ⚠ Do **NOT** restrict this search to
`adsk.core.SurfaceTypes.PlaneSurfaceType`, or to any surface type: a sliced slab is bounded by a mix
of the two planar cut faces and ruled side faces, and a type filter can pick the wrong face or miss
the cut face, which makes the step-I loft fail with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`.
Use this **same all-faces-by-centroid** rule everywhere a slab end face is needed: the twist key here,
the crown base at S23, and the loft sections at S24.

⚠ **Key the twist on the segment's HEEL-FACE cone distance, NOT its centroid.** The loft samples each
segment's heel face, so that face is what must land at the right azimuth; centroid-keying leaves the
loft's mid-face section rotated by half a segment and the mid-faces overlap.

Apply the rotation with a free-move:

```
matrix = adsk.core.Matrix3D.create()
matrix.setToRotation(ang, axisVector, apexPoint)
moveInput = designComponent.features.moveFeatures.createInput2(bodyCollection)
moveInput.defineAsFreeMove(matrix)
designComponent.features.moveFeatures.add(moveInput)
```

Use `defineAsFreeMove` with a matrix and not `defineAsRotate`, which rejects a `SketchLine` axis
(`[PB-MOVE-ROTATE]`). **A zero angle is a no-op, not a move**: `setToRotation(0, axis, origin)` builds
the identity and Fusion refuses it with `RuntimeError: 3 : invalid transform`, so return early rather
than moving by it.

**What the proof checks, and what it substitutes.** Each segment's share of the total, read off the
built body's own azimuth rather than trusted from the plan; the post-twist heel face where the twist
put it; that the segments straddle `R_mean`, so something really is left unrotated at mid-face; that
the toe-to-heel total is the crown-gear law's own; and that a ratio pair comes out with **different**
twists on its two members, which is the `1 / sin γ` check. The rotation is a real rigid motion of a
real body, applied with the bench's equivalent of the free move; the move feature itself is not
exercised.

**From:** `spec/bevelgear/instructions.md` L666–679, `spec/bevelgear/spiral-tooth-trace.md` L185–214, `.claude/skills/generate-gear/PLAYBOOK.md` L178–179 L731–752 L791–800

## S23 `[GO]` Spiral: the lengthwise crown

Proof function `stepSpiralCrown`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCrownCases, stepSpiralCrown, assertSpiralCrown) -->

### H. Lengthwise crown (relief)

Crown the tooth by scaling each segment **except the outermost (heel) one** down by a **monotonic**
factor — full at the heel, growing smoothly toward the toe — **about a sketch point on the ROOT edge
of its heel face**, not the heel-face centroid.

For each segment compute its **heel-distance fraction** `u = (R_heel − R_heelFace) / span`, where
`R_heelFace` is the `distAlong` of that segment's heel face **found by the step-G all-faces-by-centroid
rule but RECOMPUTED here, AFTER the step-G twist has moved the slabs** — do not reuse pre-twist values
— and `R_heel` and `span` come from step A. `u` runs 0 at the held-full heel and grows toward the toe.
**"Outermost (heel) segment" = the one with the GREATEST post-twist heel-face `distAlong`**: sort the
segments by their recomputed heel-face `distAlong` and skip the last. Then:

```
factor = 1 − _CROWN_PER_RAD · (|total| / 2) · u
```

`total` is the full toe→heel twist from step G, so `|total| / 2` is the per-end peak twist magnitude
and the maximum relief — now at the **toe** — keeps the magnitude the old per-end peak had, just
relocated. This makes relief **grow monotonically from the (full) heel to the toe**, so slab heights
stay **strictly ordered heel→toe** and the natural cone taper is never reversed. If a computed
`factor` comes out ≤ 0, from an extreme twist, **`raise` a self-diagnosing error** naming the gear, the
segment's `u` and the factor — never scale by a non-positive factor. **`_CROWN_PER_RAD` is a tunable
class constant with default `0.5`** — 0 disables the crown; **set it to 0.5 and do not leave it unset
or 0.**

⚠ **Do NOT key the relief on `|ang|`, the twist magnitude.** That is **symmetric** about mid-face,
maximal at BOTH ends, so because the heel slab is held full the slab *just inside* the heel becomes the
**most**-relieved one and dips below both its neighbours — a notch that reverses the heel→toe taper.
This was the observed bug: the heel-adjacent slab came out at factor `0.932` while the next slab
inward was `0.972`, taller. Key the relief on the monotonic heel-distance `u`, never on `|ang|`.

**Three gotchas.**

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex — per `[PB-CONSTRUCTION-NEEDS-ACTIVE]`. `scaleFeatures` is the ONE exception to
   never-activate: it needs the Design occurrence as the **active** edit target, so call
   **`designOccurrence.activate()`** — a method on the `Occurrence` — before the crown scales and
   restore afterwards, in a `finally`, with **`design.activateRootComponent()`** — a method on
   `Design`. ⚠ Do **NOT** write `design.rootComponent.activate()` or `someComponent.activate()`: a
   `Component` has **no** `.activate()` method and raises `AttributeError`. Only `Occurrence` has
   `.activate()`, and the root is re-activated through `Design.activateRootComponent()`.
2. **Skip the outermost (heel) segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone at S25 trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, NOT its centroid — otherwise the crowned tooth
   lifts off the gear base.** `scaleFeatures` shrinks **uniformly** toward the base point, so a base
   point at the heel-face **centroid**, at mid tooth-height, pulls the tooth's **root** edge upward by
   `(1 − factor) · (½ tooth height)`: the tooth no longer seats on the gear body's root cone, floats
   above the base, and the Combine-Join leaves a gap, clearly visible for ratio pairs such as
   module 2 / driving 19 / pinion 13, which is the original symptom that exposed this. Put the base
   point on the **root** instead: of the heel face's vertices — `heelFace.vertices`, each `.geometry` a
   world `Point3D` — take the **two with the smallest perpendicular distance to the shaft axis**, the
   line through `apex` along `axisDir`, where the perpendicular distance is
   `|(p − apex) − ((p − apex) · axisDir) · axisDir|`; those are the two **root corners**, since the tip
   corners are the farthest from the axis. Place the base sketch point at their **midpoint**, mapped
   into the heel-face sketch with `sketch.modelToSketchSpace(...)`. The heel face is a planar cut, so
   that midpoint lies on it. A uniform scale about a point keeps every line and plane through that
   point invariant, so anchoring on the root keeps the root edge on the seating cone while the tip is
   relieved progressively toward the toe — which is exactly the lengthwise crown intended. Finding the
   heel face itself is unchanged: still the max-`distAlong`-centroid face per step G; only the point
   *on* it changes from centroid to root-edge midpoint.

The scale itself is
`designComponent.features.scaleFeatures.createInput(bodyCollection, basePoint, adsk.core.ValueInput.createByReal(factor))`
followed by `designComponent.features.scaleFeatures.add(scaleInput)`.

**What the proof substitutes, and what it costs.** decad has no scale feature, so the proof rebuilds
each slab with its section scaled by the same factor and checks the factor law, the monotonicity, that
the heel segment is held full, that no factor is non-positive, that `_CROWN_PER_RAD` is 0.5, and that
each crowned segment's own reach off the built body is its factor times the reach it would have had
full. What that does not exercise is the scale feature and its base point. `u` itself runs past 1 at
the toe, because the eight cut planes at `span/6` reach 8/6 of a span beyond the parent tooth plane.

**From:** `spec/bevelgear/instructions.md` L681–693, `.claude/skills/generate-gear/PLAYBOOK.md` L576–581 L782–790

## S24 `[GO]` Spiral: loft the curved tooth

Proof function `stepSpiralLoft`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralLoftCases, stepSpiralLoft, assertSpiralLoft) -->

### I. Loft → curved tooth

⚠ **Re-sort the segments by their heel-face cone distance HERE, AFTER the twist (G) and the crown (H)
— do NOT reuse the pre-twist slice/centroid order from step F.** The twist rotates each slab about the
shaft axis, and for high-twist *unequal-ratio* pairs that rotation changes the slabs' along-cone
(`distAlong`) order enough to **reorder adjacent slabs**; lofting in the stale pre-twist order then
assembles the cross-sections out of sequence and the crowned tooth comes out distorted, so the two
gears interfere. For equal and low-twist pairs the two orders coincide, which is why equal-teeth gears
mesh even with the stale order while unequal ratios distort — this is the single thing that makes a
ratio pair like 31/17 fail while 31/31 looks fine.

So compute that order **now** — the segment indices sorted by the `distAlong` of each segment's own
heel-face centroid, by the all-faces-by-centroid rule of S22 — and loft a NewBody through, in that
order:

1. first the **toe-most segment's apex-side (toe-facing) face** — the toe segment is `order[0]`, and
   its toe face is added first so the loft is pushed past the toe cone and the toe trim bites;
2. then the **heel-facing face of every segment, iterated in `order`** — each segment's farthest-along-
   the-element face by post-twist centroid, the last reaching past the heel cone.

Name the resulting body **`{gear} Spiral Tooth`**. Then remove the segment scaffolding with
`designComponent.features.removeFeatures.add(...)`; the loft has captured their faces.

The loft is the same call shape as S15: `loftFeatures.createInput(operation)` then one
`loftInput.loftSections.add(face)` per section in order, then `loftFeatures.add(loftInput)`
(`[PB-LOFT]`).

**What the proof substitutes, and what it costs.** decad's `Loft` takes exactly two profiles, so the
proof builds the chain as one lofted piece per consecutive section pair, laid apart. It asserts the
order is strictly increasing in post-twist heel-face cone distance, that the first section really is
the toe segment's apex-side face, that every piece has volume and has its heel face outside its toe
face, and that a ψ > 0 case really produced a twist. What it does not show is the evaluator making one
body out of the whole chain.

**From:** `spec/bevelgear/instructions.md` L695, `.claude/skills/generate-gear/PLAYBOOK.md` L715–719 L761–765

## S25 `[GO]` Spiral: the flush trim

Proof function `stepSpiralFlushTrim`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralFlushTrimCases, stepSpiralFlushTrim, assertSpiralFlushTrim) -->

### J. Flush trim and mesh phase

Return

```python
return cut_conical_ends(designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)
```

— the same toe-then-heel two-cone trim the straight tooth takes at S16 — so the curved tooth's ends
sit **flush** on the gear base. Every caller obligation and every failure mode S16 states applies
unchanged; the only difference is that the target is the `{gear} Spiral Tooth` body rather than the
straight loft.

The toe and heel **mesh phasing** is handled **outside** this hook, by `_createGearBody`'s mesh-rotate
step at S31. The pinion's extra phase is 0 by default, because the mid-face section is unrotated and
already meshes: `_pinionMeshPhase(pinionTeeth)` returns `_PINION_MESH_PHASE_TEETH · 2π / pinionTeeth`
in **radians**, with `_PINION_MESH_PHASE_TEETH` a module constant whose default is **0**.

**What the proof substitutes, and what it costs.** The same as S16 — neither cut is performed, the
operands are laid apart and the crossings are solved from their own readings — but on the **curved**
tooth this time, so the flush band is checked against geometry the twist and crown have moved.

**From:** `spec/bevelgear/instructions.md` L339–358 L697 L723–749 L804–811 L828–830

## S26 `[GO]` Circular-pattern the tooth

Proof function `stepCircularPattern`.

<!-- proof-run: proofkit3d.RunSolid(patternCases, stepCircularPattern, assertCircularPattern) -->

Circular-pattern the remaining tooth piece around the **shaft-axis edge** — the same in-sketch profile
edge used for the revolve, **not** the §2 construction line:

```
bodies = adsk.core.ObjectCollection.create()
bodies.add(toothKeeper)
patternInput = designComponent.features.circularPatternFeatures.createInput(bodies, shaftAxisEdge)
patternInput.quantity   = adsk.core.ValueInput.createByReal(<this gear's Teeth Number>)
patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
patternInput.isSymmetric = False
pattern = designComponent.features.circularPatternFeatures.add(patternInput)
```

**Pin all three explicitly**; do not rely on Fusion's defaults staying equal to them
(`[PB-CIRCULAR-PATTERN]`). The number of copies equals this gear's Teeth Number. Although the pitch
diameter shrinks from heel toward apex, the *angular* spacing around the shaft axis stays constant at
`360° / N` for the entire face width — the radial taper is already produced by the loft from Apex to
the heel-end tooth profile, so the pattern just rotates that single tapered tooth into N evenly spaced
copies.

`pattern.bodies` already includes the seed body plus the copies, so **do not re-add the seed**, and
**copy them into an `adsk.core.ObjectCollection` first** before handing them to Combine, because
`pattern.bodies` is a `BRepBodies` and `combineFeatures.createInput` rejects it
(`[PB-PATTERN-BODIES]`).

**THIS STEP IS SERIAL, and it is the only one in this package that is.** It is registered through
`proofkit3d.RunSolid`, not `RunSolidParallel`. The pattern increment retires the seed tooth, so the
seed cannot be measured after the step runs: its azimuth, radius, height and volume have to be read
during the build and handed to the assertion, and that hand-off leaves the case. Two cases sharing one
set of seed readings overwrite each other, and it is not a hazard that announces itself — the two gear
sides differ enough in volume that the overwrite was caught when it happened, and a pair of cases whose
seeds measured alike would have passed on each other's numbers instead. The carried readings stay
where the proof keeps them, and the proof records beside them that this step is serial because of
them. **Every other `[GO]` step in this package is registered through the parallel entry point** —
`proofkit.RunParallel` where it would otherwise take `proofkit.Run`, and
`proofkit3d.RunSolidParallel` where it would otherwise take `proofkit3d.RunSolid` — because a bevel
case builds its own sketch or document from its own parameters and measures only the geometry that
case constructed. A step that later acquires a reading carried from its build into its assertion moves
to the serial runner in the same change.

**What the proof substitutes, and what it costs.** No pattern feature exists on the bench, so the proof
rotates the seed into copies at `k = 1` and `k = N − 1` with rigid motions and asserts each keeps the
seed's volume, radius and height and sits exactly one whole pitch increment round. Adjacent teeth of
one gear converge on the apex, so a copy left coaxial with the seed is a pair the evaluator cannot
prove disjoint; each copy is laid apart along +X after its rotation, which changes no azimuth, radius,
height or volume, and every reading is taken about that copy's own axis. The azimuth is read as the
section's **centroid**, not its outermost vertex, because a tooth is symmetric about its own centreline
and which of the two tied corners an argmax picks flips under a rotation.

**From:** `spec/bevelgear/instructions.md` L751 L848–872, `.claude/skills/generate-gear/PLAYBOOK.md` L684–694

## S27 `[GO]` Combine-Join the teeth with the Gear Body

Proof function `stepCombineJoin`.

<!-- proof-run: proofkit3d.RunSolidParallel(combineCases, stepCombineJoin, assertCombineJoin) -->

Join all patterned tooth pieces with the Gear Body in a **single Combine-Join**, the Gear Body as the
target and the patterned tooth bodies as the tools:

```
tools = adsk.core.ObjectCollection.create()
for i in range(pattern.bodies.count):
    tools.add(pattern.bodies.item(i))
combineInput = designComponent.features.combineFeatures.createInput(gearBody, tools)
combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
designComponent.features.combineFeatures.add(combineInput)
```

**What the proof substitutes, and what it costs — THE STITCH.** No join is performed: the frustum's
bands are Lofts. The proof lays the operands apart and asserts the join's two consequences from their
own measured geometry — a join leaves ONE lump when the tooth's root is at or below the body's root
cone, seated rather than floating, and the joined body reaches further out than the frustum when the
tooth's tip stands proud of it — taking both readings at the **toe**, the **middle** and the **heel**
of the band the join would cover. The proof cannot show the evaluator making one boundary out of two.

⚠ **The proof sinks the tooth's root a twentieth of the tooth height below the gear body's root cone**,
which is what makes "seated" measurable as a strict inequality. **The generated module seats the tooth
exactly on the cone and must not sink it** — the sink belongs to the proof alone.

**From:** `spec/bevelgear/instructions.md` L753 L812–820, `.claude/skills/generate-gear/PLAYBOOK.md` L684–688

## S28 `[GO]` The `{gearLabel}` bore plane

Proof function `stepBorePlane`.

<!-- proof-run: proofkit.RunParallel(borePlaneCases, stepBorePlane) -->

**Skip S28, S29 and S30 entirely if Enable Bore is unchecked.**

Build the bore plane normal to the shaft at its start:

```
planeInput = designComponent.constructionPlanes.createInput()
planeInput.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
borePlane = designComponent.constructionPlanes.add(planeInput)
```

Pass the **in-sketch shaft-axis edge** — A′→G / B′→I — directly, never the §2 construction line and
never wrapped in `Path.create` (`[PB-CONSTRUCTION-PLANES]`). Distance `0.0` along the path is the
edge's **start**, so the plane is rooted at the shaft start and its sketch origin sits **on** the axis,
which is what lets the bore circle be centred on the origin at S29.

**What the proof substitutes, and what it costs.** The frame is built in the sketch engine's World and
read back: normal to the shaft axis, through the shaft edge's start, and that start is the hexagon's
front foot, which is on the axis. Fusion's `setByDistanceOnPath` is not itself exercised.

**From:** `spec/bevelgear/instructions.md` L755, `.claude/skills/generate-gear/PLAYBOOK.md` L766–777

## S29 `[GO]` The `{gearLabel} Bore` sketch

Proof function `stepBoreSketch`.

<!-- proof-run: proofkit.RunParallel(boreSketchCases, stepBoreSketch) -->

Create a sketch on the bore plane named `{gearLabel} Bore`. Sketch the bore circle centred at the
sketch origin with `sketch.sketchCurves.sketchCircles.addByCenterRadius(...)`, then **fix the circle's
centre and add a diameter dimension**:

```
circle.centerSketchPoint.isFixed = True
sketch.sketchDimensions.addDiameterDimension(circle, textPoint)   # then .parameter.value = <bore diameter, cm>
```

`[PB-CIRCLE-CENTER]`: a circle's centre is a free point even when created at (0, 0, 0) —
`addByCenterRadius` does not reuse the sketch's `originPoint` — and `addCoincident` between the centre
and `sketch.originPoint` has been observed to throw `VCS_SKETCH_SOLVING_FAILED` on exactly this kind
of `setByDistanceOnPath` plane. `isFixed` on the centre plus a diameter dimension is 2 DOF + 1 DOF = 0.
The diameter text point must be off-centre, on or near the curve (`[PB-RADIAL-DIM]`).

The bore diameter is this gear's **Bore Diameter** if specified (non-zero); otherwise
**`this gear's Pitch Diameter / 4`**.

Gate the sketch with `sketch.isFullyConstrained` (`[BEVEL-F-FULL-CONSTRAINT]` — the Bore sketch is one
of the four permanent sketches the gate covers).

**A spec gap the proof records rather than asserts.** Nothing in the spec bounds the bore diameter
against the gear body it pierces, and the auto value can exceed the body's own heel radius: at
Module 1, 31/31 teeth and Shaft Angle 35° the auto bore radius is 3.8750 mm against a heel radius of
3.2101 mm, so the through-cut would take the whole blank away. The spec admits that configuration, so
the proof logs the reading instead of failing on it.

**From:** `spec/bevelgear/instructions.md` L100–104 L755, `spec/bevelgear/fusion.md` L21–30, `.claude/skills/generate-gear/PLAYBOOK.md` L442–448 L643–647

## S30 `[GO]` The bore through-cut

Proof function `stepBoreCut`.

<!-- proof-run: proofkit3d.RunSolidParallel(boreCases, stepBoreCut, assertBoreCut) -->

Cut a cylindrical through bore through the Gear Body along the shaft axis with a symmetric
extrude-cut restricted to this gear's body:

```
extrudeInput = designComponent.features.extrudeFeatures.createInput(
    boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
extrudeInput.setSymmetricExtent(
    adsk.core.ValueInput.createByReal(2 * <Cone Distance, cm>), False)
extrudeInput.participantBodies = [gearBody]
designComponent.features.extrudeFeatures.add(extrudeInput)
```

`[PB-THROUGH-CUT]`: the second argument `isFullLength=False` means the distance is the **half-length
per side**, so `2 × Cone Distance` each way is generously past any face width. Do not pass a third,
taper, argument. Restrict the cut with `participantBodies`.

**What the proof substitutes, and what it costs — THE PIERCED BODY.** The tool is built as a **real
extrude**, which a symmetric extent produces as a prism, but no cut is performed: the target is the
frustum, whose bands are Lofts. The proof lays the tool and the band apart and asserts the cut from the
tool's own measured geometry — its diameter, that its two ends sit exactly `2 × Cone Distance` either
side of the shaft edge's start, and that both clear the frustum, which is what makes it a THROUGH cut —
and computes the material it would remove from the frustum's own profile clipped to the bore radius.
One lump with a hole and no enclosed void is not shown. The Enable-Bore-unchecked branch is in the
table too, and there the proof asserts that no diameter resolves and nothing is built.

**From:** `spec/bevelgear/instructions.md` L755 L821–827, `.claude/skills/generate-gear/PLAYBOOK.md` L720–723

## S31 `[GO]` The meshing rotation

Proof function `stepMeshRotation`.

<!-- proof-run: proofkit3d.RunSolidParallel(meshRotateCases, stepMeshRotation, assertMeshRotation) -->

**Do this here, in the Design component, before the body is moved out.** Rotate the **driving** body
by `180° / Driving Gear Teeth Number` — half a tooth pitch — about its shaft axis, with the framework
helper:

```python
rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)
```

The helper takes the rotation axis and origin from the **B→I profile edge's world endpoints**
(`[PB-MOVE-ROTATE]`). Rationale: both gears are patterned from a starting tooth in the axial plane, so
without the offset a driving tooth and a pinion tooth would both sit at the axial-plane crossing and
visually collide; the offset puts a driving **valley** where the pinion tooth crosses, giving the
interlocked meshing look.

This runs in Design **before** `moveToComponent` because a construction axis cannot be added in the
moved-out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation must use the edge's world
geometry while the body is still in Design.

The pinion additionally gets `_pinionMeshPhase(pinionTeeth)`, which returns
`_PINION_MESH_PHASE_TEETH · 2π / pinionTeeth` radians and is **0 by default**, so no move feature is
emitted for it: `rotate_body_about_edge` absorbs a zero angle for exactly this reason, since
`setToRotation(0, axis, origin)` builds the identity and Fusion refuses the move with
`RuntimeError: 3 : invalid transform`.

**What the proof checks.** That the driving angle is exactly `π / N` radians, that the pinion's phase
is zero and therefore no move is emitted at all, that the rotation changes nothing about the body but
its position, and that the body's own section really moved by half a pitch. The two bodies are laid
apart because half a tooth pitch leaves the turned body overlapping where it started.

**From:** `spec/bevelgear/instructions.md` L355–357 L757, `.claude/skills/generate-gear/PLAYBOOK.md` L178–179 L791–800

## S32 `[PROSE]` Move the finished bodies into the gear component

Relocate this gear's finished body into its `{gearLabel} Gear` component with
`body.moveToComponent(gearOccurrence)`. `moveToComponent` preserves world position and needs no
activation (`[PB-NO-CROSS-SIBLING]`). All the feature operations above ran in the single Design
component precisely so no cross-sibling sketch or `project` reference is ever needed; the visible end
state is identical.

The pinion is built and moved first, the driving gear second, each through the whole of S08 to S32
before the next begins — profile and body are **interleaved per gear** (pinion profile → pinion body →
driving profile → driving body), **not** both profiles and then both bodies.

**From:** `spec/bevelgear/instructions.md` L328–343 L386–387 L713

## S33 `[PROSE]` Cleanup

Call the framework helper:

```python
hide_construction_geometry(bevelComponent)
```

It recursively walks the Bevel Gear component tree, dedupes by `entityToken`, and hides every sketch,
construction plane and construction axis with `isLightBulbOn = False`. Construction planes and axes
are **not** hidden by `isVisible` (`[PB-HIDE-AFTER-USE]`, `[BEVEL-F-CLEANUP]`): `isVisible = False`
hides **sketches**, `isLightBulbOn = False` hides **construction planes and axes** — do not cross them.
There is no sketch-only mode and no per-mode guard; bevel always builds solids. Leave only the two
finished gear bodies visible.

The driving gear's half-tooth-pitch **meshing rotation** is performed earlier, at S31, in the Design
component before the body is moved out; it is not a cleanup step.

**Nothing in this module settles the sketch display.** `commands/_gear_command.py` calls
`geargen.settle_sketch_display` once after `generate()` returns, and every gear command runs through
that one call, so a generator must not add its own (`[PB-SETTLE-DISPLAY]`). Fusion's browser shows a
stale constraint icon for every sketch a generator authors until something makes it settle; the icon
is not evidence, and nothing about it belongs in a generated module or in a step list.

**From:** `spec/bevelgear/instructions.md` L761–766, `spec/bevelgear/fusion.md` L161–166, `.claude/skills/generate-gear/PLAYBOOK.md` L525–546 L650–662 L826–828
