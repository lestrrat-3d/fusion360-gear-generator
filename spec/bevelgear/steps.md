# Bevel gear — compiled step list

The proof for this step list is `proof/bevelgear/geometry_test.go`, `proof/bevelgear/cases_test.go`,
`proof/bevelgear/bounds_test.go`, `proof/bevelgear/lattice_test.go`, `proof/bevelgear/sketches_test.go`,
`proof/bevelgear/solids_test.go`, `proof/bevelgear/spiral_test.go` and the generated
`proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `6cb50bdb875e150a1504b22f13126f4d672c6eca` |
| `spec/bevelgear/fusion.md` | `40d165fbc2f47ffba45d7c3c0f73ca67ec488d42` |
| `spec/bevelgear/spiral-tooth-trace.md` | `c9ec08561ced7975aa0ed9ad6a330186259c0d08` |
| `spec/spurgear/instructions.md` | `486f78e9844f07a1ab7ebf4af110260aafac6c99` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `1b3078d6767d6a3f56c228e1e934c82ccfbf53fe` |

## S1 `[PROSE]` Module layout, imports, classes and constants

Write `lib/geargen/bevelgear.py`. Import explicitly — no `import *` in a gear module:

```python
import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import get_boolean, get_selection
from .utilities import find_profile_by_curve_counts
from .solids import (cut_conical_ends, slice_body_by_offset_planes, rotate_body_about_edge,
                     plane_by_angle, combine_point, circle_intersect_nearest,
                     hide_construction_geometry)
from .spurgear import SpurGearInvoluteToothDesignGenerator
from .spurproxy import VirtualSpurProxy
```

From `base.py` import **only** `get_selection` and `get_boolean`. Bevel uses a **standalone
generator**: it does not subclass `base.Generator`, has no `GenerationContext`, and registers no
Fusion user parameters — every value is precomputed in Python in internal cm and written into
geometry numerically (`[PB-PRECOMPUTED-MODE]`). There are therefore no `PARAM_*` name strings.

Two classes plus the imported proxy:

1. `BevelGearCommandInputsConfigurator` — `@classmethod def configure(cls, cmd)`,
   `@classmethod def handle_input_changed(cls, args)` and the private
   `@classmethod def _updateSpiralInputVisibility(cls, inputs)`.
2. `BevelGearGenerator` — `__init__(self, design)` storing `self.design` and
   `self.bevelOccurrence = None`; `generate(inputs)`; `deleteComponent()`.
3. `VirtualSpurProxy` is **imported from the framework**; do not define a local proxy or
   value-wrapper class.

`configure`, `handle_input_changed`, `generate` and `deleteComponent` are members this module
DEFINES for `commands/bevelgear/entry.py` and the shared `GearCommand` to call; they are not calls
this module makes.

<!-- check-step-calls: ignore configure handle_input_changed generate deleteComponent -->
<!-- check-compile: ignore handle_input_changed -->

Module-level constants, exactly these and no others:

```
INPUT_ID_PLANE = 'targetPlane'
INPUT_ID_CENTER_POINT = 'centerPoint'
INPUT_ID_PARENT = 'parentComponent'
INPUT_ID_MODULE = 'module'
INPUT_ID_SHAFT_ANGLE = 'shaftAngle'
INPUT_ID_DRIVING_TEETH = 'drivingTeeth'
INPUT_ID_PINION_TEETH = 'pinionTeeth'
INPUT_ID_DRIVING_BASE_HEIGHT = 'drivingBaseHeight'
INPUT_ID_PINION_BASE_HEIGHT = 'pinionBaseHeight'
INPUT_ID_BORE_ENABLE = 'boreEnable'
INPUT_ID_DRIVING_BORE = 'drivingBore'
INPUT_ID_PINION_BORE = 'pinionBore'
INPUT_ID_FACE_WIDTH = 'faceWidth'
INPUT_ID_TOOTH_SPACING = 'toothSpacing'
INPUT_ID_SPIRAL_ANGLE = 'spiralAngle'
INPUT_ID_HAND = 'spiralHand'
INPUT_ID_CUTTER_RADIUS = 'cutterRadius'
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'
_CROWN_PER_RAD = 0.5
_PINION_MESH_PHASE_TEETH = 0
```

Get the submodule right on every `adsk.*` name (`[PB-ADSK-MODULES]`): `Point3D`, `Vector3D`,
`Matrix3D`, `ObjectCollection`, `ValueInput` and `SelectionCommandInput` live in **`adsk.core`**;
`DimensionOrientations`, `FeatureOperations` and `Path` live in **`adsk.fusion`**. The wrong module
is a runtime `AttributeError`, not a parse error, because `adsk` is native and unimportable outside
Fusion. Look the name up rather than guessing it (`[PB-API-LOOKUP]`).

Because every value is written numerically, editing anything afterwards does not update an existing
gear — regenerate (`[PB-NUMERIC-SNAPSHOT]`). Bevel does not opt into the live
`validate_inputs` hook (`[PB-VALIDATE-INPUTS]`): its rejections are raised at execute time from
`_readInputs`.

`__init__` and `_updateSpiralInputVisibility` are members this module DEFINES, and `import` is a
Python keyword in the parenthesised import list — none of the three is a call the module makes.

<!-- check-step-calls: ignore __init__ import _updateSpiralInputVisibility -->

**From:** `spec/bevelgear/instructions.md` L128-L151 (Architecture), L165-L177 (Exact input ids),
L402-L430 (Dependencies), L647 (`_CROWN_PER_RAD`), L335-L336 (`_PINION_MESH_PHASE_TEETH`);
`.claude/skills/generate-gear/PLAYBOOK.md` L17-L41 (`Module layout & imports`), L819-L828
(`[PB-PRECOMPUTED-MODE]`), L220-L228 (`[PB-NUMERIC-SNAPSHOT]`), L317-L344
(`[PB-VALIDATE-INPUTS]`), L432-L435 (`[PB-API-LOOKUP]`, `[PB-ADSK-MODULES]`).

---

## S2 `[PROSE]` Command dialog — `configure()` adds the 17 inputs

`BevelGearCommandInputsConfigurator.configure(cls, cmd)` reads `inputs = cmd.commandInputs` and adds
these inputs **in this order**. Target Plane is first so Fusion's auto-focus lands on it
(`[PB-AUTOFOCUS-FIRST]`); Center Point follows; Parent Component is third and pre-selected.

| # | Dialog label | input id | call | unit | default | filters / limits / tooltip |
|---|---|---|---|---|---|---|
| 1 | `Target Plane` | `targetPlane` | `addSelectionInput` | — | — | filters `ConstructionPlanes`, `PlanarFaces`; `setSelectionLimits(1, 1)`; tooltip `Plane the bottom of the driving gear sits flush against` |
| 2 | `Center Point` | `centerPoint` | `addSelectionInput` | — | — | filters `ConstructionPoints`, `SketchPoints`; `setSelectionLimits(1, 1)`; tooltip `Point the driving bevel gear is centered on` |
| 3 | `Parent Component` | `parentComponent` | `addSelectionInput` | — | root component pre-selected | filters `Occurrences`, `RootComponents`; `setSelectionLimits(1, 1)`; tooltip `Component the gear pair is created under` |
| 4 | `Module` | `module` | `addValueInput` | `''` | `adsk.core.ValueInput.createByReal(1)` | — |
| 5 | `Shaft Angle` | `shaftAngle` | `addValueInput` | `deg` | `adsk.core.ValueInput.createByString('90 deg')` | — |
| 6 | `Driving Gear Teeth` | `drivingTeeth` | `addValueInput` | `''` | `adsk.core.ValueInput.createByReal(31)` | — |
| 7 | `Pinion Gear Teeth` | `pinionTeeth` | `addValueInput` | `''` | `adsk.core.ValueInput.createByReal(31)` | — |
| 8 | `Driving Gear Base Height` | `drivingBaseHeight` | `addValueInput` | `mm` | `adsk.core.ValueInput.createByReal(to_cm(0))` | — |
| 9 | `Pinion Gear Base Height` | `pinionBaseHeight` | `addValueInput` | `mm` | `adsk.core.ValueInput.createByReal(to_cm(0))` | — |
| 10 | `Enable Bore` | `boreEnable` | `addBoolValueInput` (checkbox) | — | `True` | — |
| 11 | `Driving Gear Bore Diameter` | `drivingBore` | `addValueInput` | `mm` | `adsk.core.ValueInput.createByReal(to_cm(0))` | — |
| 12 | `Pinion Gear Bore Diameter` | `pinionBore` | `addValueInput` | `mm` | `adsk.core.ValueInput.createByReal(to_cm(0))` | — |
| 13 | `Face Width` | `faceWidth` | `addValueInput` | `mm` | `adsk.core.ValueInput.createByReal(to_cm(0))` | — |
| 14 | `Tooth Spacing` | `toothSpacing` | `addValueInput` | `mm` | `adsk.core.ValueInput.createByReal(to_cm(0))` | — |
| 15 | `Mean Spiral Angle` | `spiralAngle` | `addValueInput` | `deg` | `adsk.core.ValueInput.createByString('35 deg')` | — |
| 16 | `Hand of Spiral` | `spiralHand` | `addDropDownCommandInput` | — | items `Right` selected, `Left` unselected | style `adsk.core.DropDownStyles.TextListDropDownStyle` |
| 17 | `Cutter Radius` | `cutterRadius` | `addValueInput` | `mm` | `adsk.core.ValueInput.createByReal(to_cm(0))` | — |

Signatures: `inputs.addSelectionInput(id, name, commandPrompt)` — the third argument is the tooltip
string in the table, reproduced verbatim; `inputs.addValueInput(id, name, unitType, initialValue)`;
`inputs.addBoolValueInput(id, name, True, '', True)`;
`inputs.addDropDownCommandInput(id, name, adsk.core.DropDownStyles.TextListDropDownStyle)` then
`dropdown.listItems.add(_HAND_RIGHT, True)` and `dropdown.listItems.add(_HAND_LEFT, False)`.

Selection filters are written as the named constants, never quoted literals
(`[PB-SELECTION-FILTER-ENUM]`): `adsk.core.SelectionCommandInput.ConstructionPlanes`,
`adsk.core.SelectionCommandInput.PlanarFaces`, `adsk.core.SelectionCommandInput.ConstructionPoints`,
`adsk.core.SelectionCommandInput.SketchPoints`, `adsk.core.SelectionCommandInput.Occurrences`,
`adsk.core.SelectionCommandInput.RootComponents`. The filter parameter is typed `str` and the
constant's value is that same string, so the constant is not a type requirement — it is checked for
typos at import, reads as the API's own vocabulary, and survives a renamed filter, where a literal
fails silently by selecting nothing.

The Parent selection pre-selects the root component: `parentInput.addSelection(get_design().rootComponent)`.

The `mm` and `deg` defaults are passed in **internal units** (`[PB-DIALOG-DEFAULT-UNITS]`): `to_cm(...)`
for lengths, and `createByString('90 deg')` / `createByString('35 deg')` for the two angles so the
expression engine parses them.

`configure()` calls `cls._updateSpiralInputVisibility(inputs)` as its **last** step.

`configure` is the classmethod this module DEFINES for `commands/bevelgear/entry.py` to call; it
is not a call this module makes.

<!-- check-step-calls: ignore configure BevelGearCommandInputsConfigurator.configure -->

**From:** `spec/bevelgear/instructions.md` L27-L36 (input order), L165-L196 (the input table and the
constant list), L201-L207 (filters, tooltips, pre-selection, default units), L63-L64 (`configure`
calls the visibility helper last); `.claude/skills/generate-gear/PLAYBOOK.md` L346-L348
(`[PB-AUTOFOCUS-FIRST]`), L128-L136 (`[PB-DIALOG-DEFAULT-UNITS]`), L138-L143
(`[PB-SELECTION-DECL]`), L524-L535 (`[PB-SELECTION-FILTER-ENUM]`).

---

## S3 `[PROSE]` Conditional visibility of the spiral-only inputs

Hand of Spiral and Cutter Radius are relevant only to a curved bevel, so they are **hidden whenever
ψ = 0 and shown when ψ > 0**. Mean Spiral Angle itself is the controller and is always visible.
There is no declarative show-if in the Fusion API; use the `commandInput.isVisible` property.

`@classmethod def _updateSpiralInputVisibility(cls, inputs)`:

- resolve `spiral = inputs.itemById(INPUT_ID_SPIRAL_ANGLE)`,
  `hand = inputs.itemById(INPUT_ID_HAND)`, `cutter = inputs.itemById(INPUT_ID_CUTTER_RADIUS)`;
  if any is `None`, return early;
- evaluate the spiral input's **`.expression`** with
  `get_design().unitsManager.evaluateExpression(spiral.expression, 'rad')` — internal radians; it
  does **not** read the input's `.value`;
- wrap that evaluation in `try/except`, since a half-typed expression can raise mid-edit, and on
  failure leave both inputs **shown**;
- set `hand.isVisible = (value > 0)` and `cutter.isVisible = (value > 0)`.

`@classmethod def handle_input_changed(cls, args)` is one line:
`cls._updateSpiralInputVisibility(args.inputs)` — recompute on every input change; do not branch on
which input changed. It is bound by name from `commands/bevelgear/entry.py` as the `GearCommand`'s
`input_changed` callback.

`isVisible` only hides the dialog row. The input still exists, `_readInputs` reads it normally, and
a ψ = 0 build ignores Hand and Cutter Radius anyway, so hiding is cosmetic and cannot affect
generation.

`handle_input_changed` is the classmethod this module DEFINES for the dialog's `inputChanged`
event to call. The module never calls it itself.

<!-- check-step-calls: ignore handle_input_changed -->
<!-- check-compile: ignore handle_input_changed -->

**From:** `spec/bevelgear/instructions.md` L169-L188 (Conditional visibility), L296-L302 (external
bindings); `.claude/skills/generate-gear/PLAYBOOK.md` L282-L315 (command-entry wiring).

---

## S4 `[PROSE]` `_readInputs` — read every input, in internal units, and range-check it

`generate(inputs)` calls `_readInputs(inputs)` **first**, before anything creates an occurrence.
`_readInputs` returns the 7-tuple `(parentComponent, targetPlane, centerPoint, module,
drivingTeeth, pinionTeeth, shaftAngle_deg)` and stashes the rest on `self`:
`self._drivingBaseHeight_cm`, `self._pinionBaseHeight_cm`, `self._boreEnable`,
`self._drivingBore_cm`, `self._pinionBore_cm`, `self._faceWidth_cm`, `self._toothSpacing_cm`,
`self._spiralAngle_rad`, `self._hand`, `self._cutterRadius_cm`.

**How each input is read.** Selections with `get_selection(inputs, id)`; the checkbox with
`get_boolean(inputs, INPUT_ID_BORE_ENABLE)`; the dropdown with
`inputs.itemById(INPUT_ID_HAND).selectedItem`, taking `.name` and defaulting to `_HAND_RIGHT` when
it is `None`. Every numeric and angle input is read with
`get_design().unitsManager.evaluateExpression(<input>.expression, <units>)`, with `<units>` the
input's own unit string (`''`, `'mm'` or `'deg'`) (`[PB-EVAL-EXPRESSION]`). Do **not** call
`get_value` on the checkbox: `BoolValueCommandInput` has no `.expression` and Fusion raises
`AttributeError` (`[PB-INPUT-READ]`).

<!-- check-step-calls: ignore get_value -->

**Units — the one trap.** `evaluateExpression` always returns Fusion internal units regardless of
the unit string, so the `mm` inputs come back in **cm** and the `deg` inputs in **radians**. Use
them as-is; do **not** `to_cm` them again. `Module` is read with unit `''`, so it comes back as a
raw number meaning **millimetres**, and therefore **every length derived from Module must be
`to_cm`-converted before it touches geometry**: the pitch diameters, the Cone Distance, the dedendum
`1.25 * Module`, the module-length construction extensions, and the default Face Width. Mixing a
raw-mm Module-derived length with an already-cm `mm` input makes the gear come out about ten times
off and the Face-Width bound meaningless. Convert the two angles with `math.degrees(...)` before any
degree-range check.

Both tooth counts are coerced with `int(round(...))` before validation.

Read every **selection** and stash it before anything creates an occurrence
(`[PB-SELECTION-STASH]`): a selection entity is dropped when Fusion's active component context
shifts. Bevel creates no user parameters, so nothing shifts the context during this pass — keep the
order anyway so it stays that way.

**Range checks in this pass:** `Module > 0`; both tooth counts `>= 3`; the Shaft Angle at least 30°;
the base heights, bore diameters, Face Width and Tooth Spacing non-negative; the Mean Spiral Angle
in `[0, 60)`; the Cutter Radius non-negative. Each rejection message names the offending input and
the numeric bound it broke. The COMPUTED bounds — the Maximum Shaft Angle, the Minimum Teeth floor,
each base-height window and the Maximum Face Width — and the fallbacks they cap or raise are S5,
and they run in the same pass, in the order S5 fixes.

`generate` is the method this module DEFINES for the entry point to call, `_readInputs` is a private
helper whose spelling the spec lets vary, and `int` and `round` are Python builtins.

<!-- check-step-calls: ignore generate _readInputs int round -->

**From:** `spec/bevelgear/instructions.md` L216-L234 (reading and units), L153-L161, L280-L294
(Generation Context — the 7-tuple and the stashed attributes), L100-L110 (bores and Enable Bore),
L126 (Mean Spiral Angle range), L130 (Cutter Radius);
`.claude/skills/generate-gear/PLAYBOOK.md` L103-L118 (`[PB-INPUT-READ]`), L830-L834
(`[PB-EVAL-EXPRESSION]`), L196-L201, L624-L625 (`[PB-SELECTION-STASH]`).

---

## S5 `[GO]` Resolve the derived values and the input bounds, in this order

Proof function: `stepResolveInputBounds`.

<!-- proof-run: proofkit.Run(boundsCases, stepResolveInputBounds) -->

All of this is closed form and resolves during input validation, before any geometry exists. Work in
whatever units each formula states and convert per S4; the proof works in millimetres throughout.

**Pitch diameters.** `Driving Gear Pitch Diameter = Module * Driving Gear Teeth Number` and
`Pinion Gear Pitch Diameter = Module * Pinion Gear Teeth Number`. Written `DPD` and `PPD` below.

**Cone Distance** (the diagonal, not the pitch cone distance):
`Cone Distance = sqrt((Module * Driving Gear Teeth Number)**2 + (Module * Pinion Gear Teeth Number)**2)`.
It depends on the two tooth counts only, never on the Shaft Angle. `R`, the **Pitch Cone Distance**,
is a different length: `R = (PPD / 2) / sin(γ_p)`. The two coincide as `Cone Distance = 2 * R`
exactly when the Shaft Angle is 90° and diverge everywhere else. Where a step below says "Cone
Distance" it means the diagonal; `R` is always written `R`.

**Maximum Shaft Angle.** Both pitch cone angles stay below 90° exactly while

```
cos(Shaft Angle) > -min(DPD, PPD) / max(DPD, PPD)
```

so the limit is `degrees(acos(-min(DPD, PPD) / max(DPD, PPD)))`, **capped at 150°**. The cone-angle
half is **exclusive** (`acos` is a hard singularity there) and the 150° half is inclusive. Reject a
Shaft Angle at or above the cone-angle limit, or above 150°, naming the computed limit. A 31/17 pair
gives `acos(-17/31) = 123.24°`. Equal tooth counts give `acos(-1) = 180°`, which is no constraint at
all, so the 150° cap is what binds there. Check this **after** both tooth counts are read and
coerced, since it depends on both.

**Pitch cone angles.** `tan γ_p = sin Σ * PPD / (DPD + PPD * cos Σ)` and `γ_g = Σ − γ_p`, with Σ the
Shaft Angle in radians. Compute them once, then, **in this order**:

1. **Minimum Teeth**, per gear with that gear's own γ: `teeth >= 5.27 * cos γ`, on top of the
   blanket `teeth >= 3`. The constant is `2 * (1.05 * 1.25 / 0.95 + 1.25)`. Name the computed floor
   in the rejection. At Shaft Angle 90° the floor is 3.72, i.e. 4 teeth: measured, an equal 4-tooth
   pair solves and a 3-tooth pair still fails on the heel edge. Keep `teeth >= 3` as the absolute
   floor and apply this computed floor on top of it.
2. **Base heights**, per gear, with `r` that gear's pitch radius and γ its own pitch cone angle:
   - `Minimum Base Height = 1.05 * 1.25 * Module * sin γ`
   - `Maximum Base Height = 0.95 * (r - 1.25 * Module * cos γ) * tan γ`
   - The driving fallback, when the input is 0, is `Module * Driving Gear Teeth Number / 8`.
   - The pinion fallback, when the input is 0, is the **RESOLVED** driving base height —
     after its own fallback and after the driving Maximum Base Height capped it — times
     `Pinion Gear Teeth Number / Driving Gear Teeth Number`. Then apply the **pinion's own**
     Minimum and Maximum Base Height to the result: the two gears share no pitch cone angle
     unless the tooth counts are equal.
   - Apply in both directions: raise a fallback that falls below the minimum, cap one that
     exceeds the maximum, and **reject** a user value outside either end naming the bound it broke.

   The Minimum Teeth check runs first because it is exactly the statement that the base-height
   window is non-empty, so step 2 never has to describe what to do when the minimum exceeds the
   maximum.

   Read the base height's origin carefully: it is the offset dimension between the A→Apex2 drop and
   G→H, so it is measured from **Apex 2's plane**, not from the dedendum point. Walking out along
   the dedendum line from Apex 2, the perpendicular distance to the shaft axis falls at `cos γ` per
   unit, so the heel point reaches the axis at `r * tan γ` — the true crossing. The Maximum Base
   Height above sits `1.25 * Module * sin γ` below that crossing and is therefore deliberately
   conservative, refusing a band of base heights that would in fact still build. Past the true
   crossing the profile has crossed its own axis of revolution and the revolve fails with
   `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`). Do not adopt the exact bound `0.95 * r * tan γ` without
   re-running the low-tooth-count cases.

**Bore diameters.** Only consulted when Enable Bore is checked, in which case a value of 0 means
auto: use that gear's `Pitch Diameter / 4`. When Enable Bore is unchecked, no bore is cut on either
gear and both bore inputs are ignored.

**Face Width** is resolved in §2 (S10), because its cap needs solved sketch geometry. Its default is
`Cone Distance / 6`.

**Hand.** `handSign = +1` for `Right`, `−1` for `Left`, read for the **driving** gear; the pinion is
built with the opposite hand.

**Cutter radius.** 0 means auto — use the mean cone distance `R_mean` as `r_c`.

**Three different behaviours, and they are not interchangeable.** A FALLBACK below a minimum is
RAISED to it and one above a maximum is CAPPED to it; a USER value outside either end is
REJECTED, naming the bound it broke, rather than being clamped; and a user value inside its
window is used exactly as given. The proof checks each of the three on every case, together with
the worked numbers this section publishes — the 31/31 pair at Σ = 30° with its 3.638 mm cap,
4.153 mm true crossing and 3.875 mm fallback, and the 31/17 pair's cone-angle limit — and then
draws the frustum hexagon the resolved values produce, since keeping that profile off its own
axis of revolution is what the whole bounds system is for.

A configuration can satisfy every bound here and still be refused by the sketch solver as
near-singular. That is a fact about the particular constraint net, not a validation rule: no bound
here is derived from a conditioning measurement, and a near-singular report is a real refusal of that
construction, never a tolerance to loosen.

Every name in the formulas above is the arithmetic itself — `math` module functions and Python
builtins — rather than a Fusion call.

<!-- check-step-calls: ignore acos cos degrees max min sin sqrt -->

**From:** `spec/bevelgear/instructions.md` L37-L52 (Shaft Angle and its maximum), L58-L60 (pitch
diameters), L66-L98 (base-height bounds, the true crossing, Minimum Teeth), L100-L114 (bores, Face
Width, Cone Distance vs `R`), L127-L130 (hand, cutter radius), L232-L245 (the resolve order),
L478 (the closed form for γ_p and γ_g); `.claude/skills/generate-gear/PLAYBOOK.md` L684-L690
(`[PB-REVOLVE]`).

---

## S6 `[PROSE]` `generate()` orchestration and `deleteComponent()`

```
generate(inputs)
  -> _readInputs(inputs)                       # S4, S5
  -> resolve pitch diameters and bores in Python, in cm
  -> build the occurrence tree                 # S7
  -> _buildAnchorSketch(design, plane, center)  # S8
  -> _buildGearProfiles(...)                    # S9, S10, then PER GEAR:
        pinion profile -> pinion body -> driving profile -> driving body
        -> _buildVirtualSpurProfile(...)        # S11, S12, S13
        -> _createGearBody(...)                 # S14 .. S33
              -> _transformToothBody(...)       # S18 when psi = 0, S19 .. S27 when psi > 0
  -> _hideConstructionGeometry(bevelComponent)  # S34
deleteComponent()                               # error rollback
```

The order is load-bearing: read every input first, then build the tree, then build geometry. Because
bevel registers no user parameters, nothing creates an occurrence until every selection is already
read, so the selection-context-shift hazard does not bite — keep the order so it stays that way.

`_buildGearProfiles` runs **per gear, interleaved**: the pinion's §3 profile and its whole body,
then the driving gear's — never both profiles followed by both bodies.

The per-gear geometric anchors are carried in **plain per-gear dicts** (`pinionCtx` / `drivingCtx`)
holding this gear's label, teeth, pitch diameter, γ, tooth-centre point and reference line, hexagon
vertices, shaft-edge point pair, toe and heel edges, toe and heel cone points, root axis, bore
diameter and mesh angle. `_buildVirtualSpurProfile` and `_createGearBody` write the tooth sketch,
tooth plane, `embedded` flag and virtual tooth count back into the same dict. Shared anchors are
self-stashed: `self._gearProfilesPlane`, `self._apexSketchPoint`, `self._gpSketch`, `self._apex2d`,
`self._anchorCenterPoint`, and the derived `self._coneDistance_cm`, `self._gamma_p`, `self._gamma_g`,
`self._faceWidthResolved_cm`. That class-level shape IS the intended structure — do not introduce a
`GenerationContext`-style class.

`self.bevelOccurrence` holds the top occurrence for cleanup; `self.designOccurrence`,
`self.designComponent` and `self.bevelComponent` hold the inner tree. `deleteComponent()` calls
`deleteMe()` on `self.bevelOccurrence` and is called by the entry point on any exception.

`_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane, gearLabel,
teethNumber, gamma)` is the single tooth-body hook `_createGearBody` calls after lofting the uncut
tooth. Its first line is the gate `if self._spiralAngle_rad <= 0: return cut_conical_ends(...)`.

`_pinionMeshPhase(pinionTeeth)` returns `_PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth`, in
radians.

Use `futil.log(...)` for step progress; let the entry point's `try/except` plus `deleteComponent()`
handle rollback rather than swallowing errors mid-step.

`generate` and `deleteComponent` are the two methods this module DEFINES for the entry point to
call. The underscore-prefixed names are the intended decomposition and nothing more: the spec
says private helper names may vary, so the boundaries are the requirement and the spellings are
not. `deleteMe` sits inside `deleteComponent`, on the error-rollback path the entry point takes
on an exception, so no executable path from the build reaches it.

<!-- check-step-calls: ignore generate deleteComponent deleteMe _readInputs _buildAnchorSketch _buildGearProfiles _buildVirtualSpurProfile _createGearBody _hideConstructionGeometry _pinionMeshPhase _transformToothBody _cutBore -->

**From:** `spec/bevelgear/instructions.md` L296-L351 (Method contract and call graph), L280-L294
(Generation Context), L390-L400 (Generation Order);
`.claude/skills/generate-gear/PLAYBOOK.md` L671-L673 (`[PB-LOGGING]`), L802-L804
(`[PB-TREE-CLEANUP]`).

---

## S7 `[PROSE]` Occurrence tree — `Bevel Gear` and `Design`

Create each occurrence with `parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` and
name it through `occurrence.component.name` (`[PB-OCCURRENCE-TREE]`).

- `Bevel Gear` under the user's Parent Component.
- `Design` under `Bevel Gear`. It holds every sketch, construction plane and construction axis, and
  every feature operation runs in it.

**Never call `occurrence.activate()`** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`). Bevel's
own reason: the Anchor sketch is created on the user's **external**, root-owned target plane, and an
activated occurrence resolves that external plane in its own local frame, so the build collapses onto
world XY regardless of the real plane's tilt. Building everything in the one Design component also
avoids every cross-sibling reference, which Fusion rejects outright (`[PB-NO-CROSS-SIBLING]`). The
sole exception is the spiral crown's scale feature (S32).

<!-- check-step-calls: ignore activate -->

**From:** `spec/bevelgear/instructions.md` L21-L23, L434-L442 (component setup), L387-L388 (never
activate); `spec/bevelgear/fusion.md` L149-L154 (`[BEVEL-F-NEVER-ACTIVATE]`);
`.claude/skills/generate-gear/PLAYBOOK.md` L784-L801 (`[PB-OCCURRENCE-TREE]`,
`[PB-NEVER-ACTIVATE]`, `[PB-NO-CROSS-SIBLING]`).

---

## S8 `[GO]` Anchor sketch

Proof function: `stepAnchorSketch`.

<!-- proof-run: proofkit.Run(anchorCases, stepAnchorSketch) -->

Create the sketch with `designComponent.sketches.add(targetPlane)` — **directly on the user-selected
target plane**, whether that selection is a `ConstructionPlane` or a `PlanarFace`. Do not re-derive
or offset it: a coplanar construction plane built inside the Design component resolves in that
component's own frame and silently loses the selected plane's world orientation, collapsing the whole
build onto world XY (`[PB-USE-SELECTED-PLANE]`). Name the sketch `Anchor`.

Mark the centre by projecting the user's Center Point into the sketch: `sketch.project(centerPoint)`,
taking the first returned entity as the projected centre `SketchPoint`.

**Write the call as `sketch.project(entity)`, and do not substitute `project2`.** The compiled Fusion
API reference declares `project2(entities, isLinked)` and no `project`, so every gate in this repo
reports the call as unverified; that report is expected and is not a defect to fix here. `project` is
what the shipped add-ins call and what the spur step list names, and it sits on `fusion_api.py`'s
`UNVERIFIED_CALLS`, which is reported, not blocking, and explicitly not waived. The two are not
interchangeable in any case: `project2` takes a list and returns a list, so swapping the name alone
would be wrong.

<!-- check-step-calls: ignore project2 -->

Draw the Anchor Line with `sketch.sketchCurves.sketchLines.addByTwoPoints(...)` from raw
`adsk.core.Point3D.create(...)` coordinates, its two endpoints seeded at exactly **±0.5 cm** from the
projected centre along the sketch-local X axis, so the seeded length is 10 mm. Then:

- `sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)` — the intersection, which
  pins the centre onto the line;
- `sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)` — the centre bisects the
  line. Use **both**, not midpoint alone;
- `sketch.sketchDimensions.addDistanceDimension(anchorLine.startSketchPoint,
  anchorLine.endSketchPoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation,
  textPoint)` — **without** assigning `.parameter.value`. The dimension simply locks the length at
  the seeded 10 mm; the value is arbitrary because this is only a reference line;
- `sketch.geometricConstraints.addHorizontal(anchorLine)` — sketch-local, per
  `[PB-REFLINE-DIRECTION]`. It works on any tilted target plane; a world-axis lock would mis-orient
  the figure.

The line's absolute direction is arbitrary — nothing downstream depends on it, because §2 derives
every direction relative to the projected anchor line — but it must not be a free degree of freedom.
With midpoint, length and Horizontal the line has zero DOF.

**Stash the projected-centre `SketchPoint`** on `self` (as `self._anchorCenterPoint`) so §2
re-projects *this* point rather than the raw user-selected one.

Gate the sketch at the end of the step: `if not sketch.isFullyConstrained: raise ...` naming the
sketch (`[BEVEL-F-FULL-CONSTRAINT]`, `[PB-FULL-CONSTRAINT]`). A free DOF here is a generation defect,
not a warning.

The proof substitutes two things and says so at the call site: the engine's Midpoint already carries
both rows the coincident's point-on-line row implies, so it emits the midpoint alone; and the
engine's aligned length dimension is unsigned, which admits the end-for-end flip, so the proof writes
the length as the signed x span the Horizontal constraint has just made equal to it.

**From:** `spec/bevelgear/instructions.md` L444-L448; `spec/bevelgear/fusion.md` L21-L30
(`[BEVEL-F-FULL-CONSTRAINT]`); `.claude/skills/generate-gear/PLAYBOOK.md` L438-L447
(`[PB-FULL-CONSTRAINT]`, `[PB-REFLINE-DIRECTION]`), L506-L513 (`[PB-SKETCHCURVES]`), L805-L815
(`[PB-USE-SELECTED-PLANE]`), L602-L603 (`[PB-DRIVING-DIM]`).

---

## S9 `[PROSE]` Gear Profiles Plane

Build a construction plane through the Anchor Line at 90° to the target plane:

```
planeInput = designComponent.constructionPlanes.createInput()
planeInput.setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
gearProfilesPlane.name = 'Gear Profiles Plane'
```

Pass the `SketchLine` **directly** to `setByAngle`; never wrap it in `adsk.fusion.Path.create` first
(`[PB-CONSTRUCTION-PLANES]`). Build it off the **original `targetPlane`** as the reference — this is
the other place the target-plane orientation reaches the bodies, and substituting a different plane
here also collapses the gear onto XY (`[PB-USE-SELECTED-PLANE]`).

<!-- check-step-calls: ignore Path.create -->

Stash the plane as `self._gearProfilesPlane`.

**From:** `spec/bevelgear/instructions.md` L450-L452;
`.claude/skills/generate-gear/PLAYBOOK.md` L742-L753 (`[PB-CONSTRUCTION-PLANES]`), L805-L815
(`[PB-USE-SELECTED-PLANE]`).

---

## S10 `[GO]` Gear Profiles sketch — the §2 lattice

Proof function: `stepGearProfiles`.

<!-- proof-run: proofkit.Run(latticeCases, stepGearProfiles) -->

`sketch = designComponent.sketches.add(gearProfilesPlane)`, named `Gear Profiles`. Stash it as
`self._gpSketch`. One sketch holds the whole lattice for both gears.

**Three rules govern every line in this step.**

1. **Every line is a construction line**: set `line.isConstruction = True` on the lattice lines, the
   toe lines M→N and O→P, and the short reference and connector lines M→C, N→A, O→D, P→B, A→G, B→I,
   C→K/K′, D→L/L′ alike. The solid features consume only the per-gear Profile sketches, never a §2
   curve.
2. **Every line uses the COINCIDENT style, never sharing** (`[BEVEL-F-COINCIDENT-STYLE]`, a stricter
   delta to `[PB-SHARE-XOR-COINCIDENT]`). Create the line from raw
   `adsk.core.Point3D.create(x, y, 0)` coordinates for BOTH endpoints and pin each endpoint that
   meets an existing point with exactly one
   `sketch.geometricConstraints.addCoincident(line.startSketchPoint, existingPoint)` (or
   `endSketchPoint`). Never pass an existing `SketchPoint` into `addByTwoPoints` to share it.
   Sharing without a coincident leaves the sketch **under**-constrained and the gate fails on "Gear
   Profiles"; sharing **and also** coinciding is redundant and the solve fails outright with
   `VCS_SKETCH_SOLVING_FAILED - failed to create offset`. This covers the short reference and
   connector lines too — a regen that shared only those came out about fourteen coincidents short.
   No §2 line is exempt.
3. **Each named line is created ONCE and reused** (`[BEVEL-F-LINE-ONCE]`). The helper that creates a
   module-length extension must RETURN the line and the caller keeps that reference. Never draw a
   second line between the same two points just to obtain a handle: the duplicate carries its own
   constraints over the same segment, over-determines the coupled net, and the solve fails with
   `VCS_SKETCH_OVER_CONSTRAINTS`. A duplicate whose endpoints carry only per-end coincidents has
   been observed to solve and even pass the gate, so do not rely on the solver to catch one.

**Every length dimension in this sketch is
`adsk.fusion.DimensionOrientations.AlignedDimensionOrientation`.** The figure has no axis-aligned
line in it — the shaft axes sit at the Shaft Angle to each other and the whole lattice tilts with the
target plane — so `HorizontalDimensionOrientation` or `VerticalDimensionOrientation` would each
dimension a projection instead of a length. Wherever a step below says "a dimensional constraint with
length = X" it means
`sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
followed by `.parameter.value = <number in cm>`. The offset dimensions are a different call,
`sketch.sketchDimensions.addOffsetDimension(line, entityTwo, textPoint)`, which takes no orientation.

<!-- check-step-calls: ignore HorizontalDimensionOrientation VerticalDimensionOrientation -->

A helper that turns a 2-D coordinate into a `Point3D` **must tolerate both a raw `(x, y)` tuple and
an object with `.x`/`.y`** (`[PB-POINT-HELPER]`), because this step mixes seed tuples with solved
`.geometry` points and hands both to the same helper. A helper written only as
`Point3D.create(p.x, p.y, 0)` throws `AttributeError: 'tuple' object has no attribute 'x'` the moment
a seed reaches it — a pure runtime crash no parse or type check sees. Branch on the type, or
standardise every call site on one form.

**The whole figure's POSITION is computed in this sketch's own 2-D coordinates**
(`[BEVEL-F-APEX-LOCAL]`); never compute a §2 position from a world round-trip, which is the single
biggest source of the XY-collapse. The one permitted world use is reading the target-plane normal as
a *direction* to pick which side the gear grows on (`[BEVEL-F-GROW-SIDE]`) — a one-bit comparison,
not a position. Read that normal as `targetPlane.geometry.normal` for **both** selection kinds: a
`BRepFace`'s `geometry` and a `ConstructionPlane`'s `geometry` are each a `core.Plane` carrying
`.normal`.

Now the construction, in order.

**Project the anchor centre.** `sketch.project(self._anchorCenterPoint)` — the Anchor sketch's centre
`SketchPoint`, NOT the raw user-selected centre. Both are coincident, but projecting the
anchor-sketch point keeps the chain inside the Design component; projecting the raw external point is
a cross-component reference that can resolve inconsistently. Call the result `c`, and let `d` be the
projected anchor line's 2-D unit direction and `perp = (-d.y, d.x)` the in-plane perpendicular, its
sign chosen so it points **toward the target-plane normal**.

**centre → Apex.** A construction line from `c`, perpendicular to the projected anchor line
(`sketch.geometricConstraints.addPerpendicular(centerToApex, projectedAnchorLine)`). Seed its far end
— the **Apex** — at the 2-D point `c + perp * (R * cos γ_g + <resolved Driving Gear Base Height>)`.
Seed it at **that** distance, not at `Driving Gear Pitch Diameter`: the constraint net closes this
line at exactly that value, and for the default 31/31 pair at Shaft Angle 90° the old
pitch-diameter seed sat 11.6 mm past where the solve puts it (31 mm seeded against 19.375 mm solved),
which is a seed waiting to pick the wrong branch (`[PB-SEED-NEAR]`). Pin the start with exactly one
`addCoincident(centerToApex.startSketchPoint, projectedCenter)`. Add **no** length constraint on this
line.

**Driving Gear Shaft Axis, Apex → B.** From the Apex in the `-perp` direction, its far end seeded at
`apex - perp * (R * cos γ_g)`, which is `c + perp * (<resolved Driving Gear Base Height>)` — measured
from the Apex, not from `c`. The closure at Apex 2 drives `|Apex→B|` to `R * cos γ_g`, so B solves to
exactly one base height above `c`. Constrain it
`sketch.geometricConstraints.addParallel(drivingShaftAxis, centerToApex)`. **Do NOT use
`addVertical`**: that forces the line to the sketch's world-vertical, which is wrong on a tilted
target plane and over-constrains the figure. Pin the beginning to the Apex with a coincident. Do
**not** dimension the length.

<!-- check-step-calls: ignore addVertical -->

**Pinion Gear Shaft Axis, Apex → A.** The driving-shaft direction rotated about the Apex by the Shaft
Angle. Rotating has two senses and they place A on opposite sides; choosing wrong mirrors the whole
gear onto the wrong side of the target plane. **Select the sense this way: form both candidate A
positions — the driving direction rotated by +Shaft Angle and by −Shaft Angle — and keep the
candidate whose endpoint has the greater X coordinate in this sketch.** Compare the two X values and
take the larger; do not rotate one fixed sense and flip only when its X comes out negative, because
when both candidates have a positive X that shortcut keeps the wrong one. Call the chosen unit
direction `pinionDir`. Pin the beginning to the Apex with a coincident; do not dimension the length.

**The Shaft Angle dimension.**
`sketch.sketchDimensions.addAngularDimension(pinionShaftAxis, drivingShaftAxis, textPoint)` with
`.parameter.value = <Shaft Angle in radians>`. Place the text point **inside the Σ wedge** so it
measures Σ and not its supplement (`[PB-ANGULAR-DIM]`): use the interior bisector,
`apex + normalize(pinionDir + drivingDir) * (PPD / 4)`, with `drivingDir` the unit Apex→B direction.
The angular dimension fixes the magnitude only; the side is held by the seed rule above together with
the Apex 2 closure below.

**A → Apex 2, the PPD/2 drop.** From A, perpendicular to the **Pinion** Gear Shaft Axis, drawn toward
the side where Apex 2 will lie. ⚠️ Apex 2 sits in the interior wedge **between** the two shaft axes,
so this drop must point toward the OTHER shaft axis, at point B. Pick the perpendicular sense by the
sign of its dot product with the **A→B** direction, never against a generic "toward the anchor line"
reference. `addPerpendicular(aDrop, pinionShaftAxis)`, a dimensional constraint with
`length = Pinion Gear Pitch Diameter / 2`, and a coincident pinning its beginning to A.

**Naming convention used from here on:** "A→Apex2" always means **this perpendicular drop line**, not
the Apex→A shaft axis. The two share point A but are different lines. The same holds for "B→Apex2"
versus the Apex→B shaft axis.

**B → Apex 2, the DPD/2 drop.** From B, perpendicular to the **Driving** Gear Shaft Axis, toward the
other shaft axis at point A: pick the sense by the sign of its dot product with the **B→A**
direction. ⚠️ **Do NOT choose this sense by a "toward the anchor line" reference**, i.e. the `-perp`
grow direction: the Driving Gear Shaft Axis is itself parallel to that direction, so the
perpendicular's dot with it is about zero — a degenerate test that silently picks an arbitrary,
usually wrong, side. This is the critical failure. If this drop seeds Apex 2 on the wrong side of the
driving shaft while the A drop seeds it on the correct side, the coincidence that closes the two
drops makes the solver **flip the entire frame to the mirror solution**: A jumps to the opposite
side, the pinion dedendum C collapses onto the driving dedendum D, the toe ends up outside the heel,
the revolved frustum is degenerate, and the conical end-cut finds no cone face at the toe midpoint.
Both drops must aim at the same interior-wedge point. `addPerpendicular(bDrop, drivingShaftAxis)`, a
dimensional constraint with `length = Driving Gear Pitch Diameter / 2`, and a coincident pinning its
beginning to B.

**Close them.** `addCoincident(aDrop.endSketchPoint, bDrop.endSketchPoint)`. That point is **Apex 2**.
At Shaft Angle 90° the four points Apex, A, Apex 2, B form a rectangle; at other angles the figure is
a non-rectangular quadrilateral and the lengths of Apex→A and Apex→B adjust so the two drops coincide.

**Seed the along-shaft lengths** with the closed-form cone geometry — seed coordinates only, the
lengths stay undimensioned and are fixed by the Apex 2 closure (`[BEVEL-F-DRIVEN-DIMS]`):
`|Apex→A| = R * cos γ_p` and `|Apex→B| = R * cos γ_g`, with γ_p, γ_g and R from S5. Both cosines are
positive for every Shaft Angle the range check admits, which is what the Maximum Shaft Angle
guarantees. Seeding A or B merely by a pitch diameter is wrong for Σ ≠ 90° and can send the solver to
the wrong branch. The quadrilateral deliberately lies well above the anchor line, and the Apex's
offset keeps the whole figure above that line across the supported Shaft Angle range.

**Pitch Line.** A construction line from Apex to Apex 2, each end coincident-pinned.

**The two dedendum lines.** From Apex 2, one to each side, each perpendicular to the Pitch Line
(`addPerpendicular(dedendum, pitchLine)`) with a dimensional constraint `length = Module * 1.25`. The
one drawn **toward** the anchor line is the **Driving Gear Dedendum**, its end point **D**; the one
drawn **away** from the anchor line is the **Pinion Gear Dedendum**, its end point **C**.

**Root Axes.** A construction line Apex→D and one Apex→C, coincident at both ends. These are the
driving and pinion Root Axis respectively.

**A → E.** From A, collinear with the line from Apex to A, extending for a length equal to Module —
seed only, **no dimensional constraint** (`[BEVEL-F-DRIVEN-DIMS]`). Apply
`sketch.geometricConstraints.addCollinear(aToE, pinionShaftAxis)` and a coincident between the end of
Apex→A and the beginning of this new line. Its end is point **E**.

**C → E.** A construction line from C to E, each end coincident-pinned to its point. Then
`addPerpendicular(aToE, cToE)`.

**B → F** and **D → F.** The driving twins: B→F collinear with Apex→B, no dimension; D→F drawn from D
to F with both ends pinned; `addPerpendicular(bToF, dToF)`. Its end is point **F**.

**E → G.** From E, collinear with **line A→E** — the collinear names A→E, **never the Apex→A shaft
axis further up the chain**, even though both describe the same infinite line
(`[BEVEL-F-COLLINEAR-CHAIN]`, `[PB-COLLINEAR-CHAIN]`). Length equal to Module as a seed, no
dimensional constraint. Coincident between E and the beginning of this line. Its end is point **G**.
Measured 2026-09-03 on this lattice: `addCollinear(E→G, Apex→A)` raised
`RuntimeError: 3 : failed to create offset: VCS_SKETCH_OVER_CONSTRAINTS` at the second such call, the
first having been absorbed; `addCollinear(E→G, A→E)` builds.

**C → H.** From C, a line of length equal to Module as a seed, no dimensional constraint, with C
coincident to its beginning. Its end is point **H**. `addCollinear(cToH, pinionDedendum)` — the
collinear names **line Apex2→C**, the Pinion Dedendum line C is the endpoint of.

**G → H.** Connect G and H, both ends coincident-pinned. Then
**`addPerpendicular(eToG, gToH)`**.

⚠️ **That perpendicular is required in Fusion and is omitted in the proof, and the reason is a
difference between the two engines rather than a choice.** `addOffsetDimension` in Fusion is a
distance dimension whose documentation requires the second entity to be "a line that is parallel to
the first", and it controls only the perpendicular distance — one equation. So the parallelism has to
exist before the base-height offset below can be applied at all, and this perpendicular is what
supplies it: E→G runs along the pinion shaft, so making G→H perpendicular to it makes G→H parallel to
the A→Apex2 drop. Perpendicular plus offset is two equations for two freedoms and nothing is
redundant. The proof harness's offset emits **two** rows, holding both endpoints of the target line at
the same signed perpendicular distance, so it carries the parallelism itself; adding the
perpendicular there is a third row for the same two freedoms and the lattice comes back
overconstrained at DOF 0 with the two base-height offsets named as the redundant pair. Leave it out
of the proof, never weaken the gate (`[PB-NO-OVERCONSTRAIN]`).

**F → I**, **D → J** and **I → J.** The driving twins of E→G, C→H and G→H, with the same collinear
targets (`addCollinear(fToI, bToF)` naming **B→F**, never Apex→B; `addCollinear(dToJ,
drivingDedendum)` naming **line Apex2→D**) and the same
**`addPerpendicular(fToI, iToJ)`**, required in Fusion and omitted in the proof for the reason just
given. The ends are points **I** and **J**.

**The driving base-height offset.**
`sketch.sketchDimensions.addOffsetDimension(bDrop, iToJ, textPoint).parameter.value = <resolved
Driving Gear Base Height, cm>` — between the **B→Apex2 perpendicular drop** (the DPD/2 drop, NOT the
Apex→B shaft axis) and J→I. J→I is **already parallel** to that drop by construction, since J→I ⊥ F→I
which runs along the driving shaft, so add **no** extra `addParallel` (`[PB-OFFSET-DIM]`). The value
is the driving base height **after** its Maximum Base Height has been applied (a fallback capped to
it, a user value already rejected if it exceeded it), because the offset set here is what drives the
heel edge D→J toward the shaft axis.

**The pinion base-height offset.**
`addOffsetDimension(aDrop, gToH, textPoint).parameter.value = <resolved Pinion Gear Base Height, cm>`
— between the **A→Apex2 perpendicular drop** (the PPD/2 drop, not the Apex→A shaft axis) and G→H,
already parallel by construction, so again no `addParallel`. The resolved value is the one S5
computed: the pinion input when non-zero, else the RESOLVED driving base height scaled by
`Pinion Gear Teeth Number / Driving Gear Teeth Number`, then capped by the **pinion's own** Maximum
Base Height and raised to its own Minimum.

**A → G.** Draw a line from A to G, endpoints coincident-pinned.

**Constrain point I with the projected centre point.** `addCoincident(I, projectedCenter)`. This is
the closure that fixes the whole figure's position: I solves to exactly the projected centre, which
is why the Apex seed is `R * cos γ_g` plus the driving base height. The proof keeps only the single
independent row of that coincident — I on the projected anchor line — because the chain already puts
I on the perpendicular through the centre, and then ASSERTS that the solved I lands on the centre.

**K, the back-cone point.** Draw a construction line away from the Apex, starting at point G,
extending along Apex→A, and call its end **K**. Then **pin K with two point-on-line coincidents** —
`addCoincident(K, line Apex->A)` and `addCoincident(K, the Pinion Dedendum line Apex2->C extended)` —
rather than `addCollinear` on the connecting lines. By the time K is added, G and C are already
fixed, so an `addCollinear` here over-constrains the sketch and Fusion errors; the two point-on-line
coincidents locate K exactly, at the intersection of the two lines, without over-constraining. K
solves to the **back-cone apex** on the shaft axis, at `(PPD / 2) / cos γ_p` from Apex 2 along the
dedendum line — which is the virtual pitch radius §3 uses. Draw a construction line from C to K for
reference.

**K′, the tooth centre.** The §3 spur tooth is centred not at K but at **K′**, K shifted outward
along the dedendum line by **Tooth Spacing**, away from the lower corner C.

- **When Tooth Spacing is 0 (the default), build nothing here**: set K′ ≡ K and reuse the C→K
  reference line. A zero-length dimensioned line would be degenerate, and one segment gets one line
  (`[BEVEL-F-LINE-ONCE]`).
- **When Tooth Spacing > 0**: draw a construction line starting at K with its far end seeded on the
  **far side of K from C** along the dedendum direction; pin its far end the same way K is pinned to
  its line — `addCoincident(K', the Pinion Dedendum line Apex2->C extended)` — and pin its start with
  `addCoincident(start, K)`; then add a length dimension on this line equal to **Tooth Spacing**. Do
  **not** use `addCollinear`, for the same over-constraint reason as K. The far end is K′.

Build K′ **here, inside this sketch, before the end-of-step gate**, so the gate covers it. Finally
draw the tooth-centre reference line **C→K′** for §3 to use in place of C→K. Only the tooth's centre
moves; the virtual tooth number and the drawn tooth size are unchanged.

**Resolve the Maximum Face Width — here, from SOLVED geometry.** At this point A, B, C, D, H and J
all exist and are solved, so read `pointA.geometry`, `pointB.geometry`, `pointC.geometry`,
`pointD.geometry`, `pointH.geometry` and `pointJ.geometry` — **not** the pre-solve seed coordinates
(`[PB-SOLVED-GEOMETRY]`). The bound is `0.95 *` the smaller of

- the perpendicular distance from **A** to the line through **C and H** (the Pinion Gear Dedendum
  line, i.e. Apex2→C extended), and
- the perpendicular distance from **B** to the line through **D and J** (the Driving Gear Dedendum
  line, i.e. Apex2→D extended).

Compute both and take the minimum, so the bound holds for any Shaft Angle; the gear with the smaller
pitch diameter is normally the binding side. Seeds diverge substantially for asymmetric tooth counts
or non-90° shaft angles, making a seed-based bound too loose on the binding side — the toe still
crosses the axis and the cap is defeated. Then apply it: cap the auto default
`min(Cone Distance / 6, Maximum Face Width)`, and **reject** a user Face Width that exceeds it with a
message naming the maximum. Stash the result as `self._faceWidthResolved_cm`.

Rationale, which must not be dropped: the toe line M→N is C→H offset toward the Apex by Face Width,
with N pinned to line A→Apex2. When the offset reaches the perpendicular distance from A to line
C→H, N lands exactly on A; any larger value drives N past A, across the gear's own shaft axis. The
hexagon A, G, H, C, M, N is revolved about that axis, so a profile that has crossed it
self-intersects the axis of revolution and Fusion aborts the revolve with `ASM_WIRE_X_AXIS`
(`[PB-REVOLVE]`). The `0.95` keeps N clearly off A, since a near-coincident N ≈ A degenerates the toe
edge even before it strictly crosses. At Shaft Angle 90° this limit equals
`Pinion Gear Pitch Diameter ** 2 / (2 * Cone Distance)`, so the naive `Cone Distance / 6` default
exceeds it — and the gear fails to generate — for any gear ratio above roughly √2.

**M → N, the pinion toe line.** Create the line, then apply **exactly these four constraints**:

- `addCoincident(M, Pinion Root Axis)` — M lies on the Apex→C root axis;
- `addCoincident(N, line A->Apex2)` — N lies on the A→Apex2 **perpendicular DROP**, per the naming
  convention, NOT the Apex→A shaft axis. This is the pin; do not merely place N numerically.
  Load-bearing: pinning N to the shaft axis puts N *on the axis of revolution*, and the later conical
  split fails with `ASM_API_FAILED` for asymmetric tooth counts even though the symmetric 45° case
  happens to survive;
- `addParallel(mToN, cToH)` — the toe line is parallel to C→H;
- `addOffsetDimension(cToH, mToN, textPoint).parameter.value = <resolved Face Width, cm>`. Place the
  text point in the gap between C→H and M→N on the Apex side, e.g. the midpoint of the M seed and
  point C, so the dimension reads cleanly (`[PB-OFFSET-DIM]`).

**Seed M and N near their solved positions** (`[PB-SEED-NEAR]`): seed **M on Apex→C at the fraction
`1 - FaceWidth / R` of the way from the Apex to C**, and seed N by sliding from that M seed along the
C→H direction until it reaches line A→Apex2. At the default Face Width that M fraction comes to
roughly the midpoint of Apex→C, which is how §2 states the rule; stating it as the fraction is what
makes it hold when a user Face Width drops well below the default, where the literal midpoint seed
does not converge in the bench solver. Do **NOT** seed M and N just `Face Width` away from C and H —
that starts N near H, far from its constraint target, and the solve fails to converge.

Because N is pinned to A→Apex2, the Maximum Face Width is exactly the value at which N reaches A, and
the capped Face Width keeps N between Apex 2 and A. The toe's side relative to the heel follows from
the §2 frame being built correctly — in particular from the Apex 2 drops aiming at the interior
wedge — and is **not** controlled by the text point.

Let the beginning of the line be **M** and the end **N**. Draw a line from M to C, and a line from N
to A.

**L, L′ and O → P, the driving twins.** Build L exactly as K, substituting the Driving Gear Shaft
Axis and the Driving Dedendum line Apex2→D, with the reference line D→L; build L′ exactly as K′,
substituting L for K, D for C and the Driving Dedendum for the pinion's, with the reference line
**D→L′**, the same single Tooth Spacing value and the same reuse-the-existing-line rule at 0. Create
line O→P with O seeded on Apex→D at the same fraction and P slid along D→J toward line B→Apex2, then
the same four constraints:

- `addCoincident(O, Driving Root Axis)`;
- `addCoincident(P, line B->Apex2)` — the B→Apex2 perpendicular DROP, **not** the Apex→B shaft axis,
  the same trap as N;
- `addParallel(oToP, dToJ)`;
- `addOffsetDimension(dToJ, oToP, textPoint).parameter.value = <resolved Face Width, cm>`, its text
  point in the gap on the Apex side of D→J.

Let the beginning be **O** and the end **P**. Draw a line from O to D, a line from P to B, and a line
from B to I.

**Gate.** `if not sketch.isFullyConstrained: raise ...` naming the sketch
(`[BEVEL-F-FULL-CONSTRAINT]`). Do **not** reach full constraint by dimensioning the driven lengths —
Apex→A, Apex→B and the module-length extensions are determined by the perpendicular, collinear and
closing constraints (`[BEVEL-F-DRIVEN-DIMS]`, `[PB-NO-OVERCONSTRAIN]`).

`[PB-SKETCH-FIRST]` is **waived** for bevel as a bench-proof requirement: there is no
`spec/bevelgear/sketch/` program, the §2 scheme predates that gate, and a regen keeps the scheme
exactly as specified here — it may not invent a new one and may not claim bench-proven status. The
proof in `proof/bevelgear/lattice_test.go` reproduces this net and records what it measures,
including the two places this particular net cannot reach the engine's conditioning floor.

The proof makes five substitutions and names each at its call site: the two collinear chains and the
"constrain point I" coincident are each replaced by the single independent row they carry, since the
engine counts every row and reports the implied one as redundant; the two `addParallel` calls on the
toe lines and the two `addPerpendicular` calls on G→H and I→J are dropped, since the engine's offset
carries the parallelism itself; and the three angular relations that Fusion states with an unsigned
`addAngularDimension` or `addPerpendicular` are written as signed angles, because the unsigned
reading leaves the net with a second configuration in which D collapses onto C — the very frame
inversion this step's warnings describe — which the seed resolves in Fusion and the ambiguity probe
will not.

`min` is a Python builtin and `normalize` names the arithmetic of taking a unit vector, written
however the module's own vector helpers spell it.

<!-- check-step-calls: ignore min normalize -->

**From:** `spec/bevelgear/instructions.md` L450-L546 (the whole of §2), L116-L124 (Maximum Face
Width), L10-L17 (the sketch-first waiver); `spec/bevelgear/fusion.md` L69-L115
(`[BEVEL-F-COINCIDENT-STYLE]`, `[BEVEL-F-LINE-ONCE]`, `[BEVEL-F-COLLINEAR-CHAIN]`,
`[BEVEL-F-DRIVEN-DIMS]`), L117-L145 (`[BEVEL-F-APEX-LOCAL]`, `[BEVEL-F-GROW-SIDE]`), L21-L30
(`[BEVEL-F-FULL-CONSTRAINT]`); `.claude/skills/generate-gear/PLAYBOOK.md` L438-L490
(`[PB-FULL-CONSTRAINT]`, `[PB-NO-OVERCONSTRAIN]`, `[PB-COLLINEAR-CHAIN]`), L498-L505
(`[PB-API-SPELLING]`), L558-L580 (`[PB-SOLVED-GEOMETRY]`, `[PB-SEED-NEAR]`), L581-L601
(`[PB-SHARE-XOR-COINCIDENT]`), L604-L618 (`[PB-OFFSET-DIM]`, `[PB-ANGULAR-DIM]`), L230-L242
(`[PB-DIM-VALUE-SEMANTICS]`), L543-L551 (`[PB-POINT-HELPER]`).

---

## S11 `[PROSE]` `{gearLabel} Plane` — the tooth-profile plane

Run S11 through S33 **once per gear**, pinion first, then driving, with these substitutions:

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A → G → H → C → M → N → A | B → I → J → D → O → P → B |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A→G | B→I |
| toe cut edge | M→N | O→P |
| heel cut edge | C→H | D→J |
| tooth centre / reference line | K′ / C→K′ | L′ / D→L′ |
| pitch cone half angle | γ_p | γ_g |
| teeth, bore and pitch diameter | Pinion Gear … | Driving Gear … |
| §2 shaft construction line (NOT usable as the axis) | Apex→A | Apex→B |

First compute this gear's **virtual (back-cone, Tredgold) tooth number** from the closed form, **not**
by measuring Apex2→K′/L′:

```
virtualPitchRadius_mm = (pitchDia_cm * 10 / 2) / cos(gamma)
virtualTeeth          = int(math.floor(2 * virtualPitchRadius_mm / Module))
```

The `* 10` converts cm to mm: the stashed pitch diameters are internal cm while Module is the raw mm
value, and skipping the conversion makes the virtual tooth count about ten times off.

Then build the plane through the tooth-centre reference line, perpendicular to the Gear Profiles
sketch plane, using the framework helper: `plane_by_angle(designComponent, toothCenterRefLine,
gearProfilesPlane, 90)`. Name it `{gearLabel} Plane`. The helper passes the sketch line **directly**
to `setByAngle`; never wrap it in `Path.create` (`[PB-CONSTRUCTION-PLANES]`).

`Path.create` is named here only to forbid it; the helper resolves the curve itself.

<!-- check-step-calls: ignore Path.create -->

`cos` and `int` are the virtual-tooth formula's own arithmetic, not Fusion calls.

<!-- check-step-calls: ignore cos int -->

**From:** `spec/bevelgear/instructions.md` L548-L563 (§3 steps 1 and 2), L659-L672 (the per-gear
substitution table); `.claude/skills/generate-gear/PLAYBOOK.md` L180-L181 (`plane_by_angle`),
L742-L753 (`[PB-CONSTRUCTION-PLANES]`).

---

## S12 `[GO]` `{gearLabel} Tooth` sketch — the virtual spur tooth

Proof function: `stepToothProfile`.

<!-- proof-run: proofkit3d.RunSolid(toothSolidCases, stepToothProfile, assertToothProfile) -->

`toothSketch = designComponent.sketches.add(toothPlane)`, named `{gearLabel} Tooth`. Draw the spur
tooth into it with the borrowed generator, using this gear's tooth-centre point (K′ or L′) as the
anchor:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
drawer.draw(toothCenterPoint, angle=math.radians(180))
```

The **180° rotation is delivered through the `draw()` angle argument** — the spur generator rotates
the whole tooth by that angle — never a post-hoc Move or sketch rotation. This relies on spur's
radial flank-to-root pinning so the connecting lines rotate with the tooth.

`VirtualSpurProxy` is the framework's fake spur `parent`: it precomputes, in internal cm, exactly the
parameter keys the spur drawer reads through `parent.getParameter(name).value`, with defaults that
match bevel — pressure angle 20°, which is not a bevel dialog input, and `InvoluteSteps` 15 — and
returns each wrapped in a `.value` carrier. Construct it with the raw-mm module and the virtual tooth
number. Do **not** define a local copy.

**Read `proxy._lastToothEmbedded` back after `draw()` returns** and thread it to the tooth-profile
selection. It is an OUTPUT the spur generator writes during `draw()`; the framework proxy
pre-initialises the slot to absorb that write. This flag is not optional bookkeeping — it is the
deterministic selector for the tooth loop's line count. Stash it alongside the tooth sketch and plane
in this gear's dict.

**Do NOT gate this sketch.** Log it if `not toothSketch.isFullyConstrained`, never raise. It is the
one sketch bevel does not gate, and the reason is that the spur drawer labels each of its four
circles with along-path sketch text, and sketch text carries its own position along the curve
(`[PB-TEXT-HOLDS-DOF]`), so a tooth sketch whose geometry is completely determined still reads
`False`. **The exemption covers the labels and nothing else** — never read it as licence for loose
geometry. Bevel's own four sketches carry no text, which is why they gate normally.

<!-- check-step-calls: ignore isFullyConstrained -->

The proof therefore proves this sketch through the SOLID harness rather than the sketch gate: it
builds the loop, asserts the curve inventory, the tip and root radii, the tooth centre, the 180°
rotation and the embedded flag, and extrudes it so the harness has a body to judge. The spur tooth's
own constraint scheme belongs to `spec/spurgear` and is proved there.

`getParameter` is the interface the proxy SERVES to the borrowed drawer, which is what calls it;
this module never does.

<!-- check-step-calls: ignore getParameter -->

**From:** `spec/bevelgear/instructions.md` L404-L430 (Dependencies and the embedded flag),
L560-L561 (§3 step 3), L373-L376 (the exemption); `spec/bevelgear/fusion.md` L31-L58
(`[BEVEL-F-FULL-CONSTRAINT]`'s tooth-sketch exemption);
`.claude/skills/generate-gear/PLAYBOOK.md` L188-L194 (`VirtualSpurProxy`), L514-L522
(`[PB-TEXT-HOLDS-DOF]`), L649-L659 (`[PB-SKETCH-TEXT]`).

---

## S13 `[PROSE]` `{gearLabel} Tooth Axis`

A construction axis through the tooth-centre point, normal to the plane the tooth profile was drawn
on:

```
helperInput = designComponent.constructionPlanes.createInput()
helperInput.setByDistanceOnPath(toothCenterRefLine, adsk.core.ValueInput.createByReal(1.0))
helperPlane = designComponent.constructionPlanes.add(helperInput)

axisInput = designComponent.constructionAxes.createInput()
axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)
axis = designComponent.constructionAxes.add(axisInput)
axis.name = f'{gearLabel} Tooth Axis'
```

The two planes are the **Gear Profiles plane** and a helper plane built
`setByDistanceOnPath(<tooth-centre reference line>, 1.0)`, which is perpendicular to that line at its
far end, the tooth-centre point; their intersection is the line through the tooth centre normal to
the tooth plane. Use `setByTwoPlanes` (`[PB-CONSTRUCTION-AXES]`); `setByPerpendicularAtPoint` would
need a `BRepFace` this build does not have. Pass the sketch line directly to `setByDistanceOnPath`,
never through `Path.create`.

<!-- check-step-calls: ignore setByPerpendicularAtPoint Path.create -->

Creating this axis in the never-activated Design component is proven to work — `constructionAxes.add`
via `setByTwoPlanes` does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here; keep the axis.

**From:** `spec/bevelgear/instructions.md` L563 (§3 step 4), L550-L551 (pass the sketch line
directly); `.claude/skills/generate-gear/PLAYBOOK.md` L754-L757 (`[PB-CONSTRUCTION-AXES]`),
L758-L766 (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`).

---

## S14 `[PROSE]` `{gearLabel} Gear` component

Create a new component as a child of the **Bevel Gear** component — the same component that owns
Design, *not* the user's Parent Component; this intentionally overrides the looser "child of Parent
Component" phrasing so the pair nests cleanly inside Bevel Gear. Name it `{gearLabel} Gear`, i.e.
`Pinion Gear` or `Driving Gear`. The finished bodies for this gear end up here.

Fusion rejects cross-sibling sketch and project calls even when the target is activated or the
entities are wrapped in `createForAssemblyContext` proxies (`[PB-NO-CROSS-SIBLING]`), so the actual
feature operations all run in the Design component and the finished bodies are `moveToComponent`'d
here at the end (S33). The visible end state is identical.

`createForAssemblyContext` is named only to record that it does not help, and `moveToComponent` is
named here only to say where it happens; the call itself is S33.

<!-- check-step-calls: ignore createForAssemblyContext moveToComponent -->

**From:** `spec/bevelgear/instructions.md` L673;
`.claude/skills/generate-gear/PLAYBOOK.md` L796-L801 (`[PB-NO-CROSS-SIBLING]`).

---

## S15 `[GO]` `{gearLabel} Profile` sketch — the frustum hexagon

Proof function: `stepGearProfileHexagon`.

<!-- proof-run: proofkit.Run(profileCases, stepGearProfileHexagon) -->

Open a **fresh sketch on the axial (Gear Profiles) plane**, named per the table in S11 — **one
profile sketch per gear**, so `sketch.profiles` holds exactly this one hexagon loop. Do not draw both
gears' hexagons in the shared Gear Profiles sketch; that would leave two identically-shaped loops to
disambiguate.

Build the hexagon on fixed vertices per the `[PB-PROJECT-NOT-FIXED]` **recreate-share-fix** recipe,
and the order is the whole recipe:

1. Recreate the six §2 vertices as new sketch points at their exact positions —
   `sketch.sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))` for each — which is valid
   because §2 is fully constrained by now, so each `worldGeometry` is defined. `modelToSketchSpace`
   is a point-transforming **method**, not a matrix (`[PB-SPACE-METHODS]`).
2. Draw the closed hexagon, in the table's draw order, as six `SketchLine`s **sharing** those points.
3. Fix the lines' endpoints **after** the lines exist:
   `for e in lines: e.startSketchPoint.isFixed = True; e.endSketchPoint.isFixed = True`.

Setting `isFixed = True` on a bare point *before* it is consumed as a line endpoint does not leave
the sketch fully constrained. Projecting the §2 points instead of recreating them does not fix them
either: a projection is brought in associatively and still carries free DOF, so the sketch would
report under-constrained even though every point already sits in the right place.

The hexagon's **first edge is the gear's shaft axis** for the revolve, the pattern, the bore plane and
the meshing rotation, so it must be fixed well enough to carry a trustworthy world position: fixed
endpoints give that edge a well-defined `worldGeometry` (`[PB-WORLDGEO-CONSTRAINED]`). A free edge
resolves against a default world-XY frame and silently moves the body onto world XY — observed on the
driving gear, where the pinion looked fine only because it never read the edge's `worldGeometry`.

**The shaft axis used by every body operation below is this profile sketch's first edge, NOT the §2
`Apex->A` / `Apex->B` construction line.** The edge is collinear with the shaft axis but lives in the
*same* sketch as the profile, which is what Fusion's revolve, pattern and path builders accept;
reusing the §2 construction line, which lives in another sketch, fails or misbuilds.

Gate the sketch: `if not sketch.isFullyConstrained: raise ...` naming it
(`[BEVEL-F-FULL-CONSTRAINT]`).

**From:** `spec/bevelgear/instructions.md` L675-L677; `spec/bevelgear/fusion.md` L21-L30;
`.claude/skills/generate-gear/PLAYBOOK.md` L455-L469 (`[PB-PROJECT-NOT-FIXED]`), L552-L557
(`[PB-SPACE-METHODS]`), L565-L572 (`[PB-WORLDGEO-CONSTRAINED]`).

---

## S16 `[GO]` Revolve the hexagon into the Gear Body

Proof function: `stepRevolveGearBody`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepRevolveGearBody, assertRevolveGearBody) -->

The sketch holds exactly one closed loop, so take its single profile directly with
`sketch.profiles.item(0)` (`[PB-SINGLE-PROFILE]`) — do not invent a filter by `profileLoops` or by
curve type, which has spuriously rejected a valid all-line loop and made the revolve fail with "could
not find profile".

```
revolveInput = designComponent.features.revolveFeatures.createInput(
    profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
gearBody = designComponent.features.revolveFeatures.add(revolveInput).bodies.item(0)
```

The axis is the hexagon's first edge from S15. The result is the **Gear Body**, the frustum. Because
the toe edge is one edge of the revolved profile, the body already carries the conical face that edge
sweeps — that face is reused as the cutting tool below, as is the heel edge's cone.

**Hard failure to design around:** the profile must not cross the axis of revolution, or Fusion
aborts with `RuntimeError … ASM_WIRE_X_AXIS … the profile crosses the axis of revolution`
(`[PB-REVOLVE]`). The Maximum Face Width and Maximum Base Height caps of S5 and S10 are exactly what
keeps it from doing so; reproduce them.

The proof substitutes a polygonal sweep for the revolve and says so in `solids_test.go`: decad
publishes a revolved body's volume with a proven bound equal to the volume itself, so a revolved body
is Suspect at any tolerance and cannot pass the harness gate, while its loft and union of polygonal
profiles are Sound. It measures the same solid to a few parts in ten thousand and ties the reading
back to Pappus on the §2 hexagon.

`profileLoops` is named only to forbid filtering on it.

<!-- check-step-calls: ignore profileLoops -->

**From:** `spec/bevelgear/instructions.md` L679;
`.claude/skills/generate-gear/PLAYBOOK.md` L491-L497 (`[PB-SINGLE-PROFILE]`), L684-L690
(`[PB-REVOLVE]`).

---

## S17 `[GO]` Loft the Apex point to the tooth profile

Proof function: `stepLoftToothBody`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepLoftToothBody, assertLoftToothBody) -->

Select the tooth cross-section loop with

```
toothProfile = find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=(0 if embedded else 2))
```

The line count is **DETERMINED BY the `embedded` flag read back in S12, not guessed and not
accepted-either**: `wantLines = 0 if embedded else 2`. ⚠️ Do **NOT** accept "0 **or** 2 lines" — for a
given gear only ONE of those is the real tooth, and an unrelated loop, for example an inter-tooth or
annular region between the drawer's circles, can also have 2 NURBS and 2 arcs but the *other* line
count. Selecting it makes the loft below fail with
`RuntimeError ... ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`, because the impostor loop cannot form a loft
tool body. `embedded` means tip, root and flanks meet with no connecting lines (4 curves);
non-embedded means two connecting lines (6 curves), mirroring spur's own selection.

`find_profile_by_curve_counts` matches a loop by curve count and type, never by index
(`[PB-PROFILE-MATCH]`): it reads each curve's `curve.geometry.curveType` against
`adsk.core.Curve3DTypes`, whose member names end in `...CurveType` — `Line3DCurveType`,
`Arc3DCurveType`, `Circle3DCurveType`, `NurbsCurve3DCurveType`. `NurbsCurve3DType` and `Arc3DType` do
not exist and raise `AttributeError`. The helper raises with a self-diagnosing message rather than
falling back to a wrong profile.

<!-- check-step-calls: ignore curveType Curve3DTypes Line3DCurveType Arc3DCurveType Circle3DCurveType NurbsCurve3DCurveType NurbsCurve3DType Arc3DType -->

Then loft the **§2 Apex sketch point** to that profile:

```
loftInput = designComponent.features.loftFeatures.createInput(
    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
loftInput.loftSections.add(apexSketchPoint)
loftInput.loftSections.add(toothProfile)
toothBody = designComponent.features.loftFeatures.add(loftInput).bodies.item(0)
```

`apexSketchPoint` is `centerToApex.endSketchPoint` from the Gear Profiles sketch — the degenerate
point section. Use the §2 Apex **SketchPoint** directly; do **NOT** create a `ConstructionPoint` for
it (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`: construction geometry needs an active component and the Design
component is never activated). The order of `loftSections.add(...)` is the loft order
(`[PB-LOFT]`).

The result is the uncut **Tooth Body**, tapering from the Apex to the heel-end tooth profile.

The proof substitutes a shrunken section for the degenerate apex point, and axis-perpendicular
sections for the back-cone tooth plane; `solids_test.go` states both and what they cost.

`ConstructionPoint` is named only to forbid it as the loft's point section.

<!-- check-step-calls: ignore ConstructionPoint -->

**From:** `spec/bevelgear/instructions.md` L353-L366 (tooth-profile selection), L681;
`.claude/skills/generate-gear/PLAYBOOK.md` L152-L154 (`find_profile_by_curve_counts`), L639-L648
(`[PB-PROFILE-MATCH]`), L691-L695 (`[PB-LOFT]`), L758-L766
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`).

---

## S18 `[GO]` Conical end trims — the flush band

Proof function: `stepCutConicalEnds`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCutConicalEnds, assertCutConicalEnds) -->

Trim the Tooth Body to a flush band with the framework helper — do **not** re-implement the cut
machinery:

```
toothBody = cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)
```

**Two distinct bodies are involved and must not be conflated.** The cutting TOOLS are
`ConeSurfaceType` faces of the **Gear Body**, the revolved-hexagon frustum; the lofted Tooth Body has
no cone faces, so searching *it* for one finds none. The TARGET being split is the **Tooth Body**.

**Caller obligations, which stay in the generator:**

- `toeMid` = the toe edge's world midpoint — `(M_world + N_world) / 2` for the pinion,
  `(O_world + P_world) / 2` for the driving gear;
- `heelMid` = the heel edge's world midpoint — `(C_world + H_world) / 2` resp.
  `(D_world + J_world) / 2`;
- `apexWorld` = the §2 Apex sketch point's `worldGeometry`;
- `gearBody` = the revolved frustum, the cone-face source.

The helper implements the pinned behaviour: the **toe cut first**, its cone face identified by the toe
edge's world **MIDPOINT** best-first across the frustum's cone faces (`[PB-FACE-BY-MIDPOINT]` —
endpoints sit near the apex singularity and cannot be evaluated there), each candidate tried as the
actual split tool and the first that splits kept; **keeper selection after each cut**, removing
apex-containing pieces and keeping the largest (`[PB-REMOVE-PIECES]`); then the **heel cut on the
keeper alone**, which is what makes it deterministically two split features for every gear ratio. A
heel cone that does not intersect the keeper at all — common on ratio pairs, e.g. module 1 with
driving 31 and pinion 43, where the heel cone never overshoots the tooth — is raised by the helper as
the typed `solids.NonIntersectError` and caught, and the keeper is returned whole. The toe cut must
split: its failure propagates and crashes the build, which is correct, since an uncut tooth is
unusable. Only the heel cut is lenient, and only through that typed error. Every failure is
self-diagnosing with the per-face distance and error history (`[PB-SELF-DIAGNOSING]`), and each cut's
outcome is logged with `force_console=True`.

`apply_conical_cut`, `select_keeper`, `find_cone_faces_by_midpoint` and `surface_distance` are the
helper's own internals, named here only so a regen does not re-implement them; this module calls
`cut_conical_ends` and nothing below it.

<!-- check-step-calls: ignore apply_conical_cut select_keeper find_cone_faces_by_midpoint surface_distance NonIntersectError -->

This same call is what `_transformToothBody` returns immediately when ψ = 0 — a straight bevel is
byte-for-byte the prior behaviour — and it is also step J of the spiral branch, applied to the curved
tooth so its ends sit flush on the gear base.

The proof substitutes an intersection with the band between the two cones for the two split features
and their keeper selection: decad has no split-by-face, and the band that intersection leaves is the
trims' whole purpose.

**From:** `spec/bevelgear/instructions.md` L683-L709 (Conical cuts and caller obligations), L332-L334
(the ψ = 0 gate), L657 (step J);
`.claude/skills/generate-gear/PLAYBOOK.md` L159-L172 (`cut_conical_ends` and its machinery),
L707-L728 (`[PB-FACE-BY-MIDPOINT]`), L729-L736 (`[PB-SELF-DIAGNOSING]`), L737-L741
(`[PB-REMOVE-PIECES]`), L700-L706 (`[PB-SPLIT-BODY]`).

---

## S19 `[PROSE]` Spiral frame and cutter-arc geometry (§3a steps A and B)

Everything from here to S27 runs **only when ψ > 0**. At ψ = 0 the hook has already returned with
S18.

**The caller hand-off. Pin these exactly; mislabelling them silently inverts the spiral, and this is
the single biggest spiral-regen hazard.** `_createGearBody` builds four world points and passes them
positionally in the order `toeMid, heelMid, toeConeWorld, heelConeWorld`:

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

- `toeMid` = the world **midpoint of the TOE edge** — ½(M+N) resp. ½(O+P).
- `heelMid` = the world **midpoint of the HEEL edge** — ½(C+H) resp. ½(D+J).
- `toeConeWorld` = the toe edge's inner endpoint, **M** resp. **O**, which lies on the root cone
  element at the toe end.
- `heelConeWorld` = the **dedendum corner**, **C** resp. **D**, the outer end of that same root cone
  element.

⚠️ Two scrambles a fresh regen has made: do **NOT** pass the two endpoints of a *single* edge as
`toeMid`/`heelMid` — M and N both sit at the toe, so the span collapses to about zero or goes
negative and the spiral inverts; and `heelConeWorld` is the dedendum corner C/D, **never** H/J, which
lie on the Apex2→C / Apex2→D dedendum line one Module beyond C/D, off the root cone element, and skew
the cone vector away from Apex→C / Apex→D.

**Step A — the frame**, built from the geometry already constructed for this gear:

- `axisDir` = the shaft axis direction, from the two **world** endpoints of `shaftAxisEdge` (the
  in-sketch profile edge A→G / B→I), normalized;
- `coneVec` = `normalize(heelConeWorld − apexWorld)`, the dedendum (root) cone element Apex→C / Apex→D;
- `v` = `normalize(axisDir × coneVec)`, the circumferential direction;
- `tpNormal` = `normalize(coneVec × v)`, the tangent-plane normal;
- `distAlong(p)` = `(p − apex) · coneVec`, a point's cone distance.

Sample the sketch curves in **world** space wherever they are measured against a world quantity;
mixing a local-frame curve with a world axis is valid Python that silently returns wrong numbers
(`[PB-WORLD-FRAME]`).

⚠️ **The heel MUST be the outer end so `coneVec` points outward and the span is positive.** Before
building `coneVec`, check the passed midpoints and fix a swapped pair: if
`apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid ↔ heelMid` **and**
`toeConeWorld ↔ heelConeWorld`, then build `coneVec` from the corrected `heelConeWorld`. A negative
span silently inverts the entire spiral frame — the cutter-arc direction, the slice direction and the
per-segment twist — and the gear comes out completely wrong with no error.

Then `R_toe = distAlong(toeMid)`, `R_heel = distAlong(heelMid)`, `R_mean = ½(R_toe + R_heel)` and
`span = R_heel − R_toe`, now positive. These are the only quantities the rest of the build needs.

**Step B — the cutter-arc geometry**, in the tangent-plane 2-D frame with the origin at the apex,
x = `coneVec` (so a point's x is its cone distance) and y = `v`:

```
r_c      = Cutter Radius if non-zero else R_mean
handSign = +1 for _HAND_RIGHT else -1,  then NEGATED for the pinion
Cx = R_mean - r_c * sin(psi)
Cy = handSign * r_c * cos(psi)
```

⚠️ **The hand sign goes on the `cos`/`Cy` term, NOT the `sin`/`Cx` term.** This was a real bug.
Opposite hand mirrors the cutter centre **across the cone element (y = 0)**, which flips `Cy`.
Putting `handSign` on `Cx` mirrors about `x = R_mean` instead — a *different* curve that gives the
two gears unequal twist, where for equal teeth the driving and pinion traces must come out as exact
mirror images.

The trace's endpoints are circle∩circle intersections taken a hair **past** the face so the kept arc
reaches cleanly past the end trims:

```
R_lo   = R_toe  - 0.06 * span
R_hi   = R_heel + 0.06 * span
toe2d  = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0.0)
heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0.0)
```

`circle_intersect_nearest` is the framework helper: it intersects the apex circle of radius R with
the cutter circle of centre `(Cx, Cy)` and radius `r_c` and keeps the solution nearest the reference
point `(R_mean, 0)` — the branch the mean point sits on. Keeping the far branch gives a kinked or
back-bent trace.

`normalize`, `distAlong` and `distanceTo` above name the arithmetic rather than an API call; write
them however the module's own vector helpers spell them.

<!-- check-step-calls: ignore normalize distAlong distanceTo cos sin -->
<!-- check-compile: ignore distAlong -->

**From:** `spec/bevelgear/instructions.md` L565-L607 (§3a hand-off, steps A and B);
`spec/bevelgear/spiral-tooth-trace.md` L30-L63 (the drawing plane and its axes), L66-L76 (the three
cone-distance marks), L91-L104 (the spiral angle and the hand), L108-L145 (locating the cutter
centre), L149-L168 (toe and heel ends);
`.claude/skills/generate-gear/PLAYBOOK.md` L183-L184 (`circle_intersect_nearest`), L436
(`[PB-WORLD-FRAME]`).

---

## S20 `[GO]` `{gearLabel} Cone Element` sketch

Proof function: `stepConeElementSketch`.

<!-- proof-run: proofkit.Run(coneElementCases, stepConeElementSketch) -->

Open a sketch on the **axial (Gear Profiles) plane**, named `{gearLabel} Cone Element`, and draw one
construction line in it from the apex to `apex + R_heel * coneVec`:

```
coneSketch = designComponent.sketches.add(gearProfilesPlane)
coneSketch.name = f'{gearLabel} Cone Element'
coneElementLine = coneSketch.sketchCurves.sketchLines.addByTwoPoints(apexWorld, coneEnd)
coneElementLine.isConstruction = True
```

`coneVec` is the **root cone element** — `normalize(heelConeWorld − apexWorld)`, i.e. Apex→C for the
pinion and Apex→D for the driving gear — from S19. This line is the one the Trace Plane is rotated
about, so it is the single place the whole spiral frame's orientation is decided.

⚠️ **Build it on the root cone element and nothing else.** Built along `Apex→Apex2` — the pitch line —
or along the shaft axis, the tangent plane is skewed and the entire construction goes with it, with
no error anywhere downstream. The two are separated by the dedendum angle `atan(1.25 * Module / R)`,
which is small but never zero, and by the root cone angle respectively.

This is a transient auxiliary sketch of the spiral build and is **exempt from the full-constraint
gate** (`[BEVEL-F-FULL-CONSTRAINT]`); do not gate it. It happens to reach zero degrees of freedom
anyway, since both of its points are placed from raw coordinates and neither is dimensioned, and the
proof gates it on that rather than assuming it — which is what lets the proof also assert the
direction, the length and the two lines it must not have been built on.

**Coordinates — this rule governs the Cone Element sketch as well as the trace sketch of S32, and it
is the only place either is told what frame its points are in.** The world `Point3D`s from
`combine_point(...)`, and the raw apex and cone-end points of the Cone Element line, are passed
**directly** into the sketch calls, where they are consumed as **sketch-space** input; **no
`modelToSketchSpace` conversion is applied**, even though `adsk.fusion.Sketch` offers exactly that
call and the points really are model-space coordinates.

<!-- check-step-calls: ignore modelToSketchSpace -->

That is deliberate, and the reasoning is not the obvious one. The trace sketch is
construction-and-reference only: no downstream feature ever consumes it, because the twist is
computed analytically from the 2-D endpoints in S32 and the sketch exists only so the genuine cutter
arc is inspectable before cleanup hides it. The cone-element line is the one that needs the extra
sentence, because it *is* consumed — by `plane_by_angle`, which rotates about it — so an unconverted
cone-element line does place the Trace Plane somewhere other than the true tangent plane. That still
reaches no feature, because the only thing built on the Trace Plane is the inspection-only trace
sketch, and the chain ends there. **If a later revision ever makes any feature consume the trace
sketch or the Trace Plane, this shortcut stops being safe and both sketches need
`modelToSketchSpace` on every point.**

These are transient auxiliary sketches of the spiral build and are **exempt from the full-constraint
gate**; do not gate them.

`atan` above is arithmetic in the prose, naming the angle that separates the root cone from the
pitch cone; it is not a call this step makes.

<!-- check-step-calls: ignore atan -->

**From:** `spec/bevelgear/instructions.md` L609 (the cone element line and the trace plane),
L614-L616 (the coordinates rule), L617-L618 (the exemption); `spec/bevelgear/fusion.md` L59-L67
(the spiral auxiliary-sketch exemption);
`.claude/skills/generate-gear/PLAYBOOK.md` L180-L182 (`plane_by_angle`, `combine_point`).

---

## S21 `[PROSE]` `{gearLabel} Trace Plane`

Build the tangent plane as the axial plane rotated **90°** about the cone-element line of the
previous step:

```
tracePlane = plane_by_angle(designComponent, coneElementLine, gearProfilesPlane, 90)
tracePlane.name = f'{gearLabel} Trace Plane'
```

`plane_by_angle` passes the sketch line **directly** to `setByAngle`; never wrap it in `Path.create`
(`[PB-CONSTRUCTION-PLANES]`).

<!-- check-step-calls: ignore Path.create -->

This plane too is part of the spiral build's auxiliary scaffolding and is hidden in cleanup. It is
consumed by exactly one thing — the trace sketch of the next step — which is why the coordinate
shortcut the previous step states is safe.

**From:** `spec/bevelgear/instructions.md` L609 (the trace plane), L616 (what consumes it),
L617-L618 (the exemption); `spec/bevelgear/fusion.md` L59-L67 (the spiral auxiliary-sketch
exemption); `.claude/skills/generate-gear/PLAYBOOK.md` L180-L181 (`plane_by_angle`), L742-L753
(`[PB-CONSTRUCTION-PLANES]`).

## S22 `[GO]` `{gear} 2D Tooth Trace` sketch — the genuine cutter arc

Proof function: `stepSpiralTrace`.

<!-- proof-run: proofkit.Run(traceCases, stepSpiralTrace) -->

Add a sketch on the Trace Plane named `{gearLabel} 2D Tooth Trace`. In it, with
`tanW(px, py) = combine_point(apexWorld, px, coneVec, py, v)` mapping 2-D tangent-plane coordinates to
world, draw:

- **the cutter circle** — `sketch.sketchCurves.sketchCircles.addByCenterRadius(tanW(Cx, Cy), r_c)`,
  with `isConstruction = True`, its centre pinned by `circle.centerSketchPoint.isFixed = True`
  (`[PB-CIRCLE-CENTER]` — a circle's centre is FREE even when created at the origin, and
  `addCoincident` to the sketch origin has been observed to throw `VCS_SKETCH_SOLVING_FAILED`), and a
  diameter dimension `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` with
  `.parameter.value = 2 * r_c`;
- **the trace arc** —
  `sketch.sketchCurves.sketchArcs.addByThreePoints(tanW(toe2d), tanW(R_mean, 0), tanW(heel2d))`, with
  its centre coincident to the cutter circle's centre
  (`sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, circle.centerSketchPoint)`) and
  a radius dimension `sketch.sketchDimensions.addRadialDimension(arc, textPoint)` with
  `.parameter.value = r_c`, so it is the genuine cutter circle and not a look-alike spline.

⚠️ Text points must be **off-centre**, on or near the curve (`[PB-RADIAL-DIM]`): a radial or diameter
dimension rejects a text point at the curve's centre, because there is no radial direction there.
Use the mean point `tanW(R_mean, 0)` for the trace arc's radius dimension, and a point on the cutter
circle such as `tanW(Cx + r_c, Cy)` for the circle's diameter dimension.

⚠️ `addByCenterStartEnd` shares an arc's start and end but **copies** its centre
(`[PB-SHARE-XOR-COINCIDENT]`), which is why the centre coincidence above is correct rather than
redundant; found in Fusion 2026-09-02, the bevel tooth-top arc's centre stranded 22.9 mm behind its
origin and gave a 0.5743 mm arc where 22.5 mm was intended, on a sketch that raised no error.

<!-- check-step-calls: ignore addByCenterStartEnd -->

This sketch is **deliberately left with free DOF** — the arc's endpoints are pinned by the three-point
construction, not by endpoint dimensions, because dimensioning them over-constrains the solve against
the cone-element plane. It is therefore exempt from the full-constraint gate; do **not** gate it.

There is **no 3-D projection**: no `projectToSurface`, no root-cone face search, and no 3-D trace
sketch. Earlier versions projected the 2-D arc onto the root cone and measured the trace azimuth
there; that projection is fragile, because for unequal-ratio pairs the arc wraps around the cone and
comes back as multiple disjoint fragments, so the measured azimuth collapses to a fraction of the
true sweep, the pinion comes out grossly under-twisted, and the pair interferes. The analytic
crown-gear law of S32 is exact, deterministic and cannot wrap. Do not reintroduce the projection.

<!-- check-step-calls: ignore projectToSurface -->

The proof substitutes a chord-wise sample of the genuine cutter circle for the engine's arc entity,
because the harness gates every sketch and an engine arc carries an internal
"start and end equidistant from the centre" row that is either redundant when all three points are
grounded or ambiguous when the centre is free. `sketches_test.go` says so and asserts every invariant
the derivation states: the samples lie exactly on the cutter circle, the curve passes through the
mean point, its tangent there makes ψ with the cone element, its ends sit on the toe and heel apex
circles, and the opposite hand is the exact mirror across the cone element.

`tanW` is the 2-D-to-world mapping this step defines inline from `combine_point`, so its spelling
is the module's to choose.

<!-- check-step-calls: ignore tanW -->

**From:** `spec/bevelgear/instructions.md` L609-L620 (§3a step C and step D);
`spec/bevelgear/spiral-tooth-trace.md` L173-L182 (the trace arc), L218-L236 (the invariants);
`spec/bevelgear/fusion.md` L59-L67 (the exemption);
`.claude/skills/generate-gear/PLAYBOOK.md` L448-L454 (`[PB-CIRCLE-CENTER]`), L591-L601
(`[PB-SHARE-XOR-COINCIDENT]`'s arc-centre caveat), L619-L623 (`[PB-RADIAL-DIM]`).

---

## S23 `[GO]` Slice the tooth into cross-section slabs (§3a step E)

Proof function: `stepSliceToothSlabs`.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepSliceToothSlabs, assertSliceToothSlabs) -->

**Step E — slice.** Split the uncut apex→heel `toothBody` into cross-section slabs by planes
perpendicular to the cone element, spanning a touch past the toe and the heel, by a **fixed** slice
scheme of eight planes; the count is **not** user-configurable. The first cut plane is the **parent
transverse tooth plane** (`parentToothPlane`, the `{gearLabel} Plane` of S11, passed into the hook)
offset toward the apex by `span / 6`. The offset **sign is chosen per gear** so it moves toward the
apex: the parent plane's normal points opposite ways for the two gears, so pick `sign` such that
`sign * normal` points apex-ward, testing `(apex − planeOrigin) · normal`. Then step further toward
the apex in `span / 6` increments:

```
offsets = [sign * (k + 1) * span / 6 for k in range(8)]
pieces  = slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)
```

The helper splits piece by piece and keeps a piece whole when a plane misses it.

⚠️ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in **one
piece**, the offset sign was wrong or `parentToothPlane` sits outside the tooth's span — **retry the
whole cut once with the opposite sign**. If it is still one piece, **`raise` a clear self-diagnosing
error** naming the gear, the final piece count, `span` and the sign tried (`[PB-EMPTY-RESULT]`,
`[PB-SELF-DIAGNOSING]`). Do **NOT** return an unsliced result: step F then drops that one piece as
the apex scrap, leaving `segments` empty, and the crown later crashes with
`ValueError: max() iterable argument is empty` far from the cause.


The proof builds each slab at its station rather than cutting it out of a body and then moving it,
and returns the slabs as separate bodies laid a fortieth of a slab apart: decad has neither a
split-by-plane nor a scale feature, it refuses a union whose operands share a facet plane, and its
verification cannot decide whether two bodies that share a face are disjoint or overlapping.
`spiral_test.go` says what that costs — it cannot check that the cut planes really do split the
body, which is the failure step E tells the generator to retry and then raise on.

`range` is a Python builtin, the `max()` above appears inside a quoted Python error message, and
`normal` is the construction plane's own property rather than a call.

<!-- check-step-calls: ignore max normal range -->

**From:** `spec/bevelgear/instructions.md` L622 (step E);
`.claude/skills/generate-gear/PLAYBOOK.md` L174-L177 (`slice_body_by_offset_planes`), L437
(`[PB-EMPTY-RESULT]`), L729-L736 (`[PB-SELF-DIAGNOSING]`).

## S24 `[GO]` Order the slabs and drop the apex scrap (§3a step F)

Proof function: `stepDropApexScrap`.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepDropApexScrap, assertDropApexScrap) -->

**Step F — order and drop the scrap.** Sort the pieces by the `distAlong` of their centroid,
`body.physicalProperties.centerOfMass`. The first, apex-most piece is the long **apex-side scrap**
below the toe: remove it and keep the rest as the working `segments`. **Drop it by re-slicing the
list first, then delete it** — `segments = segments[1:]` before
`designComponent.features.removeFeatures.add(scrap)` (`[PB-REMOVE-PIECES]`). After the drop,
`segments` must be non-empty; if it is empty the slice failed in step E, so `raise` a clear error
rather than proceeding into the twist and crown, which assume at least one segment.


The proof builds each slab at its station rather than cutting it out of a body and then moving it,
and returns the slabs as separate bodies laid a fortieth of a slab apart: decad has neither a
split-by-plane nor a scale feature, it refuses a union whose operands share a facet plane, and its
verification cannot decide whether two bodies that share a face are disjoint or overlapping.
`spiral_test.go` says what that costs — it cannot check that the cut planes really do split the
body, which is the failure step E tells the generator to retry and then raise on.

**From:** `spec/bevelgear/instructions.md` L624 (step F);
`.claude/skills/generate-gear/PLAYBOOK.md` L437 (`[PB-EMPTY-RESULT]`), L737-L741
(`[PB-REMOVE-PIECES]`), L729-L736 (`[PB-SELF-DIAGNOSING]`).

## S25 `[GO]` Twist the slabs about the shaft axis (§3a step G)

Proof function: `stepTwistSlabs`.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepTwistSlabs, assertTwistSlabs) -->

**Step G — twist.** Rotate each segment about the **shaft axis** (`axisDir` through `apex`) so the
tooth follows the trace, **centred on R_mean so the mid-face section stays unrotated** — that section
then meshes exactly like the straight tooth, and the pinion's zero mesh nudge depends on it. The
total toe→heel twist comes from the **conjugate crown-gear generation law**: a spiral bevel is
generated by an imaginary flat crown gear, and the work gear's shaft rotation relates to the developed
crown-plane azimuth by the roll ratio `1 / sin γ`, since the generating crown gear has `N / sin γ`
teeth. Compute it **analytically — no projection, no curve sampling:**

```
phi_crown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`gamma` is this gear's **PITCH** cone angle, `self._gamma_p` for the pinion and `self._gamma_g` for
the driving gear, forwarded into the hook. ⚠️ **Never `acos(coneVec · axisDir)`**, which is the
*root* cone angle — about 14° against a pitch angle of about 29° for a 17-tooth pinion — and yields a
twist roughly 1.6 times too large. ⚠️ The two members of a meshing pair **legitimately get different
twists**: same cutter, same ψ, but γ differs, so `1 / sin γ` differs — about 2.08 for a 17-tooth
pinion against about 1.14 for a 31-tooth gear, a ratio near 1.83. That is *why* equal-teeth pairs
always meshed while ratio pairs failed under any method that gets `1 / sin γ` wrong.

Each segment's rotation is a **linear share keyed to the cone distance of its HEEL FACE** — the exact
section the later loft samples:

```
ang = -handSign * total * (R_mean - R_heelFace(seg)) / span
```

**Define a slab's heel face precisely: the face whose centroid has the GREATEST
`distAlong(face.centroid)`, searched across ALL of the slab's faces with NO surface-type filter**;
its toe face is the least-centroid one. ⚠️ Do **not** restrict the search to `PlaneSurfaceType` or any
other surface type — a sliced slab is bounded by a mix of the two planar cut faces and ruled side
faces, and a type filter can pick the wrong face or miss the cut face, which makes the step-I loft
fail with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this same **all-faces-by-centroid** rule
(max → heel, min → toe) everywhere a slab end face is needed: here, in the crown, and in the loft.
⚠️ Key the twist on the segment's **heel-face** cone distance, **not** its centroid: the loft samples
each segment's heel face, so that face is what must land at the right azimuth, and centroid-keying
leaves the loft's mid-face section rotated by half a segment and overlapping.

<!-- check-step-calls: ignore PlaneSurfaceType -->

Apply the rotation with a free move:
`rotate_body_about_edge` is not used here — build the matrix directly with
`adsk.core.Matrix3D.setToRotation(ang, axisDir, apex)` and apply it through
`designComponent.features.moveFeatures.createInput2(bodyCollection)` →
`moveInput.defineAsFreeMove(matrix)` → `designComponent.features.moveFeatures.add(moveInput)`
(`[PB-MOVE-ROTATE]`). Use `defineAsFreeMove` with a matrix, never `defineAsRotate`, which rejects a
`SketchLine` axis. A zero angle is a no-op, not a move: `setToRotation(0, axis, origin)` builds the
identity and Fusion refuses it with `RuntimeError: 3 : invalid transform`, so a computed angle of
zero must return early rather than reaching the move.

<!-- check-step-calls: ignore defineAsRotate -->


The proof builds each slab at its station rather than cutting it out of a body and then moving it,
and returns the slabs as separate bodies laid a fortieth of a slab apart: decad has neither a
split-by-plane nor a scale feature, it refuses a union whose operands share a facet plane, and its
verification cannot decide whether two bodies that share a face are disjoint or overlapping.
`spiral_test.go` says what that costs — it cannot check that the cut planes really do split the
body, which is the failure step E tells the generator to retry and then raise on.

`distAlong` and `R_heelFace` are the local helpers step A defines, and `abs` is a Python builtin.
`acos(coneVec · axisDir)` is named ONLY to forbid it — it measures the root cone angle where the
pitch cone angle is wanted — and `rotate_body_about_edge` is named only to say that this step
does not use it; its call site is the meshing rotation.

<!-- check-step-calls: ignore distAlong R_heelFace abs acos rotate_body_about_edge -->
<!-- check-compile: ignore distAlong -->

**From:** `spec/bevelgear/instructions.md` L626-L639 (step G);
`spec/bevelgear/spiral-tooth-trace.md` L186-L214 (the analytic twist and the `1/sin γ` factor);
`.claude/skills/generate-gear/PLAYBOOK.md` L767-L776 (`[PB-MOVE-ROTATE]` including its
zero-angle note), L436 (`[PB-WORLD-FRAME]`).

## S26 `[GO]` Crown the slabs lengthwise (§3a step H)

Proof function: `stepCrownSlabs`.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepCrownSlabs, assertCrownSlabs) -->

**Step H — the lengthwise crown.** Crown the tooth by scaling each segment **except the outermost
(heel) one** down by a **monotonic** factor — full at the heel, growing smoothly toward the toe —
about a sketch point on the **ROOT edge of its heel face**. For each segment compute its heel-distance
fraction

```
u      = (R_heel - R_heelFace) / span      # 0 at the held heel, 1 at the toe
factor = 1 - _CROWN_PER_RAD * (abs(total) / 2) * u
```

with `R_heelFace` found by the same all-faces-by-centroid rule but **RECOMPUTED here, AFTER the
step-G twist has moved the slabs** — do not reuse pre-twist values — and `R_heel` and `span` from
step A. `abs(total) / 2` is the per-end peak twist magnitude, so the maximum relief, now at the toe,
keeps the magnitude the old per-end peak had. This makes relief grow monotonically from the full heel
to the toe, so slab heights stay strictly ordered heel→toe and the natural cone taper is never
reversed. If a computed `factor` comes out ≤ 0, `raise` a self-diagnosing error naming the gear, the
segment's `u` and the factor; never scale by a non-positive factor. `_CROWN_PER_RAD` is a tunable
class constant with the value **0.5**; 0 would disable the crown, so set it to 0.5 and do not leave
it unset.

⚠️ **Do NOT key the relief on `abs(ang)`**, the twist magnitude. That is symmetric about mid-face and
maximal at BOTH ends, so, because the heel slab is held full, the slab *just inside* the heel becomes
the most-relieved one and dips below both its neighbours — a notch that reverses the heel→toe taper.
This was the observed bug: the heel-adjacent slab came out at factor 0.932 while the next slab inward
was 0.972, taller. Key on the monotonic heel-distance `u`, never on `abs(ang)`.

Three further gotchas:

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). `scaleFeatures` is the ONE exception to
   never-activate: it needs the Design occurrence as the **active** edit target, so call
   `self.designOccurrence.activate()` before the crown scales and restore afterwards, **in a
   `finally`**, with `self.design.activateRootComponent()`. ⚠️ Do **not** write
   `design.rootComponent.activate()` or `someComponent.activate()` — a `Component` has **no**
   `.activate()` method and raises `AttributeError`. Only `Occurrence` has `.activate()`, and the
   root is re-activated through `Design.activateRootComponent()`.
2. **Skip the outermost (heel) segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone trims it flush with the gear base. "Outermost" means the segment with the
   **GREATEST post-twist heel-face `distAlong`**: sort by that and skip the last.
3. **Anchor the scale on the heel face's ROOT edge, not its centroid.** `scaleFeatures` shrinks
   uniformly toward the base point, so a base point at the heel-face **centroid**, at mid tooth
   height, pulls the tooth's **root** edge upward by `(1 − factor) * (½ tooth height)`: the tooth no
   longer seats on the gear body's root cone, floats above the base, and the Combine-Join leaves a
   gap — clearly visible for ratio pairs such as module 2 with driving 19 and pinion 13, which is the
   symptom that exposed this. Put the base point on the root instead: of the heel face's vertices
   (`heelFace.vertices`, each `.geometry` a world `Point3D`), take the **two with the smallest
   perpendicular distance to the shaft axis** — the line through `apex` along `axisDir`, so the
   distance is `|(p − apex) − ((p − apex) · axisDir) * axisDir|` — which are the two **root corners**,
   the tip corners being farthest from the axis, and place the base sketch point at their
   **midpoint**, mapped into the heel-face sketch with `sketch.modelToSketchSpace(...)`. The heel face
   is a planar cut, so that midpoint lies on it. A uniform scale about a point keeps every line and
   plane through that point invariant, so anchoring on the root keeps the root edge on the seating
   cone while the tip is relieved progressively toward the toe, which is exactly the lengthwise crown
   intended. Finding the heel face itself is unchanged — still the max-`distAlong`-centroid face —
   only the point *on* it changes from centroid to root-edge midpoint.

Apply each scale with
`designComponent.features.scaleFeatures.createInput(inputEntities, basePoint, adsk.core.ValueInput.createByReal(factor))`
followed by `designComponent.features.scaleFeatures.add(scaleInput)`.

<!-- check-step-calls: ignore rootComponent.activate rotate_body_about_edge distAlong slabHeelFace -->
<!-- check-compile: ignore distAlong slabHeelFace -->


The proof builds each slab at its station rather than cutting it out of a body and then moving it,
and returns the slabs as separate bodies laid a fortieth of a slab apart: decad has neither a
split-by-plane nor a scale feature, it refuses a union whose operands share a facet plane, and its
verification cannot decide whether two bodies that share a face are disjoint or overlapping.
`spiral_test.go` says what that costs — it cannot check that the cut planes really do split the
body, which is the failure step E tells the generator to retry and then raise on.

`abs` is a Python builtin, and `design.rootComponent.activate` and `someComponent.activate` are
named ONLY to forbid them: a `Component` has no `.activate()` at all.

<!-- check-step-calls: ignore abs someComponent.activate design.rootComponent.activate -->

**From:** `spec/bevelgear/instructions.md` L641-L653 (step H and its three gotchas);
`.claude/skills/generate-gear/PLAYBOOK.md` L758-L766 (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`),
L788-L795 (`[PB-NEVER-ACTIVATE]` and the scale feature's exception to it), L552-L557
(`[PB-SPACE-METHODS]`).

## S27 `[GO]` Loft the curved tooth (§3a step I)

Proof function: `stepLoftSpiralTooth`.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepLoftSpiralTooth, assertLoftSpiralTooth) -->

⚠️ **Re-sort the segments by their heel-face cone distance HERE, after the twist and the crown — do
NOT reuse the pre-twist slice or centroid order from step F.** The twist rotates each slab about the
shaft axis, and for high-twist *unequal-ratio* pairs that rotation changes the slabs' along-cone order
enough to reorder adjacent slabs; lofting in the stale order assembles the cross-sections out of
sequence, the crowned tooth comes out distorted, and the two gears interfere. For equal or low-twist
pairs the two orders coincide, which is why equal-teeth gears mesh even with the stale order while
unequal ratios distort — this is the single thing that makes a ratio pair like 31/17 fail while 31/31
looks fine. So compute
`order = sorted(segment indices, key=lambda s: distAlong(slabHeelFace(s).centroid))` **now**.

Then loft a **new body** through, in that order:

1. first the **toe-most segment's apex-side (toe-facing) face** — the toe segment is `order[0]`, and
   its toe face is added first so the loft pushes past the toe cone and the toe trim bites;
2. then the **heel-facing face of every segment, iterated in `order`** — each segment's
   farthest-along-the-element face by post-twist centroid, the last of which reaches past the heel
   cone.

```
loftInput = designComponent.features.loftFeatures.createInput(
    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
loftInput.loftSections.add(toeFace)
for i in order:
    loftInput.loftSections.add(slabHeelFace(i))
curvedTooth = designComponent.features.loftFeatures.add(loftInput).bodies.item(0)
curvedTooth.name = f'{gearLabel} Spiral Tooth'
```

Then remove the segment scaffolding with `designComponent.features.removeFeatures.add(...)`; the loft
has captured their faces.

Finally return the flush trim of **step J**: `cut_conical_ends(designComponent, curvedTooth, gearBody,
toeMid, heelMid, apexWorld, gearLabel)`, the same toe-then-heel two-cone trim as the straight tooth
(S18), so the curved tooth's ends sit flush on the gear base. The toe and heel **mesh phasing** is
handled outside this hook, by `_createGearBody`'s mesh-rotate step (S32); the pinion's extra phase is
0 by default because the mid-face section is unrotated and already meshes.

The proof lofts the same nine sections pairwise and keeps the eight bands apart, since decad's loft
takes two profiles and refuses to union a boolean result with the next band across the face they
share. `spiral_test.go` says what that costs and asserts, band by band, that each starts where the
previous one ended and that they run in the post-twist order this step requires.

`distAlong`, `slabHeelFace` and `sorted` name the arithmetic and the ordering, not an API call.

<!-- check-step-calls: ignore distAlong slabHeelFace sorted -->
<!-- check-compile: ignore distAlong slabHeelFace sorted -->

**From:** `spec/bevelgear/instructions.md` L655 (step I), L657 (step J);
`.claude/skills/generate-gear/PLAYBOOK.md` L691-L695 (`[PB-LOFT]`), L737-L741
(`[PB-REMOVE-PIECES]`).

---

## S28 `[GO]` Circular pattern

Proof function: `stepCircularPattern`.

<!-- proof-run: proofkit3d.RunSolid(patternCases, stepCircularPattern, assertCircularPattern) -->

**Pattern.** Circular-pattern the remaining tooth piece around the **shaft-axis edge** — the same
in-sketch profile edge the revolve used, not the §2 construction line:

```
bodies = adsk.core.ObjectCollection.create()
bodies.add(toothBody)
patternInput = designComponent.features.circularPatternFeatures.createInput(bodies, shaftAxisEdge)
patternInput.quantity = adsk.core.ValueInput.createByReal(teethNumber)
patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
patternInput.isSymmetric = False
pattern = designComponent.features.circularPatternFeatures.add(patternInput)
```

Pin all three inputs explicitly; do not rely on Fusion's defaults staying equal to them
(`[PB-CIRCULAR-PATTERN]`). The number of copies equals this gear's Teeth Number. Although the pitch
diameter shrinks from heel toward apex, the *angular* spacing about the shaft axis stays constant at
`360° / N` for the entire face width: the radial taper is already produced by the loft from the Apex
to the heel-end tooth profile, so the pattern just rotates that single tapered tooth into N evenly
spaced copies.

The proof applies ONE pattern increment and measures where it put the tooth, consuming the seed
rather than copying it: decad has no pattern feature, and leaving N copies live in one document
makes its verification check every pair of them for interference, which it cannot decide for
teeth this close together. `solids_test.go` says so. The count and the closure are then
arithmetic over that same increment, and the tooth's angular thickness is checked against the
tooth pitch so a tooth that would not fit N times is caught.

**From:** `spec/bevelgear/instructions.md` L711 (Pattern);
`.claude/skills/generate-gear/PLAYBOOK.md` L660-L670 (`[PB-PATTERN-BODIES]`,
`[PB-CIRCULAR-PATTERN]`).

## S29 `[GO]` Combine-Join

Proof function: `stepCombineJoin`.

<!-- proof-run: proofkit3d.RunSolid(patternCases, stepCombineJoin, assertCombineJoin) -->

**Combine.** Join all patterned tooth pieces to the Gear Body in a single Combine-Join, the Gear Body
as the target and the patterned tooth bodies as the tools:

```
tools = adsk.core.ObjectCollection.create()
for i in range(pattern.bodies.count):
    tools.add(pattern.bodies.item(i))
combineInput = designComponent.features.combineFeatures.createInput(gearBody, tools)
combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
designComponent.features.combineFeatures.add(combineInput)
```

`pattern.bodies` already includes the seed body plus the copies, so do **not** re-add the seed
(`[PB-PATTERN-BODIES]`). It is a `BRepBodies`, and `combineFeatures.createInput` rejects that type, so
copy the items into a fresh `adsk.core.ObjectCollection` and pass that.

The proof joins ONE tooth, because chaining decad's faceted booleans compounds the mesh until an
operand holds a collapsed facet and the evaluator refuses it. What survives is the fact the join
exists to establish: a tooth seated on the root cone leaves one lump, not two, and the joined
body reaches further out than the frustum alone. `solids_test.go` says so.

The proof also sinks the tooth's root a twentieth of the tooth height below the gear body's root
cone, because Fusion's Combine-Join takes the exact face-to-face seating and decad refuses a
boolean whose operands graze without provably crossing. In the generated module the tooth seats
exactly on the cone; nothing here asks for a sink.

`range` is a Python builtin.

<!-- check-step-calls: ignore range -->

**From:** `spec/bevelgear/instructions.md` L713 (Combine);
`.claude/skills/generate-gear/PLAYBOOK.md` L660-L664 (`[PB-PATTERN-BODIES]`).

## S30 `[GO]` `{gearLabel} Bore` sketch

Proof function: `stepBoreSketch`.

<!-- proof-run: proofkit.Run(boreCases, stepBoreSketch) -->

Skip S30 and S31 entirely when Enable Bore is unchecked.

Build the bore plane normal to the shaft at its start, passing the **in-sketch edge**, not the §2
construction line:

```
planeInput = designComponent.constructionPlanes.createInput()
planeInput.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
borePlane = designComponent.constructionPlanes.add(planeInput)
```

Then a sketch named `{gearLabel} Bore` on that plane, with the bore circle centred at the sketch
origin — the plane is rooted at the shaft start, so the origin is already on the axis:

```
circle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
    adsk.core.Point3D.create(0, 0, 0), boreDiameter_cm / 2)
circle.centerSketchPoint.isFixed = True
dim = boreSketch.sketchDimensions.addDiameterDimension(circle, textPoint)
dim.parameter.value = boreDiameter_cm
```

**Fix the centre and dimension the diameter** (`[PB-CIRCLE-CENTER]`): a circle's centre is FREE even
when created at (0, 0, 0), because the creation call does not reuse the sketch's `originPoint`, and
`addCoincident(circle.centerSketchPoint, sketch.originPoint)` has been observed to throw
`VCS_SKETCH_SOLVING_FAILED` on exactly a `setByDistanceOnPath` plane. `isFixed` on the centre plus a
diameter dimension is 2 + 1 degrees of freedom removed and the sketch reaches zero.

The bore diameter is this gear's Bore Diameter if specified, non-zero, otherwise this gear's
`Pitch Diameter / 4`.

Gate the sketch: `if not boreSketch.isFullyConstrained: raise ...` naming it
(`[BEVEL-F-FULL-CONSTRAINT]`).

The `addCoincident` to `originPoint` above is named ONLY to forbid it; this step's pin is `isFixed`.

<!-- check-step-calls: ignore addCoincident originPoint -->

**From:** `spec/bevelgear/instructions.md` L715 (Bore), L100-L104 (bore inputs and the auto rule);
`spec/bevelgear/fusion.md` L21-L30; `.claude/skills/generate-gear/PLAYBOOK.md` L448-L454
(`[PB-CIRCLE-CENTER]`), L742-L753 (`[PB-CONSTRUCTION-PLANES]`).

---

## S31 `[GO]` Bore through-cut

Proof function: `stepBoreCut`.

<!-- proof-run: proofkit3d.RunSolid(boreSolidCases, stepBoreCut, assertBoreCut) -->

Extrude-cut the bore circle as a symmetric through-cut restricted to this Gear Body:

```
extrudeInput = designComponent.features.extrudeFeatures.createInput(
    boreSketch.profiles.item(0), adsk.fusion.FeatureOperations.CutFeatureOperation)
extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(2 * coneDistance_cm), False)
extrudeInput.participantBodies = [gearBody]
designComponent.features.extrudeFeatures.add(extrudeInput)
```

`setSymmetricExtent(distance, isFullLength)` with `isFullLength = False` means the distance is the
half-length **per side**, so `2 * Cone Distance` per side is generously past any face width
(`[PB-THROUGH-CUT]`). Do not pass a third taper argument. Restrict the cut with `participantBodies`
so it pierces only this gear.

**From:** `spec/bevelgear/instructions.md` L715;
`.claude/skills/generate-gear/PLAYBOOK.md` L696-L699 (`[PB-THROUGH-CUT]`).

---

## S32 `[GO]` Meshing rotation

Proof function: `stepMeshRotation`.

<!-- proof-run: proofkit3d.RunSolid(meshCases, stepMeshRotation, assertMeshRotation) -->

Do this **here, in the Design component, before the body is moved out**.

**Driving gear.** Rotate the driving body by `180° / Driving Gear Teeth Number` — half a tooth pitch —
about its shaft axis:

```
rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, math.pi / drivingTeeth)
```

The framework helper takes the rotation axis and origin from the **B→I profile edge's world
endpoints** and applies a free-move matrix (`[PB-MOVE-ROTATE]`). Rationale: both gears are patterned
from a starting tooth in the axial plane, so without the offset a driving tooth and a pinion tooth
would both sit at the axial-plane crossing and visually collide; half a pitch puts a driving valley
where the pinion tooth crosses, giving the interlocked meshing look.

**Pinion.** The pinion additionally gets `_pinionMeshPhase(pinionTeeth)`, which is
`_PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth` radians and therefore **0** by default. A zero
angle is a no-op, not a move: `Matrix3D.setToRotation(0, axis, origin)` builds the identity and Fusion
refuses to move a body by it with `RuntimeError: 3 : invalid transform`, measured 2026-09-02 on
exactly this pinion. `rotate_body_about_edge` absorbs a zero angle and returns early for that reason,
so the call site does not need to guard it.

This runs in Design before `moveToComponent` because a construction axis cannot be added in the
moved-out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation must use the edge's
world geometry while the body is still in Design.

`Matrix3D.setToRotation` is named here to explain what the helper builds; this step calls the helper.
`_pinionMeshPhase` is a private helper whose spelling the spec lets vary.

<!-- check-step-calls: ignore Matrix3D.setToRotation setToRotation _pinionMeshPhase -->

**From:** `spec/bevelgear/instructions.md` L717 (Meshing rotation), L335-L336, L349-L351
(`_pinionMeshPhase`), L725 (the placement note);
`.claude/skills/generate-gear/PLAYBOOK.md` L178-L179 (`rotate_body_about_edge`), L767-L776
(`[PB-MOVE-ROTATE]` and its zero-angle note).

---

## S33 `[PROSE]` Move the finished body into `{gearLabel} Gear`

`gearBody.moveToComponent(gearOccurrence)`. `moveToComponent` preserves the world position and needs
no activation (`[PB-NO-CROSS-SIBLING]`). This is the last thing `_createGearBody` does for this gear;
the loop then repeats S11 through S33 for the driving gear.

**From:** `spec/bevelgear/instructions.md` L317 (the call graph's `moveToComponent`), L673;
`.claude/skills/generate-gear/PLAYBOOK.md` L796-L801 (`[PB-NO-CROSS-SIBLING]`).

---

## S34 `[PROSE]` Cleanup

`hide_construction_geometry(self.bevelComponent)` — the framework helper. It recursively walks the
Bevel Gear component tree, deduping by `entityToken`, and hides every sketch, construction plane and
construction axis with `isLightBulbOn = False`. Construction planes and axes are **not** hidden by
`isVisible = False`; that property has no visible effect on construction geometry, so do not cross
the two (`[PB-HIDE-AFTER-USE]`, `[BEVEL-F-CLEANUP]`). Leave only the two finished gear bodies
visible. There is no sketch-only mode and no per-mode guard — bevel always builds solids.

<!-- check-step-calls: ignore isVisible entityToken -->

The driving gear's half-tooth-pitch meshing rotation is **not** a cleanup step; it happens in S32, in
the Design component, before the body is moved out.

**From:** `spec/bevelgear/instructions.md` L721-L725; `spec/bevelgear/fusion.md` L155-L160
(`[BEVEL-F-CLEANUP]`); `.claude/skills/generate-gear/PLAYBOOK.md` L185-L186
(`hide_construction_geometry`), L626-L638 (`[PB-HIDE-AFTER-USE]`), L802-L804 (`[PB-TREE-CLEANUP]`).
