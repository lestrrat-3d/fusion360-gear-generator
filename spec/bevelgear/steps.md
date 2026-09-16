# Bevel Gear — compiled step list

The proof for this gear is `proof/bevelgear/cases_test.go`, `proof/bevelgear/geometry_test.go`,
`proof/bevelgear/sketches_test.go`, `proof/bevelgear/solids_test.go`, `proof/bevelgear/spiral_test.go`
and the generated `proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `fcf0fe7558cb631a5900c39222d9650658e28c51` |
| `spec/bevelgear/fusion.md` | `754729fc9a33ad9dbf93816e0430e4b94d82e05b` |
| `spec/bevelgear/spiral-tooth-trace.md` | `c9ec08561ced7975aa0ed9ad6a330186259c0d08` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9ee2dcbaed7b5480aa69e9295e8b61acaea081f3` |

## S1 `[PROSE]` Add the twenty dialog inputs

`BevelGearCommandInputsConfigurator.configure(cls, cmd)` adds the inputs below to
`cmd.commandInputs`, in exactly this order. The display order is fixed: Target Plane first so it wins
Fusion's auto-focus (`[PB-AUTOFOCUS-FIRST]` — Fusion focuses the FIRST `SelectionCommandInput` and
ignores a later `hasFocus`), then Center Point so the user flows from plane to point, then the
pre-selected Parent Component, then the numeric and boolean fields.

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

There are **20** dialog inputs and **20 `INPUT_ID_*`** module constants, holding the table's id
strings in row order and named exactly:

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
INPUT_ID_TOE_EXTENSION = 'toeExtension'
INPUT_ID_DRIVING_TOE_RADIUS = 'drivingToeRadius'
INPUT_ID_PINION_TOE_RADIUS = 'pinionToeRadius'
```

Two further module constants hold the dropdown's item strings: `_HAND_RIGHT = 'Right'` and
`_HAND_LEFT = 'Left'`. There are **no `PARAM_*` name strings**, because bevel registers no Fusion
user parameters at all (`[PB-PRECOMPUTED-MODE]`; see S3).

Each selection input is built with `inputs.addSelectionInput(<id>, <label>, <tooltip>)` — the third
argument is the tooltip string from the table and is part of the reproduced surface, so use it
verbatim — then `selection.addSelectionFilter(...)` once per filter and
`selection.setSelectionLimits(1, 1)`. Write every filter as the named constant and never as a quoted
literal (`[PB-SELECTION-FILTER-ENUM]`): `adsk.core.SelectionCommandInput.ConstructionPlanes`,
`adsk.core.SelectionCommandInput.PlanarFaces`, `adsk.core.SelectionCommandInput.ConstructionPoints`,
`adsk.core.SelectionCommandInput.SketchPoints`, `adsk.core.SelectionCommandInput.Occurrences`,
`adsk.core.SelectionCommandInput.RootComponents`. The filter parameter is typed `str` and the
constant's value IS that string, so both spellings work; the constant is used because it is checked
for typos at import and survives a renamed filter, where a literal fails silently by selecting
nothing. The Parent Component input pre-selects the root component with
`parent.addSelection(get_design().rootComponent)`.

Numeric inputs use `inputs.addValueInput(<id>, <label>, <unit>, <ValueInput>)`, the boolean uses
`inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)`, and the dropdown
uses `inputs.addDropDownCommandInput(INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)`
followed by `hand.listItems.add(_HAND_RIGHT, True)` and `hand.listItems.add(_HAND_LEFT, False)`.

**Every `createByReal` default is in Fusion INTERNAL units — cm for length, radians for angle —
regardless of the input's unit string (`[PB-DIALOG-DEFAULT-UNITS]`).** That is why each `mm` default
of 0 is written `adsk.core.ValueInput.createByReal(to_cm(0))` and the 90° Shaft Angle is written
`adsk.core.ValueInput.createByString('90 deg')`, so the expression engine parses it rather than
reading 90 radians. `module` and `toeExtension` are unitless (`''`) and take plain reals.

`configure()` calls `cls._updateSpiralInputVisibility(inputs)` as its **last** step, so the initial
state is correct (default ψ = 35° means both spiral-only rows are shown).

<!-- check-step-calls: ignore configure -->

`configure` is a method the module DEFINES for the command entry point to call, bound by name from
`commands/bevelgear/entry.py`; the module never calls it itself.

**From:** `spec/bevelgear/instructions.md` L27-36, L145-226; `.claude/skills/generate-gear/PLAYBOOK.md`
L138-143, L128-136, L355-357, L557-568.

## S2 `[PROSE]` Drive the spiral-only inputs' conditional visibility

Hand of Spiral (`spiralHand`) and Cutter Radius (`cutterRadius`) are relevant **only** for curved
bevels, so they are **hidden whenever Mean Spiral Angle ψ = 0 and shown when ψ > 0**. Mean Spiral
Angle (`spiralAngle`) itself is the controller and is **always visible** — it is how the user reaches
ψ > 0. There is no declarative show-if in the Fusion API, so this is realized with
`commandInput.isVisible`.

Add `@classmethod def _updateSpiralInputVisibility(cls, inputs)`. It reads the `spiralAngle` input's
**`.expression`** and evaluates it in internal **radians** with
`unitsManager.evaluateExpression(spiral.expression, 'rad')` — it does **not** read the input's
`.value` — then sets `inputs.itemById(INPUT_ID_HAND).isVisible` and
`inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible` to `(value > 0)`. **Guard it:** if any of the
three inputs is `None`, return early, and wrap the expression evaluation in `try`/`except`, because a
half-typed expression can raise mid-edit; on failure leave both inputs **shown**.

`isVisible` only hides the dialog row. The input still exists and `_readInputs` reads it normally,
and the ψ = 0 build ignores Hand and Cutter anyway, so hiding is purely cosmetic and cannot affect
generation.

The dialog's `inputChanged` event drives the reactive update through a second classmethod,
a classmethod named `handle_input_changed` taking `cls` and `args`, whose whole body is
`cls._updateSpiralInputVisibility(args.inputs)` — recompute on **every** input change, which is cheap
and robust and needs no branch on which input changed. That classmethod is bound **by name**
from `commands/bevelgear/entry.py`, through `GearCommand`'s optional `input_changed=` callback.

**From:** `spec/bevelgear/instructions.md` L190-211, L276-281, L320-323;
`.claude/skills/generate-gear/PLAYBOOK.md` L302-321.

## S3 `[PROSE]` Read every input and check its range

`BevelGearGenerator.__init__(self, design)` stores `self.design` and sets `self.bevelOccurrence =
None`. `generate(inputs)` calls `_readInputs(inputs)` first, before anything creates an occurrence.

**Architecture.** Bevel uses a **standalone generator**: it does **not** subclass `base.Generator`
and uses **no `GenerationContext`**. One class builds both straight and spiral bevels — the spiral is
a branch inside the tooth-body step, gated on ψ, not a separate subclass or command. From `base.py`
import only `get_selection` and `get_boolean`; the `Generator`, `ParamNamePrefix` and
`ComponentCleaner` machinery is unused. Imports are explicit, never `import *`.

**No live Fusion user parameters (`[PB-PRECOMPUTED-MODE]`).** Every value — pitch diameters, cone
distance, base heights, bore diameters, virtual tooth counts, face width — is precomputed in Python
in internal cm and written into geometry numerically: sketch dimensions through
`dimension.parameter.value = <number>` and feature inputs through
`adsk.core.ValueInput.createByReal(<number>)`.

**Reading the raw numbers (`[PB-EVAL-EXPRESSION]`).** Read each numeric and angle input by evaluating
its expression, `design.unitsManager.evaluateExpression(input.expression, <units>)` with `''`, `'mm'`
or `'deg'` as the table gives. The values come back in Fusion **internal units — cm for length,
radians for angle — regardless of the unit string**, so a `deg` field returns radians and must be
converted with `math.degrees(...)` before any degree-range check. Read the boolean with
`get_boolean`, never `get_value`: `get_value` reaches for `input.expression`, which
`BoolValueCommandInput` does not have, and Fusion raises `AttributeError` at generation time
(`[PB-INPUT-READ]`). Read the dropdown through `inputs.itemById(INPUT_ID_HAND).selectedItem` and take
its `.name`, defaulting to `_HAND_RIGHT` when none is selected; a dropdown is never read with a
`get_*` helper. Both teeth inputs are coerced with `int(round(...))` before validation.

**Units — critical.** The `'mm'` inputs (both Base Heights, both Bore Diameters, Face Width, Tooth
Spacing, both Toe Radii) and the `'deg'` input (Shaft Angle) come back **already in internal units**;
use them as-is and do **not** `to_cm` them again. **`Module` is read with unit `''`, so it comes back
as a raw number that means millimetres** — a module of 1 is 1 mm. Every length derived from Module
must therefore be `to_cm`-converted before it touches geometry: `Pitch Diameter = to_cm(Module *
teeth)`, `Cone Distance = to_cm(...)`, the dedendum `to_cm(1.25 * Module)`, the module-length
construction extensions, and the default Face Width `Cone Distance / 6`. Mixing a raw-mm
Module-derived length with an already-cm `'mm'` input makes the gear come out about ten times off and
the Face-Width bound meaningless.

`_readInputs` returns the 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth,
pinionTeeth, shaftAngle_deg)` and stashes the rest on `self`: `self._drivingBaseHeight_cm`,
`self._pinionBaseHeight_cm`, `self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`,
`self._faceWidth_cm`, `self._toothSpacing_cm`, `self._spiralAngle_rad`, `self._hand`,
`self._cutterRadius_cm`, `self._toeExtension_pct`, `self._drivingToeRadius_cm`,
`self._pinionToeRadius_cm`. `generate()` later stashes `self._coneDistance_cm`, `self._gamma_p` and
`self._gamma_g`, and `_buildGearProfiles` stashes `self._faceWidthResolved_cm`. **Do not introduce a
`GenerationContext`-style class**: per-gear plain dicts (`pinionCtx` / `drivingCtx`) and self
attributes ARE the intended structure.

The range checks this pass makes, in this order:

- `module > 0`.
- each tooth count `>= 3`, the absolute floor.
- `shaftAngle_deg` at least **30** and **below the Maximum Shaft Angle** of S5. The Maximum Shaft
  Angle depends on both tooth counts, so check it after both are read and coerced, and put the
  computed limit in the rejection message.
- every `mm` input non-negative: both base heights, both bore diameters, Face Width, Tooth Spacing,
  both Toe Radii, and Cutter Radius.
- Mean Spiral Angle in **[0, 60)** degrees, converted from radians first.
- Toe Extension in **[0, 100]**.

<!-- check-step-calls: ignore generate -->

`generate` is a method the module DEFINES for the command entry point to call; the module never calls
it itself.

**From:** `spec/bevelgear/instructions.md` L35-37, L100-105, L124-144, L219-265, L267-315, L328-330;
`.claude/skills/generate-gear/PLAYBOOK.md` L103-118, L852-867.

## S4 `[GO]` Resolve the cone angles, the Pitch Cone Distance and the per-gear windows

Realised by the proof function `stepConeGeometry`.

<!-- proof-run: proofkit.RunParallel(sketchCases, stepConeGeometry) -->

With both tooth counts and the Shaft Angle known, compute, once, in Python:

```
PPD = Module * Pinion Gear Teeth Number          # Pinion Gear Pitch Diameter
DPD = Module * Driving Gear Teeth Number         # Driving Gear Pitch Diameter
tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)      # use atan2(sin Σ · PPD, DPD + PPD · cos Σ)
γ_g     = Σ − γ_p
R       = (PPD / 2) / sin γ_p                    # the PITCH CONE DISTANCE
Cone Distance = sqrt((Module · Driving Teeth)**2 + (Module · Pinion Teeth)**2)
```

**"Cone Distance" and "Pitch Cone Distance" are two different lengths and both are used.** The Cone
Distance is the diagonal of the two pitch diameters and depends on the tooth counts only, never on
the Shaft Angle. The Pitch Cone Distance `R` is the real apex-to-heel length along the pitch cone.
They coincide as `Cone Distance = 2 · R` **exactly when Shaft Angle is 90°**, for any pair of tooth
counts, and diverge everywhere else: an equal 31/31 pair at 30° has `Cone Distance = 43.84 mm`
against `R = 59.89 mm`, and at 140° `R = 16.49 mm`. Where a step below says "Cone Distance" it means
the diagonal; `R` is always written `R`.

**Maximum Shaft Angle.** A pitch cone angle reaching 90° turns that gear's cone inside out: `R · cos
γ` — the along-shaft seed length this spec uses for Apex→A and Apex→B, and the denominator of the
back-cone virtual pitch radius in S12 — passes through zero and changes sign, so the seed points
backwards along the shaft and the virtual radius is unbounded. Both cone angles stay below 90°
exactly while

```
cos(Shaft Angle) > -min(DPD, PPD) / max(DPD, PPD)
```

so `acos` is a hard singularity and the range check must reject a Shaft Angle **at or above**
`degrees(acos(-smaller / larger))`, naming the computed limit in the message. A 31/17 pair gives
`acos(-17/31) = 123.26°`. Equal tooth counts give `acos(-1) = 180°`, which is no constraint at all.
**The Maximum Shaft Angle is that cone-angle limit capped at 150°**, and the cone-angle half is
exclusive while the 150° half is inclusive; 150° is a practical ceiling on the figure, not a measured
one.

**Minimum Teeth, per gear, checked first.** On top of the blanket `teeth >= 3`, require

```
this gear's Teeth Number >= 5.27 * cos γ          # that gear's own γ
```

and reject below it naming the computed floor. The constant is `2 * (1.05 * 1.25 / 0.95 + 1.25) =
5.2632`, rounded **UP** to 5.27 so the published floor stays at or above the exact crossing; do not
round it down and do not substitute the exact value. At Shaft Angle 90° the floor is 3.72, i.e. **4
teeth**: with both base-height bounds applied an equal 4-tooth pair solves and a 3-tooth pair still
fails on the heel edge, because its Maximum Base Height has fallen below its Minimum.

**The base-height window, per gear, second.** Both bounds are closed-form and need no solved sketch
geometry, because `r` (that gear's pitch radius), `γ` and Module are all known before S10 draws
anything, so resolve them here during input validation.

```
Minimum Base Height = 1.05 * 1.25 * Module * sin γ
Maximum Base Height = 0.95 * (r - 1.25 * Module * cos γ) * tan γ
```

**Read the Maximum's origin carefully, because it is easy to get wrong by one dedendum.** The base
height is the offset dimension between the A→Apex2 drop and G→H (and the B→Apex2 drop and I→J), so it
is measured from **Apex 2's plane**, not from the dedendum point. Walking out along the dedendum line
from Apex 2, the perpendicular distance to the shaft axis falls at `cos γ` per unit and the
along-shaft coordinate rises at `sin γ`, so H reaches the axis when the base height reaches `r * tan
γ`. That is the true crossing. The bound above sits `1.25 * Module * sin γ` **below** it because it
starts from the dedendum corner C instead of the pitch point, and it is therefore **deliberately
conservative, not exact**: it refuses a band of base heights that would in fact still build. That is
the trade, since past the true crossing the hexagon has crossed its own axis of revolution and the
revolve fails with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`). The exact bound would be `0.95 * r * tan γ`;
do not adopt it without re-running the low-tooth-count cases.

The Minimum exists because H is placed a module beyond C along the base-height offset: unless the
base height carries H past the dedendum's own along-shaft projection `1.25 * Module * sin γ`, H lands
*behind* C and the edge C→H runs back inward instead of outward. The `1.05` margin mirrors the `0.95`
the other bounds carry.

Apply the window in **both** directions, per gear:

- when the base height was not specified, use `min(max(<the fallback>, Minimum), Maximum)`;
- when the user specified a value outside either end, reject it with a message naming the bound it
  broke, rather than proceeding.

The driving fallback is `Module * Driving Gear Teeth Number / 8`. It clears the raw projection
exactly while `Driving Gear Teeth Number > 10 * sin γ` — at Shaft Angle 90° that is 7.07, so an equal
8-tooth pair was the smallest that built and a 7-tooth pair failed. Raising the fallback to the
Minimum Base Height is what lets small tooth counts build at all; **do not instead raise the teeth
floor**, which would refuse gears that are perfectly buildable once the base height is bounded.

Order matters between the two checks: the Minimum Teeth check is exactly the statement that the
base-height window is non-empty, so running it first means the window step never has to describe what
to do when the minimum exceeds the maximum.

⚠️ **A configuration can satisfy every bound here and still be refused as near-singular by a
constraint solver.** That conditioning limit belongs to the particular lattice, not to this spec:
three independently written nets, each holding DOF 0 with nothing redundant, do not agree on which
end of the Shaft Angle range is reachable. **Do not write a Shaft Angle bound from a conditioning
measurement** and do not narrow the advertised range on one net's evidence. The proof's case table is
where such a measurement belongs; it carries two configurations this net refuses — the default pair
at Shaft Angle 30°, at conditioning `2.832e-05`, and a 31/17 pair at 120°, at `7.149e-07` — as
declared refusals rather than as gaps.

The proof draws the closing quadrilateral Apex / A / Apex 2 / B that S10 builds, solves it, and reads
the two cone angles, `R` and both along-shaft lengths off the SOLVED figure rather than restating
them, then checks each gear's window and floor and the Shaft Angle ceiling.

**Fusion API calls this step requires:** `math.atan2`, `math.acos` and `math.degrees` only; nothing in
`adsk.*` is touched here.

**From:** `spec/bevelgear/instructions.md` L37-52, L58-99, L106-114, L253-265, L505;
`.claude/skills/generate-gear/PLAYBOOK.md` L717-723.

## S5 `[PROSE]` Resolve the Face Width, the Root Length and the toe window

These four resolutions belong to the same input pass, but the Maximum Face Width cannot be evaluated
until the section 2 points A, B, C, D, H and J exist **and are solved**, so the cap is applied inside
S10 and this step states the formulas it applies. S10's proof asserts every one of them against the
solved lattice.

**Face Width.** If unspecified, default to `Cone Distance / 6`. In **every** case, default or
user-specified, the Face Width is bounded by the Maximum Face Width:

- unspecified → `min(Cone Distance / 6, Maximum Face Width)`;
- specified above the Maximum → reject with a message stating the maximum, rather than proceeding;
  the gear-body revolve in S16 would otherwise fail.

The default `Cone Distance / 6` is `R / 3`, the conventional face-width limit, **only at Shaft Angle
90°**. Below 90° it is conservative; above 90° it exceeds `R / 3` and the cap is what actually holds
it. That is deliberate — it keeps the default independent of Shaft Angle — and it is the cap, not the
default, that guarantees a buildable profile.

**Maximum Face Width.** `0.95 *` the smaller of the perpendicular distance from point A to the line
through C and H, and the perpendicular distance from point B to the line through D and J. **Compute
both from the points' SOLVED sketch geometry — `pointA.geometry`, `pointB.geometry`,
`pointC.geometry`, `pointD.geometry`, `pointH.geometry`, `pointJ.geometry` — and NOT from the
pre-solve seed coordinates (`[PB-SOLVED-GEOMETRY]`).** By the time S10 reaches this the constraint
network has located all six, so `.geometry` is exact; seeds diverge substantially for asymmetric
tooth counts or non-90° shaft angles, making a seed-based bound too loose on the binding side, so the
toe still crosses the axis and the cap is defeated. **Compute both and take the minimum**: the pinion
is normally the binding side because its smaller pitch radius gives the smaller distance, but either
gear can be the smaller one. At Shaft Angle 90° this limit equals `0.95 * min(DPD, PPD)**2 / (2 *
Cone Distance)` — the SMALLER pitch diameter, never the pinion's by name. Written with the pinion's
diameter it is wrong whenever the driving gear carries the smaller tooth count: on a Driving 17 /
Pinion 31 pair at Module 1 the real bound is 3.883 mm and the pinion form gives 13.591, so the naive
`Cone Distance / 6` default exceeds it, and the gear fails to generate, for any gear ratio above
roughly √2.

**Toe Radius, per gear.** A user value of **0 means auto-calculate**: use that gear's own inner toe
corner radius at Toe Extension 0, `this gear's Pitch Radius - Face Width / sin γ`, which is the value
that makes Toe Extension 0 today's profile exactly. A user value must be **strictly below that gear's
Toe Radius Ceiling**; reject it with a message naming the ceiling.

```
Toe Radius Ceiling = (this gear's Pitch Radius - 1.25 * Module * cos γ) * (1 - Face Width / R)
```

That ceiling is the gear's **OUTER** toe corner radius at Toe Extension 0. At or above it the point X
below falls behind the toe corner and the Toe Extension has nowhere to go.

**Root Length**, the resolved `|Ded→Toe|` — the segment C→M on the pinion and D→O on the driving
gear. At Toe Extension 0 it is the resolved Face Width re-measured along the root element rather than
perpendicular to the pitch line, which is longer by the dedendum angle's cosine:

```
|Apex->Ded| = sqrt(R**2 + (1.25 * Module)**2)
Root Length at Toe Extension 0 = Face Width * |Apex->Ded| / R
```

**Face Width still resolves exactly as it always did and still carries its cap** — the Toe Extension
adds to what Face Width resolved, it does not replace it.

**Toe Limit, per gear**, is `|Ded→X|` with X the point on the root element `Apex→Ded` at that gear's
Toe Radius:

```
γ_root    = γ - atan(1.25 * Module / R)
Toe Limit = sqrt(R**2 + (1.25 * Module)**2) - Toe Radius / sin(γ_root)
```

X is where the toe end is heading: as the Toe Extension rises the toe corner climbs `Apex→Ded` toward
X while N/P slides in along the toe-radius line to meet it, and at X the toe face has closed to
nothing.

**Toe Extension** is a single percentage in **[0, 100]** applied to **both** gears, because they share
one face and must mesh, and it is read on the **DRIVING** gear with the pinion then built to the same
root length. **0 reproduces today's toe end exactly.** A positive value resolves to

```
Root Length = Root Length at 0 + (pct / 100) * 0.99 * (min(both Toe Limits) - Root Length at 0)
```

**Toe Extension 100 stops at 0.99 of the way to the SMALLER of the two gears' Toe Limits, not at the
limit itself.** The smaller limit wins because the pair shares one root length; the other gear simply
stops short of its own X. The `0.99` is there because AT the limit the toe face has zero length, so
the revolved gear body carries **no cone at its toe end** and the toe conical cut in S18 — which must
split or the build fails — has no `ConeSurfaceType` face to find. The last percent is worth well
under a tenth of a millimetre of root length on every case in the proof's table, so the reach given up
is nil and the failure avoided is total. **Do not drop this factor.**

**A defaulted Toe Radius can leave no room at all, and that is a real configuration rather than a
defect.** On a driving gear with a large pitch cone angle the inner toe corner already sits at a
LARGER radius than the outer one — the toe dish leans toward the heel rather than away from it — so X
falls behind the toe corner and the Toe Limit comes out below the Toe Extension 0 root length.
Measured over gear ratio against Shaft Angle it is a diagonal band crossing 90° for every ratio from
about 2.75 up, and Module does not move its boundary. **Reject a Toe Extension above 0 on such a
pair**, with a message naming the gear and the Toe Radius Ceiling it needs to come below; Toe
Extension 0 still resolves, so the gear itself stays buildable exactly as before. Do **not** silently
substitute a smaller Toe Radius: that would change the toe end of a gear whose inputs asked for no
change.

**Bore diameter, per gear.** Consulted only when Enable Bore is checked. A value of 0 means
auto-calculate: use **this gear's own** `Pitch Diameter / 4`.

**From:** `spec/bevelgear/instructions.md` L100-138, L106-136, L116-122, L124-136, L551.

## S6 `[PROSE]` Create the Bevel Gear component

Create the Bevel Gear component as a child of the user's Parent Component with
`parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` (`[PB-OCCURRENCE-TREE]`) and name it
`occurrence.component.name = 'Bevel Gear'`. Stash the occurrence on `self.bevelOccurrence` so
`deleteComponent()` can roll the whole tree back on failure, and the component on
`self.bevelComponent`.

**Never call `occurrence.activate()` anywhere in this generator** (`[PB-NEVER-ACTIVATE]`,
`[BEVEL-F-NEVER-ACTIVATE]`). The bevel-specific reason: the Anchor Sketch is created on the user's
**external**, root-owned target plane, and an activated occurrence resolves that external plane in
its own local frame, so the build collapses onto world XY regardless of the real plane tilt. All
features run in the single Design component, so no cross-sibling reference is ever needed
(`[PB-NO-CROSS-SIBLING]`). The one exception is the spiral crown's scale in S25, which activates the
Design occurrence and restores the root afterwards.

<!-- check-step-calls: ignore activate -->

<!-- check-step-calls: ignore deleteComponent -->

`activate` is named here only to forbid it outside S25, and `deleteComponent` is a method the module
DEFINES for the entry point to call on failure; neither is a call this step requires.

**From:** `spec/bevelgear/instructions.md` L459-469, L736; `spec/bevelgear/fusion.md` lines
L155-160; `.claude/skills/generate-gear/PLAYBOOK.md` L817-834.

## S7 `[PROSE]` Create the Design component

Create the Design component as a child of the Bevel Gear component, again with
`occurrences.addNewComponent`, and name it `Design`. It holds every sketch, construction plane and
construction axis the build authors, and every feature operation runs in it; the finished bodies are
relocated out at the end. Stash it as `self.designOccurrence` and `self.designComponent`.

**From:** `spec/bevelgear/instructions.md` L465-469, L736.

## S8 `[GO]` Anchor sketch

Realised by the proof function `stepAnchorSketch`.

<!-- proof-run: proofkit.RunParallel(sketchCases, stepAnchorSketch) -->

Start the sketch **directly on the user-selected target plane**, `sketches.add(targetPlane)`, whether
the selection is a `ConstructionPlane` or a `PlanarFace`. Do not re-derive it and do not offset it
(`[PB-USE-SELECTED-PLANE]`): normalising it into a coplanar construction plane inside a sub-component
resolves in that component's own frame and silently loses the selected plane's world orientation, so
the whole build lies flat on XY regardless of what the user picked. Name the sketch `Anchor`.

Mark the centre by projecting the user-specified centre point into the sketch with
`sketch.project(centerPoint)`. A projected point is brought in associatively and is **not** fixed by
the projection (`[PB-PROJECT-NOT-FIXED]`).

Create a line through the projected centre point with `sketch.sketchCurves.sketchLines.addByTwoPoints`
— the curve collections live under `sketch.sketchCurves` and never directly on the sketch
(`[PB-SKETCHCURVES]`). **Seed its two endpoints at exactly ±0.5 cm from the projected centre** along
the sketch-local X, so the seeded length is 10 mm. Then:

- `sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)` — the intersection, which
  pins the centre onto the line;
- `sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)` — the centre bisects the
  line.

Use **both**, not the midpoint alone.

Add an aligned distance dimension with
`sketch.sketchDimensions.addDistanceDimension(anchorLine.startSketchPoint, anchorLine.endSketchPoint,
adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)` and **do not assign
`.parameter.value`**: the dimension simply locks the length at the seeded 10 mm. The value is
arbitrary — nothing downstream reads it, because section 2 derives every direction *relative* to the
projected anchor line.

**Then pin its direction so the sketch ends FULLY CONSTRAINED:** add
`sketch.geometricConstraints.addHorizontal(anchorLine)`. That is sketch-LOCAL
(`[PB-REFLINE-DIRECTION]`), so it works on any tilted target plane; a world-axis lock would
mis-orient the line. With midpoint, length and Horizontal the line has zero degrees of freedom.

**Stash the projected-centre `SketchPoint`** on `self._anchorCenterPoint` so S10 re-projects *this*
anchor-sketch point rather than the raw user-selected centre. After all constraints, gate the sketch:
`if not sketch.isFullyConstrained: raise` naming the sketch (`[PB-FULL-CONSTRAINT]`,
`[BEVEL-F-FULL-CONSTRAINT]`). A free degree of freedom here is a generation defect, not a warning.

The proof models the projection as reference geometry, whose coordinates the solver never moves, and
substitutes the signed horizontal distance for the aligned dimension so the line cannot swap its two
ends; Fusion gets that direction from the seed, and only `abs(target)` may ever reach a parameter
value there (`[PB-DIM-VALUE-SEMANTICS]`).

**From:** `spec/bevelgear/instructions.md` L471-475, L394-397; `spec/bevelgear/fusion.md` lines
L21-30; `.claude/skills/generate-gear/PLAYBOOK.md` L441-478, L838-848, L509-516, L239-251.

## S9 `[PROSE]` Gear Profiles Plane

Create the plane with `component.constructionPlanes.createInput()` then
`planeInput.setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)` and
`constructionPlanes.add(planeInput)`. It includes the Anchor Line and stands at 90° to the target
plane: by default it would lie flush to the anchor line's own plane, and perpendicular is what is
wanted. Name it `Gear Profiles Plane` and stash it on `self._gearProfilesPlane`.

**Build it off the original `targetPlane` as the reference** (`[PB-USE-SELECTED-PLANE]`). This is the
other place the target-plane orientation reaches the bodies; substituting a different plane here also
collapses the gear onto XY.

Pass the `SketchLine` **directly** to `setByAngle` (`[PB-CONSTRUCTION-PLANES]`). Do **not** wrap it in
`adsk.fusion.Path.create` first: `Path.create` on a sketch curve raises
`RuntimeError … InternalValidationError` whenever the curve's owner sketch is not trivially resolvable
in the current multi-component context.

<!-- check-step-calls: ignore Path.create create -->

`Path.create` is named only to forbid it.

**From:** `spec/bevelgear/instructions.md` L479, L596-597;
`.claude/skills/generate-gear/PLAYBOOK.md` L775-786, L838-848.

## S10 `[GO]` Gear Profiles sketch — the section 2 lattice

Realised by the proof function `stepGearProfiles`.

<!-- proof-run: proofkit.RunParallel(sketchCases, stepGearProfiles) -->

Create the sketch on the Gear Profiles Plane with `sketches.add(gearProfilesPlane)`, name it `Gear
Profiles`, and stash it on `self._gpSketch`.

**Every line drawn in this sketch is a construction line:** set `line.isConstruction = True` on the
lattice lines, the toe lines M→N and O→P, and the short reference and connector lines M→C, N→A′, O→D,
P→B′, A′→G, B′→I, C→K/K′ and D→L/L′ alike. The solid features later consume only the per-gear Profile
sketches of S15, never a section 2 curve directly.

**Every length dimension in this sketch is `AlignedDimensionOrientation`.**
`sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, orientation, textPoint)` takes an
`adsk.fusion.DimensionOrientations` value, and this figure has no axis-aligned line in it: the shaft
axes sit at the Shaft Angle to each other, the whole lattice tilts with the target plane, and the
sketch is not world-aligned. `HorizontalDimensionOrientation` or `VerticalDimensionOrientation` would
each dimension the line's *projection* onto a sketch axis instead of its length, so the constrained
value would be the intended one only in the accidental case where the line happens to lie along that
axis. Wherever a paragraph below says "a dimensional constraint with length = X" it means
`addDistanceDimension(..., adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
followed by `dimension.parameter.value = <number in cm>`. The offset dimensions are a different call,
`sketch.sketchDimensions.addOffsetDimension`, which takes no orientation.

**Every line in this sketch uses the COINCIDENT style and never sharing (`[BEVEL-F-COINCIDENT-STYLE]`).**
When a line must start at or connect to an already-existing point — the Apex, A, B, C, the projected
centre, any of them — create the line from raw `adsk.core.Point3D.create` coordinates for BOTH
endpoints and pin the connecting endpoint with exactly one
`sketch.geometricConstraints.addCoincident(line.startSketchPoint, existingPoint)`. Never pass the
existing `SketchPoint` into `addByTwoPoints` to share it. This is load-bearing both ways: sharing
*without* a coincident leaves this sketch **under**-constrained and the gate fails on "Gear Profiles";
sharing **and also** coinciding is redundant and the solve fails outright with `RuntimeError …
VCS_SKETCH_SOLVING_FAILED - failed to create offset`. ⚠️ **This covers the short reference and
connector lines too — the ones whose BOTH endpoints already exist: C→K, D→L, C→K′, D→L′, M→C, N→A′,
O→D, P→B′, B′→I and A′→G.** A regen that shared only those came out about fourteen coincidents short
and the gate failed. No section 2 line is exempt.

**Each named line is created ONCE and later references REUSE that line object
(`[BEVEL-F-LINE-ONCE]`).** The module-length extensions A→E, B→F, E→G and F→I and the dedendum and
closing lines C→H, D→J, G→H and I→J are *named* construction lines: when a paragraph below says "from
point E collinear to line A→E", it means the very line drawn earlier, so a helper that creates a
module extension must RETURN the line and the caller keeps that reference. Drawing a *second* line
between the same two points to obtain a reference over-determines the coupled net and the solve fails
with `RuntimeError … VCS_SKETCH_OVER_CONSTRAINTS - failed to create offset`. One segment ⇒ one line ⇒
its constraints live on that one line. An over-constraint failure is not a guaranteed tripwire,
though: a duplicate whose endpoints carry only per-end coincidents has been observed to solve and
even pass the gate, so do not rely on the solver to catch a duplicate.

**The section 2 driven lengths are NOT dimensioned (`[BEVEL-F-DRIVEN-DIMS]`).** The along-shaft
lengths Apex→A and Apex→B and the module-length extensions are DRIVEN by the closing and collinear
constraints; dimensioning them raises `VCS_SKETCH_OVER_CONSTRAINTS` (`[PB-NO-OVERCONSTRAIN]`). The
"do NOT add a dimensional constraint" notes below are as load-bearing as the dimensions that ARE
added.

### The figure, in build order

**Project the Anchor Sketch's centre `SketchPoint`** — the one stashed in S8, **not** the raw
user-selected centre point — with `sketch.project(self._anchorCenterPoint)`. Both happen to be
coincident, but projecting the anchor-sketch point keeps the chain inside the Design component and
faithful to the anchor geometry; projecting the raw external point is a cross-component reference and
can resolve inconsistently. Project the anchor line the same way.

**Write the call as `sketch.project(entity)` and do not substitute `project2`.** The compiled Fusion
API reference declares `project2(entities, isLinked)` and no `project`, so every gate in this repo
reports the call as unverified; that report is expected and is not a defect to fix here. `project` is
what the shipped add-ins call and what the spur step list names, and this repo's settled position is
to keep it and keep reporting it. The two are not interchangeable in any case: `project2` takes a list
and returns a list, so swapping the name alone would be wrong. Only a Fusion session can settle
whether `project` exists at runtime, and if it turns out not to, the fix belongs in the spec rather
than in the generated module.

<!-- check-step-calls: ignore project2 -->

`project2` is named only to forbid substituting it.

**Centre→Apex.** From the projected centre point, draw a construction line **perpendicular to the
projected anchor line** — `sketch.geometricConstraints.addPerpendicular(centerToApex, projectedAnchorLine)`.
Its far end is the **Apex**, placed in sketch-local coordinates at

```
apex2d = c + perp * (R * cos γ_g + <resolved Driving Gear Base Height>)
```

where `c` is the projected centre and `perp` is the in-plane unit vector perpendicular to the
projected anchor line, `(-d.y, d.x)` for anchor-line direction `d`. **Seed it at that distance and
not at the Driving Gear Pitch Diameter**, which is what earlier revisions said: the constraint net
closes this line at `R · cos γ_g` above point I plus the resolved driving base height, so for the
default 31/31 pair at Shaft Angle 90° the old seed sat 11.6 mm past where the solve puts it, 31 mm
seeded against 19.375 mm solved. Fusion converges from the far seed, so this was latent rather than
broken there, but a seed that disagrees with its own closure by that margin is a seed waiting to pick
the wrong branch (`[PB-SEED-NEAR]`).

**The apex POSITION is sketch-local (`[BEVEL-F-APEX-LOCAL]`) — do NOT compute it from a
world-coordinate round-trip**, which is what caused the XY collapse. The **sign of `perp`** is chosen
by the target-plane normal as a one-bit direction (`[BEVEL-F-GROW-SIDE]`): point `perp` toward the
normal, and read that normal as **`targetPlane.geometry.normal`** for BOTH selection kinds, since a
`BRepFace`'s `geometry` and a `ConstructionPlane`'s `geometry` are each a `core.Plane` carrying
`.normal`. A sketch-local rule like `perp.y >= 0` is deterministic but not tied to a physical side, so
the gear would grow inconsistently. The single permitted world use in this sketch is reading that
normal as a *direction*. Do **not** add a length constraint on this line; it is pinned later by
"Constrain Point I with the centre point".

**Driving Gear Shaft Axis.** From the apex, pointing back toward the anchor line, i.e. in the `-perp`
direction. **Seed its far end at `apex - perp * (R * cos γ_g)`, which is `c + perp * (<resolved
Driving Gear Base Height>)`** — measure from the apex, not from `c`. Earlier revisions said `c - perp
* (some length)`, which puts B on the far side of the projected centre from the apex, the wrong side
of the figure entirely; the closure at Apex 2 drives `|Apex→B|` to `R · cos γ_g`, so B solves to
exactly one base height above `c`. It must run **parallel to the centre→apex construction line**:
`sketch.geometricConstraints.addParallel(drivingShaftAxis, centerToApex)`. **Do NOT use
`addVertical`**, which forces the line to the sketch's world vertical — wrong on a tilted target plane
and it over-constrains or mis-orients the figure. Coincident its beginning to the apex. The end of
this line is point **B**. Do **not** dimension its length.

<!-- check-step-calls: ignore addVertical -->

`addVertical` is named only to forbid it.

**Pinion Gear Shaft Axis.** The Driving Gear Shaft Axis direction rotated about the apex by the Shaft
Angle. Rotating has two senses and they place point A on opposite sides; choosing wrong mirrors the
whole gear onto the wrong side of the target plane. **Select the sense this way: form both candidate
point-A positions — the driving-shaft direction rotated about the apex by +Shaft Angle and by −Shaft
Angle — and keep the candidate whose endpoint has the greater X coordinate in this sketch.** Compare
the two candidates' X and take the larger; do **not** rotate one fixed sense and flip it only when its
X comes out negative, because when *both* candidates have a positive X that shortcut keeps the wrong
one, which is exactly the side-flip to avoid. Call the chosen unit Apex→A direction `pinionDir`.

Apply an angular dimension between this line and the Driving Gear Shaft Axis equal to the Shaft
Angle: `sketch.sketchDimensions.addAngularDimension(drivingShaftAxis, pinionShaftAxis, textPoint)`
then `dimension.parameter.value = <Shaft Angle in radians>`. **Place its text point inside the Σ
wedge so it measures Σ and not its supplement 180−Σ** (`[PB-ANGULAR-DIM]`) — for example on the
interior bisector, `apex + normalize(pinionDir + drivingDir) * (PPD / 4)`, where `drivingDir` is the
unit Apex→B direction. The angular dimension fixes the angle *magnitude* only; it does not pin which
side the pinion lies on, and the text point does not prevent a frame flip. Coincident its beginning to
the apex. The end of this line is point **A**. Do **not** dimension its length.

**Seed the along-shaft lengths with the closed-form cone geometry** so the solver converges on the
right branch for any Σ. These are seed coordinates only — the lengths stay undimensioned, fixed by the
Apex 2 closure: `|Apex→A| = R * cos γ_p` and `|Apex→B| = R * cos γ_g`. Both cosines are positive for
every Shaft Angle the range check admits, which is what the Maximum Shaft Angle guarantees. Seeding A
or B merely by a pitch diameter is wrong for Σ ≠ 90° and can send the solver to the wrong branch.

**The A→Apex 2 drop.** From A, a construction line perpendicular to the Pinion Gear Shaft Axis, drawn
toward the side where Apex 2 will lie. ⚠️ **Apex 2 sits in the interior wedge *between* the two shaft
axes, so this drop must point toward the OTHER (Driving) shaft axis and point B, NOT "toward the
anchor line".** Pick the perpendicular sense by the sign of its dot product with the A→B direction.
Apply `addPerpendicular` against the Pinion Gear Shaft Axis, and a dimensional constraint with length
= **Pinion Gear Pitch Diameter / 2**, which is the pinion's pitch radius at the heel and therefore the
perpendicular distance from Apex 2 to the Pinion Gear Shaft Axis for any Shaft Angle. Coincident its
beginning to A.

**Naming convention used throughout: this perpendicular drop line — A to its far end, which becomes
Apex 2 — is what "A→Apex2" always refers to. It is NOT the Apex→A shaft axis.** The two share point A
but are different lines, one the PPD/2 perpendicular drop and the other the shaft axis. Whenever a
later step says to pin something to, or dimension against, "A→Apex2", it means this drop line. The
same holds for "B→Apex2" (the DPD/2 drop) against the Apex→B shaft axis.

**The B→Apex 2 drop.** From B, perpendicular to the Driving Gear Shaft Axis, drawn toward the side
where Apex 2 will lie. ⚠️ **This drop must point toward the OTHER (Pinion) shaft axis and point A** —
pick the sense by the sign of its dot product with the B→A direction. **Do NOT choose this sense by a
"toward the anchor line" reference, i.e. the −perp grow direction: the Driving Gear Shaft Axis is
itself parallel to that direction, so the perpendicular's dot with it is ≈ 0, a degenerate test that
silently selects an arbitrary and usually wrong side.** This is the critical failure: if this drop
seeds Apex 2 on the wrong side of the driving shaft while the pinion's drop seeds it on the correct
side, the coincidence that closes the two drops makes the solver **flip the entire frame to the mirror
solution** — point A jumps to the opposite side, the pinion dedendum C collapses onto the driving
dedendum D, the pinion inverts so the toe ends up *outside* the heel, the revolved frustum is
degenerate, and the conical end cut finds no cone face at the toe midpoint and reports `face dist =
inf`. **Both Apex 2 drops must aim at the same interior-wedge point.** Apply `addPerpendicular`
against the Driving Gear Shaft Axis and a dimensional constraint with length = **Driving Gear Pitch
Diameter / 2**. Coincident its beginning to B.

Constrain the two drops' end points together with `addCoincident`. That point is **Apex 2**. At Shaft
Angle 90° the four points Apex, A, Apex 2 and B form a rectangle; for other shaft angles the figure is
a non-rectangular quadrilateral, and the lengths of Apex→A and Apex→B adjust so the two drops of
length PPD/2 and DPD/2 coincide. The quadrilateral deliberately lies well above the anchor line: the
Apex's offset of `R · cos γ_g` plus the resolved Driving Gear Base Height keeps the whole figure above
it across the supported Shaft Angle range.

**The Pitch Line.** A construction line from Apex to Apex 2, each end coincident to its point.

**The two dedendum lines.** From Apex 2, in either direction, two construction lines each
perpendicular to the Pitch Line (`addPerpendicular` against it) with a dimensional constraint of
length **Module * 1.25**. The one drawn **towards** the anchor line is the **Driving Gear Dedendum**
and its end point is **D**; the one drawn **away** from the anchor line is the **Pinion Gear
Dedendum** and its end point is **C**.

**The two root axes.** Construction lines from the Apex to D and to C, with coincidence constraints on
both ends of each. These are the Root Axis for the driving and the pinion gear respectively.

**The pinion module chain.** From A, a construction line collinear with Apex→A, extending for a length
equal to Module — **seed only, do NOT add a dimensional constraint**. Give it
`sketch.geometricConstraints.addCollinear(lineAE, pinionShaftAxis)` and coincident the end of Apex→A
to the beginning of the new line. Its end is point **E**. Then draw a construction line from C to E,
coincident at each end to its existing point, and constrain **A→E and C→E perpendicular**. Note that E
is therefore DRIVEN to the foot of the perpendicular from C onto the shaft axis, which is one
dedendum's along-shaft projection past A rather than one module past it.

From E, a construction line collinear with **line A→E** — the collinear names A→E and **never the
Apex→A shaft axis further up the chain**, even though both describe the same infinite line
(`[BEVEL-F-COLLINEAR-CHAIN]`, `[PB-COLLINEAR-CHAIN]`). `addCollinear` carries two point-on-line rows,
and when the new line's start is already pinned to the reference line's own endpoint Fusion absorbs
one of them; when the start reaches the named line only *through an earlier collinear*, the two chains
assert the same fact independently and the sketch over-determines. Measured on this lattice,
`addCollinear(E→G, Apex→A)` raised `RuntimeError: 3 : failed to create offset:
VCS_SKETCH_OVER_CONSTRAINTS` at the second such call, the first having been absorbed, and
`addCollinear(E→G, A→E)` builds. Give the new line a length equal to Module as a seed, with no
dimensional constraint, and coincident E to its beginning. Its end is point **G**.

From C, a line of seed length Module with C coincident to its beginning; its end is point **H**, and
C→H is made collinear with **line Apex2→C**, the Pinion Dedendum line C is the endpoint of. Connect G
and H with a line, coincident at each end. **Constrain line E→G and H→G perpendicular.**

⚠️ **That perpendicular is required in Fusion and must be omitted in the proof harness, and the reason
is a difference between the two engines rather than a choice.** `addOffsetDimension` in Fusion is a
*distance* dimension with a precondition: its documentation requires the second entity to be "a line
that is parallel to the first", and it controls only the perpendicular distance. So the parallelism
has to exist before the pinion base-height offset below can be applied at all, and this perpendicular
is what supplies it — E→G runs along the pinion shaft, so making H→G perpendicular to it makes H→G
parallel to the A→Apex2 drop, which is perpendicular to that same shaft. Perpendicular plus offset is
two equations for two freedoms and nothing is redundant. The proof harness's offset constraint is not
the same shape: it emits **two** residual rows, holding *both* endpoints of the target line at the same
signed perpendicular distance from the source, so it carries the parallelism itself, and adding this
perpendicular there is a third row for the same two freedoms — measured, the lattice comes back
overconstrained at DOF 0 with 2 redundant constraints and the engine names the two base-height offsets
as the redundant pair. A proof that models Fusion's arity here fails its own gate, and the right
response is to leave the perpendicular out of the proof and say so, never to weaken the gate
(`[PB-NO-OVERCONSTRAIN]`).

**The driving module chain** is the same figure with B for A, D for C, and F, I, J for E, G, H: B→F
collinear with Apex→B, D→F with **B→F and D→F perpendicular**, F→I collinear with **line B→F**, D→J
collinear with **line Apex2→D**, I→J connecting them, and **line F→I and J→I perpendicular** — the
driving twin of the G→H case, required in Fusion and omitted in the proof for the same reason.

**The two base-height offsets.** Create an offset dimension between the **B→Apex2 perpendicular drop
line** — the DPD/2 drop, NOT the Apex→B shaft axis — and **J→I**, with
`sketch.sketchDimensions.addOffsetDimension(dropB, lineJI, textPoint)` then `dimension.parameter.value
= <resolved Driving Gear Base Height in cm>`. J→I is **already parallel** to the drop by construction,
because J→I ⊥ F→I and F→I runs along the driving shaft, so add **no** extra `addParallel`
(`[PB-OFFSET-DIM]`: a redundant parallel over-constrains and throws `VCS_SKETCH_OVER_CONSTRAINTS`).
The value is the Driving Gear Base Height if specified and non-zero, otherwise `Module * Driving Gear
Teeth Number / 8` — and either way it is the value **after** the driving gear's own Maximum Base
Height has been applied, because the offset set here is what drives the heel edge D→J toward the shaft
axis.

Create the twin offset dimension between the **A→Apex2 perpendicular drop line** — the PPD/2 drop, not
the Apex→A shaft axis — and **G→H**, already parallel by construction because G→H ⊥ E→G, so again no
parallel constraint. The value is the Pinion Gear Base Height if specified and non-zero; otherwise it
is the **RESOLVED** Driving Gear Base Height `* (Pinion Gear Teeth Number / Driving Gear Teeth
Number)`. "Resolved" means the value the driving offset actually used — after the driving side's own
fallback and after the driving Maximum Base Height capped it — and NOT the raw driving input. Then
apply the **pinion's own** Maximum Base Height to the result: the two gears have different pitch cone
angles whenever the tooth counts differ, so the driving cap does not imply the pinion's, and a
scaled-down driving height can still overshoot the pinion's own heel limit.

**Pin the figure.** Constrain point **I** with the projected centre point, `addCoincident`. This is the
one constraint that fixes the Apex's height: the driving shaft is parallel to Centre→Apex and shares
the Apex with it, so it IS the line through the centre, and I is where that line meets the driving
heel edge. The proof states only the row that is not already implied — the centre lies on line I→J —
because the bench engine counts both rows of the coincidence and reports the pair as redundant while
Fusion absorbs one.

**The two back-cone points.** Draw a construction line away from the Apex, starting from point G,
extending along Apex→A; call its end point **K**. Then **pin K with two point-on-line coincident
constraints**, `addCoincident(K, pinionShaftAxis)` and `addCoincident(K, pinionDedendumLine)`, rather
than an `addCollinear` on the connecting lines: by the time K is added G and C are already fixed, so a
collinear here over-constrains and Fusion errors, while the two point-on-line coincidents locate K
exactly, as the intersection of the two lines, without over-constraining. Draw a construction line
from C to K for reference. Build **L** on the driving side the same way — from I along Apex→B, pinned
with `addCoincident(L, drivingShaftAxis)` and `addCoincident(L, drivingDedendumLine)` — and draw D→L
for reference.

**The tooth-centre points K′ and L′ (the Tooth Spacing offset).** The section 3 spur tooth is centred
not at K but at a tooth-centre point **K′**, obtained by shifting K outward along the dedendum line by
**Tooth Spacing**, *away from the lower corner C*. **When Tooth Spacing is 0, which is the default, do
NOT build anything here** — set K′ ≡ K and reuse the C→K reference line, because a zero-length
dimensioned line would be degenerate and one segment gets ONE line (`[BEVEL-F-LINE-ONCE]`). When Tooth
Spacing > 0, draw a construction line starting at K with its far end seeded on the *far side of K from
C* along the dedendum direction, pin its far end the same way K is pinned to its line —
`addCoincident(start, K)` and `addCoincident(Kprime, pinionDedendumLine)` to keep K′ on the dedendum
line — then add a length dimension on this line equal to **Tooth Spacing**. Do **not** use
`addCollinear`, for the same over-constraint reason as K. The far end is K′. Build it **here, inside
this sketch, before the end-of-step full-constraint gate**, so the gate covers it. Finally draw the
tooth-centre reference line **C→K′** for section 3 to use in place of C→K. Only the tooth's centre
moves: the virtual tooth number and the drawn tooth size are unchanged. Build **L′** exactly as K′,
substituting L for K, D for C and the Driving Dedendum line for the pinion's; the reference line for
section 3 is **D→L′**.

**Resolve the Maximum Face Width here.** At this point all of A, B, C, D, H and J exist **and are
solved**, so resolve the Maximum Face Width of S5 from their solved `.geometry` — not the seed
coordinates — and apply it before using Face Width below: cap the auto default to it, and reject a user
value that exceeds it. Skipping this, or computing it from seeds, makes the M→N and O→P lines push N
and P across the shaft axis for asymmetric tooth counts, which fails the gear-body revolve with
`ASM_WIRE_X_AXIS`.

**The pinion toe line M→N.** **Seed BOTH ends at their closed-form solved positions, not near them**
(`[PB-SEED-NEAR]`). Seed M on `Apex→C` at the fraction `1 - <Root Length> / |Apex->C|` from the Apex.
Then seed N by sliding from that M seed along the `C->H` direction by exactly

```
(<M seed's perpendicular distance from the Pinion Gear Shaft Axis> - <Pinion Gear Toe Radius>) / cos γ_p
```

⚠️ **A seed that merely lands somewhere plausible is not enough here, and a wrong one builds the wrong
gear rather than failing to converge.** N's position is fixed by the toe line together with a LENGTH
dimension on the front face, and a length is unsigned: the toe line meets the Toe Radius on BOTH sides
of the shaft axis, so the solver takes whichever side the seed starts on. Seeded below the axis it
converges happily onto the mirror, N comes out on the far side, the revolved hexagon crosses its own
axis of revolution, and Fusion aborts the revolve with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`) at S16 —
pointing at the revolve rather than at the seed that caused it. Two earlier seeding rules are known to
do exactly that, so **do not reinstate either**: sliding from the M seed by the **Root Length**, and
sliding by the **distance from the M seed to A**. Both were written for the scheme that pinned N to the
`A->Apex2` drop, where A was N's real target. Measured on the shipped default pair, Module 1 with 31/31
teeth at Shaft Angle 90° and a Toe Extension of 50%, the Root Length slide puts the N seed at a
perpendicular distance of **−0.27 mm** from the shaft axis — past it — against a solved N at **+5.17
mm**, and Fusion refuses the revolve; the slide above puts it at 5.17 mm exactly. Do NOT seed M and N
just `Face Width` away from C and H either, which starts N near H, far from its constraint target.

**The proof cannot catch a wrong seed here, and says so beside its own seeding.** It seeds M and N at
the closed form, which is the rule above, so it proves that the constraints solve from a correct seed
and never that the module's seed is correct. A seed defect therefore reaches Fusion untested, which is
how the one described above got there.

Then apply **exactly these constraints**, all three of which are required:

- `addCoincident(M, pinionRootAxis)` — M lies on the Apex→C root axis;
- `addParallel(lineMN, lineCH)` — the toe line is parallel to C→H. M→N is a freshly drawn line at an
  arbitrary angle, so the parallel IS needed here, unlike the base-height offsets above
  (`[PB-OFFSET-DIM]`);
- `addOffsetDimension(lineCH, lineMN, textPoint)` then `dimension.parameter.value = <the Root Length
  re-measured perpendicular to the pitch line, i.e. Root Length * R / |Apex->C|>`. An offset dimension
  controls a perpendicular distance, so it carries the root length in that form. At Toe Extension 0
  the value is exactly the resolved Face Width, which is what this dimension has always been. Place the
  `textPoint` in the gap between C→H and M→N on the Apex side — the midpoint of the M seed and point C,
  `(M_seed + C) / 2` — so the dimension reads cleanly. The toe's side relative to the heel follows from
  the frame being built correctly, in particular from the Apex 2 drops aiming at the interior wedge; it
  is **not** controlled by this text point.

The beginning of the new line is point **M** and the end is point **N**. Draw a line from M to C.

**The pinion front face A′→N, which is what holds N.** ⚠️ **N is NOT pinned to line A→Apex2.** Earlier
revisions pinned it there, which fixed its station at A's and made the Maximum Face Width the value at
which N reached A. It now rides the **Pinion Gear Toe Radius** instead:

- draw a line from N to a new point **A′**, seeding A′ at N's station on the shaft axis;
- `addCoincident(Aprime, pinionShaftAxis)` — A′ lies on the **Apex→A shaft axis**. A′ is the only
  toe-end point that touches that axis, and it is a *foot*, not a corner;
- `addPerpendicular(lineNAprime, pinionShaftAxis)` — the front face stands square to the shaft, so the
  revolve sweeps it into a flat annulus;
- an aligned distance dimension on the whole line N→A′ set to the **resolved Pinion Gear Toe Radius**.

⚠️ **Pinning N itself to the Apex→A shaft axis remains forbidden** — that would put N *on the axis of
revolution*, and the later conical split fails with `ASM_API_FAILED` for asymmetric tooth counts even
though the symmetric 45° case happens to survive. A′ sits on the axis; N never does, because the Toe
Radius is strictly positive. Those three rows plus the offset and `addCoincident(M, pinionRootAxis)`
fully constrain M, N and A′ — six freedoms, six constraints — which is the arity the old drop pin and
the old N→A connector had between them.

**A′ replaces A as the hexagon's first vertex.** At Toe Extension 0 with a defaulted Toe Radius the two
coincide exactly, so nothing moves; a positive Toe Extension walks A′ along the shaft axis toward the
Apex and the shaft edge grows by that much. Draw the hexagon's shaft-axis edge **A′→G**, coincident at
both ends. It starts at the front face's foot A′, not at A.

**The driving toe line O→P** is the mirror of M→N. **Seed it the same way, at the closed-form solved
positions**: O on `Apex→D` at the fraction `1 - <Root Length> / |Apex->D|`, then P slid from that O
seed along `D→J` by `(<O seed's perpendicular distance from the Driving Gear Shaft Axis> - <Driving
Gear Toe Radius>) / cos γ_g`. The ⚠️ above applies unchanged. Then apply the same three constraints —
`addCoincident(O, drivingRootAxis)`, `addParallel(lineOP, lineDJ)` and `addOffsetDimension(lineDJ,
lineOP, textPoint)` with the same value, its text point in the gap on the Apex side of D→J, at
`(O_seed + D) / 2`. The beginning is **O** and the end is **P**. Draw a line from O to D. Build the
driving front face **B′→P** exactly as the pinion's, substituting B for A, P for N and the **Driving
Gear Toe Radius** for the pinion's: the line P→B′, `addCoincident(Bprime, drivingShaftAxis)`,
`addPerpendicular(linePBprime, drivingShaftAxis)` and a length dimension on P→B′. P is never pinned to
the Apex→B shaft axis; only B′ touches it. Draw the line B′→I.

**Gate the sketch.** `if not sketch.isFullyConstrained: raise` naming "Gear Profiles"
(`[BEVEL-F-FULL-CONSTRAINT]`). Do **NOT** reach full constraint by dimensioning the driven lengths.

**From:** `spec/bevelgear/instructions.md` L477-592, L116-123, L551; `spec/bevelgear/fusion.md`
L21-30, L69-115, L119-151; `.claude/skills/generate-gear/PLAYBOOK.md` L441-500, L501-516,
L591-597, L606-622, L635-651, L717-723.

## S11 `[PROSE]` The per-gear tooth plane

Run this and every step through S32 **once per gear — pinion first, then driving** — with these
substitutions:

| | Pinion | Driving |
|---|---|---|
| gear label | `Pinion` | `Driving` |
| tooth centre | K′ | L′ |
| tooth-centre reference line | C→K′ | D→L′ |
| pitch cone half-angle | γ_p | γ_g |
| hexagon vertices, in draw order | A′ → G → H → C → M → N → A′ | B′ → I → J → D → O → P → B′ |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A′→G | B′→I |
| toe cut edge | M→N | O→P |
| heel cut edge | C→H | D→J |
| teeth / bore / pitch-diameter inputs | Pinion Gear … | Driving Gear … |
| section 2 shaft construction line (NOT usable as the axis) | Apex→A | Apex→B |

The profile and the body are built INTERLEAVED per gear — pinion profile, then pinion body, then
driving profile, then driving body — and **not** both profiles followed by both bodies.

Create a plane that includes the tooth-centre reference line, named `{gearLabel} Plane`. Use
`setByAngle` to make it perpendicular to the Gear Profiles sketch plane, through the framework helper
`plane_by_angle(designComponent, toothCenterReferenceLine, gearProfilesPlane, 90)` from `.solids`.
Pass the sketch line **directly**; never wrap it in `Path.create` first (`[PB-CONSTRUCTION-PLANES]`).

<!-- check-step-calls: ignore Path.create -->

`Path.create` is named only to forbid it.

**From:** `spec/bevelgear/instructions.md` L594-622, L722-740, L333-338.

## S12 `[GO]` The virtual spur tooth sketch

Realised by the proof function `stepToothProfile`.

<!-- proof-run: proofkit.RunParallel(perGearSketchCases, stepToothProfile) -->

**Compute this gear's virtual (back-cone, Tredgold) tooth number from the closed form, and NOT by
measuring Apex2→K′ or Apex2→L′:**

```
virtualPitchRadius = (this gear's Pitch Diameter / 2) / cos(γ)
virtualTeeth       = 2 * virtualPitchRadius / Module        # equivalently this gear's Teeth / cos γ
```

**It is a real number and is NEVER rounded** — not floored, not ceiled, not cast to an int. The
Tredgold construction puts the equivalent spur gear's pitch radius exactly at the back-cone distance
`r / cos γ` with this gear's own module, and `z_v = z / cos γ` is a real number in every published
form of it (NPTEL Machine Design II ch. 13 eq. 13.1–13.2; Osakue et al., *FME Transactions* 49(3),
2021, §2.2; the KHK gear technical reference eq. 11.6). Rounding it rebuilds every drawn circle from
the rounded count, which draws the tooth smaller than the back cone places it and shortens the working
addendum: on the shipped default — 31 teeth, Module 1, Shaft Angle 90°, γ = 45° — the exact virtual
pitch radius is 21.9203 mm, a floored count of 43 draws 21.5 mm, and the addendum the tooth works over
falls to 0.5797 mm against a nominal 1.0 module.

**The real count reaches the spur drawer only as an angular half-thickness.** The drawer reads
`ToothNumber` as a float and uses it in one place, `π / (2 · toothNumber)`, the angle it rotates the
flank to so the pitch crossing lands there. With `z_v = 2 · r_v / Module` that angle gives a tooth
thickness of `π · Module / 2` at the pitch circle, which is the standard tooth thickness — the same
thickness the spur gear of this module carries. An INTEGER count drawn at the exact radius gives `π ·
r_v / round(z_v)` instead, which misses nominal by a different amount on each member of an unequal
pair, so the two teeth of one pair no longer carry the same thickness.

**Root sink.** Draw the root circle one **root sink**, `0.05 · 2.25 · Module`, INSIDE the dedendum
corner rather than at it. At the dedendum corner exactly, the tooth's root arc touches the gear body's
root cone only where the arc crosses the tooth's own centreline: the tooth is drawn on the back-cone
plane, so only a point on that centreline rides the cone its own polar radius names, and the arc's two
corners stand outside it — by 0.002 module on the default pair and 0.027 module on a 4/4 pair, the
largest of any pair the spec admits. The sink pushes the whole arc inside, so the Combine-Join meets
the gear body across the root rather than along one line.

The four circles the proxy is asked for:

| circle | radius |
|---|---|
| pitch | `virtualPitchRadius` |
| base | `virtualPitchRadius · cos(20°)` |
| tip | `virtualPitchRadius + Module` |
| root | `virtualPitchRadius − 1.25 · Module − rootSink` |

**Units — pin the cm→mm conversion:** the stashed pitch diameters are internal **cm** while Module is
the raw **mm** value, so compute `virtualPitchRadius_mm = (pitchDia_cm · 10 / 2) / cos(γ)` — the `· 10`
converts cm to mm — and then `virtualTeeth = 2 · virtualPitchRadius_mm / Module`. Skipping the ×10
makes the virtual tooth count about ten times off.

**Draw the tooth.** Using the `{gearLabel} Plane` and the tooth-centre point as the centre, create a
spur gear tooth profile in a sketch named `{gearLabel} Tooth`:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth, rootSink_mm=rootSink)
drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))
```

`VirtualSpurProxy` is **imported from the framework**, `from .spurproxy import VirtualSpurProxy` — do
NOT define a local copy, and define no local value-wrapper class either. It precomputes, in internal
cm, exactly the keys the spur drawer reads through `parent.getParameter(name).value`, and its defaults
match bevel: pressure angle 20°, which is not a bevel dialog input, and `InvoluteSteps` 15. It takes
`virtualTeeth` as a **REAL number** — it computes the pitch diameter as `virtualTeeth · module_mm`, so
passing the exact `2 · r_v / Module` is what makes the drawn pitch circle reach the back cone — and
`rootSink_mm` shortens the root diameter by twice its value while leaving pitch, base and tip alone.
`rootSink_mm` defaults to 0, which is what keeps spurgear's own use of the proxy unchanged.

The tooth generator is borrowed by composition: `from .spurgear import
SpurGearInvoluteToothDesignGenerator`. Its surface is `(sketch, parent, angle=0)` and
`draw(anchorPoint, angle=0)`, which runs `drawCircles()` then `drawTooth(angle)` then the
anchor-projection. **The 180° rotation is delivered through the `draw()` angle argument** — the
generator rotates the whole tooth by `angle` — and *not* by a post-hoc Move or sketch rotation; this
relies on spur's radial flank-to-root pinning so the connecting lines rotate with the tooth.

**The proxy carries `_lastToothEmbedded`, an OUTPUT the spur generator writes, and bevel MUST read it
back.** During `draw()` the spur generator decides whether the tooth is *embedded* — tip, root and
flanks meeting with no connecting lines — and records it as `self.parent._lastToothEmbedded = <bool>`;
the framework proxy pre-initialises the slot to absorb that write. **After `drawer.draw(...)` returns,
read `proxy._lastToothEmbedded` and thread it to the tooth-profile selection of S17**, stashing it
alongside the tooth sketch and plane in this gear's dict. This flag is **not optional bookkeeping**: it
is the deterministic selector for the tooth loop's line count, and skipping it to accept either count
grabs an unrelated loop and the apex→tooth loft dies with `LOFT_NO_TOOLBODY`.

`embedded` is taken from the SUNK root radius, and the sink can flip it. On the shipped default pair
at Module 1 the base circle is 20.5984 mm and the sunk root circle 20.5578 mm, so the tooth that would
otherwise be embedded is drawn NON-embedded and the drawer adds two flank-to-root lines of 0.0405 mm
each.

<!-- check-step-calls: ignore drawCircles drawTooth getParameter -->

`drawCircles`, `drawTooth` and `getParameter` are named to describe the borrowed spur drawer's own
behaviour: the drawer calls all three, on itself and on the proxy, and bevel calls none of them. Bevel
calls `draw` and reads the proxy's `_lastToothEmbedded` slot afterwards.

**After `draw()` returns do NOT hard-gate this sketch:** log it if `not
toothSketch.isFullyConstrained`, never raise. The tooth-profile sketches are exempt from the
full-constraint gate, and the exemption covers the four circle LABELS and nothing else: `drawCircles`
labels each circle with along-path sketch text, and sketch text holds a degree of freedom of its own
(`[PB-TEXT-HOLDS-DOF]`), so a sketch whose geometry is completely determined still reads `False`
purely because it is labelled. ⚠️ **Never read that exemption as licence for loose geometry.** An
earlier wording claimed the embedded tooth kept a free radial degree of freedom and that fixing it
would risk the whole spur family; all of that was wrong, and the wrong reasoning let a deformed tooth
ship — measured in Fusion on 2026-09-02 on a default 31/31 pair, the pinion's tooth-top arc came out at
0.5743 mm and the driving gear's at 17.0204 mm where both should have been the 22.5 mm tip radius,
from two sketches with byte-identical constraint counts and dimension values. The fix was one
coincident constraint on the arc's centre in the shared generator. The bevel tooth sketches have also
read `True` on some runs and `False` on others with byte-identical counts, so the answer cannot be
relied on either way: log, never raise.

The proof gates this sketch normally, because the bench has no sketch text in it, and it asserts the
four circle radii, the tip circle at `virtualPitchRadius + Module`, the tooth thickness at the pitch
circle, the embedded flag against the sunk root radius, and the loop's curve counts of exactly 2
NURBS, 2 arcs and 0 or 2 lines as the flag requires.

**From:** `spec/bevelgear/instructions.md` L594-624, L423-457, L374-384; `spec/bevelgear/fusion.md`
L31-58; `.claude/skills/generate-gear/PLAYBOOK.md` L188-203, L517-532, L624-634.

## S13 `[PROSE]` The per-gear tooth axis

Create a construction axis named `{gearLabel} Tooth Axis` through the tooth-centre point, normal to the
plane the tooth profile was drawn on, with `component.constructionAxes.createInput()` then
`axisInput.setByTwoPlanes(gearProfilesPlane, helperPlane)` then `constructionAxes.add(axisInput)`
(`[PB-CONSTRUCTION-AXES]`). `setByPerpendicularAtPoint` would need a `BRepFace` this build does not
have.

The two planes are the **Gear Profiles plane** and a **helper plane** built with
`planeInput.setByDistanceOnPath(toothCenterReferenceLine, adsk.core.ValueInput.createByReal(1.0))`,
which is perpendicular to that line at its far end, the tooth-centre point; their intersection is the
line through the tooth centre normal to the tooth plane. Pass the sketch line directly to
`setByDistanceOnPath`, never through `Path.create`.

Creating this axis in the never-activated Design component is proven to work — `constructionAxes.add`
via `setByTwoPlanes` does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here — so keep the axis.

<!-- check-step-calls: ignore setByPerpendicularAtPoint Path.create -->

`setByPerpendicularAtPoint` and `Path.create` are named only to forbid them.

**From:** `spec/bevelgear/instructions.md` L594-597, L626;
`.claude/skills/generate-gear/PLAYBOOK.md` L775-799.

## S14 `[PROSE]` The per-gear Gear component

Create a new component as a child of the **Bevel Gear** component — the same component that owns
Design, and *not* the user's Parent Component; this intentionally overrides the looser "child of
Parent Component" phrasing so the pair nests cleanly inside Bevel Gear — with
`bevelComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`, named `{gearLabel} Gear`,
so `Pinion Gear` and `Driving Gear`. The finished bodies for this gear end up here.

Fusion rejects cross-sibling sketch and project calls even when the target is activated or the
entities are wrapped in `createForAssemblyContext` proxies (`[PB-NO-CROSS-SIBLING]`), so the actual
feature operations all run in the Design component and the finished bodies are `moveToComponent`'d
here at the end, in S33. The visible end state is identical.

**From:** `spec/bevelgear/instructions.md` L736;
`.claude/skills/generate-gear/PLAYBOOK.md` L829-834.

## S15 `[GO]` The per-gear Profile sketch — the frustum hexagon

Realised by the proof function `stepProfileHexagon`.

<!-- proof-run: proofkit.RunParallel(perGearSketchCases, stepProfileHexagon) -->

Open a **fresh sketch on the axial (Gear Profiles) plane**, `sketches.add(gearProfilesPlane)`, named
per the S11 table — **one profile sketch per gear**, so `sketch.profiles` holds exactly this one
hexagon loop. Do not draw both gears' hexagons in the shared Gear Profiles sketch; that would leave two
identically shaped loops to disambiguate.

Build the hexagon on fixed vertices per the `[PB-PROJECT-NOT-FIXED]` recreate-share-fix recipe:

1. **Recreate** the six section 2 vertices as new points at their exact world-mapped positions —
   `sketch.sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))` for each — which is valid
   because section 2 is fully constrained by now. `modelToSketchSpace` is a point-transforming METHOD
   and not a matrix (`[PB-SPACE-METHODS]`): call it directly on the point, never assign it and pass it
   to `transformBy`.
2. **Share** them: draw the closed hexagon in the S11 draw order as six
   `sketch.sketchCurves.sketchLines.addByTwoPoints` calls consuming those points.
3. **Fix** the lines' endpoints **after** the lines exist — `line.startSketchPoint.isFixed = True` and
   `line.endSketchPoint.isFixed = True` for each. **Order matters:** setting `isFixed = True` on a bare
   point *before* it is consumed as a line endpoint does NOT leave the sketch fully constrained.

Projecting section 2's points instead would leave this sketch under-constrained, because a projection
is a reference and not a fix.

The hexagon's **first edge is the gear's shaft axis** for the revolve, the pattern, the bore plane AND
the meshing-rotation axis, so it must be fixed well enough to carry a trustworthy world position:
fixed endpoints give that edge a well-defined `worldGeometry` (`[PB-WORLDGEO-CONSTRAINED]`), while a
free edge resolves against a default or world-XY frame and silently moves the body onto world XY. That
was observed on the driving gear; the pinion looked fine only because it never read the edge's
`worldGeometry`.

**The shaft axis used by every body operation below is this profile sketch's first edge, NOT the
section 2 `Apex->A` / `Apex->B` construction line.** The edge is collinear with the shaft axis but lives
in the *same* sketch as the profile, which is what Fusion's revolve, pattern and path builders accept;
reusing the section 2 construction line, which lives in a different sketch, fails or misbuilds.

Gate the sketch: `if not sketch.isFullyConstrained: raise` naming it (`[BEVEL-F-FULL-CONSTRAINT]`).

**From:** `spec/bevelgear/instructions.md` L722-740; `spec/bevelgear/fusion.md` L21-30;
`.claude/skills/generate-gear/PLAYBOOK.md` L458-472, L585-590, L598-605.

## S16 `[GO]` Revolve the hexagon into the Gear Body

Realised by the proof function `stepGearBody`, checked by `assertGearBody`.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepGearBody, assertGearBody) -->

This sketch holds exactly one hexagon loop, so take its single profile with `sketch.profiles.item(0)`
and do not filter (`[PB-SINGLE-PROFILE]`): a curve-type filter in particular has spuriously rejected a
valid all-line loop and made the revolve fail with "could not find profile".

Revolve it around the **shaft-axis edge** — the hexagon's first edge — with
`component.features.revolveFeatures.createInput(profile, shaftAxisEdge,
adsk.fusion.FeatureOperations.NewBodyFeatureOperation)` then
`revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))` then
`revolveFeatures.add(revolveInput)` (`[PB-REVOLVE]`). The result is the **Gear Body**, the frustum.

Because the toe edge is one edge of the revolved profile, the body already carries the conical face
produced by sweeping it around the axis; that face is reused as a cutting tool in S18, as is the heel
edge's cone.

**Hard failure to design around: the profile must NOT cross the axis of revolution.** If it does,
Fusion aborts with `RuntimeError … ASM_WIRE_X_AXIS … the profile crosses the axis of revolution`. The
Maximum Face Width, the Maximum Base Height and the strictly positive Toe Radius are what keep it on
one side; reproduce all three caps exactly.

The proof substitutes a **polygonal sweep** for the revolve and states what that costs. decad
publishes a revolved body's volume with a proven bound equal to the volume itself, so a revolved body
is Suspect at any tolerance and cannot pass the harness gate at all; a circular loft's chorded wall
publishes a volume bound past the relative tolerance too, measured at 124.4 mm³ against a required
104.0 mm³ on the shipped pair at Module 4. What is built instead is the three bands the frustum's own
profile edges sweep — the heel cone out to the heel end, the root cone out to the dedendum corner, and
the toe dish that hollows the front face — laid apart along the shaft axis and never joined, and the
frustum is asserted as their SIGNED SUM against Pappus on the section 2 hexagon, band by band against
its own stations and ring radii, and cone half-angle by cone half-angle: the heel band and the toe
plug come out parallel, on the back-cone family, and the root band at the dedendum angle to them.
**The cost is the union**: the proof does not show the three bands closing into one watertight solid,
only that each is separately watertight and that together they have the right volume, stations and
angles.

**From:** `spec/bevelgear/instructions.md` L742, L815-824, L880-886;
`.claude/skills/generate-gear/PLAYBOOK.md` L494-500, L717-723.

## S17 `[GO]` Loft the Apex sketch point to the tooth profile

Realised by the proof function `stepToothLoft`, checked by `assertToothLoft`.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepToothLoft, assertToothLoft) -->

Loft the **section 2 Apex sketch point** — `centerToApex.endSketchPoint` from the Gear Profiles sketch,
the degenerate point-section — to this gear's section 3 Tooth profile. The result is the **Tooth
Body**.

Use the section 2 Apex **SketchPoint** directly and do **NOT** create a construction point for it
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`): construction geometry needs an ACTIVE component and the Design
component is never active, while a `SketchPoint` works as a loft point-section without one.

The calls are `component.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
then `loftInput.loftSections.add(apexSketchPoint)` and `loftInput.loftSections.add(toothProfile)` in
that order — the order of the `add` calls is the loft order (`[PB-LOFT]`) — then
`loftFeatures.add(loftInput)`.

**Select the tooth cross-section loop with `find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2,
lines=wantLines)`**, the framework helper from `.utilities`, where **the line count is DETERMINED by
the `embedded` flag and never guessed or accepted-either**: `wantLines = 0 if embedded else 2`. ⚠️ **Do
NOT accept "0 **or** 2 lines".** For a given gear only ONE of those is the real tooth, and an
**unrelated** loop — an inter-tooth or annular region between the drawn circles — can also have 2
NURBS and 2 arcs with the *other* line count. Selecting it makes this loft fail with `RuntimeError …
ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`, because the impostor loop cannot form a loft tool body. The flag
comes from the borrowed spur generator through `proxy._lastToothEmbedded`, read back in S12: embedded
means tip, root and flanks meet with no connecting lines, which is 4 curves, and non-embedded means 2
connecting lines, which is 6 — mirroring spur's own selection.

The proof substitutes a **shrunken section for the degenerate apex point** and **axis-perpendicular
sections for the back-cone tooth plane**, and its involute flanks are chorded into line segments
because decad's loft refuses a free-form section pair. What the substitutions keep is what the later
steps read: the tooth's reach, its section area at each end, and the volume the two imply. The
assertion reads the built tooth body out to the virtual tip radius laid on the back cone — at Module 4
through 8 only, since no solid case runs at Module 1 — and no case reads a tip radius off a JOINED
body, because no case joins.

**From:** `spec/bevelgear/instructions.md` L744, L374-384, L825-826;
`.claude/skills/generate-gear/PLAYBOOK.md` L152-154, L724-728, L791-799.

## S18 `[GO]` Trim the Tooth Body with the two conical end cuts

Realised by the proof function `stepConicalTrims`, checked by `assertConicalTrims`.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepConicalTrims, assertConicalTrims) -->

Trim the Tooth Body to a flush band with the framework helper and do **NOT** re-implement the cut
machinery:

```python
cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)
```

**Two distinct bodies are involved — do not conflate them.** The cutting TOOLS are `ConeSurfaceType`
faces of the **Gear Body**, the revolved-hexagon frustum; the lofted Tooth Body has no cone faces, so
searching *it* for the cone face finds none. The TARGET being split is the **Tooth Body**, the loft.

The helper implements the pinned cut behaviour: the **toe cut first**, its cone face identified by the
toe edge's world **MIDPOINT** best-first across the frustum's cone faces (`[PB-FACE-BY-MIDPOINT]` —
endpoints sit near the apex singularity, where `getParameterAtPoint` returns no result, so an
endpoint-based distance cannot see the right face), each candidate tried as the actual split tool and
the first that splits into more than one piece kept; **keeper selection after each cut**, removing
apex-containing pieces and keeping the largest (`[PB-REMOVE-PIECES]`); then the **heel cut on the
keeper alone**, since removing the apex tip first is what makes it deterministically two split
features for every gear ratio. A heel cone that does not intersect the keeper at all — common on ratio
pairs such as Module 1 with driving 31 and pinion 43, where the heel cone never overshoots the tooth —
is raised by the helper as the typed `solids.NonIntersectError` and caught, and the keeper is returned
whole. Every failure is self-diagnosing with the per-face distance and error history
(`[PB-SELF-DIAGNOSING]`), and each cut's outcome is logged with `force_console=True`.

**Caller obligations, which stay in the generator:** pass `toeMid` = the toe edge's world midpoint,
`(M_world + N_world) / 2` for the pinion and `(O_world + P_world) / 2` for the driving gear; `heelMid`
= the heel edge's world midpoint, `(C_world + H_world) / 2` and `(D_world + J_world) / 2`; `apexWorld`
= the section 2 Apex sketch point's world geometry; and `gearBody` = the revolved frustum, the
cone-face source. **The toe cut must split** — its failure propagates and crashes the build, which is
correct, since an uncut tooth is unusable. Only the heel cut is lenient, and only through the typed
`NonIntersectError`.

The helper is reached through `from . import solids`; the underlying pattern is
`component.features.splitBodyFeatures.createInput(targetBody, splittingFace, True)` then
`splitBodyFeatures.add(input)` with `isSplittingToolExtended=True` so the tool surface is extended to
fully bisect the target (`[PB-SPLIT-BODY]`). Do not write that pattern out again here; call the
helper.

<!-- check-step-calls: ignore createInput add -->

The `splitBodyFeatures` calls are named to say what the helper encodes, not to be made again by the
module.

The proof performs **neither cut**: both operands are Lofts, the tooth and each cone alike, and no
boolean here takes a Loft. It builds the tooth and the two cones, lays them apart, reads each cone's
apex and half-angle off the cone and each of the tooth's two surfaces off the tooth, solves the
stations where they cross from those readings, and checks them against the flush band. **The cost is
the split**: it does not show the evaluator dividing the tooth, selecting the keeper, or leaving a
watertight body. What it does show is that each cut lands where the flush band requires — within one
root sink of the toe corner and of the dedendum corner — and that the two ends land on DIFFERENT
surfaces of the tooth, the toe on its tip and the heel on its root, which is the observable signature
of a conical cut face rather than a planar one.

**From:** `spec/bevelgear/instructions.md` L746-772, L827-834;
`.claude/skills/generate-gear/PLAYBOOK.md` L160-172, L733-761, L762-774.

## S19 `[GO]` The spiral cone-element sketch (ψ > 0 only)

Realised by the proof function `stepConeElement`.

<!-- proof-run: proofkit.RunParallel(spiralSketchCases, stepConeElement) -->

**Everything from S19 through S27 runs ONLY when Mean Spiral Angle ψ > 0.** The tooth-body hook
`_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld, apexSketchPoint,
toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane, gearLabel, teethNumber, gamma)` has as
its first line the gate

```python
if self._spiralAngle_rad <= 0:
    return cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)
```

so a **straight bevel is byte-for-byte the prior behaviour**: the apex loft plus the two conical trims
of S18, and every spiral input is ignored. `gamma` is this gear's pitch-cone half-angle, γ_p for the
pinion and γ_g for the driving gear, forwarded to the twist law in S24. The hook is called once per
gear inside the body step, on the freshly lofted uncut apex→heel Tooth Body, before the pattern, the
combine and the bore.

**Caller hand-off — the four toe and heel world points `_createGearBody` builds and passes, in the
positional order `toeMid, heelMid, toeConeWorld, heelConeWorld`. PIN THESE EXACTLY; mislabelling them
silently inverts the spiral, and this is the single biggest spiral-regen hazard.**

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

- `toeMid` = the world **midpoint of the TOE edge** — ½(M+N) pinion, ½(O+P) driving.
- `heelMid` = the world **midpoint of the HEEL edge** — ½(C+H) pinion, ½(D+J) driving.
- `toeConeWorld` = the toe edge's inner endpoint, **M** or **O**. It lies on the **root cone element**
  (Apex→C or Apex→D) at the toe end; M is pinned onto Apex→C in section 2 and O onto Apex→D.
- `heelConeWorld` = the **dedendum corner**, **C** or **D**, the heel edge's first endpoint. C and D
  are the OUTER end of that **same root cone element**, so `coneVec = normalize(heelConeWorld − apex)`
  runs along Apex→C or Apex→D pointing outward and the heel's cone distance is greater than the toe's.

⚠️ **Two scrambles to avoid, both of which a fresh regen has made:**

- Do **NOT** pass the two endpoints of a *single* edge as `toeMid` and `heelMid`, for example M as
  `toeMid` and N as `heelMid`. M and N both sit at the **toe**, so the span between the heel midpoint's cone distance and the toe midpoint's collapses to about zero or goes negative and the spiral inverts. `toeMid` is the
  midpoint of the **toe** edge; `heelMid` is the midpoint of the **heel** edge — two different edges.
- `heelConeWorld` is the **dedendum corner C or D**, on the root axis Apex→C or Apex→D, and **never H
  or J**. H and J lie on the Apex2→C / Apex2→D dedendum line, one Module beyond C and D, **off** the
  root cone element, and using them skews `coneVec` away from the root axis.

**A. Gate and frame.** Build a world frame from the geometry already constructed for this gear:

- `axisDir` — the **shaft axis** direction, from the two **world** endpoints of `shaftAxisEdge`, the
  in-sketch profile edge A′→G or B′→I, normalized. Read it in WORLD space
  (`[PB-WORLD-FRAME]`): mixing a local-frame curve with a world axis is valid Python that silently
  returns wrong numbers, with no exception and nothing a lint can catch, and the symptom is a wrong
  spiral-twist magnitude that makes the meshing teeth interfere.
- `coneVec` — the **dedendum (root) cone element**, realized as `normalize(heelConeWorld − apexWorld)`.
- `v` = `axisDir × coneVec`, normalized — the **circumferential** direction, the sideways sense the
  tooth is displaced from the radial element.
- `tpNormal` = `coneVec × v`, normalized — the **tangent-plane normal**.
- `distAlong` — a point's **cone distance**, `(p − apex) · coneVec`.

⚠️ **The heel MUST be the OUTER end so `coneVec` points outward and `span > 0`.** Before building
`coneVec`, check the passed midpoints and **fix swapped toe and heel**: if `apex.distanceTo(heelMid) <
apex.distanceTo(toeMid)`, swap `toeMid ↔ heelMid` **and** `toeConeWorld ↔ heelConeWorld`, then build
`coneVec` from the new `heelConeWorld`. A negative `span` **silently inverts the entire spiral
frame** — it flips the cutter-arc direction, the slice direction so the first cut misses, and the
per-segment twist — and the gear comes out completely wrong with no error. The inversion can also
originate upstream in a section 2 or section 3 mislabelling of the toe and heel edges; this guard
catches it at the frame.

From the midpoints, **after** the swap guard, take `R_toe` as the toe midpoint's cone distance, `R_heel` as the heel midpoint's,
`R_mean = ½(R_toe + R_heel)` and `span = R_heel − R_toe`, the face width, now
positive. These are the only quantities the rest of the build needs.

**The sketch this step authors.** Draw a **cone-element construction line** from the Apex to `Apex +
R_heel · coneVec` in a sketch on the **axial (Gear Profiles) plane**, named `{gear} Cone Element`.
Then S20 makes the Trace Plane from it.

**The tangent plane is on the ROOT cone, not the pitch cone.** The canonical crown-gear construction
lays the trace in the plane tangent to the **pitch** cone; this implementation uses the dedendum
element instead. The two tangent planes differ only by the small dedendum angle, so the arc's shape is
essentially identical and the root cone is a defensible convenience for guiding the lengthwise tooth
curve, but it is a departure from the canonical reference and ψ then ends up measured on the root cone
rather than the pitch cone.

**Coordinates — this rule governs the `{gear} Cone Element` sketch as well as the trace sketch of
S21, and it is the only place either is told what frame its points are in.** The world `Point3D`s and
the raw `apex` and cone-end points are passed **directly** into the sketch calls, where they are
consumed as **sketch-space** input — **no `modelToSketchSpace` conversion is applied**, even though
`adsk.fusion.Sketch` offers exactly that call and the points really are model-space coordinates. This
is deliberate, and worth stating why it is harmless, because the reasoning is not the obvious one. The
trace sketch is **construction and reference only: no downstream feature ever consumes it** — the
twist is computed analytically in S24 and the sketch exists only so the genuine cutter arc is
inspectable before cleanup hides it. The cone-element line is the one that needs the extra sentence:
it *is* consumed, by `plane_by_angle`, which rotates about it to make the Trace Plane, so an
unconverted line does place that plane somewhere other than the true tangent plane. That still reaches
no feature, because the only thing built on the Trace Plane is the inspection-only trace sketch, and
the chain ends there. **If a later revision ever makes any feature consume the trace sketch or the
Trace Plane, this shortcut stops being safe and both sketches need `modelToSketchSpace` on every
point.**

<!-- check-step-calls: ignore modelToSketchSpace -->

`modelToSketchSpace` is named here only to record that it is deliberately NOT applied in these two
sketches; S15 is where the module really calls it.

**From:** `spec/bevelgear/instructions.md` L628-672, L677-679, L339-357;
`spec/bevelgear/spiral-tooth-trace.md` L30-75; `.claude/skills/generate-gear/PLAYBOOK.md` lines
L439, L180-181.

## S20 `[PROSE]` The Trace Plane

Make the tangent plane by rotating the axial plane **90°** about the cone-element line, through the
framework helper `plane_by_angle(designComponent, coneElementLine, gearProfilesPlane, 90)` from
`.solids`, and name it `{gear} Trace Plane`. Pass the sketch line directly, never via `Path.create`
(`[PB-CONSTRUCTION-PLANES]`).

**From:** `spec/bevelgear/instructions.md` L672; `spec/bevelgear/fusion.md` L59-67.

## S21 `[GO]` The 2-D tooth trace — the genuine cutter arc (ψ > 0 only)

Realised by the proof function `stepToothTrace`.

<!-- proof-run: proofkit.RunParallel(spiralSketchCases, stepToothTrace) -->

**B. Cutter-arc geometry.** Work in the tangent-plane 2-D frame with origin at the apex, **x =
coneVec** so a point's x is its cone distance, and **y = v**, circumferential. The cutter radius is
`r_c = Cutter Radius` if non-zero, **else `R_mean`**, the auto default. The hand sign is `handSign =
+1` for `Right` and `−1` for `Left`, then **negated for the pinion**, because the pair meshes with
opposite hands. The cutter-circle centre is

```
Cx = R_mean − r_c · sin ψ
Cy = handSign · r_c · cos ψ
```

⚠️ **The hand sign goes on the `cos` / `Cy` term and NOT on the `sin` / `Cx` term.** This was a real
bug. Opposite hand mirrors the cutter centre **across the cone element, y = 0**, which flips `Cy`;
putting `handSign` on `Cx` mirrors about `x = R_mean` instead, which is a *different* curve and gives
the two gears **unequal twist**, where for equal teeth the driving and pinion traces must come out as
exact mirror images.

The trace's toe and heel arc endpoints are circle∩circle intersections taken a hair **past** the face,
so the kept arc reaches cleanly past the end trims:

```
R_lo   = R_toe  − 0.06 · span
R_hi   = R_heel + 0.06 · span
toe2d  = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)
heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)
```

`circle_intersect_nearest` is the framework helper from `.solids`: it intersects the apex circle of
radius R with the cutter circle of centre `(Cx, Cy)` and radius `r_c` and keeps the solution nearest
`(R_mean, 0)`, the branch the mean point sits on, clamping to tangency on a non-overlap. Keeping the
far branch instead gives a kinked or back-bent trace.

**C. The trace sketch.** Add a sketch on the Trace Plane named **`{gear} 2D Tooth Trace`**. In it
draw, mapping 2-D coordinates to world with the framework helper `tanW(px, py) = combine_point(apex,
px, coneVec, py, v)`:

- the **cutter circle** — centre at `tanW(Cx, Cy)`, radius `r_c` — with `circle.isConstruction = True`,
  its centre pinned by `circle.centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`: a circle's
  centre is FREE even when created at the origin, and `addCoincident` to the sketch origin has been
  observed to throw `VCS_SKETCH_SOLVING_FAILED`), and a diameter dimension of `2 · r_c` through
  `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`;
- the **trace arc** — a three-point arc through `tanW(toe2d)`, `tanW(R_mean, 0)` (the mean point on the
  cone element) and `tanW(heel2d)`, built with
  `sketch.sketchCurves.sketchArcs.addByThreePoints(startPoint, point, endPoint)` — with its **centre
  coincident to the cutter circle's centre**, `addCoincident(arc.centerSketchPoint,
  circle.centerSketchPoint)`, and a **radius dimension of `r_c`** through
  `sketch.sketchDimensions.addRadialDimension(arc, textPoint)`, so it is the genuine cutter circle and
  not a look-alike spline. A fitted spline through sampled points looks similar, is not the cutter
  circle, and cannot carry the radius and centre constraints.

⚠️ **Text points per `[PB-RADIAL-DIM]`: off-centre, on or near the curve.** A radial or diameter
dimension whose `textPoint` sits at the curve's centre is rejected with `RuntimeError: 3 : …
一部の入力引数が無効です`, because at the centre there is no radial direction to place the dimension in.
Use the mean point `tanW(R_mean, 0)` for the trace arc's radius dimension, and a point on the cutter
circle such as `tanW(Cx + r_c, Cy)` for its diameter dimension.

This sketch is **deliberately left with free degrees of freedom** — the arc's endpoints are pinned by
the three-point construction, not by endpoint dimensions, because dimensioning them over-constrains
the solve against the cone-element plane. It is therefore **exempt from the full-constraint gate**
along with the `{gear} Cone Element` sketch and the `{gear} Trace Plane`; do not gate any of them. The
gate applies only to the bevel's own permanent sketches — Anchor, Gear Profiles, the two per-gear
Profile sketches, and Bore — for both straight and spiral builds.

**D. There is NO 3-D projection.** The 2-D cutter-arc sketch is the only trace geometry needed: the
spiral twist is computed **analytically** in S24, so there is **no `projectToSurface`, no root-cone
face search and no 3-D trace sketch**. Earlier versions projected the 2-D arc onto the root cone along
`tpNormal` and measured the trace azimuth there. That projection is *fragile*: for unequal-ratio pairs
the arc wraps around the cone and `projectToSurface` returns it as **multiple disjoint fragments**, so
the measured azimuth collapses to a fraction of the true sweep — the pinion comes out grossly
under-twisted and the pair interferes. Do not reintroduce it.

<!-- check-step-calls: ignore projectToSurface -->

`projectToSurface` is named only to forbid it.

The proof builds the arc from its centre, start and end where Fusion builds a three-point arc and then
constrains the centre and radius, because the bench gate exempts nothing and a three-point arc plus a
centre coincidence plus a radius dimension is one row more than the arc has freedoms. The mean point
lying on the arc becomes an assertion instead of a constraint, which is the invariant that matters,
and the proof checks all seven of the trace's invariants: apex-centred loci, the cutter radius, the
arc passing through the mean point, ψ realised at the mean point, mirror symmetry between the two
hands, the ends on the right circles, and the straight-bevel limit as ψ → 0.

**From:** `spec/bevelgear/instructions.md` L661-683; `spec/bevelgear/spiral-tooth-trace.md` lines
L90-183, L218-254; `spec/bevelgear/fusion.md` L59-67;
`.claude/skills/generate-gear/PLAYBOOK.md` L451-457, L652-656, L180-184.

## S22 `[GO]` Slice the straight tooth into cross-section slabs (ψ > 0 only)

Realised by the proof function `stepSliceTooth`, checked by `assertSliceTooth`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepSliceTooth, assertSliceTooth) -->

**E.** Split the uncut apex→heel Tooth Body into cross-section slabs by planes **perpendicular to the
cone element**, spanning a touch past the toe and the heel, by a **fixed** slice scheme of about
**eight** planes; the count is **not user-configurable**.

The first cut plane is the **parent transverse tooth plane** — `parentToothPlane`, the virtual-spur
tooth-profile plane `{gearLabel} Plane` from S11, passed into the hook — offset toward the apex by
`span / 6`. The offset **sign is chosen per gear** so it moves apex-ward: the parent plane's normal
points opposite ways for the two gears, so pick `sign` such that `sign · normal` points apex-ward, by
testing `(apex − planeOrigin) · normal`. Then a sequence of about eight planes stepped further toward
the apex in `span / 6` increments, `sign · (k + 1) · span / 6` for k = 0 … 7, with k = 0 the first cut
plane.

Split with the framework helper `slice_body_by_offset_planes(designComponent, toothBody,
parentToothPlane, offsets)` where `offsets = [sign * (k + 1) * span / 6 for k in range(8)]`. It splits
piece by piece and keeps a piece whole when a plane misses it; the caller picks the offsets and signs
and asserts the resulting piece count (`[PB-EMPTY-RESULT]`).

⚠️ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in **one
piece**, no plane cut it: the offset sign was wrong or `parentToothPlane` sits outside the tooth's
span, so **retry the whole cut once with the opposite sign**. If it is *still* one piece, **`raise` a
clear self-diagnosing error** naming the gear, the final piece count, `span` and the sign tried
(`[PB-SELF-DIAGNOSING]`). Do **NOT** return an unsliced single-piece result: S23 then drops that one
piece as the apex scrap, leaving `segments` **empty**, and the crown in S25 crashes with `ValueError:
max() iterable argument is empty` far from the cause.

The proof builds each slab directly as the piece the split would have left — the same substitution the
rest of the proof uses, since nothing here may be split — and asserts the fixed plane count, the
offsets, that the pieces tile the tooth end to end with no gap and no overlap, and each slab's volume
against its own two sections. Its slabs are cut perpendicular to the SHAFT AXIS rather than to the
cone element, because the tooth it builds stands its sections square to that axis; the two differ by
one constant factor, which every ratio the twist and the crown key on cancels.

**From:** `spec/bevelgear/instructions.md` L685, L876-878;
`.claude/skills/generate-gear/PLAYBOOK.md` L176-177, L440, L762-769.

## S23 `[GO]` Order the segments and drop the apex scrap (ψ > 0 only)

Realised by the proof function `stepDropScrap`, checked by `assertDropScrap`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepDropScrap, assertDropScrap) -->

**F.** Sort the segments by the `distAlong` of their centroid, `body.physicalProperties.centerOfMass`.
The first, apex-most one is the long **apex-side scrap** below the toe: **remove it** and keep the rest
as the working `segments`. **Drop it by re-slicing the list *first* and deleting *after* —
`segments = segments[1:]` before `designComponent.features.removeFeatures.add(scrap)`** — so the
working list never holds a body that has been removed. Use `removeFeatures.add`, which is
timeline-visible, and not a bare `deleteMe()` (`[PB-REMOVE-PIECES]`).

After dropping the scrap, **`segments` must be non-empty**, at least one cross-section. If it is empty
the slice failed in S22: `raise` a clear error rather than proceeding into the twist and the crown,
which both assume at least one segment.

**From:** `spec/bevelgear/instructions.md` L687;
`.claude/skills/generate-gear/PLAYBOOK.md` L440, L770-774.

## S24 `[GO]` Twist each segment about the shaft axis (ψ > 0 only)

Realised by the proof function `stepTwistSegments`, checked by `assertTwistSegments`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepTwistSegments, assertTwistSegments) -->

**G.** Rotate each segment about the **shaft axis** — `axisDir` through `apex` — so the tooth follows
the trace, **centred on R_mean so the mid-face section stays unrotated**. That section then meshes
exactly like the straight tooth, which is what lets the pinion's mesh nudge stay at zero.

The total toe→heel shaft-axis twist comes from the **conjugate crown-gear generation law**, the
standard Gleason/Litvin model: a spiral bevel is generated by an imaginary flat *crown gear*, and the
work gear's shaft rotation relates to the developed crown-plane azimuth by the **roll ratio `1 / sin
γ`** — the generating crown gear has `N / sin γ` teeth. Compute it **analytically, with no projection
and no curve sampling**:

```python
phi_crown = math.atan2(heel2d[1], heel2d[0]) - math.atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`phi_crown` is the angle the cutter arc's **toe and heel endpoints subtend at the apex** in the flat
2-D crown frame — apex at the origin, x the cone distance along `coneVec`, y circumferential along `v`
— exactly the `toe2d` and `heel2d` pairs from S21. `gamma` is this gear's **pitch cone angle**,
`self._gamma_p` for the pinion and `self._gamma_g` for the driving gear, already computed in S4.
`handSign` sets the direction and `total` is the magnitude.

⚠️ **Use the PITCH cone angle γ and NOT `acos(coneVec · axisDir)`**, which is the *root* or dedendum
cone angle — about 14° against a pitch angle of about 29° for a 17-tooth pinion — and yields a twist
roughly 1.6 times too large.

⚠️ The two members of a meshing pair **legitimately get different twists**: same cutter, same spiral
angle ψ, but γ differs, so `1 / sin γ` differs — about 2.08× for a 17-tooth pinion against about 1.14×
for a 31-tooth gear, a ratio of about 1.83. This is *why* equal-teeth pairs, which share γ, always
meshed while ratio pairs failed under any method that gets `1 / sin γ` wrong.

Each segment's rotation angle is a **linear share keyed to the cone distance of its HEEL FACE**, the
segment's farthest-along-the-element face, which is the exact section the later loft samples:

```python
ang = -handSign * total * (R_mean - R_heelFace(seg)) / span
```

**Define a slab's heel face precisely: the face whose centroid has the GREATEST
`distAlong`, searched across ALL of the slab's faces with NO surface-type filter.** Its
toe or apex-side face is the least-centroid one. ⚠️ Do **NOT** restrict this search to
`PlaneSurfaceType`, or to any surface type: a sliced slab is bounded by a mix of the two planar cut
faces and ruled side faces, and a type filter can pick the wrong face or miss the cut face, which makes
the S26 loft fail with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this **same
all-faces-by-centroid** rule everywhere a slab end face is needed: the twist key here, the crown base
in S25, and the loft sections in S26.

⚠️ **Key the twist on the segment's HEEL-FACE cone distance and NOT on its centroid.** The loft samples
each segment's heel face, so that face is what must land at the right azimuth; centroid-keying leaves
the loft's mid-face section rotated by half a segment and the mid-face sections overlap.

Apply the rotation as a free move by a `adsk.core.Matrix3D.setToRotation(ang, axisVector, originPoint)`
through `component.features.moveFeatures.createInput2(bodyCollection)` then
`moveInput.defineAsFreeMove(matrix)` then `moveFeatures.add(moveInput)` (`[PB-MOVE-ROTATE]`). Use
`defineAsFreeMove` with a matrix and not `defineAsRotate`, which rejects a `SketchLine` axis.

<!-- check-step-calls: ignore defineAsRotate -->

`defineAsRotate` is named only to forbid it.

The proof asserts the developed crown azimuth, the roll ratio, that the pitch and root cone angles
give measurably different twists so a case cannot pass on the wrong one, that an unequal pair's two
members get different twists while an equal pair's match, that the mid-face section is unrotated, and
that heel-face keying and centroid keying are measurably different for every segment.

**From:** `spec/bevelgear/instructions.md` L689-702; `spec/bevelgear/spiral-tooth-trace.md` lines
L186-214; `.claude/skills/generate-gear/PLAYBOOK.md` L800-809.

## S25 `[GO]` The lengthwise crown (ψ > 0 only)

Realised by the proof function `stepCrownSegments`, checked by `assertCrownSegments`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepCrownSegments, assertCrownSegments) -->

**H.** Crown the tooth by scaling each segment **except the outermost (heel) one** down by a
**monotonic** factor — full at the heel, growing smoothly toward the toe — **about a sketch point on
the ROOT edge of its heel face**, and *not* about the heel face's centroid.

For each segment compute its **heel-distance fraction** `u = (R_heel − R_heelFace) / span`, where
`R_heelFace` is the `distAlong` of that segment's heel face — found by the S24 all-faces-by-centroid
rule but **RECOMPUTED here, AFTER the twist has moved the slabs**; do not reuse pre-twist values — and
`R_heel` and `span` are from the frame in S19. `u` runs 0 at the held-full heel to 1 at the toe. **The
"outermost (heel) segment" is the one with the GREATEST post-twist heel-face `distAlong`**: sort the
segments by their recomputed heel-face `distAlong` and skip the last. Then

```python
factor = 1 - _CROWN_PER_RAD * (abs(total) / 2) * u
```

`total` is the full toe→heel twist from S24, and `|total| / 2` is the per-end peak twist magnitude, so
the maximum relief — now at the **toe** — keeps the same magnitude the old per-end peak had, just
relocated. This makes relief **grow monotonically from the (full) heel to the toe**, so slab heights
stay strictly ordered heel→toe and the natural cone taper is never reversed. If a computed `factor`
comes out ≤ 0, which extreme twist can produce, **`raise` a self-diagnosing error naming the gear, the
segment's `u` and the factor** — never scale by a non-positive factor. **`_CROWN_PER_RAD` is a tunable
class constant with default `0.5`**, where 0 disables the crown; set it to 0.5 and do not leave it
unset or 0.

⚠️ **Do NOT key the relief on `|ang|`**, the twist magnitude, i.e. the distance from the mid face.
That is **symmetric** about mid-face and maximal at BOTH ends, so because the heel slab is held full,
the slab *just inside* the heel becomes the **most**-relieved one and dips below both its neighbours —
a notch that reverses the heel→toe taper. This was the observed bug: the heel-adjacent slab came out at
factor `0.932` while the next slab inward was `0.972`, taller. Key on the monotonic heel-distance `u`,
never on `|ang|`.

Three gotchas:

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex — per `[PB-CONSTRUCTION-NEEDS-ACTIVE]`. `scaleFeatures` is the ONE exception to
   never-activate: it needs the Design occurrence as the **active** edit target, so call
   **`designOccurrence.activate()`**, a method on the `Occurrence`, before the crown scales, and
   restore afterwards — in a `finally` — with **`design.activateRootComponent()`**, a method on
   `Design`. ⚠️ Do **NOT** write `design.rootComponent.activate()` or `someComponent.activate()`: a
   `Component` has **no** `.activate()` method and raises `AttributeError`. Only `Occurrence` has
   `.activate()`, and the root is re-activated through `Design.activateRootComponent()`.
2. **Skip the outermost (heel) segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone in S27 trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge and NOT on its centroid, or the crowned tooth lifts
   off the gear base.** `scaleFeatures` shrinks **uniformly** toward the base point, so a base point at
   the heel-face **centroid**, at mid tooth-height, pulls the tooth's **root** edge *upward* by
   `(1 − factor) · (½ tooth height)`: the tooth no longer seats on the gear body's root cone, floats
   above the base, and the Combine-Join leaves a gap — clearly visible for ratio pairs, for example
   module 2 with driving 19 and pinion 13, which is the original symptom that exposed this. Put the
   base point on the **root** instead: of the heel face's vertices, `heelFace.vertices` with each
   `.geometry` a world `Point3D`, take the **two with the smallest perpendicular distance to the shaft
   axis** — the line through `apex` along `axisDir`, perpendicular distance `|(p − apex) − ((p − apex)
   · axisDir) · axisDir|` — which are the two **root corners**, since the tip corners are the farthest
   from the axis, and place the base sketch point at their **midpoint**, mapped into the heel-face
   sketch with `sketch.modelToSketchSpace(point)`. The heel face is a planar cut, so that midpoint lies
   on it. A uniform scale about a point keeps every line and plane through that point invariant, so
   anchoring on the root keeps the root edge on the seating cone — the tooth stays flush — while the
   tip is relieved progressively toward the toe, which is exactly the lengthwise crown intended.
   Finding the heel face itself is unchanged: still the max-`distAlong`-centroid face by the S24 rule;
   only the point *on* it changes from the centroid to the root-edge midpoint.

The scale itself is
`component.features.scaleFeatures.createInput(bodyCollection, baseSketchPoint,
adsk.core.ValueInput.createByReal(factor))` then `scaleFeatures.add(scaleInput)`.

<!-- check-step-calls: ignore rootComponent -->

`design.rootComponent.activate()` is named only to forbid it.

**From:** `spec/bevelgear/instructions.md` L704-716; `spec/bevelgear/fusion.md` L155-160;
`.claude/skills/generate-gear/PLAYBOOK.md` L585-590, L791-799.

## S26 `[GO]` Loft the curved tooth (ψ > 0 only)

Realised by the proof function `stepSpiralLoft`, checked by `assertSpiralLoft`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepSpiralLoft, assertSpiralLoft) -->

**I.** ⚠️ **Re-sort the segments by their heel-face cone distance HERE, AFTER the twist and the crown.
Do NOT reuse the pre-twist slice or centroid order from S23.** The twist rotates each slab about the
shaft axis, and for high-twist *unequal-ratio* pairs that rotation changes the slabs' along-cone
`distAlong` order enough to **reorder adjacent slabs**; lofting in the stale pre-twist order then
assembles the cross-sections out of sequence and the crowned tooth comes out distorted, so the two
gears interfere. For equal or low-twist pairs the two orders coincide, which is why equal-teeth gears
mesh even with the stale order while unequal ratios distort — this is the single thing that makes a
ratio pair like 31/17 fail while 31/31 looks fine.

So compute the order **now**, sorting the segment indices by the `distAlong` of each slab's own
heel-face centroid, and loft a `NewBodyFeatureOperation` through, in that order:

1. first the **toe-most segment's apex-side (toe-facing) face** — the toe segment is `order[0]`, and its
   toe face is added first to push the loft past the toe cone so the toe trim bites;
2. then the **heel-facing face of every segment, iterated in `order`** — each segment's
   farthest-along-the-element face by post-twist centroid, with the last reaching past the heel cone.

Name the resulting body **`{gear} Spiral Tooth`**. Then remove the segment scaffolding with
`removeFeatures.add`, since the loft has captured their faces.

The calls are `loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`, one
`loftInput.loftSections.add(face)` per section in that order, and `loftFeatures.add(loftInput)`.

decad's loft takes two sections, so the proof builds the chain pairwise and lays the pieces apart, and
**the cost is the single body**: it does not show the evaluator running one loft through every
section. What it does assert is the order — strictly ascending in the heel-face station, with the first
section at or beyond the toe — and that each slab carries exactly one toe face and one heel face of the
sections it was built from. **The bench cannot show a surface-type filter picking the wrong face**,
because the tooth's flanks are chorded into line segments there and every slab face comes out planar;
that limit is recorded beside the assertion.

**From:** `spec/bevelgear/instructions.md` L718, L696;
`.claude/skills/generate-gear/PLAYBOOK.md` L724-728, L770-774.

## S27 `[GO]` Trim the curved tooth flush (ψ > 0 only)

Realised by the proof function `stepSpiralTrim`, checked by `assertSpiralTrim`.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepSpiralTrim, assertSpiralTrim) -->

**J.** Return

```python
cut_conical_ends(designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)
```

— the same toe-then-heel two-cone trim the straight tooth takes in S18 — so the curved tooth's ends sit
**flush** on the gear base. The toe and heel **mesh phasing** is handled outside this hook, by the
mesh-rotate step S32; the pinion's extra phase is 0 by default because the mid-face section is
unrotated and already meshes.

The proof performs neither cut, for the reason S18 gives, and asserts that the same two cones land at
the same flush band, that the heel segment is the uncrowned one and reaches the heel, and that the
pinion's mesh phase is left at zero.

**From:** `spec/bevelgear/instructions.md` L720, L780.

## S28 `[GO]` Circular-pattern the tooth around the shaft axis

Realised by the proof function `stepCircularPattern`, checked by `assertCircularPattern`.

<!-- proof-run: proofkit3d.RunSolid(patternCases, stepCircularPattern, assertCircularPattern) -->

Circular-pattern the remaining tooth piece around the **shaft-axis edge** — the same in-sketch profile
edge the revolve used, and not the section 2 construction line. The number of copies equals this
gear's Teeth Number.

`component.features.circularPatternFeatures.createInput(bodyCollection, shaftAxisEdge)` takes an
`adsk.core.ObjectCollection` of the entities to pattern, so copy the tooth body into a fresh
`adsk.core.ObjectCollection.create()` first. **Pin all three inputs explicitly** and do not rely on
Fusion's defaults staying equal to them (`[PB-CIRCULAR-PATTERN]`):

```python
patternInput.quantity   = adsk.core.ValueInput.createByReal(<this gear's Teeth Number>)
patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
patternInput.isSymmetric = False
```

then `circularPatternFeatures.add(patternInput)`.

Although the pitch diameter shrinks from the heel toward the apex, the *angular* spacing around the
shaft axis stays constant at `360° / N` for the entire face width: the radial taper is already produced
by the loft from the Apex to the heel-end tooth profile, so the pattern just rotates that single
tapered tooth into N evenly spaced copies.

`CircularPatternFeature.bodies` already includes the seed body plus the copies, so do not re-add the
seed, and copy them into an `ObjectCollection` before handing them to the Combine in S29
(`[PB-PATTERN-BODIES]`): `pattern.bodies` is a `BRepBodies` and `combineFeatures.createInput` rejects
it.

**THIS STEP'S PROOF IS THE ONE THAT STAYS SERIAL**, on `proofkit3d.RunSolid`, where every other solid
step here takes the parallel runner. The pattern increment **retires the seed tooth**, so the seed
cannot be measured after the step runs, and its azimuth, radius, height and volume have to be read
during the build and handed to the assertion through package-level variables. That hand-off leaves the
case, and two cases running at once overwrite each other's readings and the proof reports a wrong
verdict rather than failing loudly. It is not a hazard that announces itself: the two gear sides differ
enough in volume that the overwrite was caught when it happened, and a pair of cases whose seeds
measured alike would have passed on each other's numbers instead. The proof keeps the carried readings
beside the step and records there that the step is serial because of them.

**From:** `spec/bevelgear/instructions.md` L774, L910-933;
`.claude/skills/generate-gear/PLAYBOOK.md` L693-703.

## S29 `[GO]` Combine-Join the patterned teeth into the Gear Body

Realised by the proof function `stepCombineJoin`, checked by `assertCombineJoin`.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCombineJoin, assertCombineJoin) -->

Join all patterned tooth pieces with the Gear Body in a **single** Combine-Join: the Gear Body is the
target and the patterned tooth bodies are the tools.

`component.features.combineFeatures.createInput(gearBody, toolBodyCollection)` takes the target as a
`BRepBody` and the tools as an `adsk.core.ObjectCollection`, then `combineFeatures.add(combineInput)`
with the operation left at Join.

The proof performs **no join**. It lays the operands apart and asserts the join's two consequences from
their own measured geometry: a join leaves ONE lump when the tooth's root is below the body's root
cone — seated, not floating — and the joined body reaches further out than the frustum when the tooth's
tip stands proud of it. Both readings are taken at the toe, the middle and the heel of the band the
join would cover. ⚠️ **It reads the root arc's OUTERMOST point, not the tooth's centreline**: the
centreline sits inside both root corners, so a reading taken there passes a tooth whose corners float
outside the cone, which is exactly the defect the root sink exists to remove. **The cost is the
stitch**: the proof cannot show the evaluator making one boundary out of two. **The generated module
draws its root circle one root sink inside the dedendum corner** (S12), so the root arc lies inside the
gear body's root cone across its whole width and the join overlaps along the whole root rather than
along the centreline alone, and **the proof applies that same sink** — it is one figure, not a
proof-only offset.

**Fusion has made this stitch once — loaded 2026-09-16, from the build the root sink was introduced
on.** Two configurations built with no error: the shipped default of 31 teeth on both gears at Module 1
and Shaft Angle 90°, and a 16 driving / 12 pinion pair at Module 4. The default is also the
configuration where the sink drops the root circle below the base circle, so its tooth is drawn
NON-embedded and the spur drawer adds the two flank-to-root lines, 0.0405 mm each; neither that profile
nor a Combine-Join at a sunk root had been through Fusion before that load. So the stitch this
substitution cannot show has been seen once, on those two configurations, and on nothing else in the
table.

**The heel tip radius was not measured on that load, so the tip is still checked only where the proof
checks it.** The S12 sketch case dimensions the drawn tip circle at `virtualPitchRadius + Module`, and
the S17 apex loft reads the built tooth body out to the virtual tip radius laid on the back cone — at
Module 4 through 8 only, since no solid case runs at Module 1. No case reads a tip radius off a joined
body, because no case joins. That measurement is still outstanding.

**From:** `spec/bevelgear/instructions.md` L776, L835-868;
`.claude/skills/generate-gear/PLAYBOOK.md` L693-697.

## S30 `[GO]` The bore sketch

Realised by the proof function `stepBoreSketch`.

<!-- proof-run: proofkit.RunParallel(perGearSketchCases, stepBoreSketch) -->

**Skip S30 and S31 entirely if Enable Bore is unchecked.**

Build the bore plane normal to the shaft at its start:
`planeInput.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))`. Pass the
**in-sketch shaft-axis edge**, not the section 2 construction line, and pass it directly rather than
through `Path.create`.

In a sketch named `{gearLabel} Bore`, sketch the bore circle centred at the sketch origin — the plane
is rooted at the shaft start, so the origin is on the axis — with
`sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), radius)`. Then
**fix the circle's centre and add a diameter dimension** (`[PB-CIRCLE-CENTER]`): a circle's centre is
FREE even when created at the origin, because `addByCenterRadius` does NOT reuse the sketch's
`originPoint` — its `centerSketchPoint` is a free point that happens to sit there. Set
`circle.centerSketchPoint.isFixed = True` and add
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` then `dimension.parameter.value =
<bore diameter in cm>`, which is 2 plus 1 freedoms removed. Do NOT `addCoincident` the centre to
`sketch.originPoint`; that has been observed to throw `VCS_SKETCH_SOLVING_FAILED`, at least on a
`setByDistanceOnPath` plane.

The bore diameter is this gear's Bore Diameter if specified and non-zero, otherwise **this gear's own**
`Pitch Diameter / 4`.

Gate the sketch: `if not sketch.isFullyConstrained: raise` naming it (`[BEVEL-F-FULL-CONSTRAINT]`).

<!-- check-step-calls: ignore Path.create addCoincident -->

`Path.create` and the `addCoincident`-to-origin form are named only to forbid them here; the module's
real `addCoincident` calls are in S8 and S10.

**From:** `spec/bevelgear/instructions.md` L778, L100-104;
`.claude/skills/generate-gear/PLAYBOOK.md` L451-457, L775-786.

## S31 `[GO]` Extrude-cut the bore through the Gear Body

Realised by the proof function `stepBoreCut`, checked by `assertBoreCut`.

<!-- proof-run: proofkit3d.RunSolidParallel(boreCases, stepBoreCut, assertBoreCut) -->

Cut a cylindrical **through** bore through the Gear Body along the shaft axis:

```python
extrudeInput = component.features.extrudeFeatures.createInput(
    boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
extrudeInput.setSymmetricExtent(
    adsk.core.ValueInput.createByReal(2 * coneDistance_cm), False)
extrudeInput.participantBodies = [gearBody]
component.features.extrudeFeatures.add(extrudeInput)
```

`setSymmetricExtent(distance, isFullLength)` with `isFullLength = False` means the distance is the
half-length **per side** (`[PB-THROUGH-CUT]`), so `2 × Cone Distance` per side is generously past any
face width. Do not pass a third, taper argument. Restrict the cut with `participantBodies` to this
gear's body alone.

The proof builds the tool as a **real extrude**, which a symmetric extent produces as a prism, but
performs **no cut**: the target is the frustum, whose bands are Lofts. It lays the tool and the bands
apart and asserts the cut from the tool's own measured geometry — its diameter, that its two ends sit
exactly `2 × Cone Distance` either side of the shaft edge's start, and that both clear the frustum,
which is what makes it a THROUGH cut — and computes the material it would remove from the frustum's own
profile clipped to the measured bore radius, which follows the toe dish rather than the front face on a
low-tooth-count pinion whose bore radius runs past its Toe Radius. **The cost is the pierced body**: one
lump with a hole and no enclosed void is not shown.

**From:** `spec/bevelgear/instructions.md` L778, L869-875;
`.claude/skills/generate-gear/PLAYBOOK.md` L729-732.

## S32 `[GO]` The meshing rotation (driving gear only)

Realised by the proof function `stepMeshingRotation`, checked by `assertMeshingRotation`.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepMeshingRotation, assertMeshingRotation) -->

**Do this here, in the Design component, before the body is moved out.** Rotate the driving body by
`180° / Driving Gear Teeth Number`, half a tooth pitch, about its shaft axis with the framework helper

```python
rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)
```

from `.solids`, which takes the rotation axis and origin from the **B→I profile edge's world
endpoints** (`[PB-MOVE-ROTATE]`). Both gears are patterned from a starting tooth in the axial plane, so
without this offset a driving tooth and a pinion tooth would both sit at the axial-plane crossing and
visually collide; the half-pitch turn puts a driving valley where the pinion tooth crosses, giving the
interlocked meshing look.

This runs in Design before `moveToComponent` because a construction axis cannot be added in the
moved-out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation must use the edge's world
geometry while the body is still in Design.

The **pinion** additionally gets `_pinionMeshPhase(pinionTeeth)`, its own extra rotation about its own
shaft axis in **radians**, `_PINION_MESH_PHASE_TEETH · 2π / pinionTeeth`, where
`_PINION_MESH_PHASE_TEETH` is **0** — for a straight bevel and for a spiral one alike, because the
spiral's mid-face section is left unrotated and therefore already meshes.

**A zero angle is a no-op, not a move.** `Matrix3D.setToRotation(0, axis, origin)` builds the identity
and Fusion refuses to move a body by it, with `RuntimeError: 3 : invalid transform` — measured on
2026-09-02 on the bevel pinion, whose mesh phase is 0 by default. Any caller that *computes* an angle
can legitimately arrive at zero, so `rotate_body_about_edge` absorbs it and returns early rather than
each call site guarding it; do not add a second guard, and do not remove the helper's.

The proof lays the turned copy apart from the body it was turned from — a contact the read-only
intersection cannot classify is reported Suspect — and asserts that the body turned by exactly the mesh
phase, kept its radius and its volume, and that the pinion case produced one body rather than two,
because a zero phase must be skipped rather than performed by the identity.

**From:** `spec/bevelgear/instructions.md` L780, L356-357;
`.claude/skills/generate-gear/PLAYBOOK.md` L800-809.

## S33 `[PROSE]` Move the finished bodies into their gear components

Relocate this gear's finished bodies with `body.moveToComponent(gearOccurrence)`, which preserves world
position and needs no activation (`[PB-NO-CROSS-SIBLING]`). All feature operations ran in the one
Design component precisely so no cross-sibling sketch, `project` or `Path.create` reference was ever
needed; relocating at the end is what puts the bodies where the browser shows them.

**From:** `spec/bevelgear/instructions.md` L736;
`.claude/skills/generate-gear/PLAYBOOK.md` L829-834.

## S34 `[PROSE]` Cleanup — hide the construction geometry

Call the framework helper

```python
hide_construction_geometry(bevelComponent)
```

from `.solids`. It recursively walks the Bevel Gear component tree, deduping by `entityToken`, and
hides every sketch, construction plane and construction axis with `isLightBulbOn = False`. Do not
re-implement the walk (`[BEVEL-F-CLEANUP]`, `[PB-TREE-CLEANUP]`).

**Hide construction geometry with the right property.** A `ConstructionPlane` or `ConstructionAxis` is
**not** hidden by `isVisible = False`, which has no visible effect on construction geometry; use
`isLightBulbOn = False`. So `isVisible = False` hides **sketches** and `isLightBulbOn = False` hides
**construction planes and axes** — do not cross them (`[PB-HIDE-AFTER-USE]`). Hide only after
consuming: draw, project, constrain, run the features, and only then hide. There is no sketch-only
mode and no per-mode guard, because bevel always builds solids. Leave only the two finished gear bodies
visible.

The driving gear's half-tooth-pitch **meshing rotation** is performed earlier, at S32, in the Design
component before the body is moved out; it is not a cleanup step.

**Do not add a `settle_sketch_display` call.** `commands/_gear_command.py` calls
`geargen.settle_sketch_display` once after `generate()` returns, and every gear command runs through
that one call (`[PB-SETTLE-DISPLAY]`), so nothing about it belongs in a generated module. Fusion's
browser shows a stale constraint icon for every sketch a generator authors until something makes it
settle, and **the icon is not evidence**: measured in Fusion on 2026-09-12, `isFullyConstrained`
returned `True` for the Anchor, Gear Profiles and both per-gear Profile sketches at the moment each was
built and again after the last feature, while all four icons showed otherwise.

`deleteComponent()` is the error rollback the entry point calls on an exception: it calls `deleteMe()`
on `self.bevelOccurrence`. Bevel registers no user parameters, so there are none to clean up.

<!-- check-step-calls: ignore settle_sketch_display isVisible -->

<!-- check-step-calls: ignore deleteComponent generate -->

`settle_sketch_display` is named only to forbid the module adding its own call, and `isVisible` only to
say which property does NOT hide construction geometry. `deleteComponent` and `generate` are methods
the module DEFINES for the entry point to call, not calls it makes.

**From:** `spec/bevelgear/instructions.md` L784-788, L341-342;
`spec/bevelgear/fusion.md` L161-166;
`.claude/skills/generate-gear/PLAYBOOK.md` L534-555, L659-671, L835-837.
