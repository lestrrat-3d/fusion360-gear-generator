# Bevel Gear — compiled step list

The proof for these steps is `proof/bevelgear/geometry_test.go`,
`proof/bevelgear/variables_test.go`, `proof/bevelgear/sketches_test.go`,
`proof/bevelgear/solids_test.go`, `proof/bevelgear/spiral_test.go` and the generated registration
file `proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `a06e4eef304e0f0154fe69085dc041e1f73e8172` |
| `spec/bevelgear/fusion.md` | `754729fc9a33ad9dbf93816e0430e4b94d82e05b` |
| `spec/bevelgear/spiral-tooth-trace.md` | `c9ec08561ced7975aa0ed9ad6a330186259c0d08` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `abb1123b5910f10c5c96c1ad936a38691ea3e7fb` |

## S1 `[PROSE]` Dialog inputs — `BevelGearCommandInputsConfigurator.configure`

`BevelGearCommandInputsConfigurator` is a plain class (no base) with three classmethods:
`configure(cls, cmd)`, which adds the dialog inputs to `cmd.commandInputs`;
`handle_input_changed(cls, args)`, which the dialog's `inputChanged` event delegates one line to;
and the private helper `_updateSpiralInputVisibility(cls, inputs)`. `commands/bevelgear/entry.py`
binds `configure` and `handle_input_changed` **by name**, so both names are fixed.

<!-- check-step-calls: ignore configure handle_input_changed _updateSpiralInputVisibility -->
<!-- check-compile: ignore configure handle_input_changed _updateSpiralInputVisibility -->

Those three are methods this module DEFINES for the framework to call, not calls it makes, so they
are neither calls the module must make nor Fusion API names, which is why both gates are told to
skip them.

**The twenty inputs, in the order `configure()` adds them.** The display order is fixed as the rows
below: Target Plane first so it wins Fusion's auto-focus (`[PB-AUTOFOCUS-FIRST]` — Fusion
auto-focuses the FIRST `SelectionCommandInput` and ignores a later `hasFocus`), then Center Point so
the user flows from plane to point, then the pre-selected Parent Component, then the numeric and
boolean fields.

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

There are **20** dialog inputs and **20** module-level `INPUT_ID_*` constants holding the table's id
strings in row order, named exactly:

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

Two more module constants carry the dropdown's item strings: `_HAND_RIGHT = 'Right'` and
`_HAND_LEFT = 'Left'`. There are no `PARAM_*` name strings, because bevel registers no Fusion user
parameters at all (`[PB-PRECOMPUTED-MODE]`; see S2).

**How each row is added.** A selection row is
`inputs.addSelectionInput(<id>, <label>, <tooltip>)`, then one
`selection.addSelectionFilter(...)` per filter, then `selection.setSelectionLimits(1, 1)`. The
filters are written as the named constants
`adsk.core.SelectionCommandInput.ConstructionPlanes`, `.PlanarFaces`, `.ConstructionPoints`,
`.SketchPoints`, `.Occurrences` and `.RootComponents`, never as quoted literals
(`[PB-SELECTION-FILTER-ENUM]`; the constant is checked for typos at import and survives a renamed
filter, where a literal fails silently by selecting nothing). The filter set and the limits are
contract surface, declared per input in the table (`[PB-SELECTION-DECL]`). The Parent row
pre-selects the root component with `parentSelection.addSelection(get_design().rootComponent)`.

A numeric row is `inputs.addValueInput(<id>, <label>, <unit>, <ValueInput>)` with the default built
by `adsk.core.ValueInput.createByReal(...)` or `adsk.core.ValueInput.createByString('90 deg')` as
the table says. A `createByReal` default is in Fusion INTERNAL units regardless of the unit string
(`[PB-DIALOG-DEFAULT-UNITS]`), which is why every `mm` default is written `createByReal(to_cm(0))`
and the two angles are `createByString` so the expression engine parses them.

The checkbox is `inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)` —
`isCheckBox` true, an empty resource folder, initial value `True`.

The dropdown is
`inputs.addDropDownCommandInput(INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)`,
then `hand.listItems.add(_HAND_RIGHT, True, '')` and `hand.listItems.add(_HAND_LEFT, False, '')` —
`Right` added selected, `Left` added unselected.

**Conditional visibility — the spiral-only inputs show only when ψ > 0.** Hand of Spiral and Cutter
Radius are relevant only for curved bevels, so they are hidden whenever Mean Spiral Angle ψ = 0 and
shown when ψ > 0. Mean Spiral Angle itself is the controller and is always visible: it is how the
user reaches ψ > 0. There is no declarative show-if in the Fusion API, so this is realized with
`commandInput.isVisible`:

- `_updateSpiralInputVisibility(cls, inputs)` reads `inputs.itemById(INPUT_ID_SPIRAL_ANGLE)`,
  `inputs.itemById(INPUT_ID_HAND)` and `inputs.itemById(INPUT_ID_CUTTER_RADIUS)`. If any of the
  three is `None` it returns early. It evaluates the spiral input's **`.expression`** — not its
  `.value` — with
  `get_design().unitsManager.evaluateExpression(spiral.expression, 'rad')`, which returns internal
  radians, inside a `try/except`: a half-typed expression can raise mid-edit, and on failure both
  inputs are left **shown**. Then it sets `hand.isVisible` and `cutter.isVisible` to `(value > 0)`.
- `configure()` calls `cls._updateSpiralInputVisibility(inputs)` as its **last** step, so the
  initial state is correct — the default ψ = 35° shows both.
- `handle_input_changed(cls, args)` is one line: `cls._updateSpiralInputVisibility(args.inputs)`. It
  recomputes on every input change, which is cheap and robust, so there is no branching on which
  input changed.

`isVisible` only hides the dialog row. The input still exists and S2 reads it normally, and the
ψ = 0 build ignores Hand and Cutter anyway, so hiding is purely cosmetic and cannot affect
generation.

<!-- check-step-calls: ignore settle_sketch_display -->

Do not add a `settle_sketch_display` call anywhere in this module: `commands/_gear_command.py`
already calls it once after `generate()` returns for every gear, and nothing about it belongs in a
generated module (`[PB-SETTLE-DISPLAY]`).

**From:** `spec/bevelgear/instructions.md` L25-144, L145-266, L267-293;
`.claude/skills/generate-gear/PLAYBOOK.md` L128-143, L282-316, L346-349, L525-547, L548-559,
L841-859.

## S2 `[PROSE]` Read and validate the inputs — `_readInputs`

`BevelGearGenerator` is a **standalone** generator. It does not subclass `base.Generator`, uses no
`GenerationContext`, and registers no Fusion user parameters. `__init__(self, design)` stores
`self.design` and sets `self.bevelOccurrence = None`. Its public surface is `generate(inputs)` and
`deleteComponent()`, both bound by name from `commands/bevelgear/entry.py`.

<!-- check-step-calls: ignore generate deleteComponent _readInputs get_value -->

`generate`, `deleteComponent` and `_readInputs` are methods this module defines rather than calls it
makes. `get_value` is named below only to forbid it on a boolean input, which is why it is exempted
too.

From `base.py` import **only** `get_selection` and `get_boolean`; the `Generator`,
`ParamNamePrefix` and `ComponentCleaner` machinery is unused. Imports are explicit — no `import *`
in a gear module.

**One `_readInputs` pass reads and validates everything, before anything creates an occurrence.**
It returns the 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth,
pinionTeeth, shaftAngle_deg)` and stashes the rest on `self` as `self._drivingBaseHeight_cm`,
`self._pinionBaseHeight_cm`, `self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`,
`self._faceWidth_cm`, `self._toothSpacing_cm`, `self._spiralAngle_rad`, `self._hand`,
`self._cutterRadius_cm`, `self._toeExtension`, `self._drivingToeRadius_cm` and
`self._pinionToeRadius_cm`. Later, `generate()` stashes the derived `self._coneDistance_cm`,
`self._gamma_p` and `self._gamma_g`, and the §2 step stashes `self._faceWidthResolved_cm`. Those
plain attributes and the per-gear dicts of S6 ARE the intended structure: do not introduce a
`GenerationContext`-style class.

**Read each input with the helper that matches the type it was declared with** (`[PB-INPUT-READ]`).
The three selections use `get_selection(inputs, <id>)` and are pulled out first. The checkbox uses
`get_boolean(inputs, INPUT_ID_BORE_ENABLE)` — never `get_value`, which reads `.expression` and
raises `AttributeError: 'BoolValueCommandInput' object has no attribute 'expression'`. The dropdown
is read as `inputs.itemById(INPUT_ID_HAND).selectedItem`, taking `.name` and defaulting to
`_HAND_RIGHT` when it is `None`.

**Reading the raw numbers.** Every numeric and angle input is read by evaluating its expression:
`design.unitsManager.evaluateExpression(<input>.expression, <units>)` with `''`, `'mm'` or `'deg'`
as the table gives. It ALWAYS returns Fusion internal units — cm for length, **radians** for angle —
regardless of the unit string (`[PB-EVAL-EXPRESSION]`), so a `deg` field comes back in radians and
is converted with `math.degrees(...)` before any degree-range check. Do not read via
`ValueInput.realValue`.

**Units — critical.** The `mm` inputs (Driving/Pinion Base Height, Driving/Pinion Bore Diameter,
Face Width, Tooth Spacing, Driving/Pinion Toe Radius) and the two `deg` inputs come back already in
internal units; use them as-is and do NOT `to_cm` them again. `toeExtension` is a plain unitless
percentage and needs no conversion. **`Module` is read with unit `''`, so it comes back as a raw
number that means millimetres** — a module of `1` is 1 mm. Therefore every length derived from
Module must be `to_cm`-converted before it touches geometry: Pitch Diameter = `to_cm(Module * teeth)`,
Cone Distance = `to_cm(...)`, dedendum = `to_cm(1.25 * Module)`, the module-length construction
extensions at E, F, G, H, I and J, and the default Face Width `Cone Distance / 6`. The
`VirtualSpurProxy` likewise receives Module in mm and `to_cm`'s the circle radii it serves. Mixing a
raw-mm Module-derived length with an already-cm `mm` input — comparing Face Width against the
Module-derived Maximum Face Width, say — makes the gear come out about ten times off and the
Face-Width bound meaningless.

Both teeth inputs are coerced with `int(round(...))` before validation.

**The validation order, which is load-bearing.**

1. Range checks that need nothing derived: `module > 0`; each tooth count `>= 3`;
   non-negative Base Heights, Bore Diameters, Face Width, Tooth Spacing and Cutter Radius; Toe
   Extension within `[0, 100]`; Mean Spiral Angle within `[0, 60)` degrees; Shaft Angle at least
   30 degrees.
2. **Maximum Shaft Angle**, which depends on both tooth counts, so it is checked after both are read
   and coerced. A pitch cone angle reaching 90 degrees turns that gear's pitch cone inside out:
   `R * cos γ`, which is the along-shaft seed length for Apex→A and Apex→B and the denominator of
   the back-cone virtual pitch radius, passes through zero and changes sign, so the seed points
   backwards along the shaft and the virtual radius is unbounded. Both cone angles stay below 90
   degrees exactly while

       cos(Shaft Angle) > -min(Driving Gear Pitch Diameter, Pinion Gear Pitch Diameter)
                          / max(Driving Gear Pitch Diameter, Pinion Gear Pitch Diameter)

   so `acos` is a hard singularity and the check rejects a Shaft Angle **at or above**
   `degrees(acos(-smaller / larger))`, naming the computed limit in the message. **The Maximum
   Shaft Angle is that cone-angle limit capped at 150 degrees**, and the cone-angle half is
   exclusive while the 150-degree half is inclusive. A 31/17 pair gives `acos(-17/31) = 123.24°`;
   equal tooth counts give `acos(-1) = 180°`, which is no constraint at all.
3. Compute the two pitch cone angles from the closed form, once both tooth counts and the Shaft
   Angle are known: `tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`, `γ_g = Σ − γ_p`, with
   `PPD = Module * Pinion Gear Teeth Number` and `DPD = Module * Driving Gear Teeth Number`. The
   Pitch Cone Distance is `R = (PPD / 2) / sin γ_p`.
4. **Minimum Teeth**, per gear, with that gear's own γ: `teeth >= 5.27 * cos γ`, the constant being
   `2 * (1.05 * 1.25 / 0.95 + 1.25)`. Name the computed floor in the message. It sits on top of the
   blanket `teeth >= 3`, which stays as the absolute floor. At Shaft Angle 90° the computed floor is
   3.72, so four teeth: measured, with both base-height bounds applied an equal 4-tooth pair solves
   and a 3-tooth pair still fails on the heel edge, because its Maximum Base Height has fallen below
   its Minimum.
5. **Minimum and Maximum Base Height**, per gear, both closed-form and both needing only `r`, `γ`
   and Module — no solved sketch geometry. With `r` that gear's pitch radius:

       Maximum Base Height = 0.95 * (r - 1.25 * Module * cos γ) * tan γ
       Minimum Base Height = 1.05 * 1.25 * Module * sin γ

   The base height is the offset dimension between the A→Apex2 drop and G→H (respectively the
   B→Apex2 drop and I→J), so it is measured from **Apex 2's plane**, not from the dedendum point.
   Walking out along the dedendum line from Apex 2, the perpendicular distance to the shaft axis
   falls at `cos γ` per unit and the along-shaft coordinate rises at `sin γ`, so H reaches the axis
   when the base height reaches `r * tan γ`. That is the true crossing; the bound above sits
   `1.25 * Module * sin γ` below it and is therefore deliberately conservative, refusing a band of
   base heights that would in fact still build. Past the true crossing the hexagonal frustum profile
   has crossed its own axis of revolution and the revolve aborts with `ASM_WIRE_X_AXIS`
   (`[PB-REVOLVE]`), and a heel edge that merely approaches the axis is already degenerate. The
   exact bound would be `0.95 * r * tan γ`; do not adopt it without re-running the low-tooth-count
   cases. The Minimum exists because H is placed a module beyond C along the base-height offset, so
   unless the base height carries H past the dedendum's own along-shaft projection
   `1.25 * Module * sin γ`, H lands behind C and the edge C→H runs back inward instead of outward.
   The `1.05` is the margin, mirroring the `0.95` the other bounds carry.

   Apply them in **both** directions, per gear: raise a fallback that falls below the Minimum, cap a
   fallback that rises above the Maximum, and reject a user value outside either end with a message
   naming the bound it broke. The Minimum Teeth check runs first precisely because it is the
   statement that this window is non-empty, so this step never has to describe what to do when the
   Minimum exceeds the Maximum.

   The **driving** side resolves first: its fallback, when the input is 0, is
   `Module * Driving Gear Teeth Number / 8`. The **pinion**'s fallback is the **resolved** driving
   base height times `Pinion Gear Teeth Number / Driving Gear Teeth Number` — resolved meaning after
   the driving fallback and after the driving Maximum capped it, never the raw driving input — and
   then the pinion's own Minimum and Maximum are applied to the result, because the two gears have
   different pitch cone angles whenever the tooth counts differ.

   Worked case, Module 1, Driving 31, Pinion 31, Shaft Angle 30°: each γ is 15°, the Maximum is
   `0.95 * (15.5 - 1.25*cos 15°) * tan 15° = 3.638 mm`, the true crossing is
   `15.5 * tan 15° = 4.153 mm`, and the driving fallback resolves to `3.875 mm`. The default
   therefore sits between the two and is capped to 3.638 mm, though it would not have folded
   uncapped.

**Derived values this step also resolves.** Cone Distance is
`sqrt((Module * Driving Gear Teeth Number)**2 + (Module * Pinion Gear Teeth Number)**2)` and depends
on the two tooth counts only, never on the Shaft Angle. **"Cone Distance" and "Pitch Cone Distance"
are two different lengths and both are used**: the Cone Distance just defined is the diagonal of the
two pitch diameters, while the Pitch Cone Distance `R` is the real apex-to-heel length along the
pitch cone. They coincide as `Cone Distance = 2 * R` exactly when Shaft Angle is 90°, for any pair
of tooth counts, and diverge everywhere else — an equal 31/31 pair at 30° has
`Cone Distance = 43.84 mm` against `R = 59.89 mm`, and at 140° `R = 16.50 mm`. Wherever a step below
says "Cone Distance" it means the diagonal, and `R` is always written as `R`.

Each gear's Bore Diameter is its input when non-zero, otherwise that gear's Pitch Diameter / 4; it
is consulted only when Enable Bore is checked.

Face Width, the Toe Radii, the Root Length and the Maximum Face Width are resolved later, in S6,
because the Maximum Face Width cannot be evaluated until the §2 sketch has solved.

**No live Fusion user parameters.** Every value — pitch diameters, cone distance, base heights, bore
diameters, virtual tooth counts, face width — is precomputed in Python in internal cm and written
into geometry numerically: sketch dimensions via `dimension.parameter.value = <number>` and feature
inputs via `adsk.core.ValueInput.createByReal(<number>)` (`[PB-PRECOMPUTED-MODE]`). Note the caveat
that comes with it: a sketch dimension or feature input takes the parameter's numeric value at
generation time and is not a live link, so a gear is changed by re-running the dialog
(`[PB-NUMERIC-SNAPSHOT]`).

Errors are raised, not swallowed: the entry point's `try/except` and `deleteComponent()` own
rollback, and progress is logged with `futil.log(...)` (`[PB-LOGGING]`).

**From:** `spec/bevelgear/instructions.md` L25-144, L145-266, L267-293, L294-316, L317-388,
L411-422, L423-452; `.claude/skills/generate-gear/PLAYBOOK.md` L17-41, L103-127, L196-243,
L695-698, L841-859.

## S2a `[GO]` Resolve the Variables — clamped against rejected

<!-- proof-run: proofkit.RunParallel(variableCases, stepResolveInputs) -->

S2 reads the dialog and states every formula the Variables section uses. This step is where those
formulas are APPLIED, and where the section's two different behaviours are assigned to two
different cases. It runs inside `_readInputs`, in the order S2 gives, and it creates nothing: every
value here is closed form over the twenty dialog inputs and none of it needs a sketch.

**The split is by where the value came from, not by which bound it broke.**

- A value the dialog **FELL BACK to** is **CLAMPED** into its window — raised to a minimum, capped
  to a maximum — and the build goes on. That is the driving base height from
  `Module * Driving Gear Teeth Number / 8`, the pinion base height from the resolved driving height
  times the tooth ratio, and the Face Width from `Cone Distance / 6`.
- A value the **USER TYPED** is **REJECTED**, and the build stops. The message names the input, the
  bound it broke, and that bound's number, so the user is told something they can act on
  (`[PB-VALIDATE-INPUTS]` asks for exactly that shape).

Clamping a user value would silently build a gear the user did not ask for; rejecting a fallback
would refuse a dialog the user never touched. Neither is the other.

**Which bounds close which way.**

| bound | applies to | one step inside | at the bound | one step outside |
|---|---|---|---|---|
| the documented 30 degree Shaft Angle floor | Shaft Angle | accepted | **accepted** | rejected |
| the Maximum Shaft Angle, cone-angle half | Shaft Angle | accepted | **rejected** | rejected |
| the Maximum Shaft Angle, 150 degree half | Shaft Angle | accepted | **accepted** | rejected |
| the absolute tooth floor of 3 | each tooth count | accepted | **accepted** | rejected |
| the computed Minimum Teeth floor | each tooth count | accepted | see below | rejected |
| Minimum Base Height | that gear's base height | accepted | **accepted** | rejected, or clamped for a fallback |
| Maximum Base Height | that gear's base height | accepted | **accepted** | rejected, or clamped for a fallback |
| Maximum Face Width | Face Width | accepted | **accepted** | rejected, or clamped for the default |
| Toe Radius Ceiling | that gear's Toe Radius | accepted | **rejected** | rejected |
| the [0, 100] Toe Extension range | Toe Extension | accepted | **accepted** | rejected |
| the [0, 60) Mean Spiral Angle range | Mean Spiral Angle | accepted | 0 accepted, 60 **rejected** | rejected |

Two of those close in a way that is easy to get backwards, and each is a real failure mode. The
**cone-angle half of the Maximum Shaft Angle is EXCLUSIVE**, because `acos` of the ratio is a hard
singularity there and a pitch cone angle reaching 90 degrees turns that gear's cone inside out; the
**150 degree half is INCLUSIVE**, because it is a practical ceiling on the figure rather than a
measured one. And a **Toe Radius must be STRICTLY below its ceiling**, so the ceiling itself is the
first refused value rather than the last accepted one.

**The Minimum Teeth floor is a fixed point, not a number a dialog can be set to.** Lowering a tooth
count moves that gear's pitch cone angle, which moves `5.27 * cos γ` with it, so the smallest
admissible count is found by the check refusing the count below it rather than by comparing against
a constant computed at some other count.

**A defaulted Toe Radius above its own ceiling is a real configuration, not a rejection.** On a gear
with a large pitch cone angle the inner toe corner already sits at a LARGER radius than the outer
one, so the toe dish leans toward the heel and the Toe Limit comes out at or below the Toe Extension
0 root length. Toe Extension 0 still resolves and the gear stays buildable exactly as before; a
positive Toe Extension is refused, naming the gear and the Toe Radius Ceiling its Toe Radius has to
come below. The ceiling and the Toe Limit are two readings of one fact: a Toe Radius is at or above
its ceiling exactly when its Toe Limit is at or below that root length.

**The figures the spec publishes, which are what a reader checks the module against by hand.**

    acos(-17/31)                            = 123.2564 degrees   (the spec prints 123.24)
    acos(-1)                                = 180 degrees, so an equal pair is capped at 150
    Module 1, 31/31, Shaft Angle 30:
      each pitch cone angle                 = 15 degrees
      Maximum Base Height                   = 0.95 * (15.5 - 1.25*cos 15) * tan 15 = 3.638 mm
      the true heel crossing                = 15.5 * tan 15                        = 4.153 mm
      the driving fallback                  = 1 * 31 / 8                           = 3.875 mm
      so the default is CAPPED to 3.638, and it would not have folded uncapped
      Cone Distance                         = 43.84 mm
      Pitch Cone Distance R                 = 59.89 mm
    Module 1, 31/31, Shaft Angle 140:  R    = 16.4948 mm          (the spec prints 16.50)
    the Minimum Teeth constant              = 2 * (1.05*1.25/0.95 + 1.25) = 5.2632
      at Shaft Angle 90 its floor           = 3.72, so four teeth
      at three teeth the base-height window is EMPTY and at four it is not
    the fallback clears the raw dedendum projection while teeth > 10 * sin γ,
      which at Shaft Angle 90 is 7.07, so eight teeth clear it and seven do not
    at Shaft Angle 90 the Maximum Face Width is 0.95 * smaller pitch diameter^2 / (2 * Cone Distance)
      and the naive Cone Distance / 6 default exceeds it above a ratio of sqrt(2)

Resolve these in `_readInputs` and raise on the first rejection, naming the bound. Nothing here
touches geometry, so it all happens before the component tree of S3 exists.

`stepResolveInputs` realises this step.

### What the proof establishes

The proof resolves each case's dialog, checks the resolution against the closed form, checks the
published figures above, sweeps every bound from both sides, and then draws that gear's frustum at
the resolved values as the witness that what the resolution admits stays on one side of its own axis
of revolution — the condition that would otherwise abort the revolve with `ASM_WIRE_X_AXIS`. The
sketch harness carries it; nothing about it needs a solid.

The sweep is what separates the two behaviours: for each bound a user can break it types a value one
step inside and one step outside and checks that the inside one is accepted, the outside one
rejected, which bound the rejection names, and that the message carries that bound's number rather
than only the offending value. For each bound a FALLBACK can break it checks the opposite — no
rejection at all, and a clamp that lands on the bound. A probe is skipped, visibly, when a different
bound bites first on the dialog it would have to build, which is what happens when a tooth count is
lowered past a Shaft Angle that was already near its ceiling; that order is the spec's, so the probe
has nothing to say there. The declared-refusal cases are NOT skipped in this step, because nothing
here solves the §2 lattice and one of them carries the worked base-height figures.

What the proof does NOT fix is the wording of a message. A step list cannot pin a human sentence, so
the rendered text is the proof's own; what is proved is the part that has to agree — that a
rejection happened, which input and which bound it names, and that bound's value.

Three of the published figures disagree with what the formulas give, and the proof carries the
formulas. `acos(-17/31)` is 123.2564 and rounds to 123.26 where the spec prints 123.24; the Pitch
Cone Distance at 140 degrees is 16.4948 and rounds to 16.49 where the spec prints 16.50; and the
Minimum Teeth constant is stated twice, as `5.27` and as its derivation `2 * (1.05 * 1.25 / 0.95 +
1.25)` = 5.2632, which round to different second decimals. The spec's own published floor of 3.72
follows the derivation. The module implements the rule as the spec writes it, `5.27`, which is the
more conservative of the two, and both give the same integer floor of four teeth at Shaft Angle 90.

One more reading the proof makes explicit: the Maximum Face Width's binding side is the gear with
the SMALLER pitch diameter, which is normally the pinion and which the spec's closed form at 90
degrees names as the pinion, but a dialog may put the smaller count on the driving gear and then the
driving side binds. The proof takes the minimum of the two distances, as §2 requires.

**From:** `spec/bevelgear/instructions.md` L25-144, L145-266, L317-388, L471-577;
`.claude/skills/generate-gear/PLAYBOOK.md` L317-345, L432-441, L582-596, L708-714.


## S3 `[PROSE]` Build the component tree

`generate(inputs)` reads **all** inputs first (S2), then creates occurrences. Because bevel registers
no user parameters, nothing creates an occurrence until every selection is already read, so the
selection-context-shift hazard does not bite here — but keep the order, read inputs then build the
tree then build geometry, so it stays that way (`[PB-SELECTION-STASH]`).

Create each occurrence directly with
`parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` and name it through
`occurrence.component.name` (`[PB-OCCURRENCE-TREE]`). Do not use `getOccurrence`, `addParameter`,
`parameterName` or `createSketchObject` from `base.Generator`; this generator manages its own tree.

The tree is:

- **`Bevel Gear`**, a child of the user's Parent Component, held on `self.bevelOccurrence` for
  cleanup and on `self.bevelComponent`.
- **`Design`**, a child of `Bevel Gear`, held on `self.designOccurrence` and
  `self.designComponent`. It owns every sketch, construction plane, construction axis and feature.
- **`Pinion Gear`** and **`Driving Gear`**, each a child of the **`Bevel Gear`** component — the
  same component that owns Design, NOT the user's Parent Component. They are created inside the
  per-gear body step (S23 onward) and receive the finished bodies at the end.

**Never activate any occurrence** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`). The bevel
reason is specific: the Anchor Sketch is created on the user's EXTERNAL, root-owned target plane, and
an activated occurrence resolves that external plane in its own local frame, so the build collapses
onto world XY regardless of the real plane tilt. All features run in the single Design component, so
no cross-sibling reference is ever needed (`[PB-NO-CROSS-SIBLING]` — Fusion rejects cross-sibling
sketch and project calls even when the target is activated or the entities are wrapped in
`createForAssemblyContext` proxies). The one exception in the whole module is the spiral crown's
scale step, S20.

<!-- check-step-calls: ignore createForAssemblyContext deleteMe -->

`createForAssemblyContext` is named only as a route that does not help, and `deleteMe` is described
as what `deleteComponent()` does rather than called here, which is why both are exempted.

`deleteComponent()` is the error rollback the entry point calls on an exception: it calls
`deleteMe()` on `self.bevelOccurrence` when that occurrence exists (`[PB-TREE-CLEANUP]`).

**From:** `spec/bevelgear/instructions.md` L19-24, L267-293, L317-388, L411-422, L453-464,
L689-750; `spec/bevelgear/fusion.md` L153-166; `.claude/skills/generate-gear/PLAYBOOK.md`
L802-840.

## S4 `[GO]` Anchor sketch

<!-- proof-run: proofkit.RunParallel(anchorCases, stepAnchorSketch) -->

Create the sketch with `designComponent.sketches.add(targetPlane)` and name it `Anchor` through
`sketch.name`. It is started **directly on the user-selected target plane**, whether that selection
is a `ConstructionPlane` or a `PlanarFace`; do not re-derive or offset it
(`[PB-USE-SELECTED-PLANE]` — a construction plane created in a sub-occurrence and offset from a face
in another component resolves in the sub-component's own frame and silently loses the selected
plane's world orientation, collapsing the whole build onto XY).

Mark the centre by projecting the user-selected Center Point into the sketch:
`sketch.project(centerPoint)`, keeping the resulting `SketchPoint`.

**Write the call as `sketch.project(entity)` and do not substitute `project2`.** The compiled Fusion
API reference declares `project2(entities, isLinked)` and no `project`, so every gate in this repo
reports the call as unverified; that report is expected and is not a defect to fix here. `project`
is what the shipped add-ins call and what the spur step list names, and this repo's settled position
is to keep it and keep reporting it. The two are not interchangeable in any case: `project2` takes a
list and returns a list, so swapping the name alone would be wrong. Only a Fusion session can settle
whether `project` exists at runtime.

<!-- check-step-calls: ignore project2 -->

`project2` is named above only to forbid the substitution, which is why it is exempted.

Draw the Anchor Line with `sketch.sketchCurves.sketchLines.addByTwoPoints(...)` — curve collections
live under `sketchCurves`, never directly on the sketch (`[PB-SKETCHCURVES]`) — seeding its two
endpoints at **exactly ±0.5 cm from the projected centre** along the sketch-local X, so the seeded
length is 10 mm. Then:

- `sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)` — the intersection,
  pinning the centre onto the line;
- `sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)` — the centre bisects the
  line. Use **both**, not midpoint alone;
- `sketch.sketchDimensions.addDistanceDimension(anchorLine.startSketchPoint, anchorLine.endSketchPoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
  **without assigning `.parameter.value`** — the dimension simply locks the length at the seeded
  10 mm, whose value is arbitrary because this is only a reference line. Leave it driving; never
  pass a trailing `isDriven` (`[PB-DRIVING-DIM]`);
- `sketch.geometricConstraints.addHorizontal(anchorLine)` — sketch-local, per
  `[PB-REFLINE-DIRECTION]`. It works on any tilted target plane, where a world-axis lock would
  mis-orient the figure.

The anchor line's absolute direction is arbitrary — nothing downstream depends on it, because §2
derives every direction relative to the projected anchor line — but it must not be a free degree of
freedom: with midpoint, length and Horizontal the line has zero DOF.

Stash the projected-centre `SketchPoint` on `self._anchorCenterPoint` so S6 re-projects **this**
anchor-sketch point rather than the raw user-selected centre.

After all constraints, gate the sketch: `if not sketch.isFullyConstrained: raise` naming the sketch
(`[PB-FULL-CONSTRAINT]`, `[BEVEL-F-FULL-CONSTRAINT]` — a free DOF here is a generation defect, not a
warning).

`stepAnchorSketch` realises this step.

### What the proof establishes

The proof builds the same line and gates it on the sketch engine's own verdict, so the scheme is
shown to reach DOF 0 with nothing conflicting, nothing redundant and no discrete ambiguity, across
the whole case table.

Two differences are recorded in the proof file beside the step. The projected centre is modelled as
reference geometry, which the engine coordinate-LOCKS, while Fusion's projection is associative and
still carries free degrees of freedom (`[PB-PROJECT-NOT-FIXED]`); nothing about this sketch depends
on the projection being free, so the difference costs the proof nothing. And the point-on-line
coincidence is omitted there: the engine's midpoint constraint already carries two residual rows, so
the coincidence is implied by it and adding it reports the sketch as redundant at DOF 0. Fusion
absorbs the extra row and the spec asks for both, so the Fusion build carries both and the proof
carries the midpoint alone.

One thing the proof adds. The spec's aligned distance dimension is a magnitude whose direction
Fusion captures from the seeded geometry at creation (`[PB-DIM-VALUE-SEMANTICS]`); the engine takes
a signed target instead, so the seed side crosses over as a positive horizontal distance. With the
magnitude alone the line reaches DOF 0 and still admits two configurations — it can turn end for end
— which the gate refuses, and measured, it did.

**From:** `spec/bevelgear/instructions.md` L389-410, L465-470; `spec/bevelgear/fusion.md` L19-68;
`.claude/skills/generate-gear/PLAYBOOK.md` L230-243, L432-441, L449-463, L500-507, L626-627,
L650-662, L829-840.

## S5 `[PROSE]` Gear Profiles Plane

Create the plane that carries the whole §2 figure:

```
planeInput = designComponent.constructionPlanes.createInput()
planeInput.setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)
gearProfilesPlane = designComponent.constructionPlanes.add(planeInput)
gearProfilesPlane.name = 'Gear Profiles Plane'
```

`setByAngle` takes the linear entity, the angle and the planar entity the angle is measured from
(`[PB-CONSTRUCTION-PLANES]`). Pass the `SketchLine` **directly**; do not wrap it in
`adsk.fusion.Path.create(...)` first, which raises
`RuntimeError … InternalValidationError : Utils::getObjectPath(sketchCurve, …)` whenever the
curve's owner sketch is not trivially resolvable in a multi-component context.

<!-- check-step-calls: ignore Path.create getObjectPath -->
<!-- check-compile: ignore getObjectPath -->

`Path.create` is named only to forbid it here, which is why it is exempted. `getObjectPath` is not a
call at all: it is a fragment of the C++ error text Fusion raises, quoted so the failure is
recognisable, so neither gate should read it as a Fusion name or a call this module makes.

The plane includes the Anchor Line and stands at 90 degrees to the target plane — by default it
would lie flush to the anchor line's own plane, and perpendicular is what is wanted. **Build it off
the original `targetPlane`** as the reference, not off a re-derived or offset plane: this is the
other place the target-plane orientation reaches the bodies, and substituting a different plane here
also collapses the gear onto XY (`[PB-USE-SELECTED-PLANE]`).

Hold the plane on `self._gearProfilesPlane`. Leave it visible while the features that consume it
run; S29 hides it (`[PB-HIDE-AFTER-USE]`).

This step creates a construction plane and no measurable geometry, so no proof function realises it.

**From:** `spec/bevelgear/instructions.md` L471-577; `.claude/skills/generate-gear/PLAYBOOK.md`
L650-662, L766-777, L829-840.

## S6 `[GO]` Gear Profiles sketch — the §2 lattice

<!-- proof-run: proofkit.RunParallel(latticeCases, stepGearProfiles) -->

Create the sketch with `designComponent.sketches.add(self._gearProfilesPlane)` and name it
`Gear Profiles`. Hold it on `self._gpSketch`.

**Three rules govern every line in this sketch.**

**Every line is a construction line** — set `line.isConstruction = True` on the lattice lines, the
toe lines M→N and O→P, and the short reference and connector lines M→C, N→A′, O→D, P→B′, A′→G, B′→I,
C→K, D→L, C→K′ and D→L′ alike. The solid features later consume only the per-gear Profile sketches
of S10, never a §2 curve directly.

**Every line is built in the COINCIDENT style, never by sharing** (`[BEVEL-F-COINCIDENT-STYLE]`, a
stricter delta to `[PB-SHARE-XOR-COINCIDENT]`, which allows either style where §2 allows only one).
When a §2 line must start at or connect to an already-existing point, create the line from raw
`adsk.core.Point3D.create(x, y, 0)` coordinates for BOTH endpoints and pin each connecting endpoint
with exactly one `sketch.geometricConstraints.addCoincident(line.startSketchPoint, <existingPoint>)`
or `addCoincident(line.endSketchPoint, <existingPoint>)`. Never pass an existing `SketchPoint` into
`addByTwoPoints` to share it. This is load-bearing both ways: sharing without a coincident leaves
the sketch under-constrained and the gate fails on "Gear Profiles"; sharing AND coinciding is
redundant and the solve fails outright with
`RuntimeError … VCS_SKETCH_SOLVING_FAILED - failed to create offset`. It covers the short reference
and connector lines too, the ones whose BOTH endpoints already exist — a regen that shared only
those came out about fourteen coincidents short and the gate failed.

**Each named line is created ONCE and later references reuse that object** (`[BEVEL-F-LINE-ONCE]`).
The module-length extensions A→E, B→F, E→G and F→I and the dedendum and closing lines C→H, D→J, G→H
and I→J are named construction lines: when a step below says "collinear with line A→E" it means the
very line drawn earlier, so the helper that creates a module-extension must RETURN the line and that
reference is kept. Drawing a second line between the same two points to obtain a reference
over-determines the coupled net and the solve fails with
`RuntimeError … VCS_SKETCH_OVER_CONSTRAINTS - failed to create offset`. A duplicate carrying only
per-end coincidents has been observed to solve and even pass the gate, so do not rely on the solver
to catch one.

**Every length dimension in this sketch is aligned.**
`sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
is the call, and the orientation is `AlignedDimensionOrientation` every time. This figure has no
axis-aligned line in it — the shaft axes sit at the Shaft Angle to each other and the whole lattice
tilts with the target plane — so a horizontal or vertical orientation would dimension the line's
projection onto a sketch axis instead of its length. That applies to the PPD/2 and DPD/2 drops to
Apex 2, the two `Module * 1.25` dedendum lines, the Tooth Spacing dimension on the K′ and L′ lines,
and the Toe Radius dimension on the two front faces N→A′ and P→B′. The offset dimensions are a
different call, `sketch.sketchDimensions.addOffsetDimension(line, entityTwo, textPoint)`, which
takes no orientation.

**The driven lengths are NOT dimensioned** (`[BEVEL-F-DRIVEN-DIMS]`, `[PB-NO-OVERCONSTRAIN]`): the
along-shaft lengths Apex→A and Apex→B and the module-length extensions are determined by the
closing, perpendicular and collinear constraints. The "do NOT add a dimensional constraint" notes
below are as load-bearing as the dimensions that ARE added.

### The figure, in build order

**Project the anchor.** `sketch.project(self._anchorCenterPoint)` — the Anchor Sketch's centre
`SketchPoint` stashed in S4, NOT the raw user-selected centre. Both happen to be coincident, but
projecting the anchor-sketch point keeps the chain inside the Design component; projecting the raw
external point is a cross-component reference and can resolve inconsistently. Project the Anchor
Line the same way, since the Gear Profiles plane contains it. Call the projected centre `c` and the
projected anchor line's 2-D unit direction `d`.

**The grow direction.** The in-plane perpendicular is `perp = (-d.y, d.x)`, and **its sign is chosen
by the target-plane normal, not by the sketch's local +Y** (`[BEVEL-F-GROW-SIDE]`). Read the normal
as `targetPlane.geometry.normal` for BOTH selection kinds — a `BRepFace`'s `geometry` and a
`ConstructionPlane`'s `geometry` are each an `adsk.core.Plane` carrying `.normal` — and pick the
sign of `perp` so it points toward that normal. A sketch-local rule like `perp.y >= 0` is
deterministic but not tied to a physical side, so the gear would grow inconsistently. This one-bit
direction comparison is the **only** permitted world use in §2.

**Centre → Apex.** Draw a construction line from `c` to the Apex, placing the Apex in sketch-local
2-D coordinates at

    Apex = c + perp * (R * cos γ_g + <resolved Driving Gear Base Height>)

(`[BEVEL-F-APEX-LOCAL]`). Seed it at that distance and **not** at `Driving Gear Pitch Diameter`: the
constraint net closes this line at `R·cos γ_g` above point I plus the resolved driving base height,
so for the default 31/31 pair at Shaft Angle 90° a `DPD` seed sits 11.6 mm past where the solve puts
it — 31 mm seeded against 19.375 mm solved. Fusion converges from the far seed, but a seed that
disagrees with its own closure by that margin is a seed waiting to pick the wrong branch
(`[PB-SEED-NEAR]`). The apex POSITION is sketch-local; do not compute it from a world round-trip,
which is what caused the XY collapse. Build the line by the coincident style — raw `Point3D` for
both endpoints, then exactly one
`addCoincident(centerToApex.startSketchPoint, projectedCenter)` — and add
`sketch.geometricConstraints.addPerpendicular(centerToApex, projectedAnchorLine)`. Add **no** length
constraint on this line.

**Driving Gear Shaft Axis, Apex → B.** A construction line from the Apex pointing back toward the
anchor line, in the `-perp` direction. **Seed its far end at `apex - perp * (R * cos γ_g)`, which is
`c + perp * (<resolved Driving Gear Base Height>)`** — measured from the apex, not from `c`. A seed
of the form `c - perp * <length>` puts B on the far side of the projected centre from the apex, the
wrong side of the figure entirely; the closure at Apex 2 drives `|Apex→B|` to `R·cos γ_g`, so B
solves to exactly one base height above `c`. Pin the start with
`addCoincident(drivingShaft.startSketchPoint, apexPoint)` and apply
`sketch.geometricConstraints.addParallel(drivingShaft, centerToApex)`. Do **NOT** use
`addVertical`: it forces the line to the sketch's world-vertical, which is wrong on a tilted target
plane and over-constrains or mis-orients the figure. The shaft axis must be parallel to the in-plane
apex direction, expressed as a parallel to the centre→Apex line, never an absolute Horizontal or
Vertical. The end of this line is **point B**. Do not dimension its length.

<!-- check-step-calls: ignore addVertical -->

`addVertical` is named only to forbid it, which is why it is exempted.

**Pinion Gear Shaft Axis, Apex → A.** The Driving Gear Shaft Axis direction rotated about the Apex by
the Shaft Angle. Rotating has two senses, one to each side of the driving shaft, and they place point
A on opposite sides; choosing wrong mirrors the whole gear onto the wrong side of the target plane.
**Select the sense this way: form BOTH candidate point-A positions — the driving-shaft direction
rotated about the apex by +Shaft Angle and by −Shaft Angle, each taken out to `R * cos γ_p` — and
keep the candidate whose endpoint has the greater X coordinate in the Gear Profiles sketch.**
Compare the two candidates' X and take the larger; do not rotate one fixed sense and flip it only
when its X comes out negative, because when both candidates have a positive X that shortcut keeps
the wrong one. Call the chosen unit Apex→A direction `pinionDir`. Pin the start with
`addCoincident(pinionShaft.startSketchPoint, apexPoint)`. Do not dimension the length. The end of
this line is **point A**.

**The Shaft Angle dimension.**
`sketch.sketchDimensions.addAngularDimension(pinionShaft, drivingShaft, textPoint)` with the value
set to the Shaft Angle. **Place its text point inside the Σ wedge so it measures Σ and not its
supplement 180−Σ** (`[PB-ANGULAR-DIM]` — the dimension measures the angle on the side the text point
lies): use the interior bisector of the two shaft directions,
`apex + normalize(pinionDir + drivingDir) * (PPD / 4)`, where `drivingDir` is the unit Apex→B
direction. The angular dimension fixes the angle's magnitude only; it does not by itself pin which
side the pinion lies on, which is held by the seed above together with the Apex 2 closure below.

**Naming convention used from here on.** The perpendicular drop line from A to the point that
becomes Apex 2 is what **"A→Apex2"** always refers to. It is NOT the Apex→A shaft axis: the two
share point A but are different lines, one the PPD/2 perpendicular drop and the other the shaft
axis. The same holds for **"B→Apex2"**, the DPD/2 drop, against the Apex→B shaft axis.

**A → Apex 2.** From A, a construction line perpendicular to the Pinion Gear Shaft Axis, drawn
toward the side Apex 2 will lie on. **Apex 2 sits in the interior wedge BETWEEN the two shaft axes,
so this drop must point toward the OTHER, driving shaft, toward point B** — pick the perpendicular
sense by the sign of its dot product with the A→B direction, not against a generic "toward the
anchor line" reference. Apply
`addCoincident(dropA.startSketchPoint, pointA)`,
`addPerpendicular(dropA, pinionShaft)`, and an aligned distance dimension of length
**Pinion Gear Pitch Diameter / 2**, which is the pinion's pitch radius at the heel and the
perpendicular distance from Apex 2 to the Pinion Gear Shaft Axis for any Shaft Angle.

**B → Apex 2.** From B, a construction line perpendicular to the Driving Gear Shaft Axis, drawn
toward point A — pick the perpendicular sense by the sign of its dot product with the **B→A**
direction. **Do NOT choose this sense by a "toward the anchor line" reference**, i.e. by the −perp
grow direction: the Driving Gear Shaft Axis is itself parallel to that direction, so the
perpendicular's dot with it is about zero, a degenerate test that silently selects an arbitrary and
usually wrong side. This is the critical failure. If this drop seeds Apex 2 on the wrong side of the
driving shaft while the pinion's drop seeds it on the correct side, the coincidence that closes the
two at Apex 2 makes the solver **flip the entire frame to the mirror solution**: point A jumps to
the opposite side, the pinion dedendum C collapses onto the driving dedendum D, the pinion inverts
with its toe outside its heel, the revolved frustum is degenerate, and the conical end cut finds no
cone face at the toe midpoint and reports `face dist = inf`. Both drops must aim at the same
interior-wedge point. Apply `addCoincident(dropB.startSketchPoint, pointB)`,
`addPerpendicular(dropB, drivingShaft)`, and an aligned distance dimension of length
**Driving Gear Pitch Diameter / 2**.

**Close them.** `addCoincident(dropA.endSketchPoint, dropB.endSketchPoint)`. That point is **Apex 2**.
At Shaft Angle 90° the four points Apex, A, Apex 2 and B form a rectangle; for other shaft angles the
figure is a non-rectangular quadrilateral and the lengths of Apex→A and Apex→B adjust so the two
drops of length PPD/2 and DPD/2 coincide.

**Seed the along-shaft lengths from the closed-form cone geometry** so the solver converges on the
right branch for any Shaft Angle: `|Apex→A| = R · cos γ_p` and `|Apex→B| = R · cos γ_g`. Both
cosines are positive for every Shaft Angle the range check admits, which is what the Maximum Shaft
Angle guarantees. Seeding A or B merely by a pitch diameter is wrong for Σ ≠ 90° and can send the
solver to the wrong branch. These are seed coordinates only; the lengths stay undimensioned.

The quadrilateral deliberately lies well above the anchor line: the Apex's offset of
`R·cos γ_g` plus the resolved Driving Gear Base Height keeps the whole figure above that line across
the supported Shaft Angle range.

**The Pitch Line, Apex → Apex 2.** A construction line with both ends pinned by coincidents:
`addCoincident(pitchLine.startSketchPoint, apexPoint)` and
`addCoincident(pitchLine.endSketchPoint, apex2Point)`.

**The two dedendum lines.** From Apex 2, one construction line to either side, each perpendicular to
the Pitch Line and each carrying an aligned distance dimension of length **`Module * 1.25`**. Pin
each start with `addCoincident(<line>.startSketchPoint, apex2Point)` and apply
`addPerpendicular(<line>, pitchLine)`. The line drawn **toward** the anchor line is the **Driving
Gear Dedendum**, whose end point is **D**; the one drawn **away** from the anchor line is the
**Pinion Gear Dedendum**, whose end point is **C**.

**The two root axes.** A construction line from the Apex to D and another from the Apex to C, each
with coincidents on both ends. These are the **Driving Root Axis** and the **Pinion Root Axis**.

**A → E and C → E.** From point A, a construction line collinear with Apex→A, extended by a length
equal to Module as a seed but with **no** dimensional constraint. Apply
`addCoincident(lineAE.startSketchPoint, pointA)` and
`sketch.geometricConstraints.addCollinear(lineAE, pinionShaft)` — the collinear names the line the
new line's start point actually sits on (`[PB-COLLINEAR-CHAIN]`, `[BEVEL-F-COLLINEAR-CHAIN]`). The
end of the new line is **point E**. Then draw a construction line from C to E, pinning both ends
with coincidents, and apply `addPerpendicular(lineAE, lineCE)`. E is therefore the foot of the
perpendicular from C onto the Pinion Gear Shaft Axis.

**B → F and D → F.** The driving twin: from B, a construction line collinear with Apex→B, seeded a
module long and undimensioned, with `addCollinear(lineBF, drivingShaft)`; its end is **point F**.
Then a construction line from D to F with coincidents on both ends and
`addPerpendicular(lineBF, lineDF)`.

**E → G.** From point E, a construction line collinear with **line A→E** — the collinear names A→E,
**never the Apex→A shaft axis further up the chain**, even though both describe the same infinite
line (`[BEVEL-F-COLLINEAR-CHAIN]`). Measured on this lattice, `addCollinear(E→G, Apex→A)` raised
`RuntimeError: 3 : failed to create offset: VCS_SKETCH_OVER_CONSTRAINTS` at the second such call,
the first having been absorbed, while `addCollinear(E→G, A→E)` builds. Seed it a module long with no
dimensional constraint, pin its start with a coincident to E, and call its end **point G**.

**C → H.** From point C, a construction line seeded a module long with no dimensional constraint,
its start pinned by a coincident to C, and `addCollinear(lineCH, pinionDedendum)` — collinear with
**line Apex2→C**, the Pinion Dedendum line C is the endpoint of. Its end is **point H**.

**G → H.** Connect G and H with a line, pinning both ends with coincidents, and
**`addPerpendicular(lineEG, lineGH)`**. This perpendicular is required: `addOffsetDimension` in
Fusion is a distance dimension whose documentation requires the second entity to be a line parallel
to the first, and it controls only the perpendicular distance, so the parallelism has to exist
before the pinion base-height offset below can be applied at all. E→G runs along the pinion shaft,
so making G→H perpendicular to it makes G→H parallel to the A→Apex2 drop, which is perpendicular to
that same shaft. Perpendicular plus offset is two equations for two freedoms and nothing is
redundant.

**F → I, D → J and I → J.** The driving twins, built the same way: F→I collinear with **line B→F**,
its end **point I**; D→J collinear with **line Apex2→D**, its end **point J**; then I→J connected
with coincidents on both ends and `addPerpendicular(lineFI, lineIJ)`.

**The driving base-height offset.**
`sketch.sketchDimensions.addOffsetDimension(dropB, lineIJ, textPoint).parameter.value = <resolved Driving Gear Base Height>`
— between the **B→Apex2 perpendicular drop**, the DPD/2 drop, and I→J; NOT the Apex→B shaft axis.
I→J is already parallel to the drop by construction, since I→J is perpendicular to F→I which runs
along the driving shaft, so add **no** extra parallel constraint: a redundant `addParallel` over
lines the existing constraints already fix throws `VCS_SKETCH_OVER_CONSTRAINTS` (`[PB-OFFSET-DIM]`).
The value is the base height **after** the driving gear's own Minimum and Maximum were applied in
S2, because the offset set here is what drives the heel edge D→J toward the shaft axis.

**The pinion base-height offset.**
`addOffsetDimension(dropA, lineGH, textPoint).parameter.value = <resolved Pinion Gear Base Height>`
— between the **A→Apex2 perpendicular drop**, the PPD/2 drop, and G→H; again already parallel by
construction, so no parallel constraint. The value is the pinion base height S2 resolved: the user's
value when non-zero, otherwise the RESOLVED driving base height times
`Pinion Gear Teeth Number / Driving Gear Teeth Number`, with the pinion's own Minimum and Maximum
then applied.

**Pin the figure.** `addCoincident(pointI, projectedCenter)` — point I lands on the projected centre.
This is what fixes the whole figure's height above the anchor line.

**K, the pinion back-cone centre.** Draw a construction line away from the Apex, starting at point G
and extending along Apex→A; call its end **K**. Pin its start with a coincident to G, and **pin K
with two point-on-line coincidents** —
`addCoincident(pointK, pinionShaft)` and `addCoincident(pointK, pinionDedendum)`, the latter being
the Pinion Dedendum line Apex2→C extended — rather than a collinear on the connecting lines. By the
time K is added, G and C are already fixed, so a collinear here over-constrains the sketch and
Fusion errors; the two point-on-line coincidents locate K exactly, at the intersection of the two
lines, without over-constraining. Then draw a construction line from C to K for reference, with
coincidents on both ends.

**L, the driving back-cone centre.** The same, substituting I for G, B for A and D for C: a
construction line from I along Apex→B whose end is **L**, pinned with
`addCoincident(pointL, drivingShaft)` and `addCoincident(pointL, drivingDedendum)`, no collinear;
then a construction line from D to L for reference.

**Tooth-centre points K′ and L′, the Tooth Spacing offset.** The §3 spur tooth is centred not at K
but at **K′**, obtained by shifting K outward along the dedendum line by **Tooth Spacing**, away
from the lower corner C.

- **When Tooth Spacing is 0, the default, build nothing here**: set K′ ≡ K and reuse the C→K
  reference line. A zero-length dimensioned line would be degenerate, and one segment gets one line
  (`[BEVEL-F-LINE-ONCE]`).
- **When Tooth Spacing > 0**: draw a construction line starting at K with its far end seeded on the
  far side of K from C along the dedendum direction; pin its start with `addCoincident` to K and its
  far end with `addCoincident(pointKPrime, pinionDedendum)` to keep K′ on the Pinion Dedendum line
  Apex2→C extended; then add an aligned distance dimension on this line equal to **Tooth Spacing**.
  Do not use a collinear, for the same over-constraint reason as K. The far end is **K′**. Finally
  draw the tooth-centre reference line **C → K′**, from the lower corner C to K′, for §3 to use in
  place of C→K.

Build this **here, inside the Gear Profiles sketch, before the end-of-step gate**, so the gate
covers it. Only the tooth's centre moves: the virtual tooth number and the drawn tooth size are
unchanged.

Build **L′** exactly as K′, substituting L for K, D for C and the Driving Dedendum line Apex2→D for
the pinion's; its reference line for §3 is **D → L′**. Same single Tooth Spacing value, same gate,
same reuse-the-existing-line rule at 0.

### Resolve the Maximum Face Width, then Face Width and the toe end

At this point A, B, C, D, H and J exist **and are solved**, so resolve the **Maximum Face Width**
and apply it before the toe lines below.

**Compute both distances from the points' SOLVED sketch geometry** — `pointA.geometry`,
`pointB.geometry`, `pointC.geometry`, `pointD.geometry`, `pointH.geometry`, `pointJ.geometry` — and
NOT from the pre-solve seed coordinates (`[PB-SOLVED-GEOMETRY]`). By now the constraint network has
located all six, so `.geometry` is exact; seeds diverge substantially for asymmetric tooth counts
such as Driving 17 / Pinion 31, or for non-90° shaft angles, making a seed-based bound too loose on
the binding side, so the toe still crosses the axis and the cap is defeated.

    Maximum Face Width = 0.95 * min( perpendicular distance from A to the line through C and H,
                                     perpendicular distance from B to the line through D and J )

The first line is the Pinion Gear Dedendum line Apex2→C extended, the second the Driving Gear
Dedendum line Apex2→D extended. The pinion side is normally the binding one, since its smaller pitch
radius gives the smaller distance, but compute both and take the minimum so the bound holds for any
Shaft Angle. The `0.95` keeps the inner toe corner clearly off the shaft axis, since a near-coincident
corner degenerates the toe edge even before it strictly crosses. At Shaft Angle 90° this limit equals
`Pinion Gear Pitch Diameter**2 / (2 * Cone Distance)`, so the naive `Cone Distance / 6` default
exceeds it — and the gear fails to generate — for any gear ratio above roughly √2, for example
Driving 31 / Pinion 17. Skipping this, or computing it from seeds, makes the toe line push the inner
toe corner across the shaft axis for asymmetric tooth counts, and the gear-body revolve fails with
`ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`).

**Face Width.** If the user specified a value, reject it with a message stating the maximum when it
exceeds the Maximum Face Width; otherwise use `min(Cone Distance / 6, Maximum Face Width)`. Stash
the result on `self._faceWidthResolved_cm`. The default `Cone Distance / 6` is `R / 3`, the
conventional face-width limit, **only at Shaft Angle 90°**: below 90° it is conservative and below
`R / 3`, above 90° it exceeds `R / 3` and the cap is what actually holds it. That is deliberate — it
keeps the default independent of Shaft Angle — and it is the cap, not the default, that guarantees a
buildable profile.

**Root Length**, the resolved `|Ded→Toe|`, the segment C→M on the pinion and D→O on the driving
gear. At Toe Extension 0 it is the resolved Face Width re-measured along the root element rather
than perpendicular to the pitch line, which is longer by the dedendum angle's cosine:

    |Apex->Ded| = sqrt(R**2 + (1.25 * Module)**2)
    Root Length at Toe Extension 0 = Face Width * |Apex->Ded| / R

At a positive Toe Extension it is that value plus the extension's share of the window below. Face
Width still resolves exactly as above and still carries its cap: the Toe Extension ADDS to what Face
Width resolved, it does not replace it.

**Toe Radius, per gear.** The input when non-zero; **0 means auto-calculate** and takes that gear's
own inner toe corner radius at Toe Extension 0,

    Toe Radius = this gear's Pitch Radius - Face Width / sin γ

which is the value that makes Toe Extension 0 today's profile exactly. It is the perpendicular
distance from the shaft axis at which the inner toe corner N, respectively P, rides, and with it the
radius of the flat front face the revolve produces. A user value must be **strictly below** that
gear's **Toe Radius Ceiling**,

    Toe Radius Ceiling = (this gear's Pitch Radius - 1.25 * Module * cos γ) * (1 - Face Width / R)

which is that gear's OUTER toe corner radius at Toe Extension 0. Reject a value at or above it with
a message naming the ceiling; at or above it the point X below falls behind the toe corner and the
Toe Extension has nowhere to go.

**Toe Limit**, per gear, is `|Ded→X|` where X is the point on the root element Apex→Ded at this
gear's Toe Radius:

    γ_root = γ - atan(1.25 * Module / R)
    Toe Limit = sqrt(R**2 + (1.25 * Module)**2) - Toe Radius / sin(γ_root)

X is where the toe end is heading: as the Toe Extension rises the toe corner climbs Apex→Ded toward
X while N or P slides in along the toe-radius line to meet it, and at X the toe face has closed to
nothing.

**The resolved Root Length.**

    Root Length = <Root Length at Toe Extension 0>
                  + (Toe Extension / 100) * 0.99
                    * ( min(Pinion Toe Limit, Driving Toe Limit) - <Root Length at Toe Extension 0> )

**Toe Extension 100 stops at 0.99 of the way to the smaller of the two gears' Toe Limits, not at the
Toe Limit itself.** The smaller limit wins because the pair shares one root length and the other gear
simply stops short of its own X. The `0.99` is there because AT the limit the toe face has zero
length, so the revolved gear body carries no cone at its toe end and the conical end cut, whose toe
cut must split or the build fails, has no `ConeSurfaceType` face to find. The last percent is worth
well under a tenth of a millimetre of root length on every case in the proof's table, so the reach
given up is nil and the failure avoided is total. Do not drop this factor.

**A defaulted Toe Radius can leave no room at all, and that is a real configuration rather than a
defect.** On a driving gear with a large pitch cone angle the inner toe corner already sits at a
LARGER radius than the outer one — the toe dish leans toward the heel rather than away from it — so
X falls behind the toe corner and the Toe Limit comes out below the Toe Extension 0 root length.
Measured over gear ratio against Shaft Angle it is a diagonal band that crosses 90° for every ratio
from about 2.75 up, and Module does not move its boundary. **Reject a Toe Extension above 0 on such
a pair**, with a message naming the gear and the Toe Radius Ceiling it needs to come below. Toe
Extension 0 still resolves, so the gear itself stays buildable exactly as before. Do **not** silently
substitute a smaller Toe Radius: that would change the toe end of a gear whose inputs asked for no
change.

Toe Extension is a user-specified percentage, default 0, valid range **[0, 100]**, a single value
applied to **both** gears because they share one face and must mesh, read on the **driving** gear
with the pinion then built to that same root length. 0 reproduces today's toe end exactly, which is
what makes every gear built before this input existed come out unchanged.

### The pinion toe line and front face

**M → N.** Seed it near its solved position (`[PB-SEED-NEAR]`): seed M at roughly the **midpoint of
Apex→C**, and seed N by sliding from that M-seed along the C→H direction far enough to roughly reach
the A→Apex2 drop, for example by the distance from the M-seed to A. Do NOT seed M and N just
`Face Width` away from C and H — that starts N near H, far from its constraint target, and the solve
fails to converge. Then apply exactly:

- `addCoincident(pointM, pinionRootAxis)` — M lies on the Apex→C root axis;
- `addParallel(lineMN, lineCH)` — the toe line is parallel to C→H;
- `addOffsetDimension(lineCH, lineMN, textPoint).parameter.value = <Root Length * R / |Apex->C|>` —
  the Root Length re-measured perpendicular to the pitch line, since an offset dimension controls a
  perpendicular distance. At Toe Extension 0 that value is exactly the resolved Face Width, which is
  what this dimension has always been. Place the `textPoint` in the gap between C→H and M→N on the
  Apex side, for example the midpoint of the M-seed and point C, so the dimension reads cleanly
  (`[PB-OFFSET-DIM]`). The toe's side relative to the heel follows from the §2 frame being built
  correctly, in particular from the Apex 2 drops aiming at the interior wedge; it is not controlled
  by this text point.

The beginning of the line is **M** and the end is **N**. Then draw a line from M to C, coincidents on
both ends.

**The front face N → A′, which is what holds N.** **N is NOT pinned to the A→Apex2 drop.** An earlier
scheme pinned it there, which fixed its station at A's and made the Maximum Face Width the value at
which N reached A. It now rides the **Pinion Gear Toe Radius** instead, and the line that holds it
there is the gear's front face:

- draw a line from N to a new point **A′**, seeding A′ at N's station on the shaft axis;
- `addCoincident(pointAPrime, pinionShaft)` — A′ lies on the **Apex→A shaft axis**. A′ is the only
  toe-end point that touches that axis, and it is a foot, not a corner;
- `addPerpendicular(lineNA, pinionShaft)` — the front face stands square to the shaft, so the
  revolve sweeps it into a flat annulus;
- an aligned distance dimension on the whole line N→A′ equal to the **resolved Pinion Gear Toe
  Radius**.

**Pinning N itself to the Apex→A shaft axis remains forbidden** — that would put N ON the axis of
revolution, and the later conical split fails with `ASM_API_FAILED` for asymmetric tooth counts even
though the symmetric 45° case happens to survive. A′ sits on the axis; N never does, because the Toe
Radius is strictly positive. Those three rows plus the offset above and the coincident pinning M to
the Pinion Root Axis fully constrain M, N and A′: six freedoms, six constraints.

**A′ replaces A as the hexagon's first vertex.** At Toe Extension 0 with a defaulted Toe Radius the
two coincide exactly, so nothing moves; a positive Toe Extension walks A′ along the shaft axis toward
the Apex and the shaft edge grows by that much.

Draw the line **A′ → G**, the hexagon's shaft-axis edge, with coincidents on both ends. It starts at
the front face's foot A′, not at A.

### The driving toe line and front face

**O → P**, the mirror of M→N on the driving side. Seed it the same way — O near the midpoint of
Apex→D, P slid along D→J toward the driving toe radius — then apply
`addCoincident(pointO, drivingRootAxis)`, `addParallel(lineOP, lineDJ)` and
`addOffsetDimension(lineDJ, lineOP, textPoint).parameter.value = <Root Length * R / |Apex->D|>`,
with the `textPoint` in the gap on the Apex side of D→J. The beginning is **O** and the end is **P**.
Draw a line from O to D with coincidents on both ends.

Build the driving front face **P → B′** exactly as the pinion's N→A′, substituting B for A, P for N
and the **Driving Gear Toe Radius** for the pinion's: the line P→B′,
`addCoincident(pointBPrime, drivingShaft)`, `addPerpendicular(linePB, drivingShaft)` and an aligned
distance dimension on P→B′. The same prohibition applies — P is never pinned to the Apex→B shaft
axis, only B′ touches it. Draw the line **B′ → I**.

### Gate

End the step with `if not sketch.isFullyConstrained: raise` naming `Gear Profiles`
(`[PB-FULL-CONSTRAINT]`, `[BEVEL-F-FULL-CONSTRAINT]`). Do NOT reach full constraint by dimensioning
the driven §2 lines.

Carry the per-gear anchors forward in **plain dicts**, `pinionCtx` and `drivingCtx`, holding this
gear's label, teeth, pitch diameter, γ, tooth-centre point and reference line, hexagon vertices,
shaft-edge point pair, toe and heel edges, toe and heel cone points, root axis, bore diameter and
mesh angle. Stash the shared anchors on `self`: `self._gearProfilesPlane`, `self._apexSketchPoint`,
`self._gpSketch` and `self._apex2d`.

`stepGearProfiles` realises this step.

### What the proof establishes

The proof builds the whole lattice from these constraints and gates it on the sketch engine's own
verdict: DOF 0, nothing conflicting, nothing redundant, valid profiles, a system that is not
near-singular, and no discrete ambiguity. It then reads the solve back and checks it against the
closed form — every named point's position, the two solved cone angles to nine decimals, the Pitch
Cone Distance, the two pitch radii as perpendicular distances from Apex 2 to the shafts, the two
base-height offsets and the window each has to sit in, the Minimum Teeth floor, the Maximum Face
Width computed from the solved points, the Root Length, the Toe Radius the front face holds the
inner corner at, and the virtual pitch radius `|Apex2→K|` the §3 tooth is drawn from.

Two constraints are carried differently there, and both are written at their call sites in
`proof/bevelgear/sketches_test.go`.

The G→H and I→J perpendiculars are **omitted** in the proof. The engine's offset constraint emits
two residual rows, holding both endpoints of the target line at the same signed perpendicular
distance from the source, so it carries the parallelism itself; adding the perpendicular is a third
row for the same two freedoms, and measured, the lattice comes back overconstrained at DOF 0 with
the two base-height offsets named as the redundant pair. Fusion's `addOffsetDimension` needs the
parallelism to already exist, which is why the Fusion build keeps the perpendiculars.

`addCoincident(point I, the projected centre)` is carried in the proof as the **single row that is
not already implied**: the driving shaft chain B→F→I is collinear with centre→Apex by construction,
so I already lies on the line through the centre and the lateral row of the coincidence is
dependent. The proof states the independent row instead, as the centre lying on the I→J edge, which
given the rest of the net holds exactly when I is at the centre. Fusion absorbs the dependent row;
the engine reports it.

And every side-choosing `addPerpendicular` and `addParallel` is carried there as a **signed angle of
the same arity**. Fusion's versions are unsigned and take the side from the seed, and §2 chooses
every one of those sides deliberately — the grow side, the two Apex 2 drops into the interior wedge,
the dedendum pair's toward-and-away split, the front face's side of the shaft. Left unsigned, the
engine reaches DOF 0 and still admits the mirrored answer, and measured, it found exactly the two
failures §2 warns about: the driving dedendum D landing on top of the pinion's C, and the inner toe
corner sliding through the shaft axis to its mirror station. The two perpendiculars that choose no
side, A→E against C→E and B→F against D→F, stay perpendiculars.

Two cases in the table are **declared refusals** rather than passes, recorded rather than avoided.
This net cannot reach the spec's documented 30-degree Shaft Angle floor: measured, the default pair
reads conditioning 2.831e-05 against the engine's 4e-05 trust floor and first clears at 35 degrees
(4.192e-05), which matches one of the two nets the spec reports at 2.83e-05 to three digits. And
this net cannot reach a 31/17 pair at 120 degrees, where the driving pitch cone angle has climbed to
86.8 degrees on its way to the 123.26-degree Maximum Shaft Angle: 110 degrees reads 4.274e-05 and
clears, 115 reads 1.180e-05 and 120 reads 7.529e-07, both refused. Both are properties of this
lattice, not of the range the spec states, and neither narrows what the module accepts.

**From:** `spec/bevelgear/instructions.md` L25-144, L389-410, L471-577, L689-750;
`spec/bevelgear/fusion.md` L19-68, L69-116, L117-152;
`.claude/skills/generate-gear/PLAYBOOK.md` L230-243, L432-441, L449-484, L492-499, L500-507,
L582-596, L597-604, L605-625, L626-647, L708-714.

## S7 `[PROSE]` Virtual tooth number and the `{gearLabel} Plane`

Run this and the next two steps **once per gear** — pinion first, then driving — with this gear's
parameters. The pinion uses tooth-centre **K′**, reference line **C→K′** and pitch-cone half-angle
**γ_p**; the driving gear uses **L′**, **D→L′** and **γ_g = Σ − γ_p**. Throughout §3 the tooth
centre is the §2 tooth-centre point K′ or L′ and the centre reference line is C→K′ or D→L′, which
equal K, L, C→K and D→L exactly when Tooth Spacing is 0.

**The virtual tooth number comes from the closed form, not from measuring Apex2→K′.**

    virtual pitch radius = (this gear's Pitch Diameter / 2) / cos(γ)
    virtual tooth number = floor(2 * virtual pitch radius / Module)      # an int

**Pin the cm-to-mm conversion**: the stashed pitch diameters are internal **cm** while Module is the
raw **mm** value, so compute `virtualPitchRadius_mm = (pitchDia_cm * 10 / 2) / cos(γ)` — the `* 10`
converts cm to mm — and then `virtualTeeth = floor(2 * virtualPitchRadius_mm / Module)`. Skipping the
×10 makes the virtual tooth count about ten times off. The virtual tooth number is independent of
Tooth Spacing: the spacing offset moves only the centre, not the tooth size.

**The tooth plane.** Create a plane that includes the tooth-centre reference line and stands
perpendicular to the Gear Profiles sketch plane, named `{gearLabel} Plane` — `Pinion Plane` or
`Driving Plane`. Use the framework helper
`plane_by_angle(designComponent, <tooth-centre reference line>, self._gearProfilesPlane, 90)`,
which is the `setByAngle` construction-plane pattern, and pass the relevant **sketch line directly**;
never wrap it in `Path.create` first (`[PB-CONSTRUCTION-PLANES]`).

<!-- check-step-calls: ignore Path.create -->

`Path.create` is named only to forbid it, which is why it is exempted.

Keep the plane on this gear's dict; it is the `parentToothPlane` the spiral build's slice step reads,
and S29 hides it.

This step creates a number and a construction plane, so no proof function realises it. The virtual
tooth number itself is asserted in S8, where the tooth it sizes is drawn.

**From:** `spec/bevelgear/instructions.md` L317-388, L578-594;
`.claude/skills/generate-gear/PLAYBOOK.md` L766-777.

## S8 `[GO]` `{gearLabel} Tooth` sketch — the borrowed virtual spur tooth

<!-- proof-run: proofkit.RunParallel(toothCases, stepToothSketch) -->

Create the sketch on the `{gearLabel} Plane` with `designComponent.sketches.add(<that plane>)` and
name it `{gearLabel} Tooth` — `Pinion Tooth` or `Driving Tooth`. Draw the spur gear tooth profile
into it with Module and the virtual tooth number from S7, centred on the tooth-centre point.

The tooth is **borrowed** from the spur family — `from .spurgear import
SpurGearInvoluteToothDesignGenerator` — and driven through the framework's virtual-spur proxy —
`from .spurproxy import VirtualSpurProxy`. Bevel defines **no** local proxy or value-wrapper class.
Used once per gear, inside this step:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
drawer = SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))
```

<!-- check-step-calls: ignore drawCircles drawTooth getParameter -->

`drawCircles` and `drawTooth` are named below as what `draw()` runs internally, not as calls this
module makes. `getParameter` is the other half of that: it is the call the BORROWED spur generator
makes on whatever `parent` it is handed, and bevel's side of it is to supply a `VirtualSpurProxy`
that answers it. All three are the borrowed generator's calls, not this module's, which is why they
are exempted.

The borrowed generator's surface is fixed: the constructor is `(sketch, parent, angle=0)`, and
`draw(anchorPoint, angle=0)` runs `drawCircles()` then `drawTooth(angle)` then an anchor projection,
reading its parameters through `parent.getParameter(name).value`. `VirtualSpurProxy` is that
`parent`: it precomputes, in internal cm, exactly the keys the drawer reads — `Module`,
`ToothNumber`, `PressureAngle`, the Pitch, Base, Root and Tip circle diameters and radii, and
`InvoluteSteps` — each wrapped in a `.value` carrier. Its defaults match bevel: pressure angle 20
degrees, which is **not** a bevel dialog input, and 15 involute steps. Construct it with the raw-mm
Module and the §3 virtual tooth number, exactly as written above.

**The 180-degree rotation is delivered through the `draw()` angle argument**, not by a post-hoc Move
or sketch rotation. The generator rotates the whole tooth by that angle, which relies on spur's
radial flank-to-root pinning so the connecting lines rotate with the tooth.

**Read `proxy._lastToothEmbedded` back after `draw()` returns.** During `draw()` the spur generator
decides whether the tooth is *embedded* — tip, root and flanks meeting with no connecting lines —
and records it by writing `self.parent._lastToothEmbedded = <bool>`; the framework proxy
pre-initialises the slot to absorb that write. This flag is **not optional bookkeeping**. It is the
deterministic selector for the tooth loop's line count in S12: `wantLines = 0 if embedded else 2`.
Stash it alongside the tooth sketch and plane on this gear's dict.

**Do NOT hard-gate this sketch.** Log it if `not toothSketch.isFullyConstrained`, never raise. The
two tooth-profile sketches are exempt from the full-constraint gate, and the reason is exactly one
thing: the borrowed `drawCircles` labels each of the four circles with along-path sketch text
(`[PB-SKETCH-TEXT]`), and sketch text holds a degree of freedom (`[PB-TEXT-HOLDS-DOF]`), so a tooth
sketch whose geometry is completely determined still reads `False` purely because it is labelled.
Bevel's own four sketches carry no text, which is why they gate normally. The exemption covers the
labels and nothing else; it is never licence for loose geometry. Measured, the reading is not even
stable between runs — the same labelled bevel tooth sketches read `False` in one Fusion run on
2026-09-12 and `True` in the two after it, with byte-identical counts — which is the other reason to
log rather than raise.

`stepToothSketch` realises this step.

### What the proof establishes

The proof checks the three things bevel supplies to the borrowed generator and reads back from it:
the virtual tooth number against the closed form, the 180-degree draw angle, and the curve counts the
S12 profile search keys on — 2 NURBS, 2 arcs, and 0 lines when the tooth is embedded or 2 when it is
not. It also checks that the tooth reaches the tip radius and, when it is not embedded, seats on the
root radius.

The tooth's own constraint scheme is `proof/spurgear/sketches_test.go`'s subject rather than this
one's, so the proof draws the tooth as reference geometry: from bevel's side it arrives already
placed, and reference geometry is what the engine calls geometry that is externally locked by
design. That substitution has a second, mechanical reason recorded beside it in
`proof/bevelgear/sketches_test.go`: an ordinary arc carries an internal radius-consistency row that a
fully placed tooth makes dependent, so the sketch comes back at DOF 0 with one redundant constraint
per arc and the gate refuses it, where a reference arc carries no such row. The arcs are still arcs
and the region is still the tooth.

Sketch text has no counterpart in the engine at all, so the labelled-sketch behaviour above is not
reproduced; what the proof shows is that the geometry reaches DOF 0 on its own, which is the half a
bench can see.

**From:** `spec/bevelgear/instructions.md` L389-410, L423-452, L578-594;
`spec/bevelgear/fusion.md` L19-68; `.claude/skills/generate-gear/PLAYBOOK.md` L151-195, L508-524,
L663-683.

## S9 `[PROSE]` `{gearLabel} Tooth Axis`

Create a construction axis through the tooth-centre point, normal to the plane the tooth profile was
drawn on, named `{gearLabel} Tooth Axis`:

```
helperInput = designComponent.constructionPlanes.createInput()
helperInput.setByDistanceOnPath(<tooth-centre reference line>, adsk.core.ValueInput.createByReal(1.0))
helperPlane = designComponent.constructionPlanes.add(helperInput)

axisInput = designComponent.constructionAxes.createInput()
axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)
toothAxis = designComponent.constructionAxes.add(axisInput)
toothAxis.name = '{gearLabel} Tooth Axis'
```

`setByTwoPlanes` takes the intersection of two planes (`[PB-CONSTRUCTION-AXES]`), and it is the way
in here because `setByPerpendicularAtPoint` would need a `BRepFace` this step does not have. The two
planes are the **Gear Profiles plane** and a helper plane built
`setByDistanceOnPath(<tooth-centre reference line>, 1.0)`, which is perpendicular to that line at its
far end, the tooth-centre point; their intersection is the line through the tooth centre normal to
the tooth plane. Pass the sketch line directly to `setByDistanceOnPath`, never through `Path.create`
(`[PB-CONSTRUCTION-PLANES]`).

<!-- check-step-calls: ignore setByPerpendicularAtPoint Path.create -->

`setByPerpendicularAtPoint` and `Path.create` are named only as the two routes this step does not
take, which is why they are exempted.

Creating this axis in the never-activated Design component is proven to work — `constructionAxes.add`
via `setByTwoPlanes` does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here — so keep the axis.

This step creates a construction axis and no measurable geometry, so no proof function realises it.

**From:** `spec/bevelgear/instructions.md` L578-594;
`.claude/skills/generate-gear/PLAYBOOK.md` L766-777, L778-781, L782-790.

## S10 `[GO]` `{gearLabel} Profile` sketch — the revolve hexagon

<!-- proof-run: proofkit.RunParallel(profileCases, stepGearProfileSketch) -->

Run this and every step after it **once per gear** — pinion first, then driving — with these
substitutions:

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A′ → G → H → C → M → N → A′ | B′ → I → J → D → O → P → B′ |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge, the hexagon's FIRST edge | A′→G | B′→I |
| toe cut edge | M→N | O→P |
| heel cut edge | C→H | D→J |
| teeth, bore and pitch-diameter inputs | Pinion Gear … | Driving Gear … |
| §2 shaft construction line, NOT usable as the axis | Apex→A | Apex→B |

Before this step, create this gear's own component: a new occurrence under the **Bevel Gear**
component — the same component that owns Design, *not* the user's Parent Component — named
`{gearLabel} Gear`, so `Pinion Gear` or `Driving Gear`. The finished bodies for this gear end up
there. The actual feature operations all run in the Design component and the bodies are moved across
at the end (S28), because Fusion rejects cross-sibling sketch and project calls even when the target
is activated or the entities are wrapped in `createForAssemblyContext` proxies
(`[PB-NO-CROSS-SIBLING]`). The visible end state is identical.

<!-- check-step-calls: ignore createForAssemblyContext -->

`createForAssemblyContext` is named only as a route that does not help, which is why it is exempted.

**Open a fresh sketch on the axial Gear Profiles plane** — `designComponent.sketches.add(self._gearProfilesPlane)`
— named per the table. **One profile sketch per gear**, so `sketch.profiles` holds exactly this one
hexagon loop; do not draw both gears' hexagons in the shared Gear Profiles sketch, which would leave
two identically-shaped loops to disambiguate.

Build the hexagon on fixed vertices by the recreate-share-fix recipe (`[PB-PROJECT-NOT-FIXED]`),
in this order and no other:

1. recreate the six §2 vertices as new points at their exact world-mapped positions —
   `sketch.sketchPoints.add(sketch.modelToSketchSpace(<§2 point>.worldGeometry))` for each — which is
   valid because §2 is fully constrained by now. `modelToSketchSpace` is a point-transforming
   METHOD, not a matrix: call it directly on the `Point3D` (`[PB-SPACE-METHODS]`);
2. draw the closed hexagon in the table's draw order as six
   `sketch.sketchCurves.sketchLines.addByTwoPoints(...)` calls **sharing** those points;
3. **then** fix the lines' endpoints, once the lines exist — `line.startSketchPoint.isFixed = True`
   and `line.endSketchPoint.isFixed = True` for each. Order matters: setting `isFixed` on a bare
   point before it is consumed as a line endpoint does NOT leave the sketch fully constrained.

Projecting the §2 points instead would leave the sketch under-constrained, because a projection is
associative rather than fixed.

**The hexagon's first edge is the gear's shaft axis** for the revolve, the pattern, the bore plane
AND the meshing rotation, so it must be fixed well enough to carry a trustworthy world position.
Fixed endpoints give that edge a well-defined `worldGeometry` (`[PB-WORLDGEO-CONSTRAINED]`); a free
edge resolves against a default world-XY frame and silently moves the body onto world XY, which was
observed on the driving gear — the pinion looked fine only because it never read the edge's
`worldGeometry`.

**The shaft axis used by every body operation below is this profile sketch's first edge, NOT the §2
`Apex→A` or `Apex→B` construction line.** The edge is collinear with the shaft axis but lives in the
*same* sketch as the profile, which is what Fusion's revolve, pattern and path calls accept; reusing
the §2 construction line, which belongs to a different sketch, fails or misbuilds.

Gate the sketch: `if not sketch.isFullyConstrained: raise` naming it (`[PB-FULL-CONSTRAINT]`,
`[BEVEL-F-FULL-CONSTRAINT]`).

`stepGearProfileSketch` realises this step.

### What the proof establishes

The proof builds the same six recreated vertices and the same closed hexagon, fixes the vertices
after the lines exist, and gates the result. It then checks what the revolve selects on: the sketch
holds exactly one region, that region is extrudable, its boundary is the six drawn lines, the first
two vertices lie ON the shaft axis, and **no other vertex has crossed it** — which is the condition
that would otherwise abort the revolve with `ASM_WIRE_X_AXIS`. It runs for both gears of every case,
because the two hexagons are not mirror images of each other once the tooth counts differ.

**From:** `spec/bevelgear/instructions.md` L389-410, L689-750; `spec/bevelgear/fusion.md` L19-68;
`.claude/skills/generate-gear/PLAYBOOK.md` L449-463, L485-491, L500-507, L576-581, L589-596,
L802-840.

## S11 `[GO]` Revolve the hexagon into the Gear Body

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepRevolveFrustum, assertRevolveFrustum) -->

This sketch holds exactly one hexagon loop, so take its single profile directly —
`profile = gearProfileSketch.profiles.item(0)` on the per-gear Profile sketch of S10 — rather than
filtering. When a sketch has exactly one closed region it has `profiles.count == 1` and that profile
is the one; a curve-type filter in particular has spuriously rejected a valid all-line loop and made
a revolve fail with "could not find profile" (`[PB-SINGLE-PROFILE]`).

Revolve it a full turn about the hexagon's first edge:

```
revolveInput = designComponent.features.revolveFeatures.createInput(
    profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
revolve = designComponent.features.revolveFeatures.add(revolveInput)
```

The result is the **Gear Body**, the frustum. Because the toe edge is one edge of the revolved
profile, the body already carries the conical face that edge sweeps, and that face is reused as a
cutting tool in S14; likewise the heel edge's cone.

**The profile must not cross the axis of revolution.** If it does, Fusion aborts with
`RuntimeError … ASM_WIRE_X_AXIS … the profile crosses the axis of revolution` (`[PB-REVOLVE]`). The
Maximum Face Width of S6 and the Maximum Base Height of S2 are the caps that keep it on one side,
and they are reproduced exactly.

`stepRevolveFrustum` realises this step.

### What the proof establishes

decad publishes a revolved body's volume with a proven bound equal to the volume itself, so a
revolved body is Suspect at any tolerance and cannot pass the harness gate. The proof substitutes the
three bands the frustum's three off-axis profile edges sweep — the heel cone out to the heel end, the
root cone out to the dedendum corner, and the toe-dish plug that hollows the front face — built as
truncated cones and laid apart along the shaft axis, never joined. The other three edges sweep
nothing: two lie in a plane perpendicular to the axis and one lies on the axis itself, and the proof
checks that exactly three qualify.

It then asserts the frustum from those three: the SIGNED SUM of the bands against Pappus on the §2
hexagon, each band's volume against its own measured ring radii and stations, and the three cone
half-angles — the heel band and the toe plug come out parallel, both on the back-cone family, and the
root band stands off them by the dedendum angle's complement.

**The cost is the union**: the proof does not show the three bands closing into one watertight solid,
only that each is separately watertight and that together they have the right volume, stations and
angles.

Two substitutions of shape are recorded in `proof/bevelgear/solids_test.go`. Each band's rings are
drawn as inscribed regular polygons rather than circles, because a band lofted between real circles
publishes a volume whose proven bound runs about 1.4 times decad's relative tolerance and the gate
reports it as a measurement beyond tolerance; lofted between polygons the same band publishes an
exact volume, and the cost is written down as a factor rather than tolerated, since the band is
exactly that factor times the cone it stands for at every station. And the solid tables run at
Module 4 to 8 and never at Module 1: decad's mesh bound has an absolute floor, so a figure small
enough brings every measurement inside it and the gate reports Suspect on geometry that is in fact
correct. Module is a pure scale on this figure, so a case at Module 4 through 8 proves the same shape
as one at Module 1 and clears the floor.

**From:** `spec/bevelgear/instructions.md` L689-750, L757-837;
`.claude/skills/generate-gear/PLAYBOOK.md` L485-491, L708-714.

## S12 `[GO]` Loft the Apex to the tooth profile

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepApexLoft, assertApexLoft) -->

Loft the **§2 Apex sketch point** to this gear's §3 tooth profile; the result is the **Tooth Body**.

```
loftInput = designComponent.features.loftFeatures.createInput(
    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
loftInput.loftSections.add(apexSketchPoint)
loftInput.loftSections.add(toothProfile)
toothBody = designComponent.features.loftFeatures.add(loftInput).bodies.item(0)
```

The order of `loftSections.add` is the loft order, and a section may be a single `SketchPoint` for a
degenerate end, which is what makes this a tapered, pointed body (`[PB-LOFT]`). Use the **§2 Apex
SKETCH point** directly — `centerToApex.endSketchPoint` from the Gear Profiles sketch — and do NOT
create a construction point for it: construction geometry needs an active component and the Design
component is never active (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`).

**Selecting the tooth profile.** Use
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)` from `.utilities`,
with the line count **DETERMINED BY the `embedded` flag read back in S8**:

    wantLines = 0 if embedded else 2

Do **NOT** accept "0 **or** 2 lines". For a given gear only ONE of those is the real tooth, and an
unrelated loop — an inter-tooth or annular region between the circles `drawCircles` drew — can also
have 2 NURBS and 2 arcs but the other line count. Selecting it makes this loft fail with
`RuntimeError … ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`, because the impostor loop cannot form a loft
tool body. Embedded means tip, root and flanks meet with no connecting lines, so four curves;
non-embedded means two connecting lines, so six. The helper raises with a self-diagnosing message
rather than falling back to a wrong profile (`[PB-PROFILE-MATCH]`).

`stepApexLoft` realises this step.

### What the proof establishes

decad's Loft has no point section, so the degenerate apex end is substituted by a shrunken copy of
the tooth section at the same fraction of the station — which is the section the real loft passes
through there anyway — and the tilted back-cone section plane is substituted by an
axis-perpendicular one. The proof asserts that the body tapers linearly from the apex to its section,
that it runs between the two stations the loft was built between, that it reaches the virtual tip
radius and in to its own inner radius, that its root surface lies at or inside this gear's root cone
so the tooth is seated, and that its tip surface stands proud of its inner one.

The axis-perpendicular substitution is recorded in `proof/bevelgear/solids_test.go` along with what
it keeps and what it gives up: the section is placed at the station of the profile's root corner and
scaled by `cos(γ)`, which lands the tooth's root edge where the real one runs and leaves its tip
proud, and gives up the tilt, so the tip and root corners share one station where the real tooth has
them at two. The section is also drawn as a closed polyline with the tip and root boundaries chorded,
because a lofted body whose sections carry arcs publishes a volume whose bound runs near two percent
and the gate refuses it; every reading is chorded against chorded, so the sagitta cancels.

**From:** `spec/bevelgear/instructions.md` L317-388, L689-750, L757-837;
`.claude/skills/generate-gear/PLAYBOOK.md` L151-158, L663-672, L715-719, L782-790.

## S13 `[PROSE]` The tooth-body hook and its ψ gate

`_createGearBody` calls one hook after lofting the uncut apex-to-heel tooth:

```
_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
                    apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld,
                    parentToothPlane, gearLabel, teethNumber, gamma)
```

<!-- check-step-calls: ignore _transformToothBody _createGearBody _pinionMeshPhase cut_conical_ends -->

`_transformToothBody`, `_createGearBody` and `_pinionMeshPhase` are methods this module defines
rather than calls it makes; `cut_conical_ends` is named here only to describe the gate's early
return and is required in S14, which is why all four are exempted here.

`gamma` is this gear's pitch-cone half-angle — `self._gamma_p` for the pinion and `self._gamma_g` for
the driving gear, both from S2 — forwarded to the spiral build's twist law.

**The hook's first line is the gate:** `if self._spiralAngle_rad <= 0: return cut_conical_ends(...)`.
Mean Spiral Angle ψ = 0 means a STRAIGHT bevel gear, the tooth-body build takes the original straight
path unchanged, and every spiral input is ignored. Any value above 0 builds a curved tooth and runs
S15 through S22 in place of S14. The straight order and behaviour are byte-for-byte the prior ones.

**The four toe and heel world points `_createGearBody` builds and passes in, positionally, in the
order `toeMid, heelMid, toeConeWorld, heelConeWorld`. Pin these exactly; mislabelling them silently
inverts the spiral, and it is the single biggest hazard in this build.**

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

From those §2 sketch points' **world** geometry:

- `toeMid` = the world **midpoint of the TOE edge** — ½(M+N) pinion, ½(O+P) driving;
- `heelMid` = the world **midpoint of the HEEL edge** — ½(C+H) pinion, ½(D+J) driving;
- `toeConeWorld` = the toe edge's inner endpoint, **M** or **O**. It lies on the root cone element —
  M is pinned onto Apex→C in §2, O onto Apex→D;
- `heelConeWorld` = the dedendum corner, **C** or **D**. It is the outer end of that **same** root
  cone element, so `coneVec = normalize(heelConeWorld − apex)` runs along Apex→C or Apex→D pointing
  outward.

**Two scrambles to avoid**, both of which a fresh regen has made. Do NOT pass the two endpoints of a
*single* edge as `toeMid` and `heelMid` — M and N both sit at the toe, so the span between them
collapses to about zero or goes negative and the spiral inverts; the two midpoints come from two
different edges. And `heelConeWorld` is the dedendum corner C or D, **never** H or J: H and J lie on
the Apex2→C or Apex2→D dedendum line, one Module beyond C or D, off the root cone element, and using
them skews `coneVec` away from it.

`_pinionMeshPhase(pinionTeeth)` returns the pinion's extra mesh rotation about its own shaft axis, in
radians: `_PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth`, with the class constant
`_PINION_MESH_PHASE_TEETH = 0`. It is 0 for a straight bevel and stays 0 for the spiral, because the
twist is centred on the mean cone distance and leaves the mid-face section unrotated, so that section
already meshes like the straight tooth's.

This step is a branch and a hand-off rather than a timeline entry, so no proof function realises it.
The four points it passes are what S14's and S17's proofs measure against.

**From:** `spec/bevelgear/instructions.md` L317-388, L411-422, L595-688, L689-750.

## S14 `[GO]` Conical end cuts — the straight tooth's flush band

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepConicalCut, assertConicalCut) -->

Trim the Tooth Body to a flush band with the framework helper, and do **not** re-implement the cut
machinery:

```
keeper = cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)
```

from `.solids`. **Two distinct bodies are involved and must not be conflated:** the cutting TOOLS are
`ConeSurfaceType` faces of the **Gear Body**, the revolved-hexagon frustum — the lofted Tooth Body
has no cone faces, so searching *it* finds none — and the TARGET being split is the **Tooth Body**,
the loft.

The helper implements the pinned behaviour. The **toe cut runs first**, its cone face identified by
the toe edge's world **MIDPOINT** best-first across the frustum's cone faces (`[PB-FACE-BY-MIDPOINT]`
— endpoints sit near the apex singularity, where `getParameterAtPoint` returns no result, so an
endpoint-distance search cannot see the right face). Each candidate is tried as the actual split tool
and the first that splits, producing more than one piece, is kept (`[PB-SPLIT-BODY]`). After each cut
the keeper is selected: apex-containing pieces are removed and the largest remaining is kept
(`[PB-REMOVE-PIECES]`). Then the **heel cut runs on the keeper alone**, which is what makes it
deterministically two split features for every gear ratio. A heel cone that does not intersect the
keeper at all — common on ratio pairs such as Module 1 / Driving 31 / Pinion 43, where the heel cone
never overshoots the tooth — is raised by the helper as the typed `solids.NonIntersectError` and
caught, and the keeper is returned whole. Every failure is self-diagnosing with the per-face distance
and error history (`[PB-SELF-DIAGNOSING]`), and each cut's outcome is logged with
`force_console=True`.

**Caller obligations, which stay in this module:** pass `toeMid` and `heelMid` as the world midpoints
of S13's table, `apexWorld` as the §2 Apex sketch point's world geometry, and `gearBody` as the
revolved frustum, which is the cone-face source. The toe cut must split — its failure propagates and
crashes the build, which is correct, since an uncut tooth is unusable — and only the heel cut is
lenient, and only via the typed `NonIntersectError`.

`stepConicalCut` realises this step.

### What the proof establishes

Both operands are Lofts — the tooth and each cone alike — and at the decad revision this repo pins a
boolean refuses a Loft operand, so the proof performs **neither cut**. It builds the tooth and the
two cones and lays them apart along the shaft axis, then reads each cone's apex and half-angle off
the cone it built and each of the tooth's two surfaces off the tooth, and solves the stations where
they cross from those readings.

It asserts that each cone passes through both ends of the §2 edge it was swept from, so it is that
gear's real toe or heel cone and not a look-alike; that the two are parallel, both on the back-cone
family; that the flush band along the root cone between the toe root corner and the dedendum corner
is the resolved Root Length; and that the four crossings are ordered — each cone reaches the tooth's
tip nearer the apex than its inner surface, and the toe cut's pair lies wholly below the heel cut's.
That ordering is what makes the trimmed tooth a band with length rather than a point, and the two
ends landing on different surfaces is the observable signature of a conical cut face rather than a
planar one.

**The cost is the split**: the proof does not show the evaluator dividing the tooth, selecting the
keeper, or leaving a watertight body.

**From:** `spec/bevelgear/instructions.md` L317-388, L689-750, L757-837;
`.claude/skills/generate-gear/PLAYBOOK.md` L724-730, L731-752, L753-760, L761-765.

## S15 `[PROSE]` `{gear} Cone Element` sketch and `{gear} Trace Plane` (ψ > 0)

Everything from here to S22 runs **only when ψ > 0**, once per gear, inside the tooth-body hook, on
the freshly lofted uncut apex-to-heel Tooth Body, before pattern, combine and bore. It **replaces**
S14's two conical trims with a curved tooth, and S22 puts the flush trim back at the end.

**Build the world frame first**, from the geometry already constructed for this gear:

- `axisDir` = the shaft axis direction, from the two **world** endpoints of `shaftAxisEdge` — the
  in-sketch profile edge A′→G or B′→I — normalized. Read the frame you measure against: a sketch
  curve's `.geometry` is sketch-local and `.worldGeometry` is world, and mixing a local-frame curve
  with a world axis silently returns wrong numbers (`[PB-WORLD-FRAME]`);
- `coneVec` = `normalize(heelConeWorld − apexWorld)`, the dedendum root cone element Apex→C or
  Apex→D;
- `v` = `normalize(axisDir × coneVec)`, the circumferential direction — the sideways sense the tooth
  is displaced from the radial element;
- `tpNormal` = `normalize(coneVec × v)`, the tangent-plane normal;
- `distAlong(p)` = `(p − apexWorld) · coneVec`, a point's cone distance.

**The heel MUST be the OUTER end**, so `coneVec` points outward and the span comes out positive.
Before building `coneVec`, check the passed midpoints and **fix a swapped toe and heel**: if
`apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid` with `heelMid` **and**
`toeConeWorld` with `heelConeWorld`, then build `coneVec` from the corrected `heelConeWorld`. A
negative span silently inverts the entire spiral frame — it flips the cutter-arc direction, the slice
direction so the first cut misses, and the per-segment twist — and the gear comes out completely
wrong with no error. The inversion can also originate upstream in §2 or §3 mislabelling the toe and
heel edges; this guard catches it at the frame.

From the corrected midpoints take `R_toe = distAlong(toeMid)`, `R_heel = distAlong(heelMid)`,
`R_mean = ½(R_toe + R_heel)` and `span = R_heel − R_toe`, the face width, now positive. These are the
only quantities the rest of the build needs.

**Then build the two entities this step owns.** In a sketch on the axial Gear Profiles plane named
`{gear} Cone Element`, draw a construction line from the apex to `apex + R_heel · coneVec`. Then
build the tangent plane as that axial plane rotated **90 degrees** about the cone-element line:
`plane_by_angle(designComponent, coneElementLine, self._gearProfilesPlane, 90)`, named
`{gear} Trace Plane`.

**Coordinates — this rule governs the `{gear} Cone Element` sketch as well as S16's trace sketch.**
The world `Point3D`s, including the raw apex and cone-end points of this line, are passed **directly**
into the sketch calls, where they are consumed as **sketch-space** input: no `modelToSketchSpace`
conversion is applied, even though `adsk.fusion.Sketch` offers exactly that call and the points
really are model-space coordinates.

<!-- check-step-calls: ignore modelToSketchSpace -->
<!-- check-compile: ignore distAlong -->

`modelToSketchSpace` is named in this step only to say it is deliberately NOT applied here; it is
required in S10 and S20, which is why it is exempted only in this step. `distAlong` is a local
helper this module defines a few lines above and then calls here, in S19 and in S21; it is a name of
ours rather than a Fusion API name, so the API-name gate is told to skip it while the call-coverage
gate still requires it.

This is deliberate, and it is worth stating why it is harmless, because the reasoning is not the
obvious one. The trace sketch is construction and reference only: no downstream feature ever consumes
it, the twist is computed analytically in S19 from the 2-D endpoints, and the sketch exists only so
the genuine cutter arc is inspectable before cleanup hides it. The cone-element line is the one that
needs the extra sentence, because it *is* consumed, by `plane_by_angle`, which rotates about it to
make the Trace Plane. So an unconverted cone-element line does place that plane somewhere other than
the true tangent plane. That still reaches no feature, because the only thing built on the Trace
Plane is the inspection-only trace sketch, and the whole chain ends there. **If a later revision ever
makes any feature consume the trace sketch or the Trace Plane, this shortcut stops being safe and
both sketches need `modelToSketchSpace` on every point.**

Both sketches are exempt from the full-constraint gate and are hidden in cleanup; do not gate them.

This step builds a construction line whose plane reaches no feature, and a plane that is deliberately
not the true tangent plane, so there is nothing here for a proof to measure that S16 does not measure
better. The reason is recorded in `proof/bevelgear/spiral_test.go` beside `stepSpiralTrace`, which
works in the tangent plane's 2-D frame directly.

**From:** `spec/bevelgear/instructions.md` L595-688; `spec/bevelgear/fusion.md` L19-68;
`spec/bevelgear/spiral-tooth-trace.md` L30-65, L66-79;
`.claude/skills/generate-gear/PLAYBOOK.md` L430, L766-777.

## S16 `[GO]` `{gear} 2D Tooth Trace` sketch — the genuine cutter arc

<!-- proof-run: proofkit.RunParallel(traceCases, stepSpiralTrace) -->

Add a sketch on the `{gear} Trace Plane` named **`{gear} 2D Tooth Trace`**. Work in the tangent-plane
2-D frame with the origin at the apex, **x = `coneVec`** so a point's x IS its cone distance, and
**y = `v`**, circumferential. Map 2-D coordinates to world with the framework helper
`combine_point(apexWorld, px, coneVec, py, v)`.

**The cutter radius** `r_c` is the Cutter Radius input when non-zero, **else `R_mean`**, which is the
auto default. **The hand sign** is `+1` for `Right` and `−1` for `Left`, then **negated for the
pinion**, because the pair meshes with opposite hands. The cutter-circle centre is

```
Cx = R_mean - r_c * sin(psi)
Cy = handSign * r_c * cos(psi)
```

**The hand sign goes on the `cos` / `Cy` term, NOT the `sin` / `Cx` term.** This was a real bug.
Opposite hand mirrors the cutter centre **across the cone element**, the line y = 0, which flips
`Cy`. Putting `handSign` on `Cx` mirrors about `x = R_mean` instead — a different curve that gives
the two gears unequal twist, where for equal teeth the driving and pinion traces must come out as
exact mirror images.

The trace's toe and heel arc endpoints are circle-circle intersections taken a hair **past** the face
so the kept arc reaches cleanly past the end trims:

```
R_lo  = R_toe  - 0.06 * span
R_hi  = R_heel + 0.06 * span
toe2d  = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)
heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)
```

`circle_intersect_nearest` is the framework helper from `.solids`: it intersects the apex circle of
radius R with the cutter circle, centre `(Cx, Cy)` and radius `r_c`, and keeps the solution nearest
`(R_mean, 0)`, which is the branch the mean point sits on. Keeping the far branch gives a kinked or
back-bent trace.

Draw two things in the sketch:

- **the cutter circle**, centre at `combine_point(apexWorld, Cx, coneVec, Cy, v)` and radius `r_c`,
  added with `sketch.sketchCurves.sketchCircles.addByCenterRadius(...)`, marked
  `circle.isConstruction = True`, with its centre pinned by
  `circle.centerSketchPoint.isFixed = True` — a circle's centre is a free point even when created at
  the origin, and `isFixed` is the reliable pin (`[PB-CIRCLE-CENTER]`) — and a diameter dimension
  `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` set to `2 * r_c`;
- **the trace arc**, a three-point arc through `tanW(toe2d)`, `tanW(R_mean, 0)` — the mean point on
  the cone element — and `tanW(heel2d)`, added with
  `sketch.sketchCurves.sketchArcs.addByThreePoints(startPoint, point, endPoint)`, with its centre
  made coincident to the cutter circle's centre by
  `sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, circle.centerSketchPoint)` and a
  radius dimension `sketch.sketchDimensions.addRadialDimension(arc, textPoint)` set to `r_c`, so it
  is the genuine cutter circle and not a look-alike spline.

**Text points must be OFF-CENTRE** (`[PB-RADIAL-DIM]`): a radial or diameter dimension rejects a text
point at the curve's centre with `RuntimeError: 3 : … 一部の入力引数が無効です`, because at the centre
there is no radial direction to place it. Use the mean point `tanW(R_mean, 0)` for the trace arc's
radius dimension and a point on the cutter circle such as `tanW(Cx + r_c, Cy)` for the circle's
diameter dimension.

**There is no 3-D projection.** The 2-D cutter-arc sketch is the only trace geometry needed: the
spiral twist is computed analytically in S19 from its endpoints, so there is no `projectToSurface`,
no root-cone-face search and no 3-D trace sketch. An earlier version projected the 2-D arc onto the
root cone along `tpNormal` and measured the trace azimuth there; that projection is fragile, because
for unequal-ratio pairs the arc wraps around the cone and `projectToSurface` returns it as multiple
disjoint fragments, so the measured azimuth collapses to a fraction of the true sweep — the pinion
comes out grossly under-twisted and the pair interferes. Do not reintroduce it.

<!-- check-step-calls: ignore projectToSurface -->

`projectToSurface` is named only to forbid it, which is why it is exempted.

This sketch is **deliberately left with free degrees of freedom** — the arc's endpoints are pinned by
the three-point construction, not by endpoint dimensions, which would over-constrain the solve
against the cone-element plane — and is therefore exempt from the full-constraint gate. Do not gate
it.

`stepSpiralTrace` realises this step.

### What the proof establishes

The proof builds the cutter circle and the trace arc in the tangent plane's own 2-D frame and checks
the invariants `spec/bevelgear/spiral-tooth-trace.md` §9 lists, which are what a correct trace has to
satisfy and what each common way of drawing it wrong breaks: the centre sits exactly one cutter
radius from the mean point; both ends lie on the cutter circle, so the arc's radius is `r_c`
everywhere and it is one circle; each end sits on its own apex circle at the radius the 0.06 overrun
asks for; the mean spiral angle is realized AT the mean point, read off the tangent there; the
opposite hand keeps the centre's cone distance and mirrors it across the element, which is the
reading that catches the sign living on the sin term instead; and at ψ = 0 the whole offset is
circumferential and the trace is tangent to the cone element at the mean point.

It also checks the roll ratio the twist depends on: the angle the law divides by is the **PITCH** cone
angle, a dedendum angle above the root cone's. Measuring it off the cone element instead gives the
root cone angle, which inflates the twist by about 1.6 times for a 17-tooth pinion — the difference
between a pair that meshes and one that interferes.

Two differences are recorded beside it in `proof/bevelgear/spiral_test.go`. The geometry is reference
geometry there, for the same reason S8's tooth is: a fully placed ordinary arc makes its internal
radius-consistency row dependent and the gate refuses the redundancy, where Fusion leaves this sketch
with free degrees of freedom and exempts it instead — neither engine gates it on the same terms, and
what the step proves is the arc. And the straight-bevel limit is exact only in the TANGENT: a finite
cutter still curves away from the element either side of the mean point, so ψ = 0 leaves a small
residual twist rather than none, and the straight bevel is the `r_c → ∞` limit. The generated module
does not rely on even that, because at ψ = 0 the hook returns before any of this runs.

**From:** `spec/bevelgear/instructions.md` L595-688; `spec/bevelgear/fusion.md` L19-68;
`spec/bevelgear/spiral-tooth-trace.md` L18-29, L80-89, L90-107, L108-148, L149-185, L218-239,
L240-254; `.claude/skills/generate-gear/PLAYBOOK.md` L442-448, L500-507, L643-647.

## S17 `[GO]` Slice the straight tooth into slabs

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepSliceTooth, assertSliceTooth) -->

Split the uncut apex-to-heel `toothBody` into cross-section slabs by planes **perpendicular to the
cone element**, spanning a touch past toe and heel, via a **fixed** slice scheme of about eight
planes. The count is not user-configurable.

The first cut plane is the **parent transverse tooth plane** — `parentToothPlane`, the virtual-spur
tooth-profile plane `{gearLabel} Plane` from S7, passed into the hook — offset toward the apex by
`span / 6`. **The offset sign is chosen per gear** so it moves toward the apex, because the parent
plane's normal points opposite ways for the two gears: pick `sign` so `sign * normal` points
apex-ward, by testing `(apex − planeOrigin) · normal`. Then a sequence of about eight planes steps
further toward the apex in `span / 6` increments. Split with the framework helper:

```
offsets = [sign * (k + 1) * span / 6 for k in range(8)]
pieces  = slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)
```

from `.solids`, which splits piece-by-piece and keeps a piece whole when a plane misses it. The
caller picks the offsets and signs and asserts the resulting piece count (`[PB-EMPTY-RESULT]`).

**The slice MUST actually split the tooth.** After the cut loop, if the body is still in **one
piece**, so no plane cut it, the offset sign was wrong or `parentToothPlane` sits outside the tooth's
span: **retry the whole cut once with the opposite sign**. If it is still one piece, **raise a clear
self-diagnosing error** naming the gear, the final piece count, `span`, and the sign tried
(`[PB-SELF-DIAGNOSING]`). Do **NOT** return an unsliced single-piece result: the next step then drops
that one piece as the apex scrap, leaving the segment list **empty**, and the crown later crashes
with `ValueError: max() iterable argument is empty` far from the cause.

`stepSliceTooth` realises this step.

### What the proof establishes

decad has no split, so the proof builds the slabs themselves rather than cutting one body, lays them
apart along the shaft axis, and asserts what the split has to produce: the fixed eight cuts and
therefore nine pieces, each cut at the `span / 6` station the spec names, each piece being the tooth
between its own two stations, and the pieces adding back up to the whole tooth — which is what a
split is and what a plane that missed would break. It also asserts the result is more than one piece
at all, which is the condition the retry and the raise above exist for.

One substitution is this step's own and is recorded in `proof/bevelgear/spiral_test.go`: the spec's
slice planes are perpendicular to the CONE ELEMENT and the proof's are perpendicular to the SHAFT
AXIS, placed at the station each cone-distance offset reaches along the root cone, so the offsets are
the spec's own, measured where the tooth's root sits.

**From:** `spec/bevelgear/instructions.md` L595-688, L757-837;
`.claude/skills/generate-gear/PLAYBOOK.md` L431, L724-730, L753-760.

## S18 `[GO]` Order the slabs and drop the apex scrap

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepDropScrap, assertDropScrap) -->

Sort the pieces by the `distAlong` of their centroid — `body.physicalProperties.centerOfMass` — and
drop the apex-most one, which is the long apex-side scrap below the toe. Keep the rest as the working
`segments`.

**Drop the scrap by re-slicing the list first, then deleting it**: `segments = segments[1:]` before
`designComponent.features.removeFeatures.add(scrap)`. A remove feature is the timeline-visible way to
delete a piece; do not call `deleteMe()` on the body (`[PB-REMOVE-PIECES]`).

<!-- check-step-calls: ignore deleteMe -->

`deleteMe` is named only to forbid it here, which is why it is exempted.

After dropping the scrap, **`segments` must be non-empty**, at least one cross-section. If it is
empty the slice failed in S17: raise a clear error rather than proceeding into the twist and the
crown, which both assume at least one segment (`[PB-EMPTY-RESULT]`).

`stepDropScrap` realises this step.

### What the proof establishes

The proof builds the segments that remain after the drop, laid apart along the shaft axis, and
asserts the drop: the count is the slice's count less one, the list is non-empty, the segments are
ordered outward, and the piece that was dropped is the apex-side one running from the apex end up to
the first cut. The empty case is checked the moment it is produced rather than three steps later,
which is the point of the guard.

decad has no remove feature either, so the scrap is built and then left out rather than deleted;
what the proof gives up is the timeline entry, not the ordering or the count.

**From:** `spec/bevelgear/instructions.md` L595-688, L757-837;
`.claude/skills/generate-gear/PLAYBOOK.md` L431, L761-765.

## S19 `[GO]` Twist the segments — the spiral

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepTwistSegments, assertTwistSegments) -->

Rotate each segment about the **shaft axis** — `axisDir` through `apexWorld` — so the tooth follows
the trace, **centred on `R_mean` so the mid-face section stays unrotated**. That section then meshes
exactly like the straight tooth, which is critical: the pinion's zero mesh phase depends on it.

The total toe-to-heel shaft-axis twist comes from the **conjugate crown-gear generation law**, the
standard Gleason and Litvin model. A spiral bevel is generated by an imaginary flat crown gear, and
the work gear's shaft rotation relates to the developed crown-plane azimuth by the **roll ratio
`1 / sin γ`**: the generating crown gear has `N / sin γ` teeth, and γ is this gear's **pitch** cone
angle. Compute it **analytically — no projection, no curve sampling**:

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`phi_crown` is the angle the cutter arc's toe and heel endpoints subtend at the apex in the flat 2-D
crown frame — exactly the `toe2d` and `heel2d` pairs S16 computed. `gamma` is this gear's **pitch**
cone angle, `self._gamma_p` for the pinion and `self._gamma_g` for the driving gear, both from S2.
The hand sign sets the direction and `total` is the magnitude.

**Use the PITCH cone angle γ — NOT `acos(coneVec · axisDir)`**, which is the root or dedendum cone
angle, about 14 degrees against the pitch's 29 for a 17-tooth pinion, and yields a twist about 1.6
times too large. **The two members of a meshing pair legitimately get different twists**: same
cutter, same spiral angle ψ, but γ differs, so `1 / sin γ` differs — about 2.08 times for a 17-tooth
pinion against about 1.14 for a 31-tooth gear, a ratio near 1.83. That is *why* equal-teeth pairs
with equal γ always meshed while ratio pairs failed under any method that gets `1 / sin γ` wrong.

Each segment's rotation angle is a **linear share keyed to the cone distance of its HEEL FACE**, the
segment's farthest-along-the-element face, which is the exact section the later loft samples:

```
ang = -handSign * total * (R_mean - R_heelFace(seg)) / span
```

**Define a slab's heel face precisely: the face whose centroid has the GREATEST
`distAlong(face.centroid)`, searched across ALL of the slab's faces with NO surface-type filter**;
its toe-side face is the least-centroid one. Do **NOT** restrict the search to `PlaneSurfaceType` or
any other surface type — a sliced slab is bounded by a mix of the two planar cut faces and ruled side
faces, and a type filter can pick the wrong face or miss the cut face, which makes the S21 loft fail
with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this **same all-faces-by-centroid** rule
everywhere a slab end face is needed: the twist key here, the crown base in S20, and the loft sections
in S21.

**Key the twist on the segment's HEEL-FACE cone distance, NOT its centroid.** The loft samples each
segment's heel face, so that face is what must land at the right azimuth; centroid-keying leaves the
loft's mid-face section rotated by half a segment and the flanks overlap there.

Apply the rotation as a free move:

```
rotate_body_about_edge(designComponent, segment, shaftAxisEdge, ang)
```

the framework helper from `.solids`, which builds
`adsk.core.Matrix3D.setToRotation(angleRadians, axisVector, originPoint)` and applies it through
`designComponent.features.moveFeatures.createInput2(bodyCollection)` and
`moveInput.defineAsFreeMove(matrix)` — a free move with a matrix, not `defineAsRotate`, which rejects
a `SketchLine` axis (`[PB-MOVE-ROTATE]`). The helper takes the rotation axis and origin from the
profile edge's **world** endpoints, and it absorbs a zero angle: `setToRotation(0, axis, origin)`
builds the identity and Fusion refuses to move a body by it with
`RuntimeError: 3 : invalid transform`, so a caller that computes an angle may legitimately arrive at
zero without guarding each call site.

<!-- check-step-calls: ignore defineAsRotate -->

`defineAsRotate` is named only to forbid it, which is why it is exempted.

`stepTwistSegments` realises this step.

### What the proof establishes

The proof builds each segment already turned by its own share — decad's `Placed` is an isometry and
a turned slab can be built that way directly — and then reads the law back off the segments
themselves: each segment's measured azimuth against the twist its heel-face cone distance calls for,
the measured twist RATE against the crown-gear law's total over the span, and the fact that the
section at the mean cone distance is the one left unrotated, which is why the pinion needs no extra
mesh phase. It also checks that keying on the centroid instead of the heel face would move the
result, so the distinction is not vacuous.

**From:** `spec/bevelgear/instructions.md` L595-688, L757-837;
`spec/bevelgear/spiral-tooth-trace.md` L186-217;
`.claude/skills/generate-gear/PLAYBOOK.md` L731-752, L791-801.

## S20 `[GO]` Crown the segments — lengthwise relief

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepCrownSegments, assertCrownSegments) -->

Crown the tooth by scaling each segment **except the outermost, heel one** down by a **monotonic**
factor — full at the heel and growing smoothly toward the toe — **about a sketch point on the ROOT
edge of its heel face**.

For each segment compute its **heel-distance fraction**

    u = (R_heel - R_heelFace) / span

where `R_heelFace` is the `distAlong` of that segment's heel face, **found by the S19 all-faces-by-
centroid rule but RECOMPUTED here, AFTER the twist has moved the slabs** — do not reuse pre-twist
values — and `R_heel` and `span` come from the frame in S15. `u` runs 0 at the held-full heel to 1 at
the toe. The **outermost, heel segment is the one with the GREATEST post-twist heel-face
`distAlong`**: sort the segments by their recomputed heel-face `distAlong` and skip the last. Then

    factor = 1 - _CROWN_PER_RAD * (abs(total) / 2) * u

`total` is the full toe-to-heel twist from S19, so `abs(total) / 2` is the per-end peak twist
magnitude and the maximum relief — now at the **toe** — keeps the same magnitude the old per-end peak
had, just relocated. This makes relief **grow monotonically from the full heel to the toe**, so slab
heights stay strictly ordered heel to toe and the natural cone taper is never reversed. If a computed
`factor` comes out at or below 0, from an extreme twist, **raise a self-diagnosing error** naming the
gear, the segment's `u` and the factor; never scale by a non-positive factor.

**`_CROWN_PER_RAD` is a tunable class constant, default `0.5`.** Zero disables the crown; set it to
0.5 and do not leave it unset or at 0.

**Do NOT key the relief on `abs(ang)`**, the twist magnitude, which is the distance from the mid-face.
That is **symmetric** about the mid-face and therefore maximal at BOTH ends, so because the heel slab
is held full, the slab *just inside* the heel becomes the **most**-relieved one and dips below both
its neighbours — a notch that reverses the heel-to-toe taper. This was the observed bug: the
heel-adjacent slab came out at factor 0.932 while the next slab inward was 0.972, taller. Key the
relief on the monotonic heel-distance `u`, never on `abs(ang)`.

**Three things about the scale itself.**

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex — because a `ConstructionPoint` needs an active component
   (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). `scaleFeatures` is the ONE exception to never-activate: it
   needs the Design occurrence as the **active** edit target, so call
   **`self.designOccurrence.activate()`** before the crown scales and restore afterwards, **in a
   `finally`**, with **`self.design.activateRootComponent()`**. Do NOT write
   `design.rootComponent.activate()` or `someComponent.activate()` — a `Component` has **no**
   `.activate()` method and raises `AttributeError`. Only an `Occurrence` has `.activate()`, and the
   root is re-activated through `Design.activateRootComponent()`.
2. **Skip the outermost, heel segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone in S22 trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, NOT its centroid**, or the crowned tooth lifts
   off the gear base. `scaleFeatures` shrinks **uniformly** toward the base point, so a base point at
   the heel-face **centroid**, at mid tooth-height, pulls the tooth's **root** edge upward by
   `(1 − factor) * (½ tooth height)`: the tooth no longer seats on the gear body's root cone, floats
   above the base, and the Combine-Join leaves a gap, which is clearly visible for ratio pairs such
   as Module 2 / Driving 19 / Pinion 13 and is the original symptom that exposed this. Put the base
   point on the **root** instead: of the heel face's vertices — `heelFace.vertices`, each
   `vertex.geometry` a world `Point3D` — take the **two with the smallest perpendicular distance to
   the shaft axis**, the line through `apexWorld` along `axisDir`, where the perpendicular distance
   is `|(p − apex) − ((p − apex) · axisDir) * axisDir|`. Those two are the **root corners**, since
   the tip corners are the farthest from the axis. Place the base sketch point at their
   **midpoint**, mapped into the heel-face sketch with `sketch.modelToSketchSpace(...)`. The heel
   face is a planar cut, so that midpoint lies on it. A uniform scale about a point keeps every line
   and plane through that point invariant, so anchoring on the root keeps the root edge on the
   seating cone while the tip is relieved progressively toward the toe, which is exactly the
   lengthwise crown intended. Finding the heel face itself is unchanged — still the
   max-`distAlong`-centroid face by the S19 rule; only the point *on* it changes from centroid to
   root-edge midpoint.

The feature is
`scaleInput = designComponent.features.scaleFeatures.createInput(bodyCollection, baseSketchPoint, adsk.core.ValueInput.createByReal(factor))`
then `designComponent.features.scaleFeatures.add(scaleInput)`.

<!-- check-step-calls: ignore activate -->

`activate` appears above both as the call to make on the Design **occurrence** and as the call NOT to
make on a `Component`, so it is exempted rather than read as one requirement.

`stepCrownSegments` realises this step.

### What the proof establishes

decad has no scale feature, so the proof builds each segment already scaled about the base point on
its heel face's root edge rather than scaling one after the fact. It then asserts the relief from the
bodies: each segment's volume ratio against its own crown factor cubed, which is what a uniform
scale does; the outermost segment held exactly full; and the factors strictly increasing outward, so
the relief grows monotonically from the heel to the toe and no slab dips below its neighbours. It
also computes how far a centroid anchor would lift the root edge, which is the failure the root
anchor exists to avoid, and checks that the toe-most segment is relieved at all, so the check is not
vacuous.

**From:** `spec/bevelgear/instructions.md` L595-688, L757-837; `spec/bevelgear/fusion.md` L153-166;
`.claude/skills/generate-gear/PLAYBOOK.md` L576-581, L782-790, L812-819.

## S21 `[GO]` Loft the curved tooth

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepSpiralLoft, assertSpiralLoft) -->

**Re-sort the segments by their heel-face cone distance HERE, AFTER the twist and the crown.** Do NOT
reuse the pre-twist slice or centroid order from S18. The twist rotates each slab about the shaft
axis, and for high-twist *unequal-ratio* pairs that rotation changes the slabs' along-cone
`distAlong` order enough to **reorder adjacent slabs**; lofting in the stale pre-twist order then
assembles the cross-sections out of sequence and the crowned tooth comes out distorted, so the two
gears interfere. For equal or low-twist pairs the two orders coincide, which is why equal-teeth gears
mesh even with the stale order while unequal ratios distort — this is the single thing that makes a
ratio pair like 31/17 fail while 31/31 looks fine.

So compute `order = sorted(segment indices, key = distAlong(slabHeelFace(seg).centroid))` **now**,
and loft a new body through, in that order:

1. first the **toe-most segment's apex-side, toe-facing face** — the toe segment is `order[0]`, and
   its toe face is added first to push the loft past the toe cone so the toe trim bites;
2. then the **heel-facing face of every segment, iterated in `order`** — each segment's
   farthest-along-the-element face by post-twist centroid, so the last reaches past the heel cone.

```
loftInput = designComponent.features.loftFeatures.createInput(
    adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
loftInput.loftSections.add(<toe face of order[0]>)
for i in order:
    loftInput.loftSections.add(<heel face of segments[i]>)
curvedTooth = designComponent.features.loftFeatures.add(loftInput).bodies.item(0)
```

<!-- check-compile: ignore sorted slabHeelFace distAlong -->

`sorted` is a Python builtin, and `slabHeelFace` and `distAlong` are local helpers this module
defines — the first the all-faces-by-centroid rule S19 states, the second the cone-distance
projection S15 builds. None of the three is a Fusion API name, so the API-name gate is told to skip
them; all three are still calls the module makes, so the call-coverage gate is not.

The order of `loftSections.add` is the loft order (`[PB-LOFT]`). Name the resulting body
**`{gear} Spiral Tooth`**. Then remove the segment scaffolding — the loft has captured their faces —
with `designComponent.features.removeFeatures.add(<segment>)` for each (`[PB-REMOVE-PIECES]`).

`stepSpiralLoft` realises this step.

### What the proof establishes

decad's Loft takes exactly two profiles and the spiral loft runs through one face per segment, so the
single body is not built here. What is built is the same set of sections, twisted and crowned, and
what is asserted is the ORDER they have to be assembled in — which is the thing this step is actually
about. The proof checks that the order recomputed from the POST-twist, POST-crown heel faces is
strictly increasing outward, that neighbouring sections sit one `span / 6` apart along the axis, and
that the case carries a real twist, so the ordering claim is not vacuous.

The substitution, and that the single lofted body is not shown, is recorded in
`proof/bevelgear/spiral_test.go` beside the step.

**From:** `spec/bevelgear/instructions.md` L595-688, L757-837;
`.claude/skills/generate-gear/PLAYBOOK.md` L715-719, L761-765.

## S22 `[PROSE]` Flush trim the spiral tooth

Return `cut_conical_ends(designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)`
— the framework helper from `.solids`, the **same** toe-then-heel two-cone trim S14 makes for the
straight tooth — so the curved tooth's ends sit flush on the gear base. Every obligation in S14
applies unchanged: the cutting tools are the Gear Body's `ConeSurfaceType` faces, the target is the
curved tooth, the toe cut must split, and only the heel cut is lenient and only via the typed
`NonIntersectError`.

The toe and heel **mesh phasing** is handled outside this hook, by the mesh-rotate step S27. The
pinion's extra phase is 0 by default because the mid-face section is unrotated and already meshes.

This step makes the same helper call on the same operands as S14 — nothing about the cut differs,
only which tooth body is handed to it — so S14's proof already covers it and no proof function is
written twice. That is recorded in `proof/bevelgear/spiral_test.go` beside the spiral chain.

**From:** `spec/bevelgear/instructions.md` L317-388, L595-688, L689-750.

## S23 `[GO]` Circular-pattern the tooth

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCircularPattern, assertCircularPattern) -->

Circular-pattern the remaining tooth piece — the keeper S14 or S22 returned — around the
**shaft-axis edge**, the same in-sketch profile edge the revolve used, not the §2 construction line.

```
seeds = adsk.core.ObjectCollection.create()
seeds.add(toothKeeper)
patternInput = designComponent.features.circularPatternFeatures.createInput(seeds, shaftAxisEdge)
patternInput.quantity   = adsk.core.ValueInput.createByReal(<this gear's Teeth Number>)
patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
patternInput.isSymmetric = False
pattern = designComponent.features.circularPatternFeatures.add(patternInput)
```

**Pin all three inputs explicitly** — quantity, a full 360-degree total angle, and not symmetric —
rather than relying on Fusion's defaults staying equal to them (`[PB-CIRCULAR-PATTERN]`).

Although the pitch diameter shrinks from the heel toward the apex, the *angular* spacing around the
shaft axis stays constant at `360° / N` for the entire face width: the radial taper is already
produced by the loft from the Apex to the heel-end tooth profile, so the pattern just rotates that
single tapered tooth into N evenly spaced copies.

`pattern.bodies` already includes the seed body plus the copies, so do not re-add the seed
(`[PB-PATTERN-BODIES]`).

`stepCircularPattern` realises this step.

### Why this step is the one that stays serial

Every other `[GO]` step here is registered through a parallel harness entry point, because a bevel
case builds its own sketch or its own document from its own parameters and measures only the geometry
that case constructed, so two cases of one step have nothing between them to corrupt. Running them
together is what keeps the bevel proof, the slowest package in the suite, from setting the suite's
wall time on its own.

**The Pattern step is the exception and stays serial**, on `proofkit3d.RunSolid`. The pattern
increment retires the seed tooth, so the seed cannot be measured after the step runs, and its
azimuth, radius, height and volume have to be read during the build and handed to the assertion. That
hand-off leaves the case, and two cases sharing one set of seed readings overwrite each other. It is
not a hazard that announces itself: the two gear sides differ enough in volume that the overwrite was
caught when it happened, and a pair of cases whose seeds measured alike would have passed on each
other's numbers instead. The carried readings are kept in package-level variables in
`proof/bevelgear/solids_test.go`, and the reason this step is serial is recorded beside them.

### What the proof establishes

The proof builds the seed tooth, measures it, applies one pattern increment as a rigid motion that
retires the seed exactly as the pattern retires the body it copies, and returns the copy. It asserts
that copy 1 sits one tooth pitch round from the seed, that it is the seed moved rather than reshaped
— same volume, same station, same reach from the axis — and that N such increments close the full
circle, which is what quantity and totalAngle together ask for.

**From:** `spec/bevelgear/instructions.md` L689-750, L838-861;
`.claude/skills/generate-gear/PLAYBOOK.md` L684-694.

## S24 `[GO]` Combine-Join the teeth into the Gear Body

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCombineJoin, assertCombineJoin) -->

Join all patterned tooth pieces with the Gear Body in a **single** Combine-Join: the Gear Body is the
target and the patterned tooth bodies are the tools.

```
tools = adsk.core.ObjectCollection.create()
for i in range(pattern.bodies.count):
    tools.add(pattern.bodies.item(i))
combineInput = designComponent.features.combineFeatures.createInput(gearBody, tools)
combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
designComponent.features.combineFeatures.add(combineInput)
```

Copy the patterned bodies into a fresh `ObjectCollection` first: `pattern.bodies` is a `BRepBodies`
and `combineFeatures.createInput` rejects it (`[PB-PATTERN-BODIES]`).

`stepCombineJoin` realises this step.

### What the proof establishes

Both operands are Lofts, so the proof performs no join. It lays the Gear Body's seating surface and
one tooth apart along the shaft axis and asserts the join's two consequences from their own measured
geometry: at the toe, the middle and the heel of the band the join would cover, the tooth's root sits
at or below the gear body's root cone — seated, not floating, which is what makes the join leave ONE
lump — and the tooth's tip stands proud of it, which is what makes the joined body reach further out
than the frustum.

**The proof sinks the tooth's root a twentieth of the tooth height below the gear body's root cone**,
which is what makes "seated" measurable as a strict inequality. **The generated module seats the
tooth exactly ON the cone and must not sink it**: the sink belongs to the proof alone.

**The cost is the stitch**: the proof cannot show the evaluator making one boundary out of two.

**From:** `spec/bevelgear/instructions.md` L689-750, L757-837;
`.claude/skills/generate-gear/PLAYBOOK.md` L684-688.

## S25 `[GO]` `{gearLabel} Bore` sketch

<!-- proof-run: proofkit.RunParallel(boreCases, stepBoreSketch) -->

**Skip this step and S26 entirely when Enable Bore is unchecked.** The per-gear bore diameter inputs
are then ignored on both gears.

Build the bore plane normal to the shaft at its start:

```
planeInput = designComponent.constructionPlanes.createInput()
planeInput.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))
borePlane = designComponent.constructionPlanes.add(planeInput)
```

`setByDistanceOnPath` takes a fractional distance from 0 at the start to 1 at the end
(`[PB-CONSTRUCTION-PLANES]`); pass the **in-sketch shaft-axis edge**, not the §2 construction line,
and pass it directly rather than through `Path.create`.

<!-- check-step-calls: ignore Path.create -->

`Path.create` is named only to forbid it, which is why it is exempted.

In a sketch named `{gearLabel} Bore` — `Pinion Bore` or `Driving Bore` — sketch the bore circle
centred at the sketch origin, since the plane is rooted at the shaft start and the origin is
therefore on the axis:

```
boreSketch = designComponent.sketches.add(borePlane)
circle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(
    adsk.core.Point3D.create(0, 0, 0), boreDiameter / 2)
circle.centerSketchPoint.isFixed = True
boreSketch.sketchDimensions.addDiameterDimension(circle, textPoint).parameter.value = boreDiameter
```

**Fix the centre and add a diameter dimension**, in that combination. A circle's centre is a FREE
point even when created at `(0, 0, 0)` — `addByCenterRadius` does not reuse the sketch's
`originPoint` — so two degrees of freedom plus one make zero. Do NOT make the centre coincident to
the sketch origin instead: that has been observed to throw `VCS_SKETCH_SOLVING_FAILED`, at least on a
`setByDistanceOnPath` plane. `isFixed` on the centre is the reliable pin (`[PB-CIRCLE-CENTER]`).
Place the diameter dimension's text point off the centre (`[PB-RADIAL-DIM]`).

The bore diameter is this gear's Bore Diameter input when non-zero, otherwise this gear's Pitch
Diameter / 4, as S2 resolved it.

Gate the sketch: `if not boreSketch.isFullyConstrained: raise` naming it (`[PB-FULL-CONSTRAINT]`,
`[BEVEL-F-FULL-CONSTRAINT]` — the Bore sketch is one of the four permanent sketches the gate covers).

`stepBoreSketch` realises this step.

### What the proof establishes

The proof builds the same circle with the same fixed centre and diameter dimension and gates it,
which is what shows the combination reaches DOF 0 with nothing redundant — the pairing the
coincident-to-origin alternative would break. A case with Enable Bore unchecked is skipped there,
naming the reason, rather than passing quietly.

**From:** `spec/bevelgear/instructions.md` L25-144, L389-410, L689-750;
`spec/bevelgear/fusion.md` L19-68; `.claude/skills/generate-gear/PLAYBOOK.md` L442-448, L500-507,
L643-647, L766-777.

## S26 `[GO]` Bore cut

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepBoreCut, assertBoreCut) -->

Cut the cylindrical through bore through the Gear Body along the shaft axis:

```
profile = boreSketch.profiles.item(0)
extrudeInput = designComponent.features.extrudeFeatures.createInput(
    profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
extrudeInput.setSymmetricExtent(
    adsk.core.ValueInput.createByReal(2 * coneDistance_cm), False)
extrudeInput.participantBodies = [gearBody]
designComponent.features.extrudeFeatures.add(extrudeInput)
```

`setSymmetricExtent(distance, isFullLength)` with `isFullLength = False` means the distance is the
half-length **per side**, so `2 × Cone Distance` per side reaches generously past any face width and
makes this a through cut regardless of thickness. Do not pass a third taper argument. Restrict the
cut to this gear's body with `participantBodies` (`[PB-THROUGH-CUT]`).

`stepBoreCut` realises this step.

### What the proof establishes

The tool is a real extrude — a symmetric extent produces a prism, which decad builds exactly — but
the target is the frustum, whose bands are Lofts, so the proof performs **no cut**. It lays the tool
and a band apart along the shaft axis and asserts the cut from the tool's own measured geometry: its
radius is half the Bore Diameter, its volume is the prism of that section, its two ends sit exactly
`2 × Cone Distance` either side of the shaft edge's start, and both clear the frustum — which is what
makes it a THROUGH cut. It then computes the material the cut would remove, from the frustum's own
profile clipped to the bore radius, and checks that it is a real hole: more than nothing and less
than the whole frustum.

**The cost is the pierced body**: one lump with a hole and no enclosed void is not shown.

**From:** `spec/bevelgear/instructions.md` L689-750, L757-837;
`.claude/skills/generate-gear/PLAYBOOK.md` L720-723.

## S27 `[GO]` Meshing rotation

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepMeshRotate, assertMeshRotate) -->

**Driving gear only, and here — in the Design component, before the body is moved out.** Rotate the
driving body by `180° / Driving Gear Teeth Number`, half a tooth pitch, about its shaft axis:

```
rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)
```

the framework helper from `.solids`, which takes the rotation axis and origin from the **B′→I**
profile edge's **world** endpoints and applies a free-move matrix (`[PB-MOVE-ROTATE]`). A driving
valley then sits where the pinion tooth crosses the axial plane, giving the interlocked meshing look.
Both gears are patterned from a starting tooth in the axial plane, so without the offset a driving
tooth and a pinion tooth would both sit at the axial-plane crossing and visually collide.

This runs in Design before the bodies move out because a construction axis cannot be added in the
moved-out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation must use the edge's
world geometry while still in Design.

The pinion additionally receives `_pinionMeshPhase(pinionTeeth)`, which is
`_PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth` with `_PINION_MESH_PHASE_TEETH = 0`, so it is
0 unless a spiral pair ever needs one. **A zero angle is a no-op, not a move**:
`setToRotation(0, axis, origin)` builds the identity and Fusion refuses it with
`RuntimeError: 3 : invalid transform`, measured on the bevel pinion whose mesh phase is 0 by default.
`rotate_body_about_edge` absorbs a zero angle for exactly this reason, so return early rather than
guarding at each call site.

`stepMeshRotate` realises this step.

### What the proof establishes

The proof builds the tooth, rotates it by this gear's mesh phase as a rigid motion, and asserts that
the body's own azimuth advanced by exactly that phase and that its volume is unchanged, so the
rotation moves the body rather than reshaping it. It checks the driving gear's phase is half a tooth
pitch, and that the pinion's is zero — the case where a move must not be built at all.

**From:** `spec/bevelgear/instructions.md` L317-388, L689-750;
`.claude/skills/generate-gear/PLAYBOOK.md` L782-790, L791-801.

## S28 `[PROSE]` Move the finished bodies into the per-gear components

Relocate this gear's finished body into the `{gearLabel} Gear` component created in S10:
`body.moveToComponent(<that gear's occurrence>)`. `moveToComponent` preserves world position and
needs no activation (`[PB-NO-CROSS-SIBLING]`), which is what lets every feature run in the one Design
component and the bodies end up where the browser shows them.

This step relocates a body without changing its geometry, so no proof function realises it.

**From:** `spec/bevelgear/instructions.md` L689-750;
`.claude/skills/generate-gear/PLAYBOOK.md` L802-840.

## S29 `[PROSE]` Cleanup

Call the framework helper `hide_construction_geometry(self.bevelComponent)` from `.solids`. It
recursively walks the Bevel Gear component tree, deduping by `entityToken`, and hides every sketch,
construction plane and construction axis by setting `isLightBulbOn = False`
(`[BEVEL-F-CLEANUP]`, `[PB-TREE-CLEANUP]`). Do not re-implement the walk.

Use the right property: `isVisible = False` hides **sketches**, while a `ConstructionPlane` or
`ConstructionAxis` is **not** hidden by `isVisible` and needs `isLightBulbOn = False`
(`[PB-HIDE-AFTER-USE]`). Hide only after every feature that consumes the geometry has run. There is
no sketch-only mode and no per-mode guard: bevel always builds solids.

Leave only the two finished gear bodies visible.

The driving gear's half-tooth-pitch meshing rotation is performed earlier, in S27, in the Design
component before the body is moved out; it is not a cleanup step.

This step changes visibility and no geometry, so no proof function realises it.

**From:** `spec/bevelgear/instructions.md` L751-756; `spec/bevelgear/fusion.md` L153-166;
`.claude/skills/generate-gear/PLAYBOOK.md` L650-662, L802-840.
