# Bevel Gear — compiled step list

The proof for this step list is `proof/bevelgear/geometry_test.go`, `proof/bevelgear/cases_test.go`,
`proof/bevelgear/sketches_test.go`, `proof/bevelgear/solids_test.go`,
`proof/bevelgear/spiral_test.go` and the generated `proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `692d89c0d5be5ec751731b3bedfd41400b4795f4` |
| `spec/bevelgear/fusion.md` | `754729fc9a33ad9dbf93816e0430e4b94d82e05b` |
| `spec/bevelgear/spiral-tooth-trace.md` | `c9ec08561ced7975aa0ed9ad6a330186259c0d08` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9ee2dcbaed7b5480aa69e9295e8b61acaea081f3` |

## S1 `[PROSE]` Module layout, imports and the two classes

Write `lib/geargen/bevelgear.py`. Import explicitly — **no `import *` in a gear module**:

```python
import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import get_boolean, get_selection
from .utilities import find_profile_by_curve_counts
from . import solids
from .spurproxy import VirtualSpurProxy
from .spurgear import SpurGearInvoluteToothDesignGenerator
```

Bevel uses a **standalone generator**. It does **not** subclass `base.Generator`, carries **no
`GenerationContext`**, and registers **no Fusion user parameters**: every value is precomputed in
Python in internal cm and written into geometry numerically (`[PB-PRECOMPUTED-MODE]`). From
`base.py` take only the input readers; the `Generator` / `ParamNamePrefix` / `ComponentCleaner`
machinery is unused.

Two classes, both bound **by name** from `commands/bevelgear/entry.py`:

1. `BevelGearCommandInputsConfigurator` — `@classmethod configure(cls, cmd)`,
   `@classmethod handle_input_changed(cls, args)`, and the private helper
   `@classmethod _updateSpiralInputVisibility(cls, inputs)`.
2. `BevelGearGenerator` — `__init__(self, design)` storing `self.design` and
   `self.bevelOccurrence = None`; `generate(inputs)`; `deleteComponent()`.

The virtual-spur proxy is **imported from the framework**; bevel defines no local proxy or
value-wrapper class.

State is threaded three ways and there is no context object: the input reader returns a 7-tuple
`(parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth, shaftAngle_deg)`
and stashes the rest on `self` (`self._drivingBaseHeight_cm`, `self._pinionBaseHeight_cm`,
`self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`, `self._faceWidth_cm`,
`self._toothSpacing_cm`, `self._spiralAngle_rad`, `self._hand`, `self._cutterRadius_cm`);
`generate()` later stashes `self._coneDistance_cm`, `self._gamma_p`, `self._gamma_g` and
`self._faceWidthResolved_cm`; the per-gear anchors live in **plain per-gear dicts** `pinionCtx` and
`drivingCtx`; and `self.bevelOccurrence`, `self.designOccurrence`, `self.designComponent`,
`self.bevelComponent` hold the tree. Do not introduce a `GenerationContext`-style class.

Every name in the class surface above is a MENTION rather than a call this module makes. The two
class names are the names the entry point binds to, and `generate`, `configure`,
`handle_input_changed`, `_updateSpiralInputVisibility` and `deleteComponent` are methods this module
**defines for the framework to call**: `commands/_gear_command.py` calls `generate(inputs)` inside
its execute handler and `deleteComponent()` on an exception, and the dialog's own handlers call the
two configurator methods. Nothing in the generated module calls any of them, so none of them is a
requirement, and each is exempted below.

<!-- check-step-calls: ignore BevelGearCommandInputsConfigurator BevelGearGenerator generate configure handle_input_changed _updateSpiralInputVisibility deleteComponent -->
<!-- check-compile: ignore handle_input_changed -->

**From:** `instructions.md` L267-292, `instructions.md` L294-315, `instructions.md` L423-457,
`.claude/skills/generate-gear/PLAYBOOK.md` L17-36, `.claude/skills/generate-gear/PLAYBOOK.md` L850-861

## S2 `[PROSE]` Add the 20 dialog inputs in display order

`configure(cls, cmd)` adds the inputs below to `cmd.commandInputs`, **in this order**. Target Plane
comes first so it wins Fusion's auto-focus (`[PB-AUTOFOCUS-FIRST]`), Center Point second so the user
flows from plane to point, and the pre-selected Parent Component third.

Reproduce every id, label, unit string, default and tooltip verbatim.

| # | Dialog label | input id | input type | unit | default | selection filters / tooltip |
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
| 16 | Hand of Spiral | `spiralHand` | `addDropDownCommandInput` (text list) | — | items `Right` (selected), `Left` | — |
| 17 | Cutter Radius | `cutterRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 18 | Toe Extension (%) | `toeExtension` | `addValueInput` | `''` | `createByReal(0)` | — |
| 19 | Driving Gear Toe Radius | `drivingToeRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 20 | Pinion Gear Toe Radius | `pinionToeRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |

There are **20 `INPUT_ID_*` module constants**, holding the table's id strings in row order, named
exactly: `INPUT_ID_PLANE`, `INPUT_ID_CENTER_POINT`, `INPUT_ID_PARENT`, `INPUT_ID_MODULE`,
`INPUT_ID_SHAFT_ANGLE`, `INPUT_ID_DRIVING_TEETH`, `INPUT_ID_PINION_TEETH`,
`INPUT_ID_DRIVING_BASE_HEIGHT`, `INPUT_ID_PINION_BASE_HEIGHT`, `INPUT_ID_BORE_ENABLE`,
`INPUT_ID_DRIVING_BORE`, `INPUT_ID_PINION_BORE`, `INPUT_ID_FACE_WIDTH`, `INPUT_ID_TOOTH_SPACING`,
`INPUT_ID_SPIRAL_ANGLE`, `INPUT_ID_HAND`, `INPUT_ID_CUTTER_RADIUS`, `INPUT_ID_TOE_EXTENSION`,
`INPUT_ID_DRIVING_TOE_RADIUS`, `INPUT_ID_PINION_TOE_RADIUS`. Two more module constants carry the
dropdown's list-item strings: `_HAND_RIGHT = 'Right'` and `_HAND_LEFT = 'Left'`. There are no
`PARAM_*` strings, because bevel registers no user parameters.

Selection inputs are added with `addSelectionInput(id, label, tooltip)`, then
`addSelectionFilter(...)` per the table and `setSelectionLimits(1, 1)`. Write each filter as the
named constant `adsk.core.SelectionCommandInput.ConstructionPlanes` and never as a quoted literal
(`[PB-SELECTION-FILTER-ENUM]`); the filter set and the limits are contract surface
(`[PB-SELECTION-DECL]`). The Parent selection pre-selects `get_design().rootComponent`.

The `mm`/`deg` defaults are passed in Fusion INTERNAL units (`[PB-DIALOG-DEFAULT-UNITS]`): a length
default goes through `to_cm(...)` and the Shaft Angle default is written as
`createByString('90 deg')` so the expression engine parses it.

The Hand dropdown is added with
`addDropDownCommandInput(INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)`
and its two items with `listItems.add(_HAND_RIGHT, True)` and `listItems.add(_HAND_LEFT, False)`.

`configure()` calls `cls._updateSpiralInputVisibility(inputs)` as its **last step**, so the initial
state is correct for the default ψ = 35°.

<!-- check-step-calls: ignore configure _updateSpiralInputVisibility -->

**From:** `instructions.md` L25-34, `instructions.md` L145-188,
`instructions.md` L212-217, `.claude/skills/generate-gear/PLAYBOOK.md` L128-143,
`.claude/skills/generate-gear/PLAYBOOK.md` L355-357, `.claude/skills/generate-gear/PLAYBOOK.md` L557-568

## S3 `[PROSE]` Show the spiral-only inputs only when ψ > 0

Hand of Spiral and Cutter Radius are relevant only for curved bevels, so they are **hidden whenever
Mean Spiral Angle ψ = 0 and shown when ψ > 0**. Mean Spiral Angle itself is the controller and is
always visible. There is no declarative show-if in the Fusion API; this is realized with
`isVisible`.

`_updateSpiralInputVisibility(cls, inputs)`:

- reads the `spiralAngle` input's **`.expression`** and evaluates it with
  `evaluateExpression(spiral.expression, 'rad')` — internal **radians**, and NOT the input's
  `.value`;
- sets `inputs.itemById(INPUT_ID_HAND).isVisible` and
  `inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible` to `(value > 0)`;
- **guards**: if any of the three inputs is `None`, return early; wrap the expression evaluation in
  `try/except`, because a half-typed expression can raise mid-edit, and on failure leave both
  inputs **shown**.

`handle_input_changed(cls, args)` is one line: it calls
`cls._updateSpiralInputVisibility(args.inputs)`, recomputing on **every** input change. It is bound
by name from `commands/bevelgear/entry.py` as the dialog's `inputChanged` handler.

`isVisible` only hides the dialog row. The input still exists and is read normally, and the ψ = 0
build ignores Hand and Cutter anyway, so hiding is purely cosmetic and cannot affect generation.

<!-- check-step-calls: ignore _updateSpiralInputVisibility handle_input_changed -->
<!-- check-compile: ignore handle_input_changed -->

**From:** `instructions.md` L190-211, `instructions.md` L276-280

## S4 `[PROSE]` Read every input in one pass, in internal units

Read all inputs first, before anything creates an occurrence. Because bevel registers no user
parameters nothing creates an occurrence until every selection is already read, so the
selection-context-shift hazard does not bite here — but keep the order so it stays that way.

Read each numeric and angle input by evaluating its expression with
`evaluateExpression(expr, units)` using `''` / `'mm'` / `'deg'` as appropriate. The values come back
in Fusion **internal units — cm for length, radians for angle — regardless of the unit string**
(`[PB-EVAL-EXPRESSION]`). Do not read via `realValue`.

Selections are read with `get_selection`; the Enable Bore checkbox with `get_boolean`. The Hand
dropdown is read as `itemById(INPUT_ID_HAND).selectedItem` and then `.name`, defaulting to
`_HAND_RIGHT` when there is no selection.

**Units — critical.**

- The `mm` inputs (both base heights, both bore diameters, Face Width, Tooth Spacing, both toe
  radii) and the two `deg` inputs (Shaft Angle, Mean Spiral Angle) come back **already in internal
  units**. Use them as-is; do **not** `to_cm` them again.
- **`Module` is read with unit `''`, so it comes back as a raw number that means millimetres.**
  Every length derived from Module must therefore be `to_cm`-converted before it touches geometry:
  the two pitch diameters (`to_cm(Module * teeth)`), the Cone Distance, the dedendum
  `to_cm(1.25 * Module)`, the module-length construction extensions, and the default Face Width.
  Mixing a raw-mm Module-derived length with an already-cm `mm` input makes the gear come out about
  ten times off and the Face-Width bound meaningless.
- `toeExtension` is a plain unitless percentage and needs no conversion.
- Both teeth inputs are coerced to whole numbers with `int(round(...))` before validation.

Convert the Shaft Angle and the Mean Spiral Angle to degrees with `math.degrees(...)` before any
degree-range check.

**From:** `instructions.md` L182-188, `instructions.md` L227-252,
`instructions.md` L411-421, `.claude/skills/generate-gear/PLAYBOOK.md` L863-867

## S5 `[PROSE]` Validate the ranges and resolve every bound

All of this happens during input validation, before any geometry exists. Reject with a message that
names the computed limit.

**Simple ranges.** Module > 0. Both tooth counts ≥ 3 (the blanket absolute floor). Base heights,
bore diameters, Face Width, Tooth Spacing and both toe radii non-negative. Cutter Radius
non-negative. Mean Spiral Angle in **[0, 60)** degrees. Toe Extension in **[0, 100]**.

**Pitch diameters and the Cone Distance.** `Driving Gear Pitch Diameter = Module * Driving Gear
Teeth Number`, `Pinion Gear Pitch Diameter = Module * Pinion Gear Teeth Number`, and
`Cone Distance = sqrt((Module * Driving Gear Teeth Number)**2 + (Module * Pinion Gear Teeth
Number)**2)`. The Cone Distance depends on the two tooth counts only and never on the Shaft Angle.
It is the **diagonal of the two pitch diameters** and is a different length from the Pitch Cone
Distance `R` below; the two coincide as `Cone Distance = 2 * R` exactly at Shaft Angle 90° and
diverge everywhere else.

**Shaft Angle.** At least 30° and at most the Maximum Shaft Angle. 30° is the documented geometric
floor; whether a given §2 lattice can reach it is a property of that lattice and not a validation
rule — see S10.

**Maximum Shaft Angle.** A pitch cone angle reaching 90° turns that gear's pitch cone inside out:
`R * cos γ` passes through zero and changes sign, so the along-shaft seed points backwards and the
back-cone virtual radius is unbounded. Both cone angles stay below 90° exactly while

    cos(Shaft Angle) > -min(Driving Gear Pitch Diameter, Pinion Gear Pitch Diameter)
                      / max(Driving Gear Pitch Diameter, Pinion Gear Pitch Diameter)

so the range check must reject a Shaft Angle **at or above** `degrees(acos(-smaller / larger))`,
naming the computed limit. **The Maximum Shaft Angle is that cone-angle limit capped at 150°**, and
the cone-angle half is **exclusive** while the 150° half is **inclusive**. A 31/17 pair gives
`acos(-17/31) = 123.26°`; equal tooth counts give `acos(-1) = 180°`, which is no constraint at all.
Check it **after** both tooth counts are read and coerced, since it depends on both.

**The two pitch cone angles**, from the closed form, once the counts and the Shaft Angle are known:

    tan γ_p = sin Σ * PPD / (DPD + PPD * cos Σ)
    γ_g     = Σ − γ_p
    R       = (PPD / 2) / sin γ_p          # the Pitch Cone Distance, always written as R

Then, **in this order**:

1. **Minimum Teeth.** Check each gear's count against `5.27 * cos γ` for that gear's own γ, on top
   of the blanket `teeth >= 3`, and name the computed floor. The constant is
   `2 * (1.05 * 1.25 / 0.95 + 1.25) = 5.2632`, rounded **UP** to 5.27 so the published floor stays
   at or above the exact crossing; do not round it down. At Shaft Angle 90° the floor is 3.72, i.e.
   4 teeth: an equal 4-tooth pair solves and a 3-tooth pair fails on the heel edge.
2. **The two base heights.** Both bounds are closed-form and need no solved geometry, because `r`,
   `γ` and Module are all known before §2 draws anything.

       Maximum Base Height = 0.95 * (r - 1.25 * Module * cos γ) * tan γ
       Minimum Base Height = 1.05 * 1.25 * Module * sin γ

   The maximum is measured from **Apex 2's plane** and is deliberately conservative: it sits
   `1.25 * Module * sin γ` below the true crossing `r * tan γ`, where H (resp. J) reaches the shaft
   axis and the revolve fails with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`). Do not adopt the exact
   `0.95 * r * tan γ` without re-running the low-tooth-count cases. The minimum is the dedendum's
   own along-shaft projection with the same style of margin: below it H lands *behind* C and the
   edge C->H runs back inward instead of outward.

   Apply both **per gear, in both directions**: raise a fallback below the minimum, cap a fallback
   above the maximum, and reject a **user** value outside either end naming the bound it broke.
   The driving fallback is `Module * Driving Gear Teeth Number / 8`. The pinion fallback is the
   **RESOLVED** driving base height — after its own fallback and its own cap — times
   `Pinion Gear Teeth Number / Driving Gear Teeth Number`, and then the **pinion's own** bounds are
   applied to that result, because the two gears have different pitch cone angles whenever the
   tooth counts differ.

   Order matters between the two checks: the Minimum Teeth check is exactly the statement that the
   base-height window is non-empty, so running it first means step 2 never has to describe what to
   do when the minimum exceeds the maximum.

**Bore diameters.** Only consulted when Enable Bore is checked. 0 means auto: use that gear's
`Pitch Diameter / 4`.

**Toe radii.** 0 means auto: that gear's own inner toe corner radius at Toe Extension 0,
`this gear's Pitch Radius - Face Width / sin γ`, which is the value that makes Toe Extension 0
today's profile exactly. A user value must be **strictly below** that gear's **Toe Radius Ceiling**,
`(this gear's Pitch Radius - 1.25 * Module * cos γ) * (1 - Face Width / R)`; reject it naming the
ceiling.

**A defaulted Toe Radius can leave no room at all, and that is a real configuration rather than a
defect.** On a gear with a large pitch cone angle the inner toe corner already sits at a LARGER
radius than the outer one, so the point X falls behind the toe corner and the Toe Limit comes out
below the Toe Extension 0 root length. **Reject a Toe Extension above 0 on such a pair**, naming the
gear and the Toe Radius Ceiling it needs to come below. Toe Extension 0 still resolves, so the gear
stays buildable exactly as before. Do **not** silently substitute a smaller Toe Radius: that would
change the toe end of a gear whose inputs asked for no change.

⚠️ **A configuration can satisfy every bound here and still be refused by the sketch solver as
near-singular.** That limit belongs to the particular §2 lattice, not to this spec. It is not a
validation rule, no bound here is derived from it, and a near-singular report is a real refusal of
that construction rather than a tolerance to loosen.

**From:** `instructions.md` L35-52, `instructions.md` L54-104,
`instructions.md` L106-138, `instructions.md` L124-136, `instructions.md` L246-265

## S6 `[PROSE]` Resolve the Face Width's default and the Root Length

Face Width is user-specified and positive. If unspecified, default to `Cone Distance / 6`. In
**every** case it is bounded by the Maximum Face Width, which cannot be evaluated until §2 has
solved A, B, C, D, H and J — so the cap itself is applied inside S10, and this step only resolves
what the cap is applied to:

- unspecified → `min(Cone Distance / 6, Maximum Face Width)`;
- user value above the Maximum Face Width → reject, naming the maximum, rather than proceeding.

The default `Cone Distance / 6` equals `R / 3`, the conventional face-width limit, **only at Shaft
Angle 90°**. Below 90° it is conservative; above 90° it exceeds `R / 3` and the cap is what actually
holds it. That is deliberate — it keeps the default independent of Shaft Angle — and it is the cap,
not this default, that guarantees a buildable profile.

**Toe Extension** is one percentage applied to **both** gears, because they share one face and must
mesh. It is read on the DRIVING gear and the pinion is built to the same root length. 0 reproduces
today's toe end exactly.

**Root Length** is the resolved `|Ded->Toe|`: the segment C->M on the pinion and D->O on the
driving gear. At Toe Extension 0 it is the resolved Face Width re-measured along the root element
rather than perpendicular to the pitch line, which is longer by the dedendum angle's cosine:

    |Apex->Ded| = sqrt(R**2 + (1.25 * Module)**2)
    Root Length at 0 = Face Width * |Apex->Ded| / R

Face Width still resolves exactly as it always did and still carries its cap; the Toe Extension adds
to what Face Width resolved and does not replace it.

**Toe Limit**, per gear, is `|Ded->X|` where X is the point on the root element `Apex->Ded` at that
gear's Toe Radius:

    γ_root    = γ - atan(1.25 * Module / R)
    Toe Limit = sqrt(R**2 + (1.25 * Module)**2) - Toe Radius / sin(γ_root)

**Toe Extension 100 stops at 0.99 of the way from the Toe Extension 0 root length to the smaller of
the two gears' Toe Limits, not at the Toe Limit itself.** The smaller limit wins because the pair
shares one root length. The 0.99 is there because AT the limit the toe face has zero length, the
revolved gear body carries **no cone at its toe end**, and the conical end-cut in S22 — whose toe
cut must split or the build fails — has no `ConeSurfaceType` face to find. The last percent is worth
well under a tenth of a millimetre of root length on every case in the proof's table, so the reach
given up is nil and the failure avoided is total. Do not drop this factor.

    Root Length = Root Length at 0
                + (Toe Extension / 100) * 0.99 * (min(Toe Limit_p, Toe Limit_g) - Root Length at 0)

**From:** `instructions.md` L106-122, `instructions.md` L124-136

## S7 `[PROSE]` Build the component tree

`generate(inputs)` reads every input first, then creates occurrences. Create each occurrence with
`addNewComponent(adsk.core.Matrix3D.create())` on the parent's `occurrences` collection and name it
through `occurrence.component.name` (`[PB-OCCURRENCE-TREE]`).

- `Bevel Gear` as a child of the user's Parent Component. Stash its occurrence on
  `self.bevelOccurrence` for rollback and its component on `self.bevelComponent`.
- `Design` as a child of the Bevel Gear component. It holds every sketch, construction plane and
  construction axis, and every feature operation runs in it. Stash `self.designOccurrence` and
  `self.designComponent`.

**Never call `activate()` on any occurrence** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`).
The Anchor Sketch is created on the user's **external**, root-owned target plane, and an activated
occurrence resolves that external plane in its own local frame, so the build collapses onto world XY
regardless of the real plane tilt. The sole exception is the spiral crown's scale feature in S26,
which activates the Design occurrence and restores the root in a `finally`.

All features run in the one Design component, so no cross-sibling reference is ever needed
(`[PB-NO-CROSS-SIBLING]`).

`deleteComponent()` is the error rollback the entry point calls on an exception: it calls
`deleteMe()` on `self.bevelOccurrence`.

<!-- check-step-calls: ignore deleteComponent -->

**From:** `instructions.md` L19-24, `instructions.md` L459-469,
`instructions.md` L411-421, `fusion.md` L153-160,
`.claude/skills/generate-gear/PLAYBOOK.md` L811-837

## S8 `[GO]` Anchor sketch

Proof function: `stepAnchorSketch`.

Start the Anchor Sketch **directly on the user-selected target plane**, whether the selection is a
`ConstructionPlane` or a `PlanarFace`: `sketches.add(targetPlane)` on the Design component. Do not
re-derive or offset it (`[PB-USE-SELECTED-PLANE]`) — re-deriving collapses the gear onto XY. Name
the sketch `Anchor`.

Mark the centre by projecting the user-specified centre point into the sketch with
`sketch.project(centerPoint)`.

Draw a line through the projected centre with `addByTwoPoints`. **Seed its two endpoints at exactly
±0.5 cm from the projected centre** along the sketch-local X, so the seeded length is 10 mm. Then:

- `addCoincident(projectedCenter, anchorLine)` — the intersection, which pins the centre onto the
  line;
- `addMidPoint(projectedCenter, anchorLine)` — the centre bisects the line. **Use both, not midpoint
  alone.**
- an **aligned** distance dimension with `addDistanceDimension(startPoint, endPoint,
  adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)` and **without**
  assigning `.parameter.value`: the dimension simply locks the length at the seeded 10 mm. The
  value is arbitrary; nothing downstream reads it.
- `addHorizontal(anchorLine)` — sketch-local, per `[PB-REFLINE-DIRECTION]`. The line's absolute
  direction is arbitrary, since §2 derives every direction relative to it, but it must not be a free
  degree of freedom. A world-axis lock would mis-orient the figure on a tilted target plane.

With midpoint, length and Horizontal the line has zero DOF. **Stash the projected-centre
`SketchPoint`** on `self._anchorCenterPoint`, so §2 re-projects *this* point and not the raw
user-selected centre. This line is the Anchor Line.

Gate the sketch at the end of the step: raise, naming the sketch, if `isFullyConstrained` is false
(`[PB-FULL-CONSTRAINT]`, `[BEVEL-F-FULL-CONSTRAINT]`). A free DOF is a generation defect, not a
warning.

The proof's substitution: Fusion needs both the coincident and the midpoint, and the spec is
explicit that midpoint alone is not enough; this sketch engine's midpoint carries both rows, so the
point-on-line row is redundant there and is left out. It also states the 10 mm as a **signed**
horizontal distance, because an aligned distance is a magnitude whose direction is captured from
the seed (`[PB-DIM-VALUE-SEMANTICS]`) and a net that admits the swapped endpoints is one the
harness refuses.

<!-- proof-run: proofkit.RunParallel(anchorCases, stepAnchorSketch) -->

**From:** `instructions.md` L471-475, `instructions.md` L389-397,
`fusion.md` L19-30, `.claude/skills/generate-gear/PLAYBOOK.md` L441-450,
`.claude/skills/generate-gear/PLAYBOOK.md` L239-251, `.claude/skills/generate-gear/PLAYBOOK.md` L838-848

## S9 `[PROSE]` Gear Profiles Plane

Create a construction plane that includes the Anchor Line, set at 90° so it stands perpendicular to
the anchor line's own plane: `constructionPlanes.createInput()` then
`setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)`.

**Build it off the original `targetPlane`** as the reference — do not re-derive or offset it
(`[PB-USE-SELECTED-PLANE]`). This is the other place the target-plane orientation reaches the
bodies; substituting a different plane here also collapses the gear onto XY.

Pass the `SketchLine` **directly** to `setByAngle`; never wrap it in `Path.create` first
(`[PB-CONSTRUCTION-PLANES]`).

Name the plane `Gear Profiles Plane` and stash it on `self._gearProfilesPlane`.

<!-- check-step-calls: ignore Path.create -->

**From:** `instructions.md` L477-479, `.claude/skills/generate-gear/PLAYBOOK.md` L775-786

## S10 `[GO]` Gear Profiles sketch — the §2 lattice

Proof function: `stepGearProfiles`.

Create a sketch on the Gear Profiles Plane named `Gear Profiles`, and stash it on `self._gpSketch`.
Everything below is drawn in that one sketch, and the whole sketch is one step.

**Every line drawn in this sketch is a construction line** — `isConstruction = True` for the lattice
lines, the toe lines M->N and O->P, and the short reference and connector lines M->C, N->A', O->D,
P->B', A'->G, B'->I, C->K / C->K′, D->L / D->L′ alike. The solid features later consume only the
per-gear Profile sketches, never a §2 curve directly.

**Every length dimension in this sketch is `AlignedDimensionOrientation`.**
`addDistanceDimension(pointOne, pointTwo, orientation, textPoint)` takes an
`adsk.fusion.DimensionOrientations` value, and this figure has no axis-aligned line in it: the shaft
axes sit at the Shaft Angle to each other and the whole lattice tilts with the target plane. A
horizontal or vertical orientation would dimension the line's *projection* onto a sketch axis
instead of its length. The offset dimensions are a different call, `addOffsetDimension`, which takes
no orientation.

**Build every line in the COINCIDENT style** (`[BEVEL-F-COINCIDENT-STYLE]`): create it from raw
`Point3D` coordinates with `addByTwoPoints` and pin each end that connects to an existing point with
exactly one `addCoincident(line.endpoint, existingPoint)`. Never pass an existing `SketchPoint` into
`addByTwoPoints` to share it. This covers the short reference and connector lines too, the ones
whose BOTH endpoints already exist. Sharing without a coincident leaves the sketch
under-constrained; sharing **and** coinciding is redundant and the solve fails outright with
`VCS_SKETCH_SOLVING_FAILED`. **Each named line is created ONCE** and later references reuse that
line object (`[BEVEL-F-LINE-ONCE]`); a duplicate line over the same segment carries its own
constraints and over-determines the net.

**The §2 driven lengths are NOT dimensioned** (`[BEVEL-F-DRIVEN-DIMS]`, `[PB-NO-OVERCONSTRAIN]`):
Apex->A, Apex->B and every module-length extension are determined by the perpendicular, collinear
and closing constraints.

### The projections

Project **the Anchor Sketch's centre `SketchPoint`** — the one stashed in S8 — with
`sketch.project(self._anchorCenterPoint)`, and project the Anchor Line the same way. Both happen to
be coincident with the raw user selection, but projecting the anchor-sketch point keeps the chain
inside the Design component; projecting the raw external point is a cross-component reference and
can resolve inconsistently.

**Write the call as `sketch.project(entity)`, and do not substitute `project2`.** The compiled
Fusion API reference declares `project2(entities, isLinked)` and no `project`, so every gate in this
repo reports the call as unverified; that report is expected and is not a defect to fix here.
`project` is what the shipped add-ins call and what the spur step list names, and this repo's
settled position is to keep it and keep reporting it. The two are not interchangeable in any case:
`project2` takes a list and returns a list.

<!-- check-step-calls: ignore project2 addVertical addCollinear -->

### The apex and the two shaft axes

Let `c` be the projected centre and `d` the projected anchor line's 2-D unit direction. The in-plane
perpendicular is `perp = (-d.y, d.x)`, and **the sign of `perp` is chosen by the target-plane
normal**, read as `targetPlane.geometry.normal` for BOTH selection kinds — a `BRepFace`'s
`geometry` and a `ConstructionPlane`'s `geometry` are each a `core.Plane` carrying `.normal`. That
is a one-bit direction comparison and the only permitted world use in §2
(`[BEVEL-F-GROW-SIDE]`); a sketch-local rule such as `perp.y >= 0` is deterministic but not tied to
a physical side.

Draw a construction line from the projected centre, apply `addPerpendicular` against the projected
anchor line, and place its far end — the **Apex** — in sketch-local coordinates at

    c + perp * (R * cos γ_g + <resolved Driving Gear Base Height>)

**Seed it at that distance and not at the Driving Gear Pitch Diameter.** The constraint net closes
this line at `R·cos γ_g` above point I plus the resolved driving base height, so for the default
31/31 pair at Shaft Angle 90° the old seed sat 11.6 mm past where the solve puts it, and a seed that
disagrees with its own closure by that margin is a seed waiting to pick the wrong branch
(`[PB-SEED-NEAR]`). The apex **position** is sketch-local: do **NOT** compute it from a
world-coordinate round trip, which is what caused the XY collapse (`[BEVEL-F-APEX-LOCAL]`). Do not
add a length constraint on this line.

From the apex, draw the **Driving Gear Shaft Axis** back toward the anchor line, in the `-perp`
direction. **Seed its far end at `apex - perp * (R * cos γ_g)`, which is
`c + perp * <resolved Driving Gear Base Height>`** — measure from the apex, not from `c`. Pin its
start to the apex with `addCoincident`, and make it parallel to the centre->apex line with
`addParallel(drivingShaftAxis, centerToApex)`. **Do NOT use `addVertical`**: that forces the line to
the sketch's world-vertical, which is wrong on a tilted target plane and mis-orients the figure. Its
end is point **B**, and its length is not dimensioned.

From the apex, draw the **Pinion Gear Shaft Axis**: the driving direction rotated about the apex by
the Shaft Angle. Rotating has two senses and they put point A on opposite sides, and choosing wrong
mirrors the whole gear onto the wrong side of the target plane. **Select the sense this way: form
both candidate point-A positions — the driving direction rotated by +Shaft Angle and by −Shaft
Angle — and keep the candidate whose endpoint has the greater X coordinate in this sketch.** Compare
the two and take the larger; do **not** rotate one fixed sense and flip only when its X comes out
negative, because when *both* candidates have a positive X that shortcut keeps the wrong one. Pin
the start to the apex with `addCoincident`; its end is point **A**, and its length is not
dimensioned.

Apply `addAngularDimension(pinionShaftAxis, drivingShaftAxis, textPoint)` set to the Shaft Angle.
**Place the text point inside the Σ wedge so it measures Σ and not its supplement**
(`[PB-ANGULAR-DIM]`): use the interior bisector,
`apex + normalize(pinionDir + drivingDir) * (PPD / 4)`. The angular dimension fixes the angle's
magnitude only; the pinion side is held by the seed above together with the Apex 2 closure below.

### The two drops and Apex 2

From **A**, draw a construction line perpendicular to the Pinion Gear Shaft Axis. ⚠️ **Apex 2 sits
in the interior wedge BETWEEN the two shaft axes, so this drop must point toward the OTHER shaft
axis, toward point B** — pick the perpendicular sense by the sign of its dot product with the A->B
direction, not against a generic "toward the anchor line" reference. Pin its start to A with
`addCoincident`, apply `addPerpendicular` against the Pinion Gear Shaft Axis, and dimension it with
an aligned distance of **Pinion Gear Pitch Diameter / 2**.

From **B**, draw the twin perpendicular to the Driving Gear Shaft Axis, pointing toward point A —
pick the sense by the sign of its dot product with the B->A direction. ⚠️ **Do NOT choose this sense
by a "toward the anchor line" reference:** the Driving Gear Shaft Axis is itself parallel to that
direction, so the perpendicular's dot with it is ≈ 0, a degenerate test that silently selects an
arbitrary and usually wrong side. If this drop seeds Apex 2 on the wrong side while the pinion's
drop seeds it on the correct one, the coincidence that closes the two makes the solver **flip the
entire frame to the mirror solution**: point A jumps to the opposite side, the pinion dedendum C
collapses onto the driving dedendum D, the revolved frustum is degenerate, and the conical end-cut
finds no cone face at the toe midpoint. Pin its start to B, apply `addPerpendicular`, and dimension
it with an aligned distance of **Driving Gear Pitch Diameter / 2**.

`addCoincident` the two drops' far endpoints. That point is **Apex 2**. At Shaft Angle 90° the four
points Apex, A, Apex 2, B form a rectangle; for other shaft angles the figure is a non-rectangular
quadrilateral and the lengths of Apex->A and Apex->B adjust so the two drops coincide.

**Seed the along-shaft lengths with the closed-form cone geometry** so the solver converges on the
right branch for any Σ: `|Apex->A| = R * cos γ_p` and `|Apex->B| = R * cos γ_g`, with γ_p, γ_g and R
from S5. Both cosines are positive for every Shaft Angle the range check admits, which is what the
Maximum Shaft Angle is there to guarantee. Seeding A or B merely by a pitch diameter is wrong for
Σ ≠ 90° and can send the solver to the wrong branch.

The quadrilateral deliberately lies well above the anchor line; the apex's offset keeps the whole
figure above that line across the supported Shaft Angle range.

### The pitch line, the dedendum lines and the root axes

Draw the **Pitch Line** from Apex to Apex 2, coincident at both ends.

From Apex 2, draw two construction lines perpendicular to the Pitch Line
(`addPerpendicular` against it), each with an aligned distance dimension of `Module * 1.25`. The one
drawn **toward** the anchor line is the **Driving Gear Dedendum**, ending at point **D**; the one
drawn **away** from it is the **Pinion Gear Dedendum**, ending at point **C**.

Draw the two **Root Axes**, Apex->D and Apex->C, coincident at both ends.

### The two dedendum chains

Per gear, with the pinion's names given first:

- From **A** (B), draw a construction line collinear with the shaft axis, extending for a length
  equal to Module — a **seed only**, with no dimensional constraint. Coincident its start to A (B)
  and apply `addCollinear` against the **Apex->A** (Apex->B) shaft axis. Its end is **E** (F).
- Draw a line from **C** (D) to **E** (F), coincident at both ends, and constrain **A->E and C->E**
  (B->F and D->F) with `addPerpendicular`. This is what puts E (F) at the dedendum corner's own
  station on the shaft axis.
- From **E** (F), draw a line collinear with **line A->E** (B->F) — the collinear names A->E,
  **never the Apex->A shaft axis further up the chain**, even though both describe the same infinite
  line (`[BEVEL-F-COLLINEAR-CHAIN]`, `[PB-COLLINEAR-CHAIN]`; naming the axis raises
  `VCS_SKETCH_OVER_CONSTRAINTS`). Length equal to Module as a seed, no dimension. Coincident E (F)
  and the new line's start. Its end is **G** (I).
- From **C** (D), draw a line of seed length Module, coincident at C (D), collinear with **line
  Apex2->C** (Apex2->D), the dedendum line C (D) is the endpoint of. Its end is **H** (J).
- Connect **G and H** (I and J) with a line, coincident at both ends, and **constrain E->G and H->G
  (F->I and J->I) with `addPerpendicular`**.

⚠️ **Those two perpendiculars are required in Fusion and are omitted in the proof harness, and the
reason is a difference between the two engines rather than a choice.** `addOffsetDimension` in
Fusion is a *distance* dimension whose documentation requires the second entity to be a line
parallel to the first, and it controls only the perpendicular distance. So the parallelism has to
exist before the base-height offset below can be applied at all, and this perpendicular is what
supplies it: E->G runs along the pinion shaft, so making H->G perpendicular to it makes H->G
parallel to the A->Apex2 drop. Perpendicular plus offset is two equations for two freedoms and
nothing is redundant. The proof harness's offset constraint emits **two** residual rows, holding
both endpoints of the target line at the same signed perpendicular distance from the source, so it
carries the parallelism itself; adding the perpendicular there is a third row for the same two
freedoms and the lattice comes back overconstrained at DOF 0 with the two base-height offsets named
as the redundant pair. The right response is to leave the perpendicular out of the proof and say so,
never to weaken the gate.

### The two base-height offsets

Create an **offset dimension between the B->Apex2 perpendicular drop line — the DPD/2 drop, NOT the
Apex->B shaft axis — and J->I**: `addOffsetDimension(dropB, lineJI, textPoint)` and then
`.parameter.value = <resolved Driving Gear Base Height>`. J->I is **already parallel** to the drop
by construction, since J->I ⟂ F->I and F->I runs along the driving shaft, so add **no** extra
`addParallel` (`[PB-OFFSET-DIM]`). The value is the resolved driving base height from S5 — after the
fallback and after the cap — because the offset set here is what drives the heel edge D->J toward
the shaft axis.

Create the twin **offset dimension between the A->Apex2 drop — the PPD/2 drop, not the Apex->A shaft
axis — and G->H**, already parallel for the same reason, set to the resolved pinion base height.

### Closing the figure

`addCoincident(pointI, projectedCenter)`. This is what closes the whole lattice: I is on B->F, which
is collinear with the driving shaft axis, which passes through the projected centre by construction,
so the coincidence pins the one translation the figure still has.

The proof states the independent row of that coincidence — I on the projected anchor line — because
one of its two rows is already implied and the sketch engine reports the implied one as redundant
where Fusion absorbs it. It then asserts that the solved I IS the projected centre.

### K, L and the two tooth centres

Draw a construction line away from the Apex, starting from **G** (I), extending along Apex->A
(Apex->B), and call its end **K** (L). Then **pin K with two point-on-line coincident constraints** —
`addCoincident(K, line Apex->A)` and `addCoincident(K, the Pinion Dedendum line Apex2->C extended)` —
rather than `addCollinear` on the connecting lines. By the time K is added, G and C are already
fixed, so an `addCollinear` here over-constrains the sketch and Fusion errors; the two point-on-line
coincidents locate K exactly, at the intersection of the two lines, without over-constraining
(`[BEVEL-F-COLLINEAR-CHAIN]`). Draw a construction line from C (D) to K (L) for reference.

**Tooth-centre point K′ (L′) — the Tooth Spacing offset.** The §3 tooth is centred not at K but at
K′, obtained by shifting K outward along the dedendum line by **Tooth Spacing**, *away from the
lower corner C* (D).

- **When Tooth Spacing is 0 — the default — do NOT build anything here.** Set K′ ≡ K and reuse the
  existing C->K reference line: a zero-length dimensioned line would be degenerate, and one segment
  gets one line (`[BEVEL-F-LINE-ONCE]`).
- When Tooth Spacing > 0: draw a construction line starting at K with its far end seeded on the far
  side of K from C along the dedendum direction; pin its start with `addCoincident` to K and its far
  end with `addCoincident(K′, the Pinion Dedendum line Apex2->C extended)` — the same way K is
  pinned to its line, and **not** `addCollinear`, for the same over-constraint reason. Add an
  aligned length dimension on this line equal to Tooth Spacing. The far end is K′. Then draw the
  **tooth-centre reference line C->K′** for §3 to use in place of C->K.

Build this **here, inside this sketch, before the end-of-step full-constraint gate**, so the gate
covers it. Only the tooth's centre moves; the virtual tooth number and the drawn tooth size are
unchanged.

### The Maximum Face Width, from solved geometry

At this point A, B, C, D, H and J all exist **and are solved**, so resolve the **Maximum Face
Width** here and apply it before the toe lines below: cap the auto default to it, and reject a user
value that exceeds it.

It is `0.95 *` the smaller of the perpendicular distance from **A** to the line through C and H, and
the perpendicular distance from **B** to the line through D and J.

**Compute both from the points' SOLVED sketch geometry — `pointA.geometry`, `pointB.geometry`,
`pointC.geometry`, `pointD.geometry`, `pointH.geometry`, `pointJ.geometry` — and NOT from the
pre-solve seed coordinates** (`[PB-SOLVED-GEOMETRY]`). The constraint network has located all six by
now, so `.geometry` is exact; seeds diverge substantially for asymmetric tooth counts or non-90°
shaft angles, making a seed-based bound too loose on the binding side, and the toe then still
crosses the axis.

The pinion is normally the binding side, its smaller pitch radius giving the smaller distance, but
compute both and take the minimum so the bound holds for any Shaft Angle. At Shaft Angle 90° this
limit equals `0.95 * min(DPD, PPD)**2 / (2 * Cone Distance)` — the SMALLER pitch diameter, never the
pinion's by name, since the pinion is only usually the smaller one. Written with the pinion's
diameter it is wrong whenever the driving gear carries the smaller tooth count: on a Driving 17 /
Pinion 31 pair at Module 1 the real bound is 3.883 mm and the pinion form gives 13.591, so the naive
`Cone Distance / 6` default exceeds it and the gear fails to generate for any gear ratio above
roughly √2.

The `0.95` keeps the inner toe corner clearly off the shaft axis, since a near-coincident corner
degenerates the toe edge even before it strictly crosses. A profile that has crossed its own axis of
revolution self-intersects that axis and Fusion aborts the revolve with `ASM_WIRE_X_AXIS`
(`[PB-REVOLVE]`).

### The two toe ends

Create line **M->N**. **Seed BOTH ends at their closed-form solved positions, not near them**
(`[PB-SEED-NEAR]`). Seed M on `Apex->C` at the fraction `1 - <Root Length> / |Apex->C|` from the
Apex. Then seed N by sliding from that M seed along the `C->H` direction by exactly

    (<M seed's perpendicular distance from the Pinion Gear Shaft Axis> - <Pinion Gear Toe Radius>)
    / cos γ_p

⚠️ **A seed that merely lands somewhere plausible is not enough here, and a wrong one builds the
wrong gear rather than failing to converge.** N's position is fixed by the toe line together with a
LENGTH dimension on the front face, and a length is unsigned: the toe line meets the Toe Radius on
BOTH sides of the shaft axis, so the solver takes whichever side the seed starts on. Seeded below
the axis it converges happily onto the mirror, N comes out on the far side, and the revolved hexagon
crosses its own axis of revolution — Fusion then aborts the revolve with `ASM_WIRE_X_AXIS` at S19,
pointing at the revolve rather than at the seed that caused it.

Two earlier seeding rules are known to do exactly that, so do not reinstate either: sliding from the
M seed by the **Root Length**, and sliding by the **distance from the M seed to A**. Measured on the
shipped default pair, module 1 with 31/31 teeth at Shaft Angle 90° and a Toe Extension of 50%, the
Root Length slide puts the N seed at a perpendicular distance of **−0.27 mm** from the shaft axis —
past it — against a solved N at **+5.17 mm**. The slide above puts it at 5.17 mm exactly. Do NOT
seed M and N just `Face Width` away from C and H either; that starts N near H, far from its
constraint target.

**The proof cannot catch a wrong seed here, and says so beside its own seeding.** It seeds M and N
at the closed form, which is the rule above, so it proves that the constraints solve from a correct
seed and never that this module's seed is correct. A seed defect therefore reaches Fusion untested,
which is how the one described above got there.

Then apply **exactly these three constraints** — all are required, and the front face below is what
holds N off the shaft axis:

- `addCoincident(M, Pinion Root Axis)` — M lies on the Apex->C root axis;
- `addParallel(lineMN, lineCH)` — the toe line is parallel to C->H;
- `addOffsetDimension(lineCH, lineMN, textPoint)` with
  `.parameter.value = <Root Length * R / |Apex->C|>` — the Root Length re-measured perpendicular to
  the pitch line, because an offset dimension controls a perpendicular distance. At Toe Extension 0
  that value is exactly the resolved Face Width, which is what this dimension has always been. Place
  the `textPoint` in the gap between C->H and M->N on the Apex side, at `(M_seed + C) / 2`, so the
  dimension reads cleanly (`[PB-OFFSET-DIM]`).

Let the line's beginning be **M** and its end **N**. Draw a line from M to C.

**The front face N->A', which is what holds N.** ⚠️ **N is NOT pinned to line A->Apex2.** It rides
the Pinion Gear Toe Radius instead, and the line that holds it there is the gear's front face:

- draw a line from N to a new point **A'**, seeding A' at N's station on the shaft axis;
- `addCoincident(A', line Apex->A)` — A' lies on the Apex->A shaft axis. A' is the only toe-end
  point that touches that axis, and it is a *foot*, not a corner;
- `addPerpendicular(lineNA, line Apex->A)` — the front face stands square to the shaft, so the
  revolve sweeps it into a flat annulus;
- an **aligned** distance dimension on the whole line N->A' equal to the **resolved Pinion Gear Toe
  Radius**.

⚠️ **Pinning N itself to the Apex->A shaft axis remains forbidden** — that would put N *on the axis
of revolution*, and the later conical split fails with `ASM_API_FAILED` for asymmetric tooth counts
even though the symmetric 45° case happens to survive. A' sits on the axis; N never does, because
the Toe Radius is strictly positive. Those three rows plus the offset above and the coincident on M
fully constrain M, N and A': six freedoms, six constraints.

**A' replaces A as the hexagon's first vertex.** At Toe Extension 0 with a defaulted Toe Radius the
two coincide exactly, so nothing moves; a positive Toe Extension walks A' along the shaft axis
toward the Apex and the shaft edge grows by that much.

Draw a construction line away from the Apex, starting from **I**, extending along Apex->B, and call
its end **L**. Pin L the same way as K, with `addCoincident(L, line Apex->B)` and
`addCoincident(L, the Driving Dedendum line Apex2->D extended)`; do not use `addCollinear`. Draw a
construction line from D to L for reference, and build **L′** exactly as K′, substituting L for K, D
for C and the Driving Dedendum for the pinion's; the reference line for §3 is **D->L′**.

Create line **O->P**, the mirror of M->N on the driving side. Seed it the same way: O on `Apex->D` at
the fraction `1 - <Root Length> / |Apex->D|`, then P slid from that O seed along `D->J` by
`(<O seed's perpendicular distance from the Driving Gear Shaft Axis> - <Driving Gear Toe Radius>)
/ cos γ_g`. The ⚠️ above applies unchanged. Apply the same three constraints:
`addCoincident(O, Driving Root Axis)`, `addParallel(lineOP, lineDJ)`, and
`addOffsetDimension(lineDJ, lineOP, textPoint)` set to the same perpendicular root length, with the
text point at `(O_seed + D) / 2`. Let the beginning be **O** and the end **P**. Draw a line from O
to D.

Build the driving front face **P->B'** exactly as the pinion's N->A', substituting B for A, P for N
and the **Driving Gear Toe Radius**: the line P->B', `addCoincident(B', line Apex->B)`,
`addPerpendicular(linePB, line Apex->B)` and an aligned length dimension on P->B'. The same ⚠️
applies: P is never pinned to the Apex->B shaft axis, only B' touches it.

Draw the two hexagon shaft-axis edges, **A'->G** and **B'->I**, coincident at both ends. The spec's
prose lists A'->G before A' exists; it cannot be drawn there, and the ordering is a defect in the
spec rather than a choice made here.

Gate the sketch at the end of the step on `isFullyConstrained` and raise, naming the sketch
(`[BEVEL-F-FULL-CONSTRAINT]`).

The proof's remaining substitutions, each recorded beside the thing it cannot reach: the two toe
offsets drop their `addParallel` for the same arity reason the two base-height offsets drop their
perpendicular; every `addCollinear` becomes the single point-on-line row that is not already
implied, which makes the two readings of `[PB-COLLINEAR-CHAIN]` identical there and leaves Fusion
the only place that tells them apart; and every aligned length dimension is stated as the signed
axis-aligned distance the engine offers, with the magnitude the spec names asserted after the solve.
The proof's case table records the Shaft Angles this particular lattice cannot reach — 30°, 150° and
a 17/31 pair at 120° — as declared refusals rather than narrowing the range the spec states.

<!-- proof-run: proofkit.RunParallel(gearProfileCases, stepGearProfiles) -->

**From:** `instructions.md` L477-592, `instructions.md` L37-52,
`instructions.md` L116-122, `fusion.md` L69-115, `fusion.md` L117-151,
`.claude/skills/generate-gear/PLAYBOOK.md` L479-493, `.claude/skills/generate-gear/PLAYBOOK.md` L591-613,
`.claude/skills/generate-gear/PLAYBOOK.md` L637-651, `.claude/skills/generate-gear/PLAYBOOK.md` L716-723

## S11 `[PROSE]` Per-gear component

Run S11 through S30 **once per gear — pinion first, then driving** — with these substitutions:

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A' -> G -> H -> C -> M -> N -> A' | B' -> I -> J -> D -> O -> P -> B' |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A'->G | B'->I |
| toe cut edge | M->N | O->P |
| heel cut edge | C->H | D->J |
| tooth centre / reference line | K′ / C->K′ | L′ / D->L′ |
| pitch cone half-angle | γ_p | γ_g |
| teeth / bore / pitch-diameter inputs | Pinion Gear … | Driving Gear … |
| §2 shaft construction line (NOT usable as the axis) | Apex->A | Apex->B |

Create a new component as a child of the **Bevel Gear** component — the same component that owns
Design, and *not* the user's Parent Component — named `{gearLabel} Gear`, so `Pinion Gear` and
`Driving Gear`. The finished bodies for this gear end up there.

Fusion rejects cross-sibling sketch and project calls even when the target is activated or the
entities are wrapped in assembly-context proxies (`[PB-NO-CROSS-SIBLING]`), so the actual feature
operations all run in the Design component and the finished bodies are moved here at the end with
`moveToComponent`. The visible end state is identical.

Per-gear geometric anchors are carried in a plain dict — this gear's label, teeth, pitch diameter,
γ, tooth-centre point and reference line, hexagon vertices, shaft-edge point pair, toe and heel
edges, toe and heel cone points, root axis, bore diameter and mesh angle — which later steps also
write the tooth sketch, the tooth plane, the `embedded` flag and the virtual tooth count back into.

**From:** `instructions.md` L722-740, `instructions.md` L294-315,
`instructions.md` L386-388

## S12 `[PROSE]` Tooth plane

Compute this gear's virtual tooth geometry first, because the tooth sketch in S13 is drawn from it.

**Virtual (back-cone, Tredgold) tooth number**, from the closed form and **not** by measuring
Apex2->K′:

    virtual pitch radius = (this gear's Pitch Diameter / 2) / cos γ
    virtual tooth number = 2 * virtualPitchRadius / Module        # equivalently teeth / cos γ

**It is a real number and is NEVER rounded** — not floored, not ceiled, not cast to an int. The
Tredgold construction puts the equivalent spur gear's pitch radius exactly at the back-cone distance
`r / cos γ` with this gear's own module, and `z_v = z / cos γ` is a real number in every published
form of it. Rounding it rebuilds every drawn circle from the rounded count, which draws the tooth
smaller than the back cone places it and shortens the working addendum: on the shipped default — 31
teeth, Module 1, Shaft Angle 90°, γ = 45° — the exact virtual pitch radius is 21.9203 mm, a floored
count of 43 draws 21.5 mm, and the addendum the tooth works over falls to 0.5797 mm against a
nominal 1.0 module.

**Units — pin the cm→mm conversion.** The stashed pitch diameters are internal **cm** while Module
is the raw **mm** value, so compute `virtualPitchRadius_mm = (pitchDia_cm * 10 / 2) / cos γ` — the
`* 10` converts cm to mm — and then `virtualTeeth = 2 * virtualPitchRadius_mm / Module`. Skipping
the ×10 makes the virtual tooth count about ten times off.

**Root sink.** Draw the root circle one **root sink** `0.05 * 2.25 * Module` INSIDE the dedendum
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
| base | `virtualPitchRadius * cos(20 deg)` |
| tip | `virtualPitchRadius + Module` |
| root | `virtualPitchRadius - 1.25 * Module - rootSink` |

Now create the plane. Create a construction plane that includes the **tooth-centre reference line**
C->K′ (D->L′), perpendicular to the Gear Profiles sketch plane, named `{gearLabel} Plane`. Use the
framework helper `plane_by_angle(designComponent, referenceLine, gearProfilesPlane, 90)`, which
wraps `setByAngle`; pass the sketch line **directly** and never through `Path.create`
(`[PB-CONSTRUCTION-PLANES]`).

<!-- check-step-calls: ignore Path.create -->

**From:** `instructions.md` L594-621, `instructions.md` L231-245,
`.claude/skills/generate-gear/PLAYBOOK.md` L188-203

## S13 `[GO]` Virtual spur tooth sketch

Proof function: `stepVirtualSpurTooth`.

Create a sketch on the `{gearLabel} Plane` named `{gearLabel} Tooth`, and draw the borrowed spur
tooth into it, centred on the tooth-centre point K′ (L′):

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth, rootSink_mm=rootSink)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))
```

The proxy is the framework's; do **NOT** define a local copy. It precomputes, in internal cm,
exactly the keys the spur drawer reads through `parent.getParameter(name).value`, and its own
defaults match bevel: pressure angle 20°, which is not a bevel dialog input, and `InvoluteSteps`
15. `virtualTeeth` is the **real** number from S12 — the proxy computes the pitch diameter as
`virtualTeeth * module_mm`, so passing the exact `2 * r_v / Module` is what makes the drawn pitch
circle reach the back cone — and `rootSink_mm` shortens the root diameter by twice its value while
leaving pitch, base and tip alone.

**The real count reaches the spur drawer only as an angular half-thickness.** The drawer reads
`ToothNumber` as a float and uses it in one place, `pi / (2 * toothNumber)`, the angle it rotates the
flank to so the pitch crossing lands there. With `z_v = 2 * r_v / Module` that angle gives a tooth
thickness of `pi * Module / 2` at the pitch circle, which is the standard tooth thickness. An
INTEGER count drawn at the exact radius gives `pi * r_v / round(z_v)` instead, which misses nominal
by a different amount on each member of an unequal pair, so the two teeth of one pair no longer
carry the same thickness.

**The 180° rotation is delivered through the `draw()` angle argument**, not a post-hoc move or
sketch rotation: the spur generator rotates the whole tooth by that angle, and this relies on spur's
radial flank-to-root pinning so the connecting lines rotate with the tooth.

**After `draw()` returns, read `proxy._lastToothEmbedded` back.** The spur generator decides during
`draw()` whether the tooth is *embedded* — tip, root and flanks meeting with no connecting lines —
and records it on the proxy, which pre-initialises the slot to absorb that write. Thread the flag to
the tooth-profile selection in S20 by stashing it alongside the tooth sketch and plane. This is not
optional bookkeeping: it is the deterministic selector for the tooth loop's line count, and skipping
it grabs an unrelated loop and the apex-to-tooth loft dies with `LOFT_NO_TOOLBODY`.

**Do NOT hard-gate this sketch.** Log through `futil.log` if `isFullyConstrained` is false, and never
raise: the two tooth-profile sketches are exempt from the full-constraint gate because the spur
drawer labels each of the four circles with along-path sketch text, and sketch text holds a DOF
(`[PB-TEXT-HOLDS-DOF]`), so a tooth sketch whose geometry is completely determined can still read
`False` purely because it is labelled. **That exemption covers the labels and nothing else** — it is
never licence for loose geometry (`[BEVEL-F-FULL-CONSTRAINT]`).

The proof reproduces the spur scheme with this gear's own virtual count, the root sink and the 180°
angle, and holds it to DOF 0 with nothing waived. It asserts the drawn pitch radius against the
exact back-cone radius, the pitch-circle tooth thickness against `pi * Module / 2`, the sunk root
radius, and the loop the profile search keys on.

<!-- check-step-calls: ignore getParameter _lastToothEmbedded -->
<!-- proof-run: proofkit.RunParallel(toothCases, stepVirtualSpurTooth) -->

**From:** `instructions.md` L594-626, `instructions.md` L423-457,
`instructions.md` L374-385, `fusion.md` L31-58,
`.claude/skills/generate-gear/PLAYBOOK.md` L188-203, `.claude/skills/generate-gear/PLAYBOOK.md` L517-532

## S14 `[PROSE]` Tooth axis

Create a construction axis through the tooth-centre point, normal to the plane the tooth profile was
drawn on, named `{gearLabel} Tooth Axis`. Build it with `constructionAxes.createInput()` then
`setByTwoPlanes(planeA, planeB)` (`[PB-CONSTRUCTION-AXES]`); `setByPerpendicularAtPoint` would need
a `BRepFace` this build does not have.

The two planes are the **Gear Profiles plane** and a **helper plane** built with
`setByDistanceOnPath(<tooth-centre reference line>, 1.0)`, which is perpendicular to that line at its
far end, the tooth-centre point. Their intersection is the line through the tooth centre normal to
the tooth plane. Pass the sketch line **directly** to `setByDistanceOnPath`, never through
`Path.create`.

Creating this axis in the never-activated Design component is proven to work: `constructionAxes.add`
via `setByTwoPlanes` does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here. Keep the axis.

<!-- check-step-calls: ignore setByPerpendicularAtPoint Path.create -->

**From:** `instructions.md` L626, `.claude/skills/generate-gear/PLAYBOOK.md` L775-799

## S15 `[GO]` Per-gear Profile sketch — the frustum hexagon

Proof function: `stepGearProfileHexagon`.

Open a **fresh sketch on the axial Gear Profiles plane**, named per the S11 table — **one profile
sketch per gear**, so `sketch.profiles` holds exactly this one hexagon loop. Do not draw both gears'
hexagons in the shared Gear Profiles sketch; that would leave two identically-shaped loops to
disambiguate.

Build the hexagon on fixed vertices per the `[PB-PROJECT-NOT-FIXED]` recreate-share-fix recipe:

1. recreate the six §2 vertices as brand-new sketch points at their exact world-mapped positions,
   with `sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))` — valid because §2 is fully
   constrained by now, so each source's `worldGeometry` is defined;
2. draw the closed hexagon in the table's draw order as six `SketchLine`s **sharing** those points;
3. fix the lines' endpoints **after** the lines exist, by setting `isFixed = True` on each line's
   start and end sketch point.

**The order is load-bearing.** A projected point is brought in associatively but still carries free
DOF, so a sketch hung off projections reads under-constrained even though every point already sits
in the right place; and setting `isFixed` on a bare point *before* it is consumed as a line endpoint
does not leave the sketch fully constrained either.

The hexagon's **first edge is the gear's shaft axis** for the revolve, the pattern, the bore plane
AND the meshing-rotation axis, so it must be fixed well enough to carry a trustworthy world
position: fixed endpoints give that edge a well-defined `worldGeometry`
(`[PB-WORLDGEO-CONSTRAINED]`). A free edge resolves against a default world-XY frame and silently
moves the body onto world XY — observed on the driving gear, where the pinion looked fine only
because it never read the edge's `worldGeometry`.

Gate this sketch on `isFullyConstrained` and raise, naming it (`[BEVEL-F-FULL-CONSTRAINT]`).

The proof asserts that the sketch closes exactly one extrudable region of six lines, that its area
is the closed form's, that both endpoints of the first edge sit on the shaft axis, and that the
inner toe corner never does.

<!-- proof-run: proofkit.RunParallel(hexagonCases, stepGearProfileHexagon) -->

**From:** `instructions.md` L722-740, `instructions.md` L389-397,
`fusion.md` L21-30, `.claude/skills/generate-gear/PLAYBOOK.md` L458-472,
`.claude/skills/generate-gear/PLAYBOOK.md` L585-590, `.claude/skills/generate-gear/PLAYBOOK.md` L598-605

## S16 `[GO]` Revolve the Gear Body

Proof function: `stepRevolveGearBody`.

This sketch holds exactly one hexagon loop, so take its single profile with `profiles.item(0)` and
do not filter (`[PB-SINGLE-PROFILE]`). Revolve it around the **shaft-axis edge** — the profile
sketch's own first edge, NOT the §2 `Apex->A` / `Apex->B` construction line. The edge is collinear
with the shaft axis but lives in the *same* sketch as the profile, which is what Fusion's revolve,
pattern and path builders accept; reusing the §2 construction line fails or misbuilds.

`revolveFeatures.createInput(profile, axisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
then `setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))`, then `add(input)`
(`[PB-REVOLVE]`). Let the result be the **Gear Body**, the frustum.

Because the toe edge is one edge of the revolved profile, the body already carries the conical face
that edge sweeps, and likewise the heel edge's cone. Those faces are the cutting tools in S22 and
S30, so nothing builds a fresh surface for them.

**The hard failure to design around:** the profile must not cross the axis of revolution, or Fusion
aborts with `ASM_WIRE_X_AXIS`. That is what the Maximum Face Width cap in S10 and the strictly
positive Toe Radius exist to prevent.

The proof substitutes a **polygonal sweep** for the revolve, because decad publishes a revolved
body's volume with a proven bound equal to the volume itself, so a revolved body is Suspect at any
tolerance and cannot pass the harness gate. It builds the three bands the frustum's profile edges
sweep — the root cone out to the dedendum corner, the back cone out to the heel end, and the
toe-dish plug that hollows the front face — lays them apart and never joins them, and asserts the
frustum as their SIGNED SUM against Pappus on the §2 hexagon, band by band against its own stations
and ring radii, and cone half-angle by cone half-angle. THE COST IS THE UNION.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepRevolveGearBody, assertRevolveGearBody) -->

**From:** `instructions.md` L738-742, `instructions.md` L790-824,
`.claude/skills/generate-gear/PLAYBOOK.md` L494-500, `.claude/skills/generate-gear/PLAYBOOK.md` L716-723

## S17 `[GO]` Loft the Tooth Body

Proof function: `stepLoftToothBody`.

Loft the **§2 Apex sketch point** to this gear's §3 Tooth profile. Use the Apex SKETCH point
directly — `centerToApex.endSketchPoint` from the Gear Profiles sketch, the degenerate
point-section — and do **NOT** create a construction point for it
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`; the Design component is never active).

Select the tooth cross-section loop with
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)`, where the line count
is **DETERMINED BY the `embedded` flag read back in S13** and never guessed:
`wantLines = 0 if embedded else 2`. ⚠️ Do **NOT** accept "0 **or** 2 lines": for a given gear only
ONE of those is the real tooth, an unrelated loop between the drawn circles can also have 2 NURBS
and 2 arcs with the *other* line count, and selecting it makes this loft fail with
`RuntimeError ... ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY` because the impostor loop cannot form a loft
tool body. `embedded` means tip, root and flanks meet with no connecting lines, so 4 curves;
non-embedded means 2 connecting lines, so 6.

Then `loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`, add the apex
point section and the tooth profile to `loftSections`, in that order, and `add(input)`
(`[PB-LOFT]`). Let the result be the **Tooth Body**.

The proof substitutes a shrunken section for the degenerate apex point and states the cost. It draws
the tooth on a plane PARALLEL TO THE BACK CONE — the real construction — so the tooth's own tip and
root cone angles come out at the gear's, and asserts that the body reaches the exact virtual tip
radius at the heel and carries the volume a figure scaled linearly about the apex sweeps.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepLoftToothBody, assertLoftToothBody) -->

**From:** `instructions.md` L744-745, `instructions.md` L374-385,
`instructions.md` L825-826, `.claude/skills/generate-gear/PLAYBOOK.md` L724-728,
`.claude/skills/generate-gear/PLAYBOOK.md` L791-799

## S18 `[PROSE]` The tooth-body hook and its gate

`_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane, gearLabel,
teethNumber, gamma)` is the single tooth-body hook, called after lofting the uncut apex-to-heel
tooth and before pattern, combine and bore. `gamma` is this gear's pitch-cone half-angle, forwarded
to the spiral build's twist law.

**Its first line is the gate:** if `self._spiralAngle_rad <= 0`, return `cut_conical_ends(...)` —
the straight tooth of S22, byte for byte the prior behaviour. Everything from S19 to S28 runs only
when ψ > 0.

**The caller builds the four toe and heel arguments exactly per this table, and mislabelling them
silently inverts the spiral.** This is the single biggest spiral-regeneration hazard.

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

From those §2 sketch points' **world** geometry, the caller computes and passes positionally, in the
order `toeMid, heelMid, toeConeWorld, heelConeWorld`:

- `toeMid` — the world **midpoint of the TOE edge**, ½(M+N) or ½(O+P);
- `heelMid` — the world **midpoint of the HEEL edge**, ½(C+H) or ½(D+J);
- `toeConeWorld` — the toe edge's inner endpoint, **M** or **O**, which lies on the root cone
  element at the toe end because M is pinned onto Apex->C in §2;
- `heelConeWorld` — the **dedendum corner**, **C** or **D**, the outer end of that same root cone
  element.

⚠️ **Two scrambles to avoid, both of which a fresh regeneration has made.** Do **NOT** pass the two
endpoints of a *single* edge as `toeMid` and `heelMid`: M and N both sit at the toe, so the span
collapses to about zero or goes negative and the spiral inverts. And `heelConeWorld` is the dedendum
corner C (D), **never H or J**, which lie on the `Apex2->C` (`Apex2->D`) dedendum line one module
beyond C (D) and OFF the root cone element; using them skews the cone vector away from Apex->C
(Apex->D).

`_pinionMeshPhase(pinionTeeth)` returns the pinion's extra mesh rotation in radians,
`_PINION_MESH_PHASE_TEETH * 2 * pi / pinionTeeth`, with `_PINION_MESH_PHASE_TEETH` defaulting to 0.

<!-- check-step-calls: ignore _transformToothBody _pinionMeshPhase -->

**From:** `instructions.md` L317-373, `instructions.md` L628-648

## S19 `[GO]` Spiral frame, cutter-arc geometry and the 2-D trace sketch (ψ > 0)

Proof function: `stepSpiralTrace`.

**A — the frame.** Build a world frame from the geometry already constructed for this gear:

- `axisDir` — the shaft axis direction, from the two **world** endpoints of `shaftAxisEdge`, the
  in-sketch profile edge A'->G (B'->I), normalized;
- `coneVec` — the dedendum (root) cone element, `normalize(heelConeWorld - apexWorld)`;
- `v = axisDir × coneVec`, normalized — the **circumferential** direction;
- `tpNormal = coneVec × v`, normalized — the tangent-plane normal;
- a point's **cone distance** — its distance from the apex measured along the cone element —
  is `(p - apexWorld) · coneVec`, and everything below is measured with it.

⚠️ **The heel MUST be the OUTER end so `coneVec` points outward and `span > 0`.** Before building
`coneVec`, check the passed midpoints and **fix swapped toe and heel**: if
`apexWorld.distanceTo(heelMid) < apexWorld.distanceTo(toeMid)`, swap `toeMid` with `heelMid` **and**
`toeConeWorld` with `heelConeWorld`, then build `coneVec` from the corrected `heelConeWorld`. A
negative span **silently inverts the entire spiral frame** — the cutter-arc direction, the slice
direction and the per-segment twist — and the gear comes out completely wrong with no error.

Then `R_toe` is the toe midpoint's cone distance, `R_heel` the heel midpoint's,
`R_mean = (R_toe + R_heel) / 2`,
and `span = R_heel - R_toe`, the face width, now positive.

⚠️ Read the shaft-axis endpoints in **world** space, and measure every angle and distance here
against world quantities (`[PB-WORLD-FRAME]`). Mixing a sketch-local curve with a world axis is
valid Python that silently returns wrong numbers — a wrong spiral twist that makes meshing teeth
interfere — with no exception and nothing a lint can catch.

**B — the cutter-arc geometry.** Work in the tangent-plane 2-D frame with the origin at the apex,
`x = coneVec` so a point's x is its cone distance, and `y = v`. The cutter radius is the Cutter
Radius input if non-zero, **else `R_mean`**, the auto default. The hand sign is `+1` for
`_HAND_RIGHT` else `−1`, then **negated for the pinion**, because the pair meshes with opposite
hands. The cutter-circle centre is

```
Cx = R_mean - r_c * sin(psi)
Cy = handSign * r_c * cos(psi)
```

⚠️ **The hand sign goes on the `cos`/`Cy` term, NOT the `sin`/`Cx` term.** This was a real bug.
Opposite hand mirrors the cutter centre **across the cone element (y = 0)**, which flips `Cy`.
Putting `handSign` on `Cx` mirrors about `x = R_mean` instead, a *different* curve that gives the
two gears unequal twist; for equal teeth the driving and pinion traces must come out as exact mirror
images.

The trace's toe and heel endpoints are taken a hair **past** the face so the kept arc reaches
cleanly past the end trims: with `R_lo = R_toe - 0.06 * span` and `R_hi = R_heel + 0.06 * span`,
call `circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)` and
`circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)` — the framework helper, which intersects
the apex circle of that radius with the cutter circle and keeps the solution nearest the mean point,
the branch the mean point sits on.

**C — the 2-D trace sketch.** Draw a **cone-element construction line** Apex→(Apex + R_heel·coneVec)
in a sketch on the axial Gear Profiles plane, named `{gearLabel} Cone Element`. Make the tangent
plane by rotating the axial plane **90°** about that line:
`plane_by_angle(designComponent, coneElementLine, gearProfilesPlane, 90)`, named
`{gearLabel} Trace Plane`. Add a sketch on it named `{gearLabel} 2D Tooth Trace`, and in it, with
`combine_point(apexWorld, px, coneVec, py, v)` mapping 2-D coordinates to world points, draw:

- the **cutter circle** — centre at the mapped `(Cx, Cy)`, radius `r_c` — as construction geometry,
  with its centre pinned by `centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`) and a diameter
  dimension of `2 * r_c`;
- the **trace arc** — a three-point arc through the mapped toe point, the mapped mean point
  `(R_mean, 0)` and the mapped heel point, built with `addByThreePoints` — with its **centre
  coincident to the cutter circle's centre** and a **radius dimension equal to `r_c`**, so it is the
  genuine cutter circle and not a look-alike spline.

⚠️ Text points per `[PB-RADIAL-DIM]`: off-centre and on or near the curve. Use the mapped mean point
for the trace arc's radius dimension and a point on the cutter circle such as the mapped
`(Cx + r_c, Cy)` for the circle's diameter dimension; a text point at the centre is rejected outright.

**Coordinates — this rule governs the Cone Element sketch as well as the trace sketch.** The world
`Point3D`s from `combine_point`, and the raw apex and cone-end points of the Cone Element line, are
passed **directly** into the sketch calls, where they are consumed as **sketch-space** input, with
**no `modelToSketchSpace` conversion applied**, even though `Sketch` offers exactly that call and
the points really are model-space coordinates. This is deliberate and it is harmless for a reason
that is not the obvious one: the trace sketch is construction and reference only, no downstream
feature ever consumes it, and the twist is computed analytically in S21. The cone-element line *is*
consumed, by `plane_by_angle`, so an unconverted line does place the Trace Plane somewhere other
than the true tangent plane — and that still reaches no feature, because the only thing built on the
Trace Plane is the inspection-only trace sketch. **If a later revision ever makes any feature consume
the trace sketch or the Trace Plane, this shortcut stops being safe and both sketches need
`modelToSketchSpace` on every point.**

This sketch is **deliberately left with free DOF** — the arc's endpoints are pinned by the
three-point construction, not by endpoint dimensions, because dimensioning them over-constrains the
solve against the cone-element plane — and it is therefore **exempt from the full-constraint gate**.
Do not gate it.

**D — no 3-D projection.** The 2-D cutter-arc sketch is the only trace geometry needed: the spiral
twist is computed **analytically** in S21, so there is **no `projectToSurface`, no root-cone face
search and no 3-D trace sketch**. Earlier versions projected the 2-D arc onto the root cone and
measured the trace azimuth there; that projection is fragile — for unequal-ratio pairs the arc wraps
around the cone and comes back as **multiple disjoint fragments**, so the measured azimuth collapses
to a fraction of the true sweep, the pinion comes out grossly under-twisted and the pair interferes.
Do not reintroduce it.

<!-- check-step-calls: ignore projectToSurface modelToSketchSpace -->

The proof's substitution is recorded at the construction: this sketch engine's arc carries an
automatic radius-consistency row, so an arc whose two ends are both pinned comes back
over-constrained and one that leaves an end to that row admits the mirrored end, and neither passes
the gate. The proof constrains the genuine cutter circle and pins the trace's two ends on it, and
asserts the arc's own defining properties — its radius, its centre, its ends on the toe and heel
apex circles, ψ realised at the mean point, the mirror symmetry of the two hands, and the
`1 / sin γ` roll ratio — instead of constraining them.

<!-- proof-run: proofkit.RunParallel(spiralSketchCases, stepSpiralTrace) -->

**From:** `instructions.md` L628-683, `instructions.md` L140-144,
`spiral-tooth-trace.md` L30-63, `spiral-tooth-trace.md` L66-145,
`spiral-tooth-trace.md` L149-182, `fusion.md` L59-67,
`.claude/skills/generate-gear/PLAYBOOK.md` L439, `.claude/skills/generate-gear/PLAYBOOK.md` L451-457,
`.claude/skills/generate-gear/PLAYBOOK.md` L652-656

## S20 `[GO]` Slice the tooth into slabs and drop the apex scrap (ψ > 0)

Proof function: `stepSliceToothSlabs`.

**E — slice.** Split the uncut apex-to-heel Tooth Body into cross-section slabs by planes
**perpendicular to the cone element**, spanning a touch past toe and heel, via a **fixed** scheme of
about 8 planes; the count is not user-configurable.

The first cut plane is the **parent transverse tooth plane** — `parentToothPlane`, the virtual-spur
tooth-profile plane `{gearLabel} Plane` from S12, passed into the hook — offset toward the apex by
`span / 6`. The offset **sign is chosen per gear** so that it moves toward the apex: the parent
plane's normal points opposite ways for the two gears, so pick the sign for which
`sign * normal` points apex-ward, testing `(apex - planeOrigin) · normal`. Then step further toward
the apex in `span / 6` increments, so the offsets are `sign * (k + 1) * span / 6` for k = 0…7.

Split with the framework helper
`slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)`, which splits
piece by piece and keeps a piece whole when a plane misses it.

⚠️ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in **one
piece**, the offset sign was wrong or the parent plane sits outside the tooth's span — **retry the
whole cut once with the opposite sign**. If it is *still* one piece, **`raise` a clear
self-diagnosing error** naming the gear, the final piece count, the span and the sign tried
(`[PB-SELF-DIAGNOSING]`, `[PB-EMPTY-RESULT]`). Do **NOT** return an unsliced single-piece result:
step F then drops that one piece as the apex scrap, leaving `segments` **empty**, and the crown
later crashes with `max()` on an empty iterable far from the cause.

**F — order and drop the scrap.** Sort the pieces by `distAlong` of their centroid, read from
`physicalProperties.centerOfMass`. The first, apex-most piece is the long **apex-side scrap** below
the toe: **remove it**, and keep the rest as the working `segments`. Drop it by re-slicing the list
*first* — `segments = segments[1:]` — and only *then* removing the scrap with `removeFeatures.add`,
which is timeline-visible (`[PB-REMOVE-PIECES]`). After the drop, **`segments` must be non-empty**;
if it is empty the slice failed, so `raise` a clear error rather than proceeding into the twist and
the crown, which both assume at least one segment.

The proof substitutes built slabs for a split, because decad has no split at all, and states the
cost. It asserts the fixed scheme's plane positions, the segment count after the scrap is dropped,
that the span is positive, and that no segment lies at or beyond the scrap's own plane.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepSliceToothSlabs, assertSliceToothSlabs) -->

**From:** `instructions.md` L685-688, `.claude/skills/generate-gear/PLAYBOOK.md` L176-178,
`.claude/skills/generate-gear/PLAYBOOK.md` L440, `.claude/skills/generate-gear/PLAYBOOK.md` L762-774

## S21 `[GO]` Twist the segments (ψ > 0)

Proof function: `stepTwistSegments`.

**G — the twist.** Rotate each segment about the **shaft axis** — `axisDir` through `apexWorld` —
so the tooth follows the trace, **centred on R_mean so the mid-face section stays unrotated**. That
section then meshes exactly like the straight tooth, which is what the pinion's zero mesh nudge
depends on.

The total toe-to-heel shaft-axis twist comes from the **conjugate crown-gear generation law**: a
spiral bevel is generated by an imaginary flat crown gear, and the work gear's shaft rotation
relates to the developed crown-plane azimuth by the **roll ratio `1 / sin γ`**, with γ this gear's
**pitch cone angle**. Compute it analytically — no projection, no curve sampling:

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`phi_crown` is the angle the cutter arc's toe and heel endpoints subtend at the apex in the flat 2-D
crown frame — exactly the two endpoint pairs from S19 step B. `gamma` is `self._gamma_p` for the
pinion and `self._gamma_g` for the driving gear, from S5. The hand sign sets the direction; `total`
is the magnitude.

⚠️ **Use the PITCH cone angle γ, NOT `acos(coneVec · axisDir)`**, which is the *root/dedendum* cone
angle — about 14° against a pitch of about 29° for a 17-tooth pinion — and yields a twist about 1.6
times too large.

⚠️ **The two members of a meshing pair legitimately get different twists**: same cutter, same ψ, but
γ differs, so `1 / sin γ` differs — about 2.08× for a 17-tooth pinion against about 1.14× for a
31-tooth gear, a ratio of about 1.83. This is *why* equal-teeth pairs always meshed while ratio
pairs failed under any method that gets `1 / sin γ` wrong.

Each segment's rotation angle is a **linear share keyed to the cone distance of its HEEL FACE**, the
segment's farthest-along-the-element face, which is the exact section the later loft samples:

```
ang = -handSign * total * (R_mean - R_heelFace(seg)) / span
```

⚠️ **Key the twist on the segment's HEEL-FACE cone distance, NOT its centroid.** The loft samples
each segment's heel face, so that face is what must land at the right azimuth; centroid-keying
leaves the loft's mid-face section rotated by half a segment and the mid faces overlap.

**Define a slab's heel face precisely: the face whose centroid has the GREATEST cone distance,
searched across ALL of the slab's faces with NO surface-type filter**;
its toe-side face is the least-centroid one. ⚠️ Do **NOT** restrict this search to
`PlaneSurfaceType`, or to any surface type — a sliced slab is bounded by a mix of the two planar cut
faces and ruled side faces, and a type filter can pick the wrong face or miss the cut face, which
makes the loft in S23 fail with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this same
all-faces-by-centroid rule everywhere a slab end face is needed: here, at the crown base in S22, and
at the loft sections in S23.

Apply the rotation with a free move: `moveFeatures.createInput2(bodies)`, then
`defineAsFreeMove(matrix)` where the matrix is built with
`Matrix3D.setToRotation(ang, axisVector, originPoint)`, then `add(input)` (`[PB-MOVE-ROTATE]`). Use
`defineAsFreeMove` with a matrix and not `defineAsRotate`, which rejects a `SketchLine` axis. **A
zero angle is a no-op, not a move:** `setToRotation(0, axis, origin)` builds the identity and Fusion
refuses it with `invalid transform`, so a computed angle that lands on zero returns early.

<!-- check-step-calls: ignore defineAsRotate -->

The proof builds the segments already rotated, since a free move by a rotation matrix and a body
built turned end up in the same place. It asserts that the segment at R_mean turns by exactly zero,
that the share is linear in the heel-face cone distance, that the direction follows the hand sign,
and that the two members of a pair twist alike exactly when their two pitch cone angles do.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepTwistSegments, assertTwistSegments) -->

**From:** `instructions.md` L689-702, `spiral-tooth-trace.md` L186-214,
`.claude/skills/generate-gear/PLAYBOOK.md` L800-809, `.claude/skills/generate-gear/PLAYBOOK.md` L439

## S22 `[GO]` Crown the segments lengthwise (ψ > 0)

Proof function: `stepCrownSegments`.

**H — the lengthwise crown.** Crown the tooth by scaling each segment **except the outermost, heel
one** down by a **monotonic** factor: full at the heel and growing smoothly toward the toe, **about
a sketch point on the ROOT EDGE of its heel face** and never the heel-face centroid.

For each segment compute its **heel-distance fraction** `u = (R_heel - R_heelFace) / span`, where
`R_heelFace` is that segment's heel face's `distAlong`, found by the same all-faces-by-centroid rule
as S21 but **RECOMPUTED here, AFTER the twist has moved the slabs** — do not reuse pre-twist values.
`u` runs 0 at the held-full heel to 1 at the toe. The **outermost (heel) segment is the one with the
GREATEST post-twist heel-face `distAlong`**: sort the segments by their recomputed heel-face
distance and skip the last. Then

```
factor = 1 - _CROWN_PER_RAD * (abs(total) / 2) * u
```

`total` is the full toe-to-heel twist from S21, so `abs(total) / 2` is the per-end peak twist
magnitude and the maximum relief — now at the **toe** — keeps the magnitude the old per-end peak
had, just relocated. Relief therefore grows **monotonically from the full heel to the toe**, so slab
heights stay strictly ordered heel to toe and the natural cone taper is never reversed. If a
computed `factor` comes out ≤ 0, `raise` a self-diagnosing error naming the gear, the segment's `u`
and the factor; never scale by a non-positive factor. **`_CROWN_PER_RAD` is a tunable class
constant, default `0.5`** — 0 disables the crown, so set it to 0.5 and do not leave it unset.

⚠️ **Do NOT key the relief on `abs(ang)`, the twist magnitude.** That is symmetric about the mid
face, maximal at BOTH ends, so with the heel slab held full the slab *just inside* the heel becomes
the most-relieved one and dips below both its neighbours, reversing the heel-to-toe taper. This was
the observed bug: the heel-adjacent slab came out at factor 0.932 while the next slab inward was
0.972, and therefore taller. Key it on the monotonic heel distance `u`.

Three further points:

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). `scaleFeatures` is the ONE exception to never-activate:
   it needs the Design occurrence as the **active** edit target, so call `designOccurrence.activate()`
   before the crown scales and restore afterward, in a `finally`, with
   `design.activateRootComponent()`. ⚠️ Do **NOT** write `design.rootComponent.activate()` or
   `someComponent.activate()`: a `Component` has **no** `activate()` method and raises
   `AttributeError`. Only an `Occurrence` has it.
2. **Skip the outermost (heel) segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone in S24 trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, not its centroid, or the crowned tooth lifts off
   the gear base.** `scaleFeatures` shrinks uniformly toward the base point, so a base point at the
   heel-face centroid — mid tooth height — pulls the tooth's root edge *upward* by
   `(1 - factor) * (half the tooth height)`, the tooth no longer seats on the gear body's root cone,
   it floats above the base, and the Combine-Join leaves a gap, clearly visible for ratio pairs such
   as module 2 with driving 19 and pinion 13, which is the symptom that exposed this. Put the base
   point on the **root** instead: of the heel face's vertices, each `.geometry` a world `Point3D`,
   take the **two with the smallest perpendicular distance to the shaft axis** — the line through
   the apex along `axisDir`, so the distance is
   `abs((p - apex) - ((p - apex) · axisDir) * axisDir)` — which are the two **root corners**, the tip
   corners being the farthest from the axis, and place the base sketch point at their **midpoint**,
   mapped into the heel-face sketch with `modelToSketchSpace`. The heel face is a planar cut, so that
   midpoint lies on it. A uniform scale about a point keeps every line through that point invariant,
   so anchoring on the root keeps the root edge on the seating cone while the tip is relieved
   progressively toward the toe, which is exactly the lengthwise crown intended.

Build the scale with `scaleFeatures.createInput(bodies, basePoint, adsk.core.ValueInput.createByReal(factor))`
and `add(input)`.

<!-- check-step-calls: ignore defineAsRotate -->

The proof builds each slab at its crowned size about the named anchor, because decad has no scale
feature, and states the cost. It asserts that the factors are positive and strictly monotonic from
the held-full heel toward the toe, that the toe-most relief is
`_CROWN_PER_RAD * abs(total)/2 * u` exactly, and that a centroid anchor would lift the root edge.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepCrownSegments, assertCrownSegments) -->

**From:** `instructions.md` L704-716, `.claude/skills/generate-gear/PLAYBOOK.md` L585-590,
`.claude/skills/generate-gear/PLAYBOOK.md` L791-799

## S23 `[GO]` Loft the spiral tooth (ψ > 0)

Proof function: `stepLoftSpiralTooth`.

**I — the loft.** ⚠️ **Re-sort the segments by their heel-face cone distance HERE, AFTER the twist
and the crown — do NOT reuse the pre-twist slice or centroid order.** The twist rotates each slab
about the shaft axis, and for high-twist *unequal-ratio* pairs that rotation changes the slabs'
along-cone order enough to **reorder adjacent slabs**; lofting in the stale order assembles the
cross-sections out of sequence, the crowned tooth comes out distorted, and the two gears interfere.
For equal or low-twist pairs the two orders coincide, which is why equal-teeth gears mesh even with
the stale order and a ratio pair like 31/17 does not — this is the single thing that makes such a
pair fail while 31/31 looks fine.

So sort the segment indices **now**, by the cone distance of each segment's own heel-face centroid
found by the all-faces rule of S21, and loft a NewBody through, in that order:

1. first the **toe-most segment's apex-side (toe-facing) face** — the toe segment is `order[0]`, and
   adding its toe face first pushes the loft past the toe cone so the toe trim bites;
2. then the **heel-facing face of every segment, iterated in `order`**, each being that segment's
   farthest-along-the-element face by post-twist centroid, the last of which reaches past the heel
   cone.

Name the resulting body **`{gearLabel} Spiral Tooth`**. Then remove the segment scaffolding with
`removeFeatures.add`, since the loft has captured their faces.

The proof substitutes a **chain of pairwise lofts in that order**, because decad's Loft takes exactly
two sections, and states the cost. It asserts that the order really is sorted by post-twist heel-face
cone distance and that every adjacent pair lofts into a single lump.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepLoftSpiralTooth, assertLoftSpiralTooth) -->

**From:** `instructions.md` L718, `.claude/skills/generate-gear/PLAYBOOK.md` L724-728,
`.claude/skills/generate-gear/PLAYBOOK.md` L770-774

## S24 `[GO]` Flush-trim the spiral tooth (ψ > 0)

Proof function: `stepTrimSpiralTooth`.

**J — the flush trim.** Return
`cut_conical_ends(designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)` —
the framework helper, the same toe-then-heel two-cone trim the straight tooth takes — so the curved
tooth's ends sit **flush** on the gear base.

The toe and heel **mesh phasing is handled outside this hook**, by the mesh-rotate step in S29; the
pinion's extra phase is 0 by default precisely because the mid-face section is unrotated and already
meshes.

The proof performs neither cut, for the reason S25 gives, and holds the curved tooth to the same
cone readings as the straight one. It additionally asserts that the mid-face section is unrotated
and that the pinion's mesh phase is 0.

<!-- proof-run: proofkit3d.RunSolidParallel(spiralCases, stepTrimSpiralTooth, assertTrimSpiralTooth) -->

**From:** `instructions.md` L720, `instructions.md` L827-834

## S25 `[GO]` Conical end cuts — the straight tooth (ψ = 0)

Proof function: `stepCutConicalEnds`.

Trim the Tooth Body to a flush band with the framework helper — return
`cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)` from
`.solids`. Do **NOT** re-implement the cut machinery.

**Two distinct bodies are involved and they must not be conflated:** the cutting TOOLS are
`ConeSurfaceType` faces of the **Gear Body**, the revolved-hexagon frustum — the lofted Tooth Body
has no cone faces, so searching *it* for the cone face finds none — and the TARGET being split is
the **Tooth Body**.

The helper implements the pinned cut behaviour: the **toe cut first**, its cone face identified by
the toe edge's world **MIDPOINT** best-first across the frustum's cone faces
(`[PB-FACE-BY-MIDPOINT]` — endpoints sit near the apex singularity and report an unevaluable
distance), each candidate tried as the actual split tool and the first that splits kept; **keeper
selection after each cut**, removing apex-containing pieces and keeping the largest
(`[PB-REMOVE-PIECES]`); then the **heel cut on the keeper alone**, removing the apex tip first being
what makes it deterministically two split features for every gear ratio. A heel cone that does not
intersect the keeper at all — common on ratio pairs, for example module 1 with driving 31 and pinion
43, where the heel cone never overshoots the tooth — is raised by the helper as the typed
`solids.NonIntersectError` and caught, and the keeper is returned whole. Every failure is
self-diagnosing with the per-face distance and error history (`[PB-SELF-DIAGNOSING]`).

**Caller obligations, which stay in the generator:** pass `toeMid` as the toe edge's world midpoint
`(M_world + N_world) / 2` or `(O_world + P_world) / 2`; `heelMid` as the heel edge's world midpoint
`(C_world + H_world) / 2` or `(D_world + J_world) / 2` — the same edge-midpoint pairs as the S18
hand-off; `apexWorld` as the §2 Apex sketch point's world geometry; and `gearBody` as the revolved
frustum, the cone-face source. The toe cut must split, and its failure propagates and crashes the
build, which is correct since an uncut tooth is unusable; only the heel cut is lenient, and only
through the typed `NonIntersectError`.

The proof performs **neither cut**, because both operands are Lofts, and states the cost. It builds
the tooth and the two cones and lays them apart, reads each cone's apex and half-angle off the cone
and each of the tooth's two surfaces off the tooth, solves the stations where they cross from those
readings, and checks them against the flush band. THE COST IS THE SPLIT: what it does show is that
each cut lands where the flush band requires and that the two ends land on DIFFERENT surfaces of the
tooth, which is the observable signature of a conical cut face rather than a planar one.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCutConicalEnds, assertCutConicalEnds) -->

**From:** `instructions.md` L746-772, `instructions.md` L827-834,
`.claude/skills/generate-gear/PLAYBOOK.md` L159-172, `.claude/skills/generate-gear/PLAYBOOK.md` L733-761

## S26 `[GO]` Circular-pattern the tooth

Proof function: `stepCircularPattern`.

Circular-pattern the remaining tooth piece around the **shaft-axis edge** — the same in-sketch
profile edge used for the revolve, not the §2 construction line.

`circularPatternFeatures.createInput(bodies, axisEdge)`, then pin all three inputs explicitly
(`[PB-CIRCULAR-PATTERN]`): `quantity = ValueInput.createByReal(<this gear's Teeth Number>)`,
`totalAngle = ValueInput.createByString('360 deg')`, `isSymmetric = False`. Then `add(input)`.

Although the pitch diameter shrinks from heel toward apex, the *angular* spacing around the shaft
axis stays constant at `360° / N` for the entire face width: the radial taper is already produced by
the loft from Apex to the heel-end tooth profile, so the pattern just rotates that single tapered
tooth into N evenly spaced copies.

`CircularPatternFeature.bodies` already includes the seed body plus the copies, so do not re-add the
seed, and copy them into a fresh `adsk.core.ObjectCollection` before handing them to the combine —
`pattern.bodies` is a `BRepBodies` and `combineFeatures.createInput` rejects it
(`[PB-PATTERN-BODIES]`).

**This is the one bevel step whose proof stays SERIAL.** The pattern increment retires the seed
tooth, so the seed cannot be measured after the step runs, and its azimuth, radius, height and
volume have to be read during the build and handed to the assertion. That hand-off leaves the case,
and two cases sharing one set of seed readings overwrite each other. It is not a hazard that
announces itself: the two gear sides differ enough in volume that the overwrite was caught when it
happened, and a pair of cases whose seeds measured alike would have passed on each other's numbers
instead.

The proof also measures the copies one at a time, each in a document of its own, because decad
verifies every PAIR of live bodies in a document and its disjoint/overlap partition proof resolves
neither way for the tooth pairs of a real gear. What that gives up is the proof that the copies are
mutually disjoint; what it keeps is the placement law and the fact that no copy is deformed by its
placement.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCircularPattern, assertCircularPattern) -->

**From:** `instructions.md` L774, `instructions.md` L888-912,
`.claude/skills/generate-gear/PLAYBOOK.md` L693-703

## S27 `[GO]` Combine-Join the teeth into the Gear Body

Proof function: `stepCombineTeeth`.

Join all patterned tooth pieces with the Gear Body in a **single Combine-Join**: the Gear Body as the
target and the patterned tooth bodies as the tools.
`combineFeatures.createInput(gearBody, toolCollection)` with
`operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`, then `add(input)`.

The proof performs **no join**, because the operands are Lofts, and states the cost. It lays them
apart and asserts the join's two consequences from their own measured geometry: a join leaves ONE
lump when the tooth's root is below the body's root cone — seated, not floating — and the joined body
reaches further out than the frustum when the tooth's tip stands proud of it. Both readings are
taken at the toe, the middle and the heel of the band the join would cover. THE COST IS THE STITCH.

The seating reading is the **root arc's OUTERMOST point and never the tooth's centreline**: the
centreline sits inside both root corners, so a reading taken there passes a tooth whose corners
float outside the cone, which is exactly the defect the root sink exists to remove. The generated
module draws its root circle one root sink inside the dedendum corner (S12), and **the proof applies
that same sink** — it is one figure, not a proof-only offset. On the shipped default pair the unsunk
corner floats 0.002 module outside the root cone and on a 4/4 pair 0.027 module, both of which the
sink of 0.1125 module clears.

<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCombineTeeth, assertCombineTeeth) -->

**From:** `instructions.md` L776, `instructions.md` L835-846,
`instructions.md` L605-620, `.claude/skills/generate-gear/PLAYBOOK.md` L693-697

## S28 `[GO]` Bore sketch

Proof function: `stepBoreSketch`.

Skip this step and S29 entirely if Enable Bore is unchecked.

Build the bore plane normal to the shaft at its start: `constructionPlanes.createInput()` then
`setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))`. Pass the in-sketch
profile edge, not the §2 construction line.

In a sketch named `{gearLabel} Bore`, sketch the bore circle centred at the sketch origin — the plane
is rooted at the shaft start, so the origin is on the axis. **Fix the circle's centre and add a
diameter dimension** set to the bore diameter (`[PB-CIRCLE-CENTER]`):
`addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), r)` does NOT reuse the sketch's `originPoint`
— its `centerSketchPoint` is a free point that happens to sit there — so pin it with
`centerSketchPoint.isFixed = True` and add `addDiameterDimension(circle, textPoint)`. Do NOT
`addCoincident` the centre to the sketch origin; that has been observed to throw
`VCS_SKETCH_SOLVING_FAILED` on a `setByDistanceOnPath` plane.

The bore diameter is this gear's Bore Diameter if non-zero, otherwise this gear's
`Pitch Diameter / 4`.

Gate this sketch on `isFullyConstrained` and raise, naming it (`[BEVEL-F-FULL-CONSTRAINT]`).

<!-- proof-run: proofkit.RunParallel(boreCases, stepBoreSketch) -->

**From:** `instructions.md` L778, `instructions.md` L100-104,
`fusion.md` L21-30, `.claude/skills/generate-gear/PLAYBOOK.md` L451-457

## S29 `[GO]` Cut the bore

Proof function: `stepBoreCut`.

Extrude-cut the bore circle as a **symmetric through-cut** restricted to this Gear Body:
`extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)`, then
`setSymmetricExtent(adsk.core.ValueInput.createByReal(2 * <Cone Distance>), False)` — the second
argument `isFullLength = False` means the distance is the half-length *per side*, and
`2 * Cone Distance` is generously past any face width (`[PB-THROUGH-CUT]`). Do not pass a third,
taper argument. Restrict the cut with `participantBodies = [thisGearBody]`. Then `add(input)`.

The proof builds the tool as a **real extrude**, which a symmetric extent produces as a prism, but
performs no cut, because the target is the frustum and its bands are Lofts. It lays the tool and the
bands apart and asserts the cut from the tool's own measured geometry — its diameter, that its two
ends sit exactly `2 * Cone Distance` either side of the shaft edge's start, and that both clear the
frustum, which is what makes it a THROUGH cut — and computes the material it would remove from the
frustum's own profile clipped to the bore radius. THE COST IS THE PIERCED BODY: one lump with a hole
and no enclosed void is not shown. The table carries both sides of the Enable Bore branch and both
the auto and a user diameter.

<!-- proof-run: proofkit3d.RunSolidParallel(boreSolidCases, stepBoreCut, assertBoreCut) -->

**From:** `instructions.md` L778, `instructions.md` L847-853,
`.claude/skills/generate-gear/PLAYBOOK.md` L729-732

## S30 `[GO]` Meshing rotation

Proof function: `stepMeshRotation`.

**Driving gear only, and here in the Design component, before the body is moved out.** Rotate the
driving body by `180° / Driving Gear Teeth Number` — half a tooth pitch — about its own shaft axis
with the framework helper `rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)`,
which takes the rotation axis and origin from the B->I profile edge's **world** endpoints
(`[PB-MOVE-ROTATE]`).

Both gears are patterned from a starting tooth in the axial plane, so without the offset a driving
tooth and a pinion tooth would both sit at the axial-plane crossing and visually collide; the
half-pitch offset puts a driving valley where the pinion tooth crosses, giving the interlocked
meshing look.

This runs in Design before `moveToComponent` because a construction axis cannot be added in the
moved-out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation must use the edge's
world geometry while still in Design.

The pinion additionally gets `_pinionMeshPhase(pinionTeeth)`, which is 0 by default. A zero angle is
a no-op rather than a move — Fusion refuses the identity transform — and `rotate_body_about_edge`
absorbs that for exactly this reason, so no call site guards it.

<!-- check-step-calls: ignore _pinionMeshPhase -->
<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepMeshRotation, assertMeshRotation) -->

**From:** `instructions.md` L780, `instructions.md` L356-358,
`.claude/skills/generate-gear/PLAYBOOK.md` L800-809

## S31 `[PROSE]` Move the finished body into its gear component

Relocate the finished body with `moveToComponent(targetOccurrence)`, which preserves the world
position and needs no activation (`[PB-NO-CROSS-SIBLING]`). The target is the `{gearLabel} Gear`
occurrence created in S11.

This is the last per-gear step. Run S11 through S31 for the pinion first, then for the driving gear.

**From:** `instructions.md` L736, `.claude/skills/generate-gear/PLAYBOOK.md` L829-834

## S32 `[PROSE]` Cleanup

Call the framework's `hide_construction_geometry(self.bevelComponent)` from `.solids`. It recursively
walks the Bevel Gear component tree, deduping by `entityToken`, and hides every sketch, construction
plane and construction axis by setting `isLightBulbOn = False` — construction planes and axes are
**not** hidden by `isVisible` (`[PB-HIDE-AFTER-USE]`, `[BEVEL-F-CLEANUP]`). Leave only the two
finished gear bodies visible.

There is no sketch-only mode and no per-mode guard: bevel always builds solids.

The driving gear's half-tooth-pitch meshing rotation is performed in S30, not here.

**Do not add a display-settling call.** `commands/_gear_command.py` calls
`geargen.settle_sketch_display` once after `generate()` returns, every gear command runs through that
one call, and nothing about it belongs in a generated module (`[PB-SETTLE-DISPLAY]`).

<!-- check-step-calls: ignore settle_sketch_display -->

**From:** `instructions.md` L784-788, `fusion.md` L161-166,
`.claude/skills/generate-gear/PLAYBOOK.md` L534-555, `.claude/skills/generate-gear/PLAYBOOK.md` L659-671
