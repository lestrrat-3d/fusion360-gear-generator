# Bevel gear — compiled step list

The proof for this gear is `proof/bevelgear/geometry_test.go`, `proof/bevelgear/sketches_test.go`,
`proof/bevelgear/solids_test.go`, `proof/bevelgear/spiral_test.go` and the generated
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

## S01 `[PROSE]` Module layout, imports and module constants

Write `lib/geargen/bevelgear.py` as a standalone module. It does **not** subclass `base.Generator`
and carries **no `GenerationContext`** — no context class and none of the `base.Generator` context
machinery. One generator handles both straight and spiral bevels; there is no separate spiral
subclass or command.

Import explicitly; never `import *` (Module layout). From `base.py` import **only** the input
readers `get_selection` and `get_boolean`. From `.solids` import `cut_conical_ends`,
`apply_conical_cut`, `select_keeper`, `find_cone_faces_by_midpoint`, `surface_distance`,
`slice_body_by_offset_planes`, `rotate_body_about_edge`, `plane_by_angle`, `combine_point`,
`circle_intersect_nearest` and `hide_construction_geometry`. From `.utilities` import
`find_profile_by_curve_counts`. From `.spurproxy` import `VirtualSpurProxy`. From `.spurgear`
import `SpurGearInvoluteToothDesignGenerator`. From `.misc` import `to_cm`. Also `import math` and
`import adsk.core, adsk.fusion`, and `from ...lib import fusion360utils as futil`.

**Do not re-implement any framework helper** and do not define a local proxy or value-wrapper class;
re-defining one of those names is a contract violation.

The module declares **twenty `INPUT_ID_*` string constants** plus `_HAND_RIGHT` and `_HAND_LEFT`,
and **no `PARAM_*` names at all**, because bevel registers **no live Fusion user parameters**: every
value is precomputed in Python in internal cm and written into geometry numerically — sketch
dimensions through `dimension.parameter.value = <number>` and feature inputs through
`ValueInput.createByReal(<number>)` (`[PB-PRECOMPUTED-MODE]`, `[PB-NUMERIC-SNAPSHOT]`).

```python
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
_HAND_RIGHT = 'Right'
_HAND_LEFT = 'Left'
```

Two further constants are declared **on `BevelGearGenerator`**, not at module level, because both
are read through `self` from inside the tooth-body build:

| constant | declared on | default | what it does |
|---|---|---|---|
| `_CROWN_PER_RAD` | `BevelGearGenerator` | `0.5` | scales the spiral lengthwise crown — S19 |
| `_PINION_MESH_PHASE_TEETH` | `BevelGearGenerator` | `0.0` | the pinion's extra mesh rotation in tooth-fractions |

⚠️ `_CROWN_PER_RAD` reaches built geometry and nothing derives it. `0.5` was hand-entered on
2026-06-07 on the hand-coded generator this spec replaced, and no measurement, published source or
Fusion load stands behind it since. Keep it at `0.5` so a regen reproduces today's gear, and do not
write a derivation the repository does not have.

Two classes are declared, and the entry point binds both **by name**:
`BevelGearCommandInputsConfigurator` with `configure(cls, cmd)` and `handle_input_changed(cls,
args)`, and `BevelGearGenerator` with `__init__(self, design)` (storing `self.design` and
`self.bevelOccurrence = None`), `generate(inputs)` and `deleteComponent()`.

<!-- check-compile: ignore handle_input_changed distAlong slabHeelFace sorted -->

`handle_input_changed`, and the pseudo-code names `distAlong`, `slabHeelFace` and `sorted` that the
spiral steps below write inline, are excluded from the Fusion-API reality check: the first is a
method this module DEFINES for the framework to call, and the other three are this spec's own
notation for a reading and an ordering rather than API methods.

<!-- check-step-calls: ignore configure handle_input_changed generate deleteComponent distAlong slabHeelFace sorted to_cm cut_conical_ends apply_conical_cut select_keeper find_cone_faces_by_midpoint surface_distance slice_body_by_offset_planes rotate_body_about_edge plane_by_angle combine_point circle_intersect_nearest hide_construction_geometry find_profile_by_curve_counts VirtualSpurProxy SpurGearInvoluteToothDesignGenerator get_selection get_boolean -->

The helper names above are listed as **imports** and as the module's own public surface, not as calls
this step requires; every one of them is required by the step that actually calls it, and
`configure`, `handle_input_changed`, `generate` and `deleteComponent` are methods the module defines
for the framework to call rather than calls the module makes.

**From:** `spec/bevelgear/instructions.md` L302–L331, L380–L405, L531–L544;
`.claude/skills/generate-gear/PLAYBOOK.md` L17–L40, L850–L867

## S02 `[PROSE]` Command dialog — the twenty inputs, in display order

`BevelGearCommandInputsConfigurator.configure(cls, cmd)` adds **twenty** inputs to
`cmd.commandInputs` in exactly the row order below. Target Plane is first so it wins Fusion's
auto-focus, which ignores a later `hasFocus` (`[PB-AUTOFOCUS-FIRST]`); Center Point follows so the
user flows from plane to point; the pre-selected Parent Component is third.

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

The tooltip strings in the table are the third argument to `addSelectionInput` and are reproduced
surface — use them verbatim. Each selection input sets its filters with `addSelectionFilter` and its
limits with `setSelectionLimits(1, 1)`; filters are the named enum constants, never quoted literals
(`[PB-SELECTION-FILTER-ENUM]`, `[PB-SELECTION-DECL]`). The Parent selection pre-selects
`get_design().rootComponent` through `addSelection`.

The Hand dropdown is a `DropDownStyles.TextListDropDownStyle`, with `Right` added selected and
`Left` added unselected through `listItems.add`.

Numeric `mm` and `deg` defaults are passed in **internal units** whatever the display unit string
says — `createByReal(to_cm(0))` for a 0 mm length, `createByString('90 deg')` for the angle so the
expression engine parses it (`[PB-DIALOG-DEFAULT-UNITS]`). `toeExtension` is a plain unitless
percentage and takes no `to_cm` conversion.

`configure()` calls `cls._updateSpiralInputVisibility(inputs)` as its **last** step, so the initial
state is right for the default Mean Spiral Angle of 35 degrees.

Calls this step requires: `addSelectionInput`, `addSelectionFilter`, `setSelectionLimits`,
`addValueInput`, `addBoolValueInput`, `addDropDownCommandInput`, `createByReal`, `createByString`,
`to_cm`, `addSelection`, `listItems.add`, `_updateSpiralInputVisibility`.

**From:** `spec/bevelgear/instructions.md` L37–L46, L228–L272, L295–L308;
`.claude/skills/generate-gear/PLAYBOOK.md` L53–L61, L128–L143, L355–L358, L557–L568

## S03 `[PROSE]` Conditional visibility of the two spiral-only inputs

Hand of Spiral and Cutter Radius matter only for a curved bevel, so they are **hidden whenever Mean
Spiral Angle ψ = 0 and shown when ψ > 0**. Mean Spiral Angle itself is the controller and is
**always visible** — it is how the user reaches ψ > 0. There is no declarative show-if in the Fusion
API, so this is realized with the `isVisible` property.

Add `@classmethod _updateSpiralInputVisibility(cls, inputs)`. It reads the `spiralAngle` input's
**`.expression`** and evaluates it with `unitsManager.evaluateExpression(spiral.expression, 'rad')`
— internal **radians**, and it does **not** read the input's `.value`. It then sets
`inputs.itemById(INPUT_ID_HAND).isVisible` and `inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible`
to `(value > 0)`.

Guard it: if any of the three inputs is `None`, return early; wrap the expression evaluation in
`try`/`except`, because a half-typed expression can raise mid-edit, and on failure leave both inputs
**shown**.

`isVisible` only hides the dialog row. The input still exists and S04 reads it normally, and a ψ = 0
build ignores Hand and Cutter Radius anyway, so hiding is purely cosmetic and cannot affect
generation.

`@classmethod def handle_input_changed(cls, args)` simply calls
`cls._updateSpiralInputVisibility(args.inputs)` — recompute on **every** input change, which is
cheap and robust and needs no branch on which input changed. `commands/bevelgear/entry.py` binds it
by name as the dialog's `inputChanged` callback.

Calls this step requires: `evaluateExpression`, `itemById`.

**From:** `spec/bevelgear/instructions.md` L273–L294, L389–L393;
`.claude/skills/generate-gear/PLAYBOOK.md` L291–L321, L863–L867

## S04 `[PROSE]` Read and validate every input

`generate(inputs)` calls `_readInputs(inputs)` **first**, before anything creates an occurrence.
Bevel registers no user parameters, so nothing creates an occurrence until every selection is read
and the selection-context-shift hazard does not bite here — keep the order so it stays that way
(`[PB-SELECTION-STASH]`).

`_readInputs` returns the 7-tuple `(parentComponent, targetPlane, centerPoint, module,
drivingTeeth, pinionTeeth, shaftAngle_deg)` and stashes the rest on `self`:
`self._drivingBaseHeight_cm`, `self._pinionBaseHeight_cm`, `self._boreEnable`,
`self._drivingBore_cm`, `self._pinionBore_cm`, `self._faceWidth_cm`, `self._toothSpacing_cm`,
`self._spiralAngle_rad`, `self._hand`, `self._cutterRadius_cm`, `self._toeExtension_pct`,
`self._drivingToeRadius_cm`, `self._pinionToeRadius_cm`.

Read each input with the helper its declared type fixes (`[PB-INPUT-READ]`): selections with
`get_selection`, the Enable Bore checkbox with `get_boolean`, the Hand dropdown through
`itemById(INPUT_ID_HAND).selectedItem` reading `.name` with `_HAND_RIGHT` as the default when none
is selected, and every numeric or angle input by evaluating its expression with
`evaluateExpression` under the unit string `''`, `'mm'` or `'deg'` (`[PB-EVAL-EXPRESSION]`).

**Units — critical.** `evaluateExpression` returns Fusion **internal** units (cm for length, radians
for angle) whatever the unit string says. So the `'mm'` inputs and the `'deg'` input come back
already internal — use them as they are and do **not** `to_cm` them again. **`Module` is read with
unit `''`, so it comes back as a raw number meaning millimetres**: a Module of 1 is 1 mm. Every
length derived from Module must therefore be `to_cm`-converted before it touches geometry — the two
Pitch Diameters, the Cone Distance, the dedendum `1.25 * Module`, every §2 seed length and the
default Face Width `Cone Distance / 6`. Mixing a raw-mm Module-derived length with an already-cm
`'mm'` input makes the gear come out about ten times off and the Face-Width bound meaningless.

Coerce both teeth inputs to whole numbers with `int(round(...))` before validating.

Validate, in this order:

1. `module > 0`; both teeth `>= 3`; non-negative base heights, bore diameters, face width, tooth
   spacing, toe radii and cutter radius; Toe Extension in `[0, 100]`; Mean Spiral Angle in
   `[0, 60)` degrees after converting from radians with `math.degrees`.
2. **Shaft Angle at least 30 degrees and below the Maximum Shaft Angle.** Convert to degrees before
   the range check. The Maximum Shaft Angle depends on both tooth counts, so check it after both
   are read and coerced, and name the computed limit in the message. It is
   `min(degrees(acos(-smaller_pitch_diameter / larger_pitch_diameter)), 150)`, and the cone-angle
   half is **exclusive** while the 150-degree half is **inclusive**. A 31/17 pair gives
   `acos(-17/31) = 123.26°`; equal tooth counts give `acos(-1) = 180°`, which is no constraint at
   all, and 150 then caps it. ⚠️ 30 degrees is the documented floor but is not known to be
   reachable: of three independently written §2 lattices, two refuse the default pair at 30 degrees
   on conditioning and first clear at 35. That is a property of the construction, so this floor is
   the geometric one and the reachable floor is the proof's to record.
3. Compute `γ_p` and `γ_g` from the closed form, once: `tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`
   and `γ_g = Σ − γ_p`, with `PPD` the Pinion Gear Pitch Diameter `Module * Pinion Gear Teeth
   Number` and `DPD` the Driving Gear Pitch Diameter `Module * Driving Gear Teeth Number`. The
   **Pitch Cone Distance** is `R = (PPD / 2) / sin γ_p`. Keep `R` distinct from the **Cone
   Distance** `sqrt(PPD**2 + DPD**2)`, which depends on the two tooth counts alone and never on the
   Shaft Angle; the two coincide as `Cone Distance = 2 * R` exactly when the Shaft Angle is 90
   degrees and diverge everywhere else.
4. **Minimum Teeth**, per gear, with that gear's own `γ`: `teeth >= 5.27 * cos γ`, on top of the
   blanket `teeth >= 3`, naming the computed floor in the message. The constant is
   `2 * (1.05 * 1.25 / 0.95 + 1.25) = 5.2632`, rounded **up** to 5.27 so the published floor stays
   at or above the exact crossing; do not round it down. At Shaft Angle 90 the floor is 3.72, so
   **4 teeth**: with both base-height bounds applied an equal 4-tooth pair solves and a 3-tooth pair
   still fails on the heel edge. The blanket check admits that 3-tooth pair, so keep both.
5. **Minimum and Maximum Base Height**, per gear, both closed form:
   `Minimum Base Height = 1.05 * 1.25 * Module * sin γ` and
   `Maximum Base Height = 0.95 * (r - 1.25 * Module * cos γ) * tan γ` for that gear's own pitch
   radius `r`. Apply them in both directions: **raise** a fallback below the minimum, **cap** a
   fallback above the maximum, and **reject** a user value outside either end naming the bound it
   broke. The base height is the offset between the A→Apex2 drop and G→H, so it is measured from
   **Apex 2's plane**, not from the dedendum point: walking out the dedendum line from Apex 2 the
   perpendicular distance to the shaft axis falls at `cos γ` per unit and the along-shaft coordinate
   rises at `sin γ`, so H reaches the axis at `r * tan γ`, which is the true crossing. The bound
   above sits `1.25 * Module * sin γ` below it and is **deliberately conservative, not exact**. The
   exact form `0.95 * r * tan γ` is not to be adopted without re-running the low-tooth-count cases.

Order matters between 4 and 5: the Minimum Teeth check is exactly the statement that the base-height
window is non-empty, so running it first means step 5 never has to describe what to do when the
minimum exceeds the maximum.

**The bore bound is not part of this pass.** Validate the two bore diameters here only as
non-negative numbers. The whole Maximum Bore Diameter resolves in S08, because its toe term needs
the Root Length, hence the resolved Face Width, hence solved §2 geometry, and the bound is the
minimum of the two terms.

The two base-height fallbacks are the driving gear's `Module * Driving Gear Teeth Number / 8` and
the pinion's, which is the **resolved** driving base height times `Pinion Gear Teeth Number /
Driving Gear Teeth Number`. "Resolved" means after the driving side's own fallback and after the
driving Maximum Base Height capped it, never the raw driving input; the pinion's own Minimum and
Maximum Base Height then apply on top, because the two gears have different pitch cone angles
whenever the tooth counts differ.

⚠️ A configuration can satisfy every bound here and still be refused by the sketch solver as
near-singular. That limit belongs to the particular lattice, not to this spec, so it is never a
validation rule and no bound here is derived from it. Treat a near-singular report as a real refusal
of that construction, never as a tolerance to loosen.

**This step is `[PROSE]`, and this is what that costs.** The bounds themselves are proved — the §2
lattice case computes each of them from the closed form and from the solved figure, and refuses a
resolved bore diameter above its maximum. What no case reaches is the **generated module raising on
the user's value**, because the proof never runs that module. That refusal has been seen once in
Fusion, on 2026-09-16, when an over-maximum bore stopped the build; the message's wording was not
read back. There is no case to add for it.

Calls this step requires: `get_selection`, `get_boolean`, `evaluateExpression`, `itemById`,
`math.degrees`, `math.acos`, `math.atan2`, `math.sin`, `math.cos`, `math.tan`, `to_cm`.

<!-- check-step-calls: ignore get_value -->

`get_value` is named only to forbid it on the Enable Bore checkbox: it reads `input.expression`,
which a `BoolValueCommandInput` does not have, and Fusion raises `AttributeError` at generation time
(`[PB-INPUT-READ]`).

**From:** `spec/bevelgear/instructions.md` L45–L62, L91–L137, L189–L197, L333–L378, L407–L432,
L500–L515

## S05 `[PROSE]` Build the component tree

Create the occurrence tree directly with `parent.occurrences.addNewComponent(...)`, passing
`adsk.core.Matrix3D.create()` (`[PB-OCCURRENCE-TREE]`). Bevel does not use `getOccurrence`,
`addParameter`, `parameterName` or `createSketchObject` from `base.Generator`.

Create the **Bevel Gear** component as a child of the user's Parent Component and name it
`Bevel Gear`. Create the **Design** component as a child of Bevel Gear and name it `Design`; it
holds every sketch, construction plane and construction axis the build uses. `self.bevelOccurrence`
holds the top occurrence for cleanup, and `self.designOccurrence`, `self.designComponent` and
`self.bevelComponent` hold the inner tree.

**Never activate any occurrence** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`). The Anchor
sketch is created on the user's **external**, root-owned target plane, and an activated occurrence
resolves that external plane in its own local frame, so the build collapses onto world XY whatever
the real plane tilt is. Every feature runs in the single Design component, so no cross-sibling
reference is ever needed (`[PB-NO-CROSS-SIBLING]`). The sole exception is the spiral crown's
`scaleFeatures` step — see S19.

`deleteComponent()` is the error rollback the entry point calls on an exception: it calls
`deleteMe()` on the top occurrence. There are no registered user parameters to clean up.

Calls this step requires: `addNewComponent`, `Matrix3D.create`, `deleteMe`.

**From:** `spec/bevelgear/instructions.md` L29–L34, L394–L399, L588–L598, L638–L646;
`spec/bevelgear/fusion.md` L210–L215; `.claude/skills/generate-gear/PLAYBOOK.md` L811–L837

## S06 `[GO]` Anchor sketch

Start the Anchor sketch, named `Anchor`, **directly on the user-selected target plane**, whether the
selection is a `ConstructionPlane` or a `PlanarFace`. Do not re-derive or offset it
(`[PB-USE-SELECTED-PLANE]`): a coplanar construction plane built inside the sub-component resolves
in that component's own frame and silently collapses the gear onto world XY. Call `sketches.add`
with the selected entity itself.

Mark the centre by projecting the user-specified centre point into the sketch with
`sketch.project(entity)`.

Draw a line through the projected centre. **Seed its two endpoints at exactly ±0.5 cm from the
projected centre** along the sketch-local X, so the seeded length is 10 mm, with
`sketchCurves.sketchLines.addByTwoPoints`. Apply **both** `addCoincident(projectedCenter,
anchorLine)` — the intersection, pinning the centre onto the line — **and**
`addMidPoint(projectedCenter, anchorLine)`, so the centre bisects it. Use both, not midpoint alone.

Add an aligned distance dimension with `addDistanceDimension` and **do not assign
`.parameter.value`**: the dimension simply locks the length at the seeded 10 mm. The value is
arbitrary, since this is only a reference line.

Then pin its direction so the sketch ends fully constrained: `addHorizontal(anchorLine)`, which is
**sketch-local** and therefore works on any tilted target plane (`[PB-REFLINE-DIRECTION]`). A
world-axis lock would mis-orient the figure. The anchor line's absolute direction is arbitrary —
§2 derives every direction relative to it — but it must not be a free degree of freedom, and with
midpoint, length and Horizontal the line has zero.

**Stash the projected-centre `SketchPoint` on `self` as `self._anchorCenterPoint`**, so S08
re-projects *this* anchor-sketch point rather than the raw user-selected centre.

Gate the sketch on `isFullyConstrained` and **raise**, naming the sketch, if it is not
(`[BEVEL-F-FULL-CONSTRAINT]`, `[PB-FULL-CONSTRAINT]`). A free DOF here is a generation defect, not a
warning.

The proof reproduces this sketch and holds it to DOF 0. **It writes the midpoint alone**: the sketch
engine's midpoint constraint already emits the row `addCoincident(projectedCenter, anchorLine)`
would add, so writing both leaves the bench sketch redundant at DOF 0 and the gate refuses it. Both
are required in Fusion, and what the proof loses is that it does not exercise Fusion's own pairing
of the two. It also writes the length dimension in the engine's **signed** form, because the line
reversed end for end satisfies the midpoint, the length and the Horizontal equally and the engine's
probe reports that as a second configuration; the signed target carries the seed side across, which
is the documented mapping, and only `abs(target)` may go into Fusion's own `parameter.value`
(`[PB-DIM-VALUE-SEMANTICS]`). That costs nothing here, since every §2 direction is relative to this
line and the grow side is a one-bit read of the target plane's normal the bench has no plane to
take.

<!-- proof-run: proofkit.RunParallel(bevAnchorCases, stepAnchorSketch) -->

Calls this step requires: `sketches.add`, `project`, `sketchCurves.sketchLines.addByTwoPoints`,
`geometricConstraints.addCoincident`, `geometricConstraints.addMidPoint`,
`geometricConstraints.addHorizontal`, `sketchDimensions.addDistanceDimension`, `isFullyConstrained`.

<!-- check-step-calls: ignore project2 addVertical -->

`project2` is named only to forbid substituting it, and `addVertical` only to forbid it as the
direction pin; S08 gives the full reason for each.

**From:** `spec/bevelgear/instructions.md` L648–L652; `spec/bevelgear/fusion.md` L21–L30;
`.claude/skills/generate-gear/PLAYBOOK.md` L441–L450, L838–L848

## S07 `[PROSE]` Gear Profiles Plane

Create a construction plane through the Anchor Line with `constructionPlanes.createInput()` and
`setByAngle(anchorLine, ValueInput.createByString('90 deg'), targetPlane)`. Ninety degrees is
deliberate: by default the plane would lie flush to the anchor line's own plane, and this figure has
to stand perpendicular to it.

**Build it off the original `targetPlane` as the reference** and do not re-derive or offset it
(`[PB-USE-SELECTED-PLANE]`). This is the second place the target-plane orientation reaches the
bodies, and substituting a different plane here also collapses the gear onto world XY.

Pass the sketch line **directly** to `setByAngle`; never wrap it in `Path.create` first
(`[PB-CONSTRUCTION-PLANES]`), which raises `InternalValidationError` whenever the curve's owner
sketch is not trivially resolvable in a multi-component context.

Name the plane `Gear Profiles Plane` and stash it as `self._gearProfilesPlane`.

Calls this step requires: `constructionPlanes.createInput`, `setByAngle`,
`ValueInput.createByString`, `constructionPlanes.add`.

<!-- check-step-calls: ignore Path.create setByOffset -->

`Path.create` and `setByOffset` are named only to forbid them here: the first raises on a sketch
curve in a multi-component context, and the second is the "normalize the selected plane" move that
loses the plane's world orientation.

**From:** `spec/bevelgear/instructions.md` L654–L656;
`.claude/skills/generate-gear/PLAYBOOK.md` L775–L786, L838–L848

## S08 `[GO]` Gear Profiles sketch — the §2 lattice

Create a sketch on the Gear Profiles Plane, named `Gear Profiles`, and stash it as
`self._gpSketch`. This one sketch carries the whole lattice for both gears.

**Every line drawn in this sketch is a construction line** — set `isConstruction = True` on the
lattice lines, the toe lines M→N and O→P, the front faces N→A′ and P→B′, and the short
reference and connector lines M→C, O→D, A′→G, B′→I, C→K′ and D→L′ alike. The solid features
consume only the per-gear Profile sketches, never a §2 curve.

**Every length dimension in this sketch is `AlignedDimensionOrientation`.** `addDistanceDimension`
takes an `adsk.fusion.DimensionOrientations` value, and this figure has no axis-aligned line in it:
the two shaft axes sit at the Shaft Angle to each other, the whole lattice tilts with the target
plane, and the sketch is not world-aligned. A horizontal or vertical orientation would dimension the
line's *projection* onto a sketch axis instead of its length. Wherever a length is given below —
the PPD/2 and DPD/2 drops, the two `Module * 1.25` dedendum lines, the Tooth Spacing dimension on
the K′ and L′ lines, and the Toe Radius dimension on N→A′ and P→B′ — it means an aligned
distance dimension of that value. The offset dimensions are a different call, `addOffsetDimension`,
which takes no orientation.

**Every §2 line is built in the COINCIDENT style and never by sharing**
(`[BEVEL-F-COINCIDENT-STYLE]`). Create each line from raw `Point3D` coordinates with
`addByTwoPoints` and pin each connecting endpoint with exactly one
`addCoincident(line.endpoint, <existing point>)`. Sharing an existing `SketchPoint` without a
coincident leaves the sketch under-constrained; sharing **and** coinciding is redundant and the
solve fails outright with `VCS_SKETCH_SOLVING_FAILED`. This covers the short reference and connector
lines too, whose **both** endpoints already exist: `C→K`, `D→L`, `C→K′`, `D→L′`, `M→C`, `O→D`,
`N→A′` and `B′→I` each take one coincident per end. A regen that shared only those came out about
fourteen coincidents short. The two exceptions are `A′→G`, which CREATES A′, and `P→B′`, which
CREATES B′: each of those has one endpoint that does not exist yet and takes one coincident rather
than two.

**Each named line is created ONCE and later references reuse that line object**
(`[BEVEL-F-LINE-ONCE]`). A second line drawn between the same two points carries its own constraints
over the same segment, over-determines the net, and the solve fails with
`VCS_SKETCH_OVER_CONSTRAINTS`. A duplicate whose endpoints carry only per-end coincidents has been
observed to solve and pass the gate, so the solver is not a reliable tripwire for one.

**The whole figure is positioned in sketch-local 2-D coordinates** (`[BEVEL-F-APEX-LOCAL]`); never
compute a §2 position from a world round-trip, which is the single biggest source of the XY-collapse
bug. The one permitted world use is reading the target plane's normal as a *direction* to choose
which side the gear grows on (`[BEVEL-F-GROW-SIDE]`).

**Every §2 seed is load-bearing geometry.** Fifteen constraint sites admit a mirrored solution that
satisfies every constraint, and no Fusion constraint pins any of them: every geometric constraint
Fusion offers is unsigned or undirected, `addAngularDimension` takes an unsigned value plus a text
point for the quadrant, `addOffsetDimension` is unsigned, and `addDistanceDimension` is a magnitude
whose side comes from the seed (`[BEVEL-F-MIRROR-FIGURE]`). On a 43/31 pair at Shaft Angle 75
degrees with Tooth Spacing above zero that is thirteen independent binary choices and **8192**
distinct figures; on the default pair at 90 degrees with Tooth Spacing 0 it is ten choices and
**1024**. Do **not** add a Fusion constraint to pin a side, and do not set `isFixed` on a §2 point,
which over-constrains a net the closure already determines and turns a parametric lattice into
placed geometry.

Build the figure in this order.

1. **Project the Anchor sketch's centre point**, `self._anchorCenterPoint`, with
   `sketch.project(entity)` — NOT the raw user-selected centre point. Both happen to be coincident,
   but projecting the anchor-sketch point keeps the chain inside the Design component; projecting
   the raw external point is a cross-component reference and can resolve inconsistently. Project the
   anchor line the same way. **Write the call as `sketch.project(entity)` and do not substitute
   `project2`.** The compiled Fusion API reference declares `project2(entities, isLinked)` and no
   `project`, so every gate in this repo reports the call as unverified; that report is expected and
   is not a defect to fix here. The two are not interchangeable in any case — `project2` takes a
   list and returns a list — and only a Fusion session can settle whether `project` exists at
   runtime.

2. **centre → Apex.** From the projected centre `c`, draw a construction line perpendicular to the
   projected anchor line, in the sketch's own 2-D frame, with `addPerpendicular`. Its far end is the
   **Apex**, seeded in sketch-local coordinates at `c + perp·(R·cos γ_g + <resolved Driving Gear
   Base Height>)`, where `perp` is the in-plane unit perpendicular to the projected anchor line,
   `(-d.y, d.x)` for the anchor-line direction `d`. Seed it at that distance and **not** at the
   Driving Gear Pitch Diameter: the constraint net closes this line at `R·cos γ_g` above point I
   plus the resolved driving base height, so on the default 31/31 pair at Shaft Angle 90 degrees
   the old seed sat 11.6 mm past where the solve puts it — 31 mm seeded against 19.375 mm solved
   (`[PB-SEED-NEAR]`). The **sign of `perp`** is chosen by the target-plane normal as a one-bit
   direction, read as `targetPlane.geometry.normal` for **both** selection kinds, since a
   `BRepFace`'s `geometry` and a `ConstructionPlane`'s `geometry` are each a `core.Plane` carrying
   `.normal`. Pin the start with exactly one `addCoincident` to the projected centre. Do **not** add
   a length constraint on this line.

3. **Apex → B, the Driving Gear Shaft Axis**, pointing from the apex back toward the anchor line, in
   the `-perp` direction. **Seed its far end at `apex - perp·(R·cos γ_g)`, which is
   `c + perp·<resolved Driving Gear Base Height>`** — measured from the apex, not from `c`. Apply
   `addParallel(drivingShaftAxis, centerToApex)`. Do **not** use `addVertical`, which forces the
   line to the sketch's world-vertical and is wrong on a tilted target plane. Coincident its
   beginning with the apex. Its end is **point B**. Do not dimension its length.

4. **Apex → A, the Pinion Gear Shaft Axis.** Form **both** candidate point-A positions — the
   driving-shaft direction rotated about the apex by `+Shaft Angle` and by `−Shaft Angle` — and
   keep the candidate whose endpoint has the **greater X coordinate** in this sketch. Compare the
   two and take the larger; do **not** rotate one fixed sense and flip it only when its X comes out
   negative, because when *both* candidates have a positive X that shortcut keeps the wrong one.
   Call the chosen unit direction `pinionDir`. Seed A at `apex + pinionDir·(R·cos γ_p)`. Apply an
   angular dimension between this line and the Driving Gear Shaft Axis equal to the Shaft Angle with
   `addAngularDimension`, and **place its text point inside the Σ wedge** so it measures Σ and not
   its supplement (`[PB-ANGULAR-DIM]`) — for example on the interior bisector,
   `apex + normalize(pinionDir + drivingDir)·(PPD/4)`. Coincident its beginning with the apex; its
   end is **point A**. Do not dimension its length.

5. **A → Apex 2, the PPD/2 drop.** ⚠️ Apex 2 sits in the interior wedge *between* the two shaft
   axes, so this drop must point toward the **other** shaft axis, toward point B, and the
   perpendicular sense is picked by the sign of its dot product with the A→B direction — never
   against a generic "toward the anchor line" reference. Apply `addPerpendicular` against the Pinion
   Gear Shaft Axis and a dimensional constraint of length **Pinion Gear Pitch Diameter / 2**, which
   is the perpendicular distance from Apex 2 to the Pinion Gear Shaft Axis at any Shaft Angle.
   Coincident its beginning with A. **Naming convention: "A→Apex2" always means this drop line and
   never the Apex→A shaft axis.** The two share point A and are different lines. The same holds for
   "B→Apex2" against the Apex→B shaft axis.

6. **B → Apex 2, the DPD/2 drop.** ⚠️ This drop must point toward the **Pinion** shaft axis, toward
   point A; pick the perpendicular sense by the sign of its dot product with the B→A direction. Do
   **not** choose it by a "toward the anchor line" reference: the Driving Gear Shaft Axis is itself
   parallel to that grow direction, so the perpendicular's dot with it is about zero — a degenerate
   test that silently selects an arbitrary and usually wrong side. Both drops must aim at the *same*
   interior-wedge point; if this one seeds Apex 2 on the wrong side while the pinion's seeds it on
   the right one, the coincidence that closes them makes the solver **flip the whole figure**, and
   A, C, D, G, H, K, M, N and A′ all land at negative X. Nothing in the build refuses the mirrored
   figure; what catches it is the end-of-§2 gate below. Apply `addPerpendicular` against the Driving
   Gear Shaft Axis and a dimensional constraint of length **Driving Gear Pitch Diameter / 2**.
   Coincident its beginning with B.

7. **Apex 2.** Constrain the end points of the two drops with `addCoincident`. At Shaft Angle 90
   degrees Apex, A, Apex 2 and B form a rectangle; at other angles the figure is a non-rectangular
   quadrilateral and the lengths of Apex→A and Apex→B adjust so the two drops coincide. Seed the
   along-shaft lengths with the closed-form cone geometry — `|Apex→A| = R·cos γ_p` and
   `|Apex→B| = R·cos γ_g` — so the solver converges on the right branch at any Shaft Angle. Both
   cosines are positive for every angle the range check admits, which is what the Maximum Shaft
   Angle guarantees. Seeding A and B merely by a pitch diameter is wrong for Σ ≠ 90 degrees.

8. **The Pitch Line, Apex → Apex 2**, with `addCoincident` at each end.

9. **The two dedendum lines from Apex 2**, each of length `Module * 1.25` and perpendicular to the
   Pitch Line through `addPerpendicular`. The one toward the anchor line is the **Driving Gear
   Dedendum**, ending at **point D**; the one away from it is the **Pinion Gear Dedendum**, ending
   at **point C**. **Seed the two ends by dot product against the shaft axes, not by "towards or
   away from the anchor line".** Let `u` be either unit perpendicular to the Pitch Line: the pinion
   dedendum direction is the `u` with `u · <unit Apex→A> > 0`, and the driving one is its negation,
   which satisfies `(−u) · <unit Apex→B> > 0`. Those two dot products are exactly `sin γ_p` and
   `sin γ_g`, strictly positive for every admitted configuration, unlike the anchor-line test.
   Seed **C = Apex 2 + 1.25 · Module · <pinion dedendum direction>** and
   **D = Apex 2 + 1.25 · Module · <driving dedendum direction>**. ⚠️ These two sites are where the
   "C collapses onto D" symptom lives, and each is held by its seed alone: the perpendicular fixes
   the direction and the dimension the magnitude, and neither picks a side. Flip the pinion seed and
   C solves exactly onto D; flip the driving seed and D solves onto C. The collapsed figure inverts
   that gear — the toe ends up outside the heel, the revolved frustum is degenerate, and the conical
   end cut finds no cone face at the toe midpoint.

10. **The two Root Axes**, Apex → C and Apex → D, with `addCoincident` at each end.

11. **A → E and B → F.** From A, draw a line collinear with Apex→A through `addCollinear`, seeded
    at **`E = A + <unit Apex→A> · (1.25 · Module · sin γ_p)`** and with **no** dimensional
    constraint. E is the foot of the perpendicular dropped from C onto the Pinion Gear Shaft Axis,
    which is what `C→E ⊥ A→E` closes it on, so `|Apex→E| = R · cos γ_p + 1.25 · Module · sin γ_p`.
    Coincident the end of Apex→A with the beginning of the new line. Draw C → E, coincident at each
    end, and apply `addPerpendicular` between A→E and C→E. Repeat on the driving side with
    **`F = B + <unit Apex→B> · (1.25 · Module · sin γ_g)`**, collinear with Apex→B, closed by
    `D→F ⊥ B→F`.

12. **E → G and C → H.** From E draw a line collinear with **line A→E** — the collinear names A→E
    and **never the Apex→A shaft axis** further up the chain, even though both describe the same
    infinite line (`[BEVEL-F-COLLINEAR-CHAIN]`, `[PB-COLLINEAR-CHAIN]`); naming the axis raises
    `VCS_SKETCH_OVER_CONSTRAINTS`. Seed its far end at
    **`G = A + <unit Apex→A> · <resolved Pinion Gear Base Height>`**, so
    `|E→G| = <resolved Pinion Gear Base Height> − 1.25 · Module · sin γ_p`, with **no** dimensional
    constraint; that length is strictly positive because the Minimum Base Height keeps every
    resolved base height above `1.25 * Module * sin γ` with a 1.05 margin. From C draw a line seeded
    at **`H = Apex 2 + <unit Apex2→C> · (<resolved Pinion Gear Base Height> / sin γ_p)`**, so
    `|C→H| = <resolved Pinion Gear Base Height> / sin γ_p − 1.25 · Module`, again undimensioned, and
    collinear with **line Apex2→C**. ⚠️ These two seeds pick the side of the unsigned pinion
    base-height offset below: flip them and G sits one base height on the Apex side of A instead of
    beyond it, H follows, and the pinion's heel end folds back inside the figure.

13. **G → H, and the pinion base-height offset.** Connect G and H with a line, coincident at each
    end, and **apply `addPerpendicular` between E→G and H→G**. That perpendicular is required in
    Fusion: `addOffsetDimension` is a distance dimension whose documentation requires the second
    entity to be a line parallel to the first, so the parallelism has to exist before the offset can
    be applied at all, and E→G runs along the pinion shaft, so making H→G perpendicular to it makes
    H→G parallel to the A→Apex2 drop. Then create an **offset dimension between the A→Apex2 drop
    line and G→H** with `addOffsetDimension` and set its value to the **resolved** Pinion Gear Base
    Height. Add **no** extra parallel constraint (`[PB-OFFSET-DIM]`). ⚠️ `addOffsetDimension` is
    unsigned and does not pick which side of the drop G→H lands on — the G and H seeds are the only
    thing that does.

14. **F → I and D → J, then I → J and the driving base-height offset**, exactly as 12 and 13 with
    B→F for A→E, Apex2→D for Apex2→C, and the **resolved** Driving Gear Base Height for the offset
    value. Seed **`I = B + <unit Apex→B> · <resolved Driving Gear Base Height>`** and
    **`J = Apex 2 + <unit Apex2→D> · (<resolved Driving Gear Base Height> / sin γ_g)`**. The offset
    is taken between the **B→Apex2 drop line** — the DPD/2 drop, not the Apex→B shaft axis — and
    **J→I**, which is already parallel to the drop by construction. ⚠️ The driving pair's seeds
    carry the whole figure, because "constrain point I with the centre point" below hangs everything
    off I: flip this side and the entire lattice drops by twice the resolved Driving Gear Base
    Height, gear and pinion together, with every relative length still correct, which is why nothing
    downstream refuses it.

15. **A′ → G**, the pinion hexagon's shaft-axis edge. It starts at the front face's foot A′, not at
    A; the two coincide at Toe Extension 0 with a defaulted Toe Radius. **This line is what CREATES
    A′** — nothing above it does — so draw it with its start seeded at
    `A' = Apex + <unit Apex→A> · <the along-shaft coordinate of N>`, the foot of the perpendicular
    from N onto the pinion shaft axis. The front face N→A′ below is what PINS A′ to that axis; until
    then A′ is a free endpoint at its seed. Draw it **here** rather than after the front face, so the
    hexagon's edges are created in the walk order `A′ → G → H → C → M → N` the Profile sketch's
    first-edge rule depends on.

16. **Constrain point I with the centre point** using `addCoincident`.

17. **K and L.** Draw a construction line away from the Apex starting at G, ending at **K**, and pin
    K with **two point-on-line coincidents** — `addCoincident(K, line Apex→A)` and
    `addCoincident(K, the Pinion Dedendum line Apex2→C extended)` — rather than `addCollinear` on
    the connecting lines. By the time K is added G and C are already fixed, so a collinear here
    over-constrains the sketch and Fusion errors, while the two point-on-line coincidents locate K
    exactly at the intersection. Draw a reference line C → K. Do the same on the driving side with
    I, Apex→B, the Driving Dedendum Apex2→D, ending at **L**, and a reference line D → L.

18. **The tooth centres K′ and L′.** **When Tooth Spacing is 0, build nothing here** — set K′ ≡ K
    and L′ ≡ L and reuse the existing C→K and D→L reference lines, since a zero-length dimensioned
    line is degenerate and one segment gets one line. When Tooth Spacing > 0, draw a construction
    line starting at K with its far end seeded at
    **`K′ = Apex 2 + <unit Apex2→C> · (<the pinion's virtual pitch radius> + Tooth Spacing)`**,
    which is K plus Tooth Spacing along Apex2→C, on the far side of K from C. **"Virtual pitch
    radius" here is the exact back-cone radius `(Pinion Gear Pitch Diameter / 2) / cos γ_p` that S10
    defines, never a radius rebuilt from a tooth count**: reading it as a rounded count times half a
    Module puts the seed 0.4203 mm short on the shipped default geometry, which is 420 times the
    gate tolerance below. Pin its far end the same way K is pinned — `addCoincident(start, K)` and
    `addCoincident(K′, the Pinion Dedendum line Apex2→C extended)` — then add a **length dimension
    on this line equal to Tooth Spacing**; do not use `addCollinear`, for the same over-constraint
    reason as K. ⚠️ That length dimension is unsigned, so the point-on-line pin plus the length
    admit K′ one Tooth Spacing on the C side of K just as readily, the two candidates sitting
    `2 × Tooth Spacing` apart, and this seed is the only thing that rules the wrong one out. A
    flipped K′ tightens the mesh by the clearance the input asked to add and builds a gear that
    looks right. Finally draw the tooth-centre reference line **C → K′**. Build all of this **inside
    this sketch, before its end-of-step full-constraint gate**, so the gate covers it. The driving
    side is the same with L for K, D for C and Apex2→D for the pinion's dedendum, its reference line
    is **D → L′**, and its seed is
    **`L′ = Apex 2 + <unit Apex2→D> · (<the driving gear's virtual pitch radius> + Tooth Spacing)`**
    with the same exact back-cone radius `(Driving Gear Pitch Diameter / 2) / cos γ_g`. Only the
    tooth's centre moves; the virtual tooth number and the drawn tooth size are unchanged.

19. **Resolve the Maximum Face Width and apply it.** All of A, B, C, D, H and J now exist **and are
    solved**, so compute the bound from their solved `.geometry` — `pointA.geometry`,
    `pointB.geometry`, `pointC.geometry`, `pointD.geometry`, `pointH.geometry`, `pointJ.geometry` —
    and **never from the pre-solve seed coordinates** (`[PB-SOLVED-GEOMETRY]`). It is `0.95 *` the
    smaller of the perpendicular distance from A to the line through C and H, and the perpendicular
    distance from B to the line through D and J. Cap the auto default `Cone Distance / 6` to it and
    reject a user Face Width that exceeds it, naming the maximum. Seeds diverge substantially for
    asymmetric tooth counts and non-90-degree shaft angles, so a seed-based bound is too loose on the
    binding side and the toe still crosses the axis. The pinion side is normally binding, but compute
    both and take the minimum: at Shaft Angle 90 degrees the limit equals
    `0.95 * min(Driving Gear Pitch Diameter, Pinion Gear Pitch Diameter)**2 / (2 * Cone Distance)`,
    the **smaller** pitch diameter and never the pinion's by name. Written with the pinion's it is
    wrong whenever the driving gear carries the smaller count: on a Driving 17 / Pinion 31 pair at
    Module 1 the real bound is 3.883 mm and the pinion form gives 13.591. Stash the result as
    `self._faceWidthResolved_cm`. A profile that crosses its own axis of revolution fails the S14
    revolve with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`).

20. **Resolve the Root Length, the two Toe Radii and each gear's Maximum Bore Diameter.** With the
    Face Width resolved the **Root Length** follows: at Toe Extension 0 it is
    `Face Width * |Apex→Ded| / R` with `|Apex→Ded| = sqrt(R**2 + (1.25 * Module)**2)`, the resolved
    Face Width re-measured along the root element rather than perpendicular to the pitch line. Each
    gear's **Toe Radius** is the user's value, or, at the default 0,
    `this gear's Pitch Radius - Face Width / sin γ`, which is the inner toe corner radius at Toe
    Extension 0 and therefore the value that reproduces today's profile exactly. A user value must be
    **strictly below** that gear's **Toe Radius Ceiling**,
    `(this gear's Pitch Radius - 1.25 * Module * cos γ) * (1 - Face Width / R)`; reject it naming the
    ceiling. Each gear's **Toe Limit** is `|Ded→X| = sqrt(R**2 + (1.25 * Module)**2) - Toe Radius /
    sin(γ_root)` with the root cone angle `γ_root = γ - atan(1.25 * Module / R)`. **Toe Extension 100
    stops at 0.99 of the way from the Toe Extension 0 root length to the smaller of the two gears'
    Toe Limits**, not at the Toe Limit itself: at the limit the toe face has zero length, the
    revolved body carries no cone at its toe end, and the conical end cut has no `ConeSurfaceType`
    face to find. The last percent is worth well under a tenth of a millimetre of root length on
    every case in the proof's table. Do not drop that factor. ⚠️ **A defaulted Toe Radius can leave
    no room at all, and that is a real configuration rather than a defect**: on a driving gear with a
    large pitch cone angle the inner toe corner already sits at a larger radius than the outer one,
    so X falls behind the toe corner and the Toe Limit comes out below the Toe Extension 0 root
    length. **Reject a Toe Extension above 0 on such a pair**, naming the gear and the Toe Radius
    Ceiling it needs to come below; Toe Extension 0 still resolves and the gear stays buildable. Do
    **not** silently substitute a smaller Toe Radius. Then, for each gear and only when Enable Bore
    is checked, resolve **Maximum Bore Diameter = 2 * 0.95 * min(r_heel, r_toe)** with
    `r_heel = r - <this gear's RESOLVED Base Height> / tan γ` and
    `r_toe = (|Apex→Ded| - Root Length) * sin γ_root`, cap an auto-calculated bore
    (`min(this gear's Pitch Diameter / 4, Maximum Bore Diameter)`) to it, and reject a user value
    above it naming the maximum. `r_toe` at Toe Extension 0 is exactly that gear's Toe Radius
    Ceiling, and it shrinks as the Toe Extension climbs. **The flat FRONT face is deliberately not
    protected**: its radius is the Toe Radius, which is not on the body's outer envelope, so a bore
    wider than it only exits through the toe cone and the frustum stays whole. This is the only step
    at which the whole bound can resolve, which is why S04 leaves the bore diameters unbounded.
    Stash the resolved radii as `self._drivingToeRadiusResolved_cm` and
    `self._pinionToeRadiusResolved_cm`.

21. **M → N, the pinion toe line.** **Seed BOTH ends at their closed-form solved positions, not near
    them** (`[PB-SEED-NEAR]`). Seed M on Apex→C at the fraction `1 - <Root Length> / |Apex→C|` from
    the Apex. Then seed N by sliding from that M seed along the **C→H** direction by exactly
    `(<M seed's perpendicular distance from the Pinion Gear Shaft Axis> - <Pinion Gear Toe Radius>)
    / cos γ_p`. The slide runs in the C→H sense — from C toward H, the same outward sense as
    Apex2→C continued — and **along it the perpendicular distance from the Pinion Gear Shaft Axis
    FALLS at `cos γ_p` per unit**, while the along-shaft coordinate rises at `sin γ_p`. That is why
    the quantity is divided by `cos γ_p`: the slide gives back exactly the difference between the M
    seed's perpendicular distance and the Toe Radius, so N lands at the Toe Radius. **Read as a rise
    it is the wrong sign**, and it is the natural misreading, because C→H runs outward from the
    figure while the dedendum line leans back toward the axis as it goes; the compile round that
    first wrote this rule reported 20 of its 21 lattice cases failing to converge until it corrected
    the sign. The pinion's C→H unit direction is
    `sin γ_p · <unit Apex→A> - cos γ_p · <the A→Apex2 drop direction>`, and the negative second term
    is the whole of the rule. ⚠️ A seed that merely lands somewhere plausible is not enough, and a
    wrong one builds the wrong gear rather than failing to converge: N's position is fixed by the toe
    line together with an unsigned LENGTH dimension on the front face, and the toe line meets the Toe
    Radius on **both** sides of the shaft axis, so the solver takes whichever side the seed starts
    on. Seeded below the axis it converges happily onto the mirror and Fusion aborts the S14 revolve
    with `ASM_WIRE_X_AXIS`, pointing at the revolve rather than at the seed. Two earlier seeding
    rules do exactly that and must not be reinstated: sliding from the M seed by the **Root Length**,
    and sliding by the **distance from the M seed to A**. Measured on the shipped default pair at a
    50% Toe Extension, the Root Length slide puts the N seed at a perpendicular distance of
    **−0.27 mm** from the shaft axis, past it, against a solved N at **+5.17 mm**. Do not seed M and
    N just `Face Width` away from C and H either, which starts N near H. Then apply **exactly these
    three constraints**: `addCoincident(M, Pinion Root Axis)`, so M lies on the Apex→C root axis;
    `addParallel(M→N, C→H)`; and
    `addOffsetDimension(C→H, M→N, textPoint).parameter.value = <the Root Length re-measured
    perpendicular to the pitch line, i.e. Root Length * R / |Apex→C|>`. An offset dimension controls
    a perpendicular distance, so it carries the root length in that form; at Toe Extension 0 the
    value is exactly the resolved Face Width, which is what this dimension has always been. Place the
    `textPoint` in the gap between C→H and M→N on the Apex side, for example at `(M_seed + C)/2`
    (`[PB-OFFSET-DIM]`). ⚠️ The toe's side relative to the heel is held by the M seed and by nothing
    else: `addOffsetDimension` is unsigned, so a correctly built frame still admits M→N one root
    length on the *far* side of C→H, where the toe lands outside the heel and the revolved frustum is
    degenerate, and the text point does not control it either. Let the beginning of this line be
    **point M** and its end **point N**, then draw a line M → C.

22. **The front face N → A′, which is what holds N.** ⚠️ **N is NOT pinned to line A→Apex2.** It
    rides the **Pinion Gear Toe Radius** instead, and the line that holds it there is the gear's
    front face. Draw a line from N to A′, seeding A′ at N's station on the shaft axis. Apply
    `addCoincident(A′, line Apex→A)`, so A′ lies on the **Apex→A shaft axis**; A′ is the only
    toe-end point that touches that axis, and it is a *foot*, not a corner. Apply
    `addPerpendicular(N→A′, line Apex→A)`, so the front face stands square to the shaft and the
    revolve sweeps it into a flat annulus. Add `addDistanceDimension` on the whole line equal to the
    **resolved Pinion Gear Toe Radius**. ⚠️ **Pinning N itself to the Apex→A shaft axis remains
    forbidden**: that would put N on the axis of revolution, and the later conical split fails with
    `ASM_API_FAILED` for asymmetric tooth counts even though the symmetric 45-degree case happens to
    survive. Those three rows plus the offset above and `addCoincident(M, Pinion Root Axis)` fully
    constrain M, N and A′ — six freedoms, six constraints.

23. **O → P, O → D, P → B′ and B′ → I, the driving mirror.** Seed O on Apex→D at the fraction
    `1 - <Root Length> / |Apex→D|`, then P slid from that O seed along **D→J** by
    `(<O seed's perpendicular distance from the Driving Gear Shaft Axis> - <Driving Gear Toe
    Radius>) / cos γ_g`. The falling-distance rule carries over word for word, and reading it as a
    rise is the same wrong sign. Apply `addCoincident(O, Driving Root Axis)`,
    `addParallel(O→P, D→J)`, and `addOffsetDimension(D→J, O→P, textPoint)` with the Root Length
    re-measured perpendicular to the pitch line, its text point on the Apex side of D→J, for example
    at `(O_seed + D)/2`. Let the beginning be **point O** and the end **point P**, then draw O → D.
    Build the front face as the pinion's, substituting B for A, P for N and the **Driving Gear Toe
    Radius**: the line P → B′ — which is what **creates B′** — `addCoincident(B′, line Apex→B)`,
    `addPerpendicular(P→B′, line Apex→B)` and a length dimension on P→B′. P is never pinned to the
    Apex→B shaft axis; only B′ touches it. Finally draw a line from B′ to I.

24. **Gate the sketch on `isFullyConstrained` and raise**, naming the sketch, if it is not. Do not
    reach full constraint by dimensioning the *driven* §2 lines — Apex→A, Apex→B and the extension
    lines — which the perpendicular, collinear and closing constraints already determine
    (`[BEVEL-F-DRIVEN-DIMS]`, `[PB-NO-OVERCONSTRAIN]`). Undimensioned does not mean unpinned: each
    of them has a closed-form seed above, and the seed is what picks the figure.

25. **Gate the solved figure against its own seeds** (`[BEVEL-F-SEED-HELD]`). After the
    full-constraint gate passes, compare every named point's solved `.geometry` against the
    closed-form position this step seeded it at, and **raise** naming the first point that has moved,
    with its solved position and its seeded one. Check them in creation order — Apex, B, A, Apex 2,
    C, D, E, F, G, H, I, J, K, K′, M, N, A′, L, L′, O, P, B′ — so the message names the earliest
    site that flipped rather than a downstream symptom. **The list is 22 points only when Tooth
    Spacing is above zero; at Tooth Spacing 0 K′ and L′ are not built at all, so drop those two and
    compare 20** — comparing a K′ that was never created is the one way this gate raises on a
    correct figure. The tolerance is **0.001 mm**, which is 1e-4 in internal cm: two orders below the
    tenth of a millimetre of root length the spec already treats as negligible reach, and orders
    above any residue a solve can leave on a figure whose dimensions this module sets exactly.
    Compare in the sketch's own 2-D frame, against the same seed values this step computed, with no
    world round-trip. **This gate is the only measure that catches all 8192 figures**, and it names
    the point that moved instead of leaving Fusion to report an opaque failure later. **Never treat
    the revolve's `ASM_WIRE_X_AXIS` as the tripwire**: several of the flips build a valid-looking
    gear on the wrong side and reach no error at all.

The proof reproduces the whole lattice and holds it to DOF 0 with nothing redundant, nothing
conflicting and no discrete ambiguity, and it carries the same 22-point comparison as its copy of the
end-of-§2 gate. Three departures are recorded at their sites in `proof/bevelgear/sketches_test.go`:
the two base-height perpendiculars and the two toe parallels are omitted, because the engine's offset
constraint emits a row per endpoint and so carries the parallelism Fusion's `addOffsetDimension`
demands in advance — writing them leaves the lattice overconstrained at DOF 0 with the two
base-height offsets named as the redundant pair — and one row of
`addCoincident(I, projectedCenter)` is omitted for the same reason. Where §2 pins a side with an
unsigned constraint plus a seed, the proof writes the signed angle of the same arity, because the
engine signs its angle and offset dimensions and Fusion signs neither. **The proof seeds at the
closed form, so it proves the constraints solve from a correct seed and never that the module's seed
is correct**; a seed defect therefore reaches Fusion untested, which is how the wrong N seed got
there. **Shaft Angle 30 degrees stays in the case table as a declared refusal**, because this
particular net's conditioning falls below the engine's 4e-5 trust floor there; that is a property of
the net, not of the geometry, and the advertised range is not narrowed for it.

<!-- proof-run: proofkit.RunParallel(bevLatticeCases, stepGearProfiles) -->

Calls this step requires: `sketches.add`, `project`, `isConstruction`,
`sketchCurves.sketchLines.addByTwoPoints`, `geometricConstraints.addCoincident`,
`geometricConstraints.addPerpendicular`, `geometricConstraints.addParallel`,
`geometricConstraints.addCollinear`, `sketchDimensions.addDistanceDimension`,
`sketchDimensions.addAngularDimension`, `sketchDimensions.addOffsetDimension`,
`adsk.fusion.DimensionOrientations.AlignedDimensionOrientation`, `adsk.core.Point3D.create`,
`isFullyConstrained`, `math.sin`, `math.cos`, `math.tan`, `math.atan`, `math.sqrt`.

<!-- check-step-calls: ignore project2 addVertical addSymmetry isFixed addHorizontal -->

`project2` is named only to forbid substituting it for `project`; `addVertical` and `addHorizontal`
only to forbid an absolute direction lock on the shaft axes; `addSymmetry` and `isFixed` only to
record the two mechanisms that could fix a side and are refused, so the question is not re-opened.

**From:** `spec/bevelgear/instructions.md` L49–L62, L103–L135, L139–L227, L654–L805;
`spec/bevelgear/fusion.md` L69–L169, L171–L206;
`.claude/skills/generate-gear/PLAYBOOK.md` L458–L493, L591–L613, L635–L651, L720–L723

## S09 `[PROSE]` `{gearLabel} Plane` — the per-gear tooth plane

Run this and S10 through S13 **once per gear, pinion first and driving second**, with that gear's own
parameters: the pinion uses tooth centre **K′**, reference line **C→K′** and half-angle **γ_p**; the
driving gear uses **L′**, **D→L′** and **γ_g**.

Carry the per-gear anchors in **plain per-gear dicts** — `pinionCtx` and `drivingCtx` — built in
`_buildGearProfiles`. These key strings are reproduced surface exactly like the input ids: never
rename a key, split the dict, wrap it in a class, or carry one of these values under a different
shape. Both gears carry the same **eighteen** keys and no others.

| key | value it carries | type / unit | written | read by |
|---|---|---|---|---|
| `label` | `'Pinion'` or `'Driving'` | `str` | `_buildGearProfiles` | every `{gearLabel}` name and the `gearLabel` argument of `_transformToothBody` / `cut_conical_ends` |
| `teeth` | this gear's Teeth Number | `int` | `_buildGearProfiles` | Pattern `quantity`; the Meshing rotation angle; `_transformToothBody`'s `teethNumber` |
| `gamma` | this gear's pitch cone angle | `float`, radians | `_buildGearProfiles` | S10 step 1; `_transformToothBody`'s `gamma` |
| `pitchDiameter_cm` | this gear's Pitch Diameter | `float`, internal cm | `_buildGearProfiles` | S10 step 1, the virtual pitch radius |
| `toothCenterPoint` | the tooth-centre point K′ / L′ | `SketchPoint` | `_buildGearProfiles` | S10 step 3, as the spur drawer's `draw(anchorPoint, …)` anchor |
| `toothCenterRefLine` | the tooth-centre reference line C→K′ / D→L′ | `SketchLine` | `_buildGearProfiles` | S09's `plane_by_angle`; S11's `setByDistanceOnPath` helper plane |
| `hexVertices` | the six profile vertices in draw order — A′, G, H, C, M, N / B′, I, J, D, O, P | `list[SketchPoint]`, length 6 | `_buildGearProfiles` | S13 |
| `toeEdgePoints` | the toe edge's two endpoints — M and N / O and P, in that order | `tuple[SketchPoint, SketchPoint]` | `_buildGearProfiles` | the `toeMid` midpoint; its FIRST element is `toeConeWorld` |
| `heelEdgePoints` | the heel edge's two endpoints — C and H / D and J, in that order | `tuple[SketchPoint, SketchPoint]` | `_buildGearProfiles` | the `heelMid` midpoint; its FIRST element is `heelConeWorld`, the dedendum corner C / D, **NEVER** H / J |
| `boreDiameter_cm` | this gear's Bore Diameter, already resolved AND already bounded | `float`, internal cm | `_buildGearProfiles` | S25 |
| `toothPlane` | the `{gearLabel} Plane` construction plane | `ConstructionPlane` | S09 | `_transformToothBody`'s `parentToothPlane` |
| `toothSketch` | the `{gearLabel} Tooth` sketch | `Sketch` | S10 | tooth-profile selection |
| `toothEmbedded` | the drawer's `_lastToothEmbedded`, read back off the proxy | `bool` | S10 | tooth-profile selection, as `wantLines = 0 if toothEmbedded else 2` |
| `toothAxis` | the `{gearLabel} Tooth Axis` construction axis | `ConstructionAxis` | S11 | nothing — no step reads this key back, and Cleanup hides the axis by entity kind. It is listed so a regen that stashes the axis is not read as having invented a key |
| `gearOccurrence` | the `{gearLabel} Gear` occurrence | `Occurrence` | S12 | `moveToComponent`'s destination |
| `profileSketch` | the `{gearLabel} Profile` sketch | `Sketch` | S13 | S14's single profile |
| `shaftAxisEdge` | that sketch's first edge — A′→G / B′→I | `SketchLine` | S13 | the S14 revolve axis; the S22 pattern axis; the S25 bore plane's `setByDistanceOnPath`; the S26 meshing rotation; `_transformToothBody`'s `shaftAxisEdge` |
| `gearBody` | the revolved Gear Body | `BRepBody` | S14 | the S25 bore extrude's `participantBodies`. The Combine, the Meshing rotation and `moveToComponent` all run inside the same method as the Revolve and use the local body, so the key exists for the Bore alone |

**Six values are used where they are made and NEVER enter the dict**, so a regen that stashes one has
invented an entry: the **Root Axis** Apex→C / Apex→D, consumed inside S08 by
`addCoincident(M, Pinion Root Axis)`; the **toe and heel cone points**, which are the first element
of `toeEdgePoints` / `heelEdgePoints`; the **shaft-edge point pair**, which would only duplicate the
first two `hexVertices`; the **virtual tooth number**, computed in S10 step 1 and consumed in step 3;
the **root sink**, computed and consumed in the same two steps; and the **meshing rotation angle**,
computed in S26 from `teeth`.

Shared anchors are self-stashed: `self._gearProfilesPlane`, `self._apexSketchPoint`, `self._gpSketch`,
`self._apex2d` and the S06 `self._anchorCenterPoint`.

**The step itself.** Create a construction plane that includes this gear's tooth-centre reference
line, named `{gearLabel} Plane`, perpendicular to the Gear Profiles sketch plane. Build it with the
framework helper `plane_by_angle(designComponent, toothCenterRefLine, gearProfilesPlane, 90)`, which
is `setByAngle` through a sketch line passed **directly** — never wrapped in `Path.create` first
(`[PB-CONSTRUCTION-PLANES]`). Stash it under `toothPlane`.

Calls this step requires: `plane_by_angle`.

<!-- check-step-calls: ignore Path.create -->

`Path.create` is named only to forbid wrapping the sketch line in it.

**From:** `spec/bevelgear/instructions.md` L434–L488, L807–L814, L835–L836;
`.claude/skills/generate-gear/PLAYBOOK.md` L775–L786

## S10 `[GO]` `{gearLabel} Tooth` sketch — the virtual spur tooth

**Step 1 — the virtual tooth number and the root sink, computed and consumed here.** Compute this
gear's virtual (back-cone, Tredgold) tooth number from the closed form and **not** by measuring
Apex2→K′: the virtual pitch radius is `(this gear's Pitch Diameter / 2) / cos(γ)` and the virtual
tooth number is `2 · virtualPitchRadius / Module`, equivalently `this gear's Teeth / cos(γ)`.

**It is a real number and is NEVER rounded** — not floored, not ceiled, not cast to an int. The
Tredgold construction puts the equivalent spur gear's pitch radius exactly at `r / cos γ` with this
gear's own module, and `z_v = z / cos γ` is a real number in every published form of it. Rounding it
rebuilds every drawn circle from the rounded count, which draws the tooth smaller than the back cone
places it and shortens the working addendum: on the shipped default — 31 teeth, Module 1, Shaft Angle
90 degrees, γ = 45 degrees — the exact virtual pitch radius is 21.9203 mm, a floored count of 43
draws 21.5 mm, and the addendum the tooth works over falls to 0.5797 mm against a nominal 1.0 module.
The real count reaches the drawer only as an angular half-thickness: it reads `ToothNumber` as a
float and uses it in one place, `π / (2 · toothNumber)`, and with `z_v = 2 · r_v / Module` that gives
a tooth thickness of `π · Module / 2` at the pitch circle, the standard thickness. An integer count
drawn at the exact radius gives `π · r_v / round(z_v)` instead, which misses nominal by a different
amount on each member of an unequal pair.

**Root sink.** Draw the root circle one **root sink** `0.05 · 2.25 · Module` **inside** the dedendum
corner rather than at it. At the dedendum corner exactly, the tooth's root arc touches the gear
body's root cone only where the arc crosses the tooth's own centreline — the tooth is drawn on the
back-cone plane, so only a point on that centreline rides the cone its own polar radius names, and
the arc's two corners stand outside it, by 0.002 module on the default pair and 0.027 module on a 4/4
pair, the largest of any pair the spec admits. The sink pushes the whole arc inside, so the S23
Combine-Join meets the gear body across the root rather than along one line.

So the four circles the proxy is asked for are:

| circle | radius |
|---|---|
| pitch | `virtualPitchRadius` |
| base | `virtualPitchRadius · cos(20°)` |
| tip | `virtualPitchRadius + Module` |
| root | `virtualPitchRadius − 1.25 · Module − rootSink` |

**Units — pin the cm→mm conversion:** the stashed pitch diameters are internal **cm** while Module
is the raw **mm** value, so compute `virtualPitchRadius_mm = (pitchDia_cm · 10 / 2) / cos(γ)` — the
`· 10` converts cm to mm — and then `virtualTeeth = 2 · virtualPitchRadius_mm / Module`. Skipping the
×10 makes the virtual tooth count about ten times off.

**Step 3 — draw the tooth.** Create a sketch on the `{gearLabel} Plane`, named `{gearLabel} Tooth`,
and draw a spur gear tooth profile centred on the tooth-centre point with the **borrowed** spur tooth
generator:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth, rootSink_mm=rootSink)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))   # the 180 degree tooth rotation IS the draw() angle
```

The proxy is the framework's `VirtualSpurProxy` from `lib/geargen/spurproxy.py` — import it and do
**not** define a local copy. It precomputes, in internal cm, exactly the parameter keys the spur
drawer reads through `parent.getParameter(name).value`, with its own defaults matching bevel: a
pressure angle of 20 degrees, which is not a bevel dialog input, and `InvoluteSteps` 15.
`virtualTeeth` is a **real** number here, and the proxy computes the pitch diameter as
`virtualTeeth · module_mm`, so passing the exact `2 · r_v / Module` is what makes the drawn pitch
circle reach the back cone; `rootSink_mm` shortens the root diameter by twice its value and leaves
pitch, base and tip alone.

**The 180-degree rotation is delivered through the `draw()` angle argument**, never by a post-hoc
Move or sketch rotation: the drawer rotates the whole tooth by `angle`, which relies on spur's radial
flank-to-root pinning so the connecting lines rotate with the tooth.

**The proxy carries `_lastToothEmbedded`, an OUTPUT the drawer writes, and bevel MUST read it back.**
During `draw()` the drawer decides whether the tooth is *embedded* — tip, root and flanks meeting
with no connecting lines — and records it on the proxy. After `draw()` returns, read
`proxy._lastToothEmbedded` and stash it under `toothEmbedded`. This is not optional bookkeeping: it
is the deterministic selector for the tooth loop's line count in S15, and skipping it and accepting
either count grabs an unrelated loop and the loft dies with `LOFT_NO_TOOLBODY`.

**After `draw()` returns do NOT hard-gate this sketch**: log it with `futil.log` if
`not toothSketch.isFullyConstrained`, and never raise. The tooth-profile sketches are exempt from the
full-constraint gate, and the exemption covers **the labels and nothing else**: the drawer labels
each of the four circles with along-path sketch text, and sketch text holds a DOF
(`[PB-TEXT-HOLDS-DOF]`), so a sketch whose geometry is completely determined still reads `False`
purely because it is labelled. ⚠️ Never read that as licence for loose geometry. Its earlier wording
claimed the embedded tooth kept a free radial DOF; that was wrong, the residual DOF was the tooth-top
arc's **centre**, which `addByCenterStartEnd` copies rather than shares, and the fix was one
coincident constraint in the shared drawer. That wrong reasoning let a deformed tooth ship: measured
in Fusion on 2026-09-02 on a default 31/31 pair, the pinion's tooth-top arc came out at 0.5743 mm and
the driving gear's at 17.0204 mm where both should have been the 22.5 mm tip radius.

Stash the sketch under `toothSketch`.

**The tooth-top arc is a CENTRE-POINT arc with a pinned centre and no dimension.** The drawer creates
it with `addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)` and then pins the
copied centre with `addCoincident(arc.centerSketchPoint, localOrigin)`, carrying no radius and no
diameter dimension. Bevel's own step authors no arc. **Never write that Fusion draws this arc as a
three-point arc and dimensions its radius**: that sentence sat in a proof comment once and sent an
investigation chasing a reflected centre this construction has no room for, because the centre is
pinned. The §3a trace arc in S16 is the other case and genuinely is a three-point arc with a radius
dimension, and the two must not be written from one template.

The proof reproduces the drawer's own scheme with this gear's virtual tooth count and the sunk root.
It writes the faithful `addCoincident(arc.centerSketchPoint, localOrigin)` and the sketch solves with
it, so the cost is nil and `proof/bevelgear/sketches_test.go` records that at the site. It asserts
the four radii against the closed form, that the root radius is one root sink inside the dedendum
corner, that the drawn pitch radius is the exact back-cone radius and not the floored count's, that
the arc's centre gap is zero, and that the tooth loop carries 2 NURBS, 2 arcs and the line count the
embedded flag determines. What it cannot reach is Fusion's own profile finder selecting that loop and
the loft consuming it, which is what the 2026-09-16 load showed.

<!-- proof-run: proofkit.RunParallel(bevToothCases, stepToothProfile) -->

Calls this step requires: `sketches.add`, `VirtualSpurProxy`,
`SpurGearInvoluteToothDesignGenerator`, `draw`, `math.radians`, `math.cos`, `futil.log`,
`isFullyConstrained`.

<!-- check-step-calls: ignore addByCenterStartEnd addCoincident getParameter round floor addByThreePoints addRadialDimension -->

`addByCenterStartEnd`, `addCoincident` and `getParameter` are named to describe what the **borrowed
drawer** does — the first two inside `draw()`, and `getParameter` being how the drawer reads its
parameters out of whatever `parent` it is handed. Bevel's own part is to CONSTRUCT the
`VirtualSpurProxy` that answers those reads; this module never calls `getParameter` itself, and a
bevel module that did would be reaching for a `base.Generator` helper it does not inherit. `round`
and `floor` are named only to forbid rounding the virtual tooth count; `addByThreePoints` and
`addRadialDimension` are named only to forbid the three-point-with-radius reading of the tooth-top
arc.

**From:** `spec/bevelgear/instructions.md` L546–L556, L600–L634, L807–L838, L1324–L1360;
`spec/bevelgear/fusion.md` L31–L58; `.claude/skills/generate-gear/PLAYBOOK.md` L188–L204,
L517–L532

## S11 `[PROSE]` `{gearLabel} Tooth Axis`

Create a construction axis through the tooth-centre point, normal to the plane the tooth profile was
drawn on, named `{gearLabel} Tooth Axis`. Build it with `constructionAxes.createInput()` and
`setByTwoPlanes(planeA, planeB)` (`[PB-CONSTRUCTION-AXES]`), because `setByPerpendicularAtPoint`
would need a `BRepFace` this build does not have.

The two planes are the **Gear Profiles plane** and a **helper plane** built with
`setByDistanceOnPath(<tooth-centre reference line>, 1.0)`, which is perpendicular to that line at its
far end, the tooth-centre point; their intersection is the line through the tooth centre normal to
the tooth plane. Pass the sketch line **directly** to `setByDistanceOnPath`; never wrap it in
`Path.create` (`[PB-CONSTRUCTION-PLANES]`).

Creating this axis in the never-activated Design component is proven to work — `constructionAxes.add`
through `setByTwoPlanes` does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here. Keep the axis. Stash it
under `toothAxis`; nothing reads that key back, and Cleanup hides the axis by entity kind.

Calls this step requires: `constructionAxes.createInput`, `setByTwoPlanes`, `constructionAxes.add`,
`constructionPlanes.createInput`, `setByDistanceOnPath`, `constructionPlanes.add`.

<!-- check-step-calls: ignore setByPerpendicularAtPoint Path.create -->

`setByPerpendicularAtPoint` and `Path.create` are named only to forbid them.

**From:** `spec/bevelgear/instructions.md` L807–L810, L839;
`.claude/skills/generate-gear/PLAYBOOK.md` L787–L799

## S12 `[PROSE]` `{gearLabel} Gear` component

Create a new component as a child of the **Bevel Gear** component — the same component that owns
Design, and **not** the user's Parent Component; this intentionally overrides the looser "child of
Parent Component" phrasing so the pair nests cleanly inside Bevel Gear. Name it `{gearLabel} Gear`,
so `Pinion Gear` and `Driving Gear`. Stash the occurrence under `gearOccurrence`.

The finished bodies for this gear end up here. Fusion rejects cross-sibling sketch and `project`
calls even when the target is activated or the entities are wrapped in `createForAssemblyContext`
proxies (`[PB-NO-CROSS-SIBLING]`), so every feature operation runs in the Design component and the
finished bodies are moved here with `moveToComponent` at the end of S26. The visible end state is
identical.

Calls this step requires: `addNewComponent`.

<!-- check-step-calls: ignore createForAssemblyContext -->

`createForAssemblyContext` is named only to record that it does not help.

**From:** `spec/bevelgear/instructions.md` L971–L985;
`.claude/skills/generate-gear/PLAYBOOK.md` L829–L834

## S13 `[GO]` `{gearLabel} Profile` sketch — the hexagon

Run this and S14 to S26 **once per gear**, pinion fully first and then the driving gear: the profile
and the body are **interleaved per gear**, so pinion profile, pinion body, driving profile, driving
body — never both profiles and then both bodies. The substitutions are:

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A′ → G → H → C → M → N → A′ | B′ → I → J → D → O → P → B′ |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A′→G | B′→I |
| toe cut edge | M→N | O→P |
| heel cut edge | C→H | D→J |
| teeth / bore / pitch-diameter inputs | Pinion Gear … | Driving Gear … |
| §2 shaft construction line (NOT usable as the axis) | Apex→A | Apex→B |

Open a **fresh sketch on the axial (Gear Profiles) plane**, named per the table — **one profile
sketch per gear**, so `sketch.profiles` holds exactly this one hexagon loop. Do not draw both gears'
hexagons in the shared Gear Profiles sketch, which would leave two identically-shaped loops to
disambiguate.

Build the hexagon on fixed vertices per the `[PB-PROJECT-NOT-FIXED]` recreate-share-fix recipe:
recreate the six §2 vertices as new points at their exact world-mapped positions with
`sketch.modelToSketchSpace(src.worldGeometry)` and `sketchPoints.add`, which is valid because §2 is
fully constrained by now; then draw the closed hexagon in the table's draw order as six
`SketchLine`s **sharing** those points; then set `isFixed = True` on the lines' endpoints **after**
the lines exist. Order matters: fixing a bare point before it is consumed as a line endpoint does
**not** leave the sketch fully constrained. A projection would not fix the geometry at all — a
projected point is brought in associatively and still carries free DOF.

**The hexagon's first edge is the gear's shaft axis** for the revolve, the pattern, the bore plane
and the meshing rotation, so it must be fixed well enough to carry a trustworthy world position:
fixed endpoints give that edge a well-defined `worldGeometry` (`[PB-WORLDGEO-CONSTRAINED]`), while a
free edge resolves against a default world frame and silently moves the body onto world XY, which
was observed on the driving gear — the pinion looked fine only because it never read the edge's
`worldGeometry`.

**The shaft axis every body operation below uses is this sketch's first edge, NOT the §2 Apex→A or
Apex→B construction line.** The edge is collinear with the shaft axis but lives in the *same* sketch
as the profile, which is what Fusion's revolve, pattern and path builders accept; reusing the §2
construction line, which lives in another sketch, fails or misbuilds.

Gate the sketch on `isFullyConstrained` and raise, naming it, if it is not. Stash the sketch under
`profileSketch` and its first edge under `shaftAxisEdge`.

The proof reproduces this sketch and holds it to DOF 0, takes its single region, and asserts the
region's area against the hexagon's own shoelace area, that the region is valid and current and
carries no approximate trim, and that the first edge lies on the shaft axis with both endpoints at
radius zero.

<!-- proof-run: proofkit.RunParallel(bevProfileCases, stepProfileSketch) -->

Calls this step requires: `sketches.add`, `sketchPoints.add`, `modelToSketchSpace`, `worldGeometry`,
`sketchCurves.sketchLines.addByTwoPoints`, `isFixed`, `isFullyConstrained`.

<!-- check-step-calls: ignore project -->

`project` is named only to forbid it here; a projection does not fix the recreated vertices.

**From:** `spec/bevelgear/instructions.md` L503–L510, L971–L990;
`spec/bevelgear/fusion.md` L21–L30;
`.claude/skills/generate-gear/PLAYBOOK.md` L458–L472, L585–L590, L598–L605

## S14 `[GO]` Revolve the Gear Body

This sketch holds exactly one hexagon loop, so take its single profile with `profiles.item(0)` and do
not filter (`[PB-SINGLE-PROFILE]`): a curve-type filter has spuriously rejected a valid all-line loop
and made a revolve fail with "could not find profile".

Revolve it around the **shaft-axis edge** with `revolveFeatures.createInput(profile, axis,
operation)`, then `setAngleExtent(False, ValueInput.createByString('360 deg'))`, then
`revolveFeatures.add(input)` (`[PB-REVOLVE]`). The result is the **Gear Body**, the frustum; stash it
under `gearBody`.

Because the toe edge is one edge of the revolved profile, the body already carries the conical face
that edge sweeps about the axis, and that face is reused as the cutting tool in S21; likewise the
heel edge's cone. ⚠️ The profile must **not** cross the axis of revolution, or Fusion aborts with
`ASM_WIRE_X_AXIS`. That is what the Maximum Face Width of S08 and the strictly positive Toe Radius
are for; reproduce both caps exactly.

The proof revolves the same hexagon a full turn about the station axis with decad's own Revolve,
which returns the whole frustum as one body, and **substitutes nothing**. It asserts the volume
against Pappus on that same hexagon, with no polygon correction and no decomposition into bands; the
two agree to 3.4e-16 relative at worst, inside the bound decad publishes for the reading, and because
that bound is `Approximate` rather than `Exact` the comparison is written with a relative tolerance.
It then asserts the body's **five faces**, matched by surface kind and by their own readings rather
than by the order the face selector hands them back: the flat heel face at the back, a disc of the
heel radius; the flat toe face at the front, a disc of the Toe Radius; and three cone faces, whose
areas are those of the cone frusta the edges C→H, M→C and N→M sweep. **Each cone's half-angle is read
off its own face** — a `decad.Cone` publishes it — rather than derived from two cap radii and a
height; the heel cone and the toe dish read `90° − γ` and the root cone reads this gear's root cone
angle. This step costs the proof nothing: the frustum is one watertight body with the right volume,
the right two flat faces and the right three cone faces, and there is no union left to owe. It is a
real Revolve, and it is the one body no boolean in this proof consumes — every boolean operand
elsewhere is a Loft, because at the pinned decad revision every boolean with a Revolve operand
verified Suspect, on 100 of 160 with the volume itself beyond tolerance, and neither gate admits that
verdict.

<!-- proof-run: proofkit3d.RunSolidParallel(bevSolidCases, stepRevolveGearBody, assertRevolveGearBody) -->

Calls this step requires: `profiles.item`, `revolveFeatures.createInput`, `setAngleExtent`,
`ValueInput.createByString`, `revolveFeatures.add`.

<!-- check-step-calls: ignore profileLoops profileCurves -->

`profileLoops` and `profileCurves` are named only to forbid filtering a sketch that holds one loop.

**From:** `spec/bevelgear/instructions.md` L199–L205, L991, L1080–L1094;
`.claude/skills/generate-gear/PLAYBOOK.md` L494–L500, L718–L723

## S15 `[GO]` Loft the Tooth Body

Loft the **§2 Apex sketch point** — the `centerToApex.endSketchPoint` from the Gear Profiles sketch,
the degenerate point-section — to this gear's `{gearLabel} Tooth` profile, with
`loftFeatures.createInput(operation)`, `loftSections.add(entity)` per section in order, and
`loftFeatures.add(input)` (`[PB-LOFT]`). The result is the **Tooth Body**.

Use the §2 Apex **SketchPoint** directly and do **not** create a construction point for it
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`): construction geometry needs an active component and the Design
component is never active, while a `SketchPoint` works as a loft point-section with no activation.

Select the tooth cross-section loop with
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)`, where the line count
is **DETERMINED BY** the `toothEmbedded` flag S10 read back off the proxy:
`wantLines = 0 if toothEmbedded else 2`. ⚠️ Do **not** accept "0 **or** 2 lines". For a given gear
only one of those is the real tooth, and an unrelated loop — an inter-tooth or annular region between
the drawer's circles — can also have 2 NURBS and 2 arcs with the *other* line count; selecting it
makes this loft fail with `RuntimeError … ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`, because the impostor
loop cannot form a loft tool body. Embedded means tip, root and flanks meet with no connecting lines,
so four curves; non-embedded means two connecting lines, so six.

The proof **substitutes a shrunken section for the degenerate apex point, and nothing else in the
placement**: the tooth plane is not substituted — it builds the real back-cone plane, tilted out of
the axis-perpendicular by γ, and takes the apex's perpendicular distance to the section as
`sK · cos γ`, which is the Pitch Cone Distance, rather than `sK`. It asserts the lofted volume
against the frustum-of-a-cone-from-the-apex closed form on that section, and reads the built body out
to the virtual tip radius laid on the back cone. **The cost is the point section**: the loft's
degenerate end is not built, and what the volume and the two cone slopes prove is the taper it has to
produce. A second substitution the spec does not name is forced by the evaluator: decad's Loft pairs
two sections segment by segment and refuses a free-form pair outright, so every flank is **chorded**
through the same involute sample points the Fusion spline interpolates, and the two arcs are chorded
too. What that costs is the wall surface between two chords. The chorded section also **opens where
the flank meets the root circle**, which is the boundary Fusion's own profile detection produces by
splitting the solid root circle at the flank: the flank-to-root stub exists only where the tooth is
not embedded, and keeping it on an embedded tooth makes the section cross itself, because the flank
starts inside the root circle there. This gear reaches the embedded shape often, since its virtual
tooth count is `z / cos γ` and climbs without bound with the pitch cone angle.

<!-- proof-run: proofkit3d.RunSolidParallel(bevSolidCases, stepLoftTooth, assertLoftTooth) -->

Calls this step requires: `loftFeatures.createInput`, `loftSections.add`, `loftFeatures.add`,
`find_profile_by_curve_counts`.

<!-- check-step-calls: ignore constructionPoints.add -->

`constructionPoints.add` is named only to forbid it as the loft's point section.

**From:** `spec/bevelgear/instructions.md` L546–L556, L628, L992–L993, L1095–L1099;
`.claude/skills/generate-gear/PLAYBOOK.md` L724–L728, L791–L799

## S16 `[GO]` Spiral: the `{gear} 2D Tooth Trace` cutter arc

Everything from here to S20 runs **only when ψ > 0**. `_transformToothBody(designComponent,
toothBody, gearBody, shaftAxisEdge, apexWorld, apexSketchPoint, toeMid, heelMid, toeConeWorld,
heelConeWorld, parentToothPlane, gearLabel, teethNumber, gamma)` is the single tooth-body hook
`_createGearBody` calls after lofting the uncut apex-to-heel tooth, and **its first line is the gate
`if self._spiralAngle_rad <= 0: return cut_conical_ends(...)`** — a straight bevel is byte-for-byte
the prior behaviour and skips S16 through S20 entirely, going straight to S21.

**Caller hand-off — the four toe and heel world points `_createGearBody` builds and passes,
positionally, in the order `toeMid, heelMid, toeConeWorld, heelConeWorld`. Pin these exactly;
mislabelling them silently inverts the spiral, and this is the single biggest spiral-regen hazard.**

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

`toeMid` is the world **midpoint of the TOE edge**, ½(M+N) or ½(O+P); `heelMid` is the world
**midpoint of the HEEL edge**, ½(C+H) or ½(D+J); `toeConeWorld` is the toe edge's inner endpoint M or
O, which lies on the root cone element Apex→C / Apex→D; and `heelConeWorld` is the **dedendum corner**
C or D, the outer end of that same element. ⚠️ Two scrambles to avoid, both of which a fresh regen
has made: do **not** pass the two endpoints of a *single* edge as `toeMid` and `heelMid`, because M
and N both sit at the toe and the span collapses to about zero or negative and the spiral inverts;
and `heelConeWorld` is **never** H or J, which lie on the Apex2→C / Apex2→D dedendum line one Module
beyond C and D, **off** the root cone element.

**A. Gate and frame.** Build a world frame from this gear's existing geometry. `axisDir` is the
shaft axis direction, from the two **world** endpoints of `shaftAxisEdge`, normalized
(`[PB-WORLD-FRAME]`). `coneVec = normalize(heelConeWorld − apexWorld)`, the dedendum root cone
element. `v = normalize(axisDir × coneVec)`, the circumferential direction. `tpNormal =
normalize(coneVec × v)` completes the frame and **nothing consumes it** — step D removed the
projection that once used it, so it is computed and left unread. `distAlong(p) = (p − apex) · coneVec`
is a point's cone distance.

⚠️ **The heel MUST be the OUTER end so `coneVec` points outward and `span > 0`.** Before building
`coneVec`, check the passed midpoints and **fix a swapped toe and heel**: if
`apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid` with `heelMid` **and**
`toeConeWorld` with `heelConeWorld`, then build `coneVec` from the corrected heel point. A negative
span silently inverts the entire spiral frame — it flips the cutter-arc direction, the slice
direction and the per-segment twist — and the gear comes out completely wrong with no error.

From the corrected midpoints take `R_toe = distAlong(toeMid)`, `R_heel = distAlong(heelMid)`,
`R_mean = ½(R_toe + R_heel)` and `span = R_heel − R_toe`, the face width, now positive. Those are the
only quantities the rest of the build needs.

**B. Cutter-arc geometry.** Work in the tangent-plane 2-D frame with the apex at the origin,
**x = coneVec** so a point's x is its cone distance, and **y = v**. The cutter radius `r_c` is the
Cutter Radius input if non-zero and **`R_mean`** otherwise. The hand sign is `+1` for `Right` and
`−1` for `Left`, **then negated for the pinion**, because the pair meshes with opposite hands. The
cutter-circle centre is

```
Cx = R_mean − r_c · sin ψ
Cy = handSign · r_c · cos ψ
```

⚠️ **The hand sign goes on the `cos` / `Cy` term, NOT the `sin` / `Cx` term.** This was a real bug.
Opposite hand mirrors the cutter centre **across the cone element**, which flips `Cy`; putting
`handSign` on `Cx` mirrors about `x = R_mean` instead, a *different* curve that gives the two gears
unequal twist, where for equal teeth the two traces must come out exact mirror images.

The trace's toe and heel endpoints are circle-circle intersections taken a hair **past** the face so
the kept arc reaches cleanly past the end trims:
`toe2d = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)` and
`heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)` with
`R_lo = R_toe − 0.06·span` and `R_hi = R_heel + 0.06·span`. The helper intersects the apex circle of
radius R with the cutter circle and keeps the solution nearest `(R_mean, 0)`, the branch the mean
point sits on; a non-overlapping pair clamps to tangency.

**C. The trace sketch.** First draw a **cone-element construction line** from the apex to
`apex + R_heel·coneVec` in a sketch on the **axial (Gear Profiles) plane**, named
`{gear} Cone Element`. Then make the tangent plane as that axial plane rotated **90 degrees** about
that line, with `plane_by_angle(designComponent, coneElementLine, axialPlane, 90)`, named
`{gear} Trace Plane`. Add a sketch on it named **`{gear} 2D Tooth Trace`**, and in it draw, with
`tanW(px, py) = combine_point(apex, px, coneVec, py, v)` mapping 2-D coordinates to world:

- the **cutter circle** — centre `tanW(Cx, Cy)`, radius `r_c` — with `isConstruction = True`, its
  centre pinned by `centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`) and a diameter dimension
  of `2·r_c`;
- the **trace arc** — a three-point arc through `tanW(toe2d)`, `tanW(R_mean, 0)` and `tanW(heel2d)`
  — with its **centre coincident to the cutter circle's centre** and a **radius dimension of `r_c`**,
  so it is the genuine cutter circle and not a look-alike spline. Text points go off-centre, on or
  near the curve (`[PB-RADIAL-DIM]`): use the mean point for the arc's radius dimension and a point
  such as `tanW(Cx + r_c, Cy)` for the circle's diameter dimension.

**Coordinates — this rule governs the `{gear} Cone Element` sketch as well as the trace sketch.** The
world `Point3D`s from `tanW(...)`, and the raw apex and cone-end points of the cone-element line, are
passed **directly** into the sketch calls, where they are consumed as **sketch-space** input, with
**no `modelToSketchSpace` conversion**, even though the points really are model-space coordinates.
That is deliberate and harmless, for a reason that is not the obvious one: the trace sketch is
construction and reference only and **no downstream feature ever consumes it** — the twist is computed
analytically in S18 and the sketch exists only so the genuine cutter arc is inspectable before
cleanup hides it. The cone-element line *is* consumed, by `plane_by_angle`, so an unconverted line
does place the Trace Plane somewhere other than the true tangent plane; that still reaches no
feature, because the only thing built on the Trace Plane is the inspection-only trace sketch. **If a
later revision ever makes any feature consume the trace sketch or the Trace Plane, this shortcut
stops being safe and both sketches need `modelToSketchSpace` on every point.**

This sketch is **deliberately left with free DOF** — the arc's endpoints are pinned by the
three-point construction, not by endpoint dimensions, since dimensioning them over-constrains the
solve against the cone-element plane. It is therefore **exempt from the full-constraint gate**; do
not gate it.

The proof reproduces the cutter circle and the arc in the tangent-plane frame and asserts the centre
against the closed form, that the hand flips `Cy` and leaves `Cx` alone, that the centre is exactly
`r_c` from the mean point so the arc passes through it, and that each endpoint sits on its own apex
circle. Because proofkit's gate waives nothing, the proof **pins those two endpoints**, each by its
signed angular position on the cutter circle measured from the cone element. Pinning an end by its
cone distance does not settle it: a circle pair meets in TWO points, so the far branch stands beside
the near one and the probe reports four configurations. §6 of `spiral-tooth-trace.md` says to keep
the branch the mean point sits on, and that is a SIDE, which only a signed constraint states; that
each pinned end really is the circle-circle intersection is then asserted rather than assumed, by
reading its cone distance back. The free-DOF sketch Fusion actually authors is therefore not the
sketch solved on the bench, and `proof/bevelgear/sketches_test.go` records that at the site.

<!-- proof-run: proofkit.RunParallel(bevSpiralSketchCases, stepSpiralTrace) -->

Calls this step requires: `plane_by_angle`, `combine_point`, `circle_intersect_nearest`,
`sketches.add`, `sketchCurves.sketchLines.addByTwoPoints`,
`sketchCurves.sketchCircles.addByCenterRadius`, `sketchCurves.sketchArcs.addByThreePoints`,
`sketchDimensions.addDiameterDimension`, `sketchDimensions.addRadialDimension`,
`geometricConstraints.addCoincident`, `isFixed`, `isConstruction`, `math.sin`, `math.cos`,
`cut_conical_ends`.

<!-- check-step-calls: ignore modelToSketchSpace projectToSurface -->

`modelToSketchSpace` is named only to record that it is deliberately NOT applied here and what would
make it necessary; `projectToSurface` only to forbid reintroducing the removed 3-D projection.

**From:** `spec/bevelgear/instructions.md` L511–L529, L841–L896;
`spec/bevelgear/spiral-tooth-trace.md` L32–L200;
`spec/bevelgear/fusion.md` L59–L67;
`.claude/skills/generate-gear/PLAYBOOK.md` L439, L451–L457, L652–L656

## S17 `[GO]` Spiral: slice the straight tooth and drop the apex scrap

**E. Slice.** Split the uncut apex-to-heel `toothBody` into cross-section slabs by planes **parallel
to the parent transverse tooth plane** — `parentToothPlane`, the `{gearLabel} Plane` from S09 — with
a **fixed scheme of exactly eight planes**. The count is not user-configurable.

⚠️ **The slice planes are NOT perpendicular to the cone element.** The parent plane carries the
tooth-centre line C→K′ / D→L′, which is the back-cone line and so perpendicular to the Pitch Line,
so **the parent plane's normal runs along the PITCH element** while `coneVec` is the **ROOT**
element. The two differ by the dedendum angle `δ_f = atan(1.25 · Module / R)`, equivalently
`atan(2.5 · sin γ_p / N_p) = atan(2.5 · sin γ_g / N_g)`: Module cancels, so `δ_f` depends only on the
tooth counts and the Shaft Angle and is the **same for both members** — 3.26 degrees on the default
31/31 pair at Shaft Angle 90 degrees, growing as the tooth counts fall. The parallel family is what
the build *requires* rather than what it happens to use: `slice_body_by_offset_planes` offsets the
parent plane with `setByOffset`, which produces parallel planes; the sign test below reads the
**parent plane's own normal**, which is meaningful only for that plane's own offsets; and the tooth
is lofted from the Apex to the profile drawn in the parent plane, so **the heel-most slab's heel face
IS the parent plane** and a consistent family has to contain it. ⚠️ **A build that follows
"perpendicular to the cone element" instead is wrong and silent**: it tilts every cut face by `δ_f`
and nothing in the pipeline fails, because parallel planes cut a cone in similar sections whatever
their orientation, the piece count and the retry gate and the S21 trims are all indifferent to slab
orientation, and the runtime gate only counts pieces. What moves is the geometry: on the default pair
at Module 4 a face corner lands `1.125 · Module · tan δ_f` = 0.26 mm along the cone from where the
parallel cut puts it, and the S18 twist keyed across one face mismatches by up to 0.0078 rad.

The first cut plane is the parent plane offset toward the apex by `span/6`, and the **offset sign is
chosen per gear** so it moves apex-ward: the parent plane's normal points opposite ways for the two
gears, so pick `sign` so that `sign·normal` points toward the apex, testing
`(apex − planeOrigin) · normal`. The other seven step further toward the apex in `span/6`
increments, so `offsets = [sign·(k+1)·span/6 for k in 0…7]` with `k = 0` the first cut plane.
**Where the eight land: the first sits `span/6` inside the HEEL and none of them lies past it** —
the parent plane is already the heel end, so there is no heel overshoot to give — **the sixth lands
at the toe, and the last two sit `span/6` and `2·span/6` PAST the toe**; those two segments are what
S21's toe cone trims away. The first sits a hair more than `span/6` inside the heel and the sixth a
fraction of a millimetre inside the toe, because `R_heel` and `R_toe` are read at the two edge
midpoints rather than on the root element.

Split the body with `slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane,
offsets)`, which splits piece by piece and keeps a piece whole when a plane misses it.

⚠️ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in **one
piece**, the offset sign was wrong or `parentToothPlane` sits outside the tooth's span — **retry the
whole cut once with the opposite sign**. If it is *still* one piece, `raise` a clear self-diagnosing
error naming the gear, the final piece count, `span` and the sign tried (`[PB-SELF-DIAGNOSING]`,
`[PB-EMPTY-RESULT]`). Do **not** return an unsliced single-piece result: step F then drops that one
piece as the apex scrap, leaving `segments` empty, and the crown later crashes with
`ValueError: max() iterable argument is empty` far from the cause.

**F. Order and drop the scrap.** Sort the segments by the `distAlong` of their centroid, read through
`physicalProperties.centerOfMass`. The first, apex-most one is the long apex-side scrap below the
toe — **remove it** and keep the rest as the working `segments`. Drop it by **re-slicing the list
first and deleting second**: `segments = segments[1:]` before `removeFeatures.add(scrap)`. After the
drop, `segments` must be **non-empty**; if it is empty the slice failed in E, so `raise` a clear error
rather than proceeding into the twist and the crown, which both assume at least one segment.

The proof **builds the slabs rather than cutting them**, because decad has no split-by-plane: each
slab is lofted between the two sections the parallel cut planes produce, and those sections are the
full back-cone section scaled about the apex, because the tooth is a cone from the apex. It asserts
the eight offsets, that the first plane sits inside the heel and none past it, that the sixth lands
at the toe and the last two past it, and that each slab's volume matches the frustum closed form. The
cost is that the evaluator is not shown dividing a body, so the piece count, the retry with the
opposite sign and the raise that follows a second single-piece result are asserted on the offsets
rather than on a split. The slabs are also **laid apart along the shaft axis**, because built where
they belong they are face to face — each one's toe face IS the next one's heel face — and decad
refuses a pair in exact contact, reporting that its read-only intersection cannot classify it. A
translation along that axis leaves the azimuth, every volume and every length untouched, and the
assertions that compare a position add the same displacement back; what it costs is that the nine
sections are never seen standing in one stack, so a slab misplaced relative to its neighbours would
still read correctly. **The choice of slice family reaches Fusion untested**, for the reason this
step gives: the proof builds its own slabs from the offsets the spec fixes and never reads the plane
the module constructs. A spiral build has been through Fusion three times on the parallel family —
2026-09-16 at `7d253d8`, 2026-09-17 at `c6ccb3a`, and 2026-09-17 at `2ad1e32` — and each time a clean
build is the reading that the eight planes cut the tooth into more than one piece and that step F
left segments for the crown. ⚠️ **That is the whole of what those loads show, and they do not
distinguish the two families**, because a build on the tilted planes completes just as silently; no
face corner and no twist angle was measured on any of the three. The 0.26 mm corner shift is still
unmeasured in Fusion, and a measurement on a loaded spiral gear is the only thing that closes it.

<!-- proof-run: proofkit3d.RunSolidParallel(bevSpiralSolidCases, stepSliceTooth, assertSliceTooth) -->

Calls this step requires: `slice_body_by_offset_planes`, `physicalProperties.centerOfMass`,
`removeFeatures.add`.

<!-- check-step-calls: ignore setByOffset -->

`setByOffset` is named to describe what `slice_body_by_offset_planes` does inside the helper, not as
a call this step makes.

**From:** `spec/bevelgear/instructions.md` L841–L845, L898–L932;
`.claude/skills/generate-gear/PLAYBOOK.md` L172–L177, L440, L762–L769

## S18 `[GO]` Spiral: twist the segments

**G. Twist.** Rotate each segment about the **shaft axis** — `axisDir` through `apexWorld` — so the
tooth follows the trace, **centred on `R_mean` so the mid-face section stays unrotated**. That
section then meshes exactly like the straight tooth, which is critical: the pinion's zero mesh nudge
depends on it.

The total toe-to-heel twist comes from the **conjugate crown-gear generation law**, the standard
Gleason and Litvin model: a spiral bevel is generated by an imaginary flat crown gear, and the work
gear's shaft rotation relates to the developed crown-plane azimuth by the **roll ratio `1/sin γ`**,
because the generating crown gear has `N/sin γ` teeth and γ is this gear's **pitch** cone angle.
Compute it **analytically — no projection, no curve sampling**:

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`phi_crown` is the angle the cutter arc's toe and heel endpoints subtend at the apex in the flat 2-D
crown frame — exactly the `toe2d` and `heel2d` pairs from S16 step B. `gamma` is this gear's pitch
cone angle, `self._gamma_p` or `self._gamma_g`, already computed in S08 and forwarded into the hook.

⚠️ **The two halves of this law are taken on two different cones, and that is deliberate rather than
an oversight.** `phi_crown` is measured in the frame whose x axis is the **ROOT** cone element
Apex→C / Apex→D, while the divisor `sin γ` is the **PITCH** cone's roll ratio, because the crown gear
the law generates against is tangent to the pitch cone. Written consistently on the root cone the
divisor would be `sin γ_root` for `γ_root = γ − δ_f`, which is a real difference and not a rounding:
at the default pair `δ_f` is 3.26 degrees, `sin γ / sin γ_root` is 1.062, and the root divisor would
twist the tooth about 6% further. **Keep the pitch angle.** `acos(coneVec · axisDir)` is exactly that
root angle, and the build that used it inflated the twist by about 1.15 times on a 17-tooth pinion
meshing a 31-tooth gear, which is the defect that kept ratio pairs from meshing at all. ⚠️ The two
members of a meshing pair **legitimately get different twists**: same cutter, same ψ, but γ differs,
so `1/sin γ` differs — about 2.08 for a 17-tooth pinion against about 1.14 for a 31-tooth gear, a
ratio of roughly 1.83. That is *why* equal-teeth pairs always meshed while ratio pairs failed under
any method that gets the roll ratio wrong.

`handSign` sets the direction and `total` is the magnitude. Each segment's rotation is a **linear
share keyed to the cone distance of its HEEL FACE**, the segment's farthest-along-the-element face,
which is the exact section the S20 loft samples:

```
ang = −handSign · total · (R_mean − R_heelFace(seg)) / span
```

**Define a slab's heel face precisely: the face whose centroid has the GREATEST
`distAlong(face.centroid)`, searched across ALL of the slab's faces with NO surface-type filter**;
its toe-side face is the least-centroid one. ⚠️ Do **not** restrict the search to `PlaneSurfaceType`
or any surface type — a sliced slab is bounded by a mix of the two planar cut faces and ruled side
faces, and a type filter can pick the wrong face or miss the cut face, which makes the S20 loft fail
with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this **same all-faces-by-centroid** rule
everywhere a slab end face is needed: here, in S19's crown base, and in S20's loft sections.

⚠️ **Key the twist on the segment's HEEL-FACE cone distance, NOT its centroid.** The loft samples
each segment's heel face, so that face is what must land at the right azimuth; centroid-keying leaves
the loft's mid-face section rotated by half a segment and the mid-face overlaps.

Apply the rotation as a free move with a `Matrix3D.setToRotation(ang, axisDir, apex)` matrix through
`moveFeatures.createInput2(bodyCollection)` and `defineAsFreeMove(matrix)` (`[PB-MOVE-ROTATE]`), never
`defineAsRotate`, which rejects a `SketchLine` axis.

The proof rotates real slabs and asserts each one's azimuth against the linear share, that the law is
`|phi_crown| / sin(gamma)` on the **pitch** angle, and that the root-cone divisor really is the
larger one. **What is not settled is whether the FRAME should move to the pitch cone to match the
divisor.** Nothing in this repository measures it: the proof asserts this step against the same
formula the module computes, so it confirms the arithmetic and says nothing about which cone the
frame belongs on, and no Fusion load has reported a trace azimuth or a face corner either. Moving the
frame would change `R_toe` and `R_heel`, which are read along `coneVec`, and would change which
element ψ is measured against, so it is its own derivation and its own change.
`proof/bevelgear/spiral_test.go` records that beside the twist assertion.

<!-- proof-run: proofkit3d.RunSolidParallel(bevSpiralSolidCases, stepTwistSegments, assertTwistSegments) -->

Calls this step requires: `moveFeatures.createInput2`, `defineAsFreeMove`,
`adsk.core.Matrix3D.create`, `setToRotation`, `math.atan2`, `math.sin`, `math.fabs`,
`physicalProperties.centerOfMass`.

<!-- check-step-calls: ignore defineAsRotate projectToSurface acos -->

`defineAsRotate` is named only to forbid it; `projectToSurface` only to forbid measuring the twist
off a projected 3-D cone trace, which wraps for ratio pairs and collapses the measurement; `acos`
only to forbid `acos(coneVec · axisDir)` as the divisor's angle.

**From:** `spec/bevelgear/instructions.md` L934–L951;
`spec/bevelgear/spiral-tooth-trace.md` L204–L235;
`.claude/skills/generate-gear/PLAYBOOK.md` L800–L809

## S19 `[GO]` Spiral: the lengthwise crown

**H. Crown.** Crown the tooth by scaling each segment **except the outermost (heel) one** down by a
**monotonic** factor — full at the heel, growing smoothly toward the toe — **about a sketch point on
the ROOT edge of its heel face**, never the heel-face centroid.

For each segment compute its **heel-distance fraction** `u = (R_heel − R_heelFace) / span`, where
`R_heelFace` is the `distAlong` of that segment's heel face, **found by the S18 all-faces-by-centroid
rule but RECOMPUTED here, AFTER the twist has moved the slabs** — do not reuse pre-twist values —
and `R_heel` and `span` are from S16 step A. **`u` runs 0 at the held-full heel to 1 at the toe, and
PAST 1 on the two segments beyond the toe**: those two segments' heel faces sit `6·span/6` and
`7·span/6` in from the parent plane, so the toe-most one reads about 1.18 before the twist and a
little more, about 1.21, after this recompute, at the default Mean Spiral Angle of 35 degrees on the
default pair. **Do not treat `8/6` = 1.33 as a ceiling.** That figure is the last plane's offset, and
that plane is a toe face rather than any segment's heel face, so nothing ever evaluates `u` there;
the twist moves the heel faces, so the recomputed `u` climbs with the Mean Spiral Angle and is
**measured at 1.351 at 55 degrees**, which the `[0, 60)` range admits. Nothing reads an upper bound on
`u`. The heel segment reads a few hundredths rather than exactly 0, because `R_heel` is read at the
heel edge's midpoint rather than on the parent plane; that segment is skipped anyway.

**"Outermost (heel) segment" is the one with the GREATEST post-twist heel-face `distAlong`** — sort
the segments by their recomputed heel-face `distAlong` and skip the last. Then

```
factor = 1 − _CROWN_PER_RAD · (|total| / 2) · u
```

`total` is the full toe-to-heel twist from S18, so `|total|/2` is the per-end peak twist magnitude and
the maximum relief — now at the **toe** — keeps the magnitude the old per-end peak had, just
relocated. This makes relief **grow monotonically from the full heel to the toe**, so slab heights
stay strictly ordered heel to toe and the natural cone taper is never reversed. If a computed
`factor` comes out **≤ 0**, `raise` a self-diagnosing error naming the gear, the segment's `u` and the
factor; never scale by a non-positive factor. `_CROWN_PER_RAD` is the class constant from S01, set to
`0.5` — 0 disables the crown, so do not leave it unset.

⚠️ **Do NOT key the relief on `|ang|`**, the twist magnitude. That is **symmetric** about mid-face,
maximal at BOTH ends, so with the heel slab held full the slab *just inside* the heel becomes the
**most**-relieved one and dips below both its neighbours, a notch that reverses the taper. This was
the observed bug: the heel-adjacent slab came out at factor 0.932 while the next slab inward was
0.972, taller. Key on the monotonic `u`, never on `|ang|`.

Three gotchas:

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex — per `[PB-CONSTRUCTION-NEEDS-ACTIVE]`. `scaleFeatures` is the ONE exception to
   never-activate: it needs the Design occurrence as the **active** edit target, so call
   `designOccurrence.activate()` before the crown scales and restore afterwards, in a `finally`, with
   `design.activateRootComponent()`. ⚠️ Do **not** write `design.rootComponent.activate()` or
   `someComponent.activate()`: a `Component` has **no** `.activate()` method and raises
   `AttributeError`. Only an `Occurrence` has it, and the root is re-activated through `Design`.
2. **Skip the outermost (heel) segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone in S21 trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, NOT its centroid, or the crowned tooth lifts off
   the gear base.** `scaleFeatures` shrinks **uniformly** toward the base point, so a base point at
   the heel-face centroid — mid tooth-height — pulls the tooth's **root** edge upward by
   `(1−factor)·(½ tooth height)`, the tooth no longer seats on the gear body's root cone, it floats
   above the base, and the Combine-Join leaves a gap, clearly visible for ratio pairs such as module
   2 with driving 19 and pinion 13, the original symptom that exposed this. Put the base point on the
   **root** instead: of the heel face's vertices, each `.geometry` a world `Point3D`, take the **two
   with the smallest perpendicular distance to the shaft axis** — the line through `apex` along
   `axisDir`, with perpendicular distance `|(p−apex) − ((p−apex)·axisDir)·axisDir|` — those being the
   two **root corners**, since the tip corners are farthest from the axis, and place the base sketch
   point at their **midpoint**, mapped into the heel-face sketch with `modelToSketchSpace`. The heel
   face is a planar cut, so that midpoint lies on it. A uniform scale about a point keeps every line
   and plane through that point invariant, so anchoring on the root keeps the root edge on the
   seating cone while the tip is relieved progressively toward the toe, which is exactly the
   lengthwise crown intended. Finding the heel face itself is unchanged — still the
   max-`distAlong`-centroid face of S18; only the point *on* it changes.

Build the feature with `scaleFeatures.createInput(inputEntities, point, scaleFactor)` and
`scaleFeatures.add(input)`.

The proof **builds each segment at its crowned size** rather than scaling one after the fact, because
decad has no scale feature; a uniform scale about a point maps the slab's two sections to scaled
sections on parallel planes, which is what the proof constructs. It asserts that the factors are
monotonic heel to toe, that the outermost segment is held exactly full, that every factor is
positive, that `u` passes 1 on the two segments beyond the toe with no ceiling read on it, and that
the base point is a fixed point of the scale so the root corner does not move. **What it does not
exercise is Fusion's `scaleFeatures` and the one never-activate exception the crown needs.**

<!-- proof-run: proofkit3d.RunSolidParallel(bevSpiralSolidCases, stepCrownSegments, assertCrownSegments) -->

Calls this step requires: `scaleFeatures.createInput`, `scaleFeatures.add`, `activate`,
`activateRootComponent`, `modelToSketchSpace`, `sketchPoints.add`, `physicalProperties.centerOfMass`,
`ValueInput.createByReal`.

<!-- check-step-calls: ignore rootComponent.activate -->

`rootComponent.activate` is named only to forbid it: a `Component` has no `activate` method.

**From:** `spec/bevelgear/instructions.md` L310–L331, L953–L965;
`spec/bevelgear/fusion.md` L210–L215;
`.claude/skills/generate-gear/PLAYBOOK.md` L585–L590, L791–L799

## S20 `[GO]` Spiral: loft the curved tooth

**I. Loft.** ⚠️ **Re-sort the segments by their heel-face cone distance HERE, AFTER the twist and the
crown — do NOT reuse the pre-twist slice or centroid order from S17.** The twist rotates each slab
about the shaft axis, and for high-twist *unequal-ratio* pairs that rotation changes the slabs'
along-cone order enough to **reorder adjacent slabs**; lofting in the stale order then assembles the
cross-sections out of sequence and the crowned tooth comes out distorted, and the two gears
interfere. For equal or low-twist pairs the two orders coincide, which is why equal-teeth gears mesh
even with the stale order while unequal ratios distort — this is the single thing that makes a ratio
pair like 31/17 fail while 31/31 looks fine.

So compute `order = sorted(segment indices, key = distAlong(slabHeelFace(seg).centroid))` **now**,
and loft a **NewBody** through, in that order: first the **toe-most segment's apex-side (toe-facing)
face** — the toe segment is `order[0]`, and its toe face is added first to push the loft past the toe
cone so the toe trim bites — then the **heel-facing face of every segment, iterated in `order`**,
each being that segment's farthest-along-the-element face by post-twist centroid, the last of which
reaches past the heel cone. Name the resulting body **`{gear} Spiral Tooth`**. Then remove the
segment scaffolding with `removeFeatures.add`, since the loft has captured their faces.

The proof lofts the chain **pair by pair in the recomputed order**, because decad's Loft takes
exactly **two** sections and the single nine-section loft is out of reach. It asserts the recomputed
order, that the chain opens on the toe-most segment, that the order is strictly increasing in
post-twist heel-face cone distance, and that the chain spans from past the toe to the parent plane's
own section. **The cost is the single loft**: the evaluator is not shown building one body through
all nine sections, so a pairing failure only a nine-section loft would reach is out of range.

<!-- proof-run: proofkit3d.RunSolidParallel(bevSpiralSolidCases, stepSpiralLoft, assertSpiralLoft) -->

Calls this step requires: `loftFeatures.createInput`, `loftSections.add`, `loftFeatures.add`,
`removeFeatures.add`, `physicalProperties.centerOfMass`.

**From:** `spec/bevelgear/instructions.md` L967–L969;
`.claude/skills/generate-gear/PLAYBOOK.md` L724–L728

## S21 `[GO]` Conical end cuts — the flush trim

**J.** Trim the Tooth Body to a flush band with the framework helper: return
`cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)` from
`.solids`, and do **not** re-implement the cut machinery. For a straight bevel this is what the ψ = 0
gate at the top of `_transformToothBody` returns directly; for a spiral it is what S20's
`{gear} Spiral Tooth` is handed to, so the curved tooth's ends sit flush on the gear base.

**Two distinct bodies are involved and must not be conflated:** the cutting **TOOLS** are
`ConeSurfaceType` faces of the **Gear Body**, the revolved-hexagon frustum — the lofted Tooth Body
has no cone faces, so searching *it* finds none — and the **TARGET** being split is the **Tooth
Body**.

The helper implements the pinned behaviour: the **toe cut first**, its cone face identified by the
toe edge's world **MIDPOINT** best-first across the frustum's cone faces (`[PB-FACE-BY-MIDPOINT]` —
endpoints sit near the apex singularity and `getParameterAtPoint` returns no result there), each
candidate tried as the actual split tool with `isSplittingToolExtended=True` and the first that
splits kept; then **keeper selection** after each cut, dropping apex-containing pieces and keeping
the largest (`[PB-REMOVE-PIECES]`); then the **heel cut on the keeper alone**, removing the apex tip
first being what makes it deterministically two split features for every gear ratio. A heel cone that
does not intersect the keeper at all — common on ratio pairs such as module 1 with driving 31 and
pinion 43, where the heel cone never overshoots the tooth — is raised by the helper as the typed
`solids.NonIntersectError` and caught, and the keeper is returned whole. Every failure is
self-diagnosing with the per-face distance and error history (`[PB-SELF-DIAGNOSING]`), and each cut's
outcome is logged with `force_console=True`.

**Caller obligations, which stay in the generator:** pass `toeMid` as the toe edge's world midpoint,
`(M_world + N_world)/2` for the pinion and `(O_world + P_world)/2` for the driving gear; `heelMid` as
the heel edge's world midpoint, `(C_world + H_world)/2` and `(D_world + J_world)/2` — the same
edge-midpoint pairs as the S16 hand-off; `apexWorld` as the §2 Apex sketch point's world geometry;
and `gearBody` as the revolved frustum, the cone-face source. **The toe cut must split**, and its
failure propagates and crashes the build, which is correct, since an uncut tooth is unusable; only
the heel cut is lenient, and only through the typed `NonIntersectError`.

The proof **performs the toe cut and substitutes for the heel cut**. Where each cut lands is read the
same way for both: build the tooth, read each cone's apex and half-angle and each of the tooth's two
surfaces off the bodies themselves, solve the stations where they cross from those readings, and
check them against the flush band. **Both half-angles are taken off the revolved gear body's own cone
faces**, since those are the faces Fusion's `ConeSurfaceType` search finds here and a `decad.Cone`
publishes the angle directly. It asserts three things: each cut lands where the flush band requires,
the toe cone meeting the gear body's root cone at M and the heel cone at C; each cone's half-angle
equals `90° − γ`; and each cone crosses the tooth's tip **inboard** of where it crosses the tooth's
root, so the trimmed end is shorter at the tip than at the root. **It does not assert that a cut
meets the tip and the root at different stations, and carries no message for that case**: both
cutting cones have their apex on the shaft axis, so a cone of wall slope `k` and apex station `a`
crosses a tooth surface of slope `m` at `a · k / (m + k)`, which is positive for every `m > 0` and
different for the tooth's two surfaces whenever the tooth has height, so such an assertion passes on
any figure this spec can build.

**The toe cut is performed.** The toe cutting cone is built as the **SOLID inside the cone**, with
its apex on the shaft axis at the station the toe edge's own lattice point M or O puts it, rather
than as a band spanning that one profile edge, which is enough to read an angle off and is not a tool
a cut can use; it is an n-gon loft, because the cut consumes it, and it **reaches past the gear
apex** so the cut leaves one piece on each side rather than shearing an apex end off against the
tool's own far cap. The step's own document lays its four bodies apart, since a gear body, a tooth
and two cutting cones standing where they belong all overlap and decad reports each overlapping pair
as an interference the gate refuses; the cut itself is performed on fresh operands in their true
relative placement, in documents of their own. The tooth and the cone are put in
one frame so the cone meets the tooth where the toe end of the flush band puts it; seating the tooth
on the gear body is a different placement, and it is the one S23 still waits on. The tooth is then
`Cut` by the cone for one piece and `Intersect`ed with it for the other, in separate documents, since
either operation retires its operands. **One typed refusal is tolerated from the `Intersect` and any
other error fails**: a `*decad.BooleanError` carrying `BooleanEmpty` says the cone took nothing off
that tooth, which is the condition the module raises as `solids.NonIntersectError`, and a probe over
one pair of operands drew it on the two Shaft Angle 142-degree cases and on no other. The proof
asserts that the two pieces add back to the whole tooth and that each piece is one lump and solid.
**The toe split costs nothing now** — the evaluator divides the tooth and both halves are measured,
except on a case the refusal claims, where the step records that it built no split rather than
passing silently.

**No heel cut is performed, because its cone is tangent to the tooth plane.** The dedendum corner
C/D and the tooth centre K′/L′ both sit on this gear's back-cone dedendum line, so the tooth plane
contains a generator of the heel cone and the two touch along the tooth's own centreline instead of
crossing it. decad refuses exactly that as `BooleanUnsupportedContact`, on 15 of the 20 cases for the
`Cut` and 17 for the `Intersect`, and rebuilding the same cone as a Revolve replaces the refusal with
a Suspect verdict the gate does not admit either, so no operand pairing at this revision puts the
heel cut in reach. The heel cone is laid apart from the tooth and the three readings are kept. **The
cost is the heel split**: for that end the proof does not show the evaluator dividing the tooth,
selecting the keeper, or leaving a watertight body. At Tooth Spacing 0 the tooth centre is that
cone's own apex, so the heel cone passes exactly through the tooth's heel-end centreline and takes
only the two corners, by `py² / (2 · r · cos γ)` for a corner `py` off the centreline at polar radius
`r`; a cut that removes that little is a cut that can miss the keeper altogether on a ratio pair,
which is the typed error the helper catches and the reason only the toe cut must split.

**What this step cannot tell apart**, recorded in the proof file beside the cut assertions: a cone
and a tilted plane read identically in everything the step measures, because in the axial section a
cone of half-angle `90° − γ` and a plane tilted by γ through the same generator are the same line.
What makes the face conical is that it is a **surface of revolution about the shaft axis** — its
crossing with the tooth's tip sits at one station at every azimuth, where a tilted plane's crossing
moves with azimuth. The half-angle the step reads comes off a `decad.Cone` face of the revolved gear
body, which is a surface of revolution by its own construction; the tool the toe cut consumes is a
faceted body swept about that axis, and a swept body cannot be anything else either, but **nothing
measures its azimuthal crossing with the tooth**. The flush-band check cannot see the half-angle
either: any band through M crosses the gear body's root ray at M whatever slope the band has, so the
half-angle assertion is what pins the angle and nothing about where a cut *lands* pins it.

<!-- proof-run: proofkit3d.RunSolidParallel(bevSolidCases, stepConicalEndCut, assertConicalEndCut) -->

Calls this step requires: `cut_conical_ends`.

<!-- check-step-calls: ignore apply_conical_cut select_keeper find_cone_faces_by_midpoint surface_distance splitBodyFeatures.createInput isSplittingToolExtended pointContainment -->

The helper's internals are named to describe the pinned behaviour the module must not re-implement,
not as calls this step makes.

**From:** `spec/bevelgear/instructions.md` L533–L538, L995–L1021, L1100–L1146, L1224–L1256;
`.claude/skills/generate-gear/PLAYBOOK.md` L159–L170, L733–L774

## S22 `[GO]` Circular pattern

Circular-pattern the remaining tooth piece around the **shaft-axis edge** — the same in-sketch
profile edge the revolve used, never the §2 construction line — with
`circularPatternFeatures.createInput(bodies, axis)`, then `add(input)`. Pin all three inputs
explicitly and do not rely on Fusion's defaults staying equal to them (`[PB-CIRCULAR-PATTERN]`):
`quantity = <this gear's Teeth Number>` as a `ValueInput`,
`totalAngle = ValueInput.createByString('360 deg')`, and `isSymmetric = False`.

Although the pitch diameter shrinks from heel toward apex, the *angular* spacing around the shaft
axis stays constant at `360° / N` for the entire face width: the radial taper is already produced by
the loft from Apex to the heel-end tooth profile, so the pattern just rotates that one tapered tooth
into N evenly spaced copies.

`CircularPatternFeature.bodies` already includes the seed body plus the copies, so do not re-add the
seed, and copy them into a fresh `adsk.core.ObjectCollection.create()` before handing them to the
Combine, because `pattern.bodies` is a `BRepBodies` and `combineFeatures.createInput` rejects it
(`[PB-PATTERN-BODIES]`).

**This is the one bevel step whose proof cases stay SERIAL**, on `proofkit3d.RunSolid`, where every
other `[GO]` step registers through the parallel entry point. The pattern increment retires the seed
tooth, so the seed cannot be measured after the step runs, and its azimuth, radius, height and volume
have to be read during the build and handed to the assertion. That hand-off leaves the case, and two
cases sharing one set of seed readings overwrite each other. It is not a hazard that announces
itself: the two gear sides differ enough in volume that the overwrite was caught when it happened,
and a pair of cases whose seeds measured alike would have passed on each other's numbers instead.
`proof/bevelgear/solids_test.go` records beside those variables that the step is serial because of
them, and a step that acquires a reading like this moves to the serial runner in the same change.

The proof builds the seed tooth, reads it, then produces the copies and retires the seed exactly as
the pattern does. It asserts that the body count equals the Teeth Number, that each copy's volume
agrees with the seed's carried reading, that each copy reaches the same radius and spans the same
length along the shaft, and that the copies sit `360/N` apart around the full circle. Each copy is
**also displaced along the shaft axis**, and that is a substitution: every tooth is lofted from the
same apex, so N copies rotated about the axis all meet at that one point, and decad cannot decide
whether two bodies touching within its chord tolerance are disjoint or overlapping. It is not an
adjacency problem — the four-tooth case, whose copies are a quarter turn apart, behaves the same way
— and a translation along the axis leaves all four readings this step asserts untouched. **The cost
is that the proof does not show N copies standing in one frame**, so it cannot see two of them
interfering.

<!-- proof-run: proofkit3d.RunSolid(bevSolidCases, stepCircularPattern, assertCircularPattern) -->

Calls this step requires: `circularPatternFeatures.createInput`, `circularPatternFeatures.add`,
`ValueInput.createByReal`, `ValueInput.createByString`, `adsk.core.ObjectCollection.create`.

**From:** `spec/bevelgear/instructions.md` L1023, L1362–L1386;
`.claude/skills/generate-gear/PLAYBOOK.md` L693–L703

## S23 `[GO]` Combine-Join

Join all patterned tooth pieces with the Gear Body in a **single** Combine-Join: the Gear Body is the
target and the patterned tooth bodies are the tools. Build it with
`combineFeatures.createInput(targetBody, toolBodies)`, set
`operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`, and `combineFeatures.add(input)`.
The tools must be an `ObjectCollection`, per S22.

The proof **performs no join, because the two operands are not in one frame.** It builds the tooth on
a section perpendicular to the build axis at the Pitch Cone Distance from the apex, scaled about the
apex — the Tredgold mapping — while the gear body is written about the shaft axis, and the tooth's
own seating on that body is derived nowhere in this proof. The two therefore do not meet when they
are put in one document, and neither sign of the rotation that relates the two planes seats them: at
one sign the union returns two lumps, and at the other decad refuses the contact on half the case
table. **The engine is not what blocks this join** — given operands that do overlap it performs the
union, returns one lump and publishes a volume bound of 8e-15 of the value, Sound. What is missing is
the tooth's real back-cone placement, and deriving it is its own change.

Until then the proof lays the operands apart and asserts the join's two consequences from their own
measured geometry: a join leaves **ONE lump** when the tooth's root is below the body's root cone —
seated, not floating — and the joined body reaches **further out** than the frustum when the tooth's
tip stands proud of it. Both readings are taken at the toe, the middle and the heel of the band the
join would cover. **The cost is the stitch**: the proof cannot show the evaluator making one boundary
out of two.

The module **draws its root circle one root sink inside the dedendum corner** in S10, so the root arc
lies inside the gear body's root cone across its whole width and the join overlaps along the whole
root rather than along the centreline alone, and **the proof applies that same sink** — it is one
figure, not a proof-only offset. ⚠️ **Read the root arc's OUTERMOST point, not the tooth's
centreline.** The centreline sits inside both root corners, so a reading taken there passes a tooth
whose corners float outside the cone, which is exactly the defect the sink exists to remove.

Fusion has made this stitch three times — 2026-09-16 on the branch the root sink was introduced on,
2026-09-16 on the branch that regenerated the module after it, and 2026-09-17 at `2ad1e32` — on the
shipped default pair and on a 16 driving / 12 pinion pair at Module 4. The second of those counted
**one solid per gear**, which is the join's own reading, since two bodies would mean a tooth floating
off the root cone. The third counted nothing and measured nothing. **The heel tip radius has never
been measured on a loaded gear**, so the tip is checked only where the proof checks it: S10 dimensions
the drawn tip circle and S15 reads the built tooth body out to the virtual tip radius laid on the back
cone, at Module 4 through 8 only, since no solid case runs at Module 1. No case reads a tip radius off
a joined body, because no case joins. That measurement is still outstanding.

<!-- proof-run: proofkit3d.RunSolidParallel(bevSolidCases, stepCombineJoin, assertCombineJoin) -->

Calls this step requires: `combineFeatures.createInput`, `combineFeatures.add`,
`adsk.fusion.FeatureOperations.JoinFeatureOperation`.

**From:** `spec/bevelgear/instructions.md` L818–L833, L1025, L1147–L1205;
`.claude/skills/generate-gear/PLAYBOOK.md` L693–L697

## S24 `[GO]` `{gearLabel} Bore` sketch

Skip this step and S25 entirely when **Enable Bore** is unchecked; no bore is cut on either gear and
the per-gear bore diameter inputs are ignored.

Build the bore plane normal to the shaft at its start with `constructionPlanes.createInput()` and
`setByDistanceOnPath(<shaft-axis edge>, ValueInput.createByReal(0.0))`, passing the **in-sketch
edge** directly and never the §2 construction line and never a `Path.create` wrapper
(`[PB-CONSTRUCTION-PLANES]`). A distance of 0 is the start of the path.

In a sketch named `{gearLabel} Bore`, sketch the bore circle centred at the **sketch origin**, since
the plane is rooted at the shaft start and the origin is therefore on the axis. Use
`sketchCurves.sketchCircles.addByCenterRadius`, then **fix the circle's centre** with
`circle.centerSketchPoint.isFixed = True` and add a diameter dimension with
`addDiameterDimension` set to this gear's resolved bore diameter (`[PB-CIRCLE-CENTER]`). A circle
created at (0,0,0) does **not** reuse the sketch's `originPoint` — its centre is a free point that
happens to sit there — and making it coincident to the origin has been observed to throw
`VCS_SKETCH_SOLVING_FAILED` on a `setByDistanceOnPath` plane. `isFixed` on the centre is the reliable
pin.

The diameter is this gear's **resolved** Bore Diameter from the dict key `boreDiameter_cm` — the
user's value if specified, otherwise `this gear's Pitch Diameter / 4`, in either case already bounded
by that gear's Maximum Bore Diameter in S08. Take that resolved number; do **not** re-derive it here,
or the cap is lost and the bore deletes the body's back face.

Gate the sketch on `isFullyConstrained` and raise, naming it, if it is not.

The proof reproduces the sketch, holds it to DOF 0, takes its single region and asserts the region's
area against `π d² / 4`, and re-asserts that the resolved diameter is the bounded auto value. A case
with Enable Bore unchecked is recorded as unmodelled, since no sketch is drawn at all.

<!-- proof-run: proofkit.RunParallel(bevBoreCases, stepBoreSketch) -->

Calls this step requires: `constructionPlanes.createInput`, `setByDistanceOnPath`,
`constructionPlanes.add`, `sketches.add`, `sketchCurves.sketchCircles.addByCenterRadius`,
`isFixed`, `sketchDimensions.addDiameterDimension`, `ValueInput.createByReal`,
`isFullyConstrained`.

<!-- check-step-calls: ignore Path.create originPoint -->

`Path.create` is named only to forbid wrapping the edge; `originPoint` only to forbid making the
circle's centre coincident to it.

**From:** `spec/bevelgear/instructions.md` L137–L141, L1027;
`spec/bevelgear/fusion.md` L21–L30;
`.claude/skills/generate-gear/PLAYBOOK.md` L451–L457, L775–L786

## S25 `[GO]` Bore extrude-cut

Extrude-cut the bore circle as a **symmetric through cut** restricted to this Gear Body. Build it
with `extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`, set
the extent with `setSymmetricExtent(ValueInput.createByReal(<half length>), False)` — the second
argument `isFullLength=False` means the value is the half-length **per side**, and no third taper
argument is passed — set `participantBodies = [this Gear Body]`, then `extrudeFeatures.add(input)`
(`[PB-THROUGH-CUT]`). Use **`2 × Cone Distance`** as the per-side half-length, generously past any
face width.

The proof builds the tool as a real extrude, which a symmetric extent produces as a prism, and
**performs the cut**. It asserts the tool first, from its own measured geometry — its diameter, that
its two ends sit exactly `2 * Cone Distance` either side of the shaft edge's start, and that both
clear the frustum, which is what makes it a THROUGH cut — and computes the material the bore takes
out. **The target is the heel cone band, lofted for this step**: it is the section of the gear body
the bore passes through, and it is a Loft, which is the form a boolean operand takes here. The
revolved gear body cannot be the target, for the reason S14 gives. The cut verifies Sound on every
case in the table. The proof asserts the pierced body's volume against the band's own n-gon closed
form less the prism the bore removes over that height, that the result is **one lump**, which is what
a through hole leaves, and that it is **solid**, which an enclosed void would not be. **What this
still does not reach is the rest of the body**: the bore is pierced through the band that stands for
the heel section, not through the whole frustum, because the frustum is a Revolve.

<!-- proof-run: proofkit3d.RunSolidParallel(bevBoreSolidCases, stepBoreCut, assertBoreCut) -->

Calls this step requires: `extrudeFeatures.createInput`, `setSymmetricExtent`,
`participantBodies`, `extrudeFeatures.add`, `ValueInput.createByReal`,
`adsk.fusion.FeatureOperations.CutFeatureOperation`.

**From:** `spec/bevelgear/instructions.md` L143–L164, L1027, L1206–L1219;
`.claude/skills/generate-gear/PLAYBOOK.md` L729–L732

## S26 `[GO]` Meshing rotation, and move the bodies out

**Do this here, in the Design component, before the body is moved out.** Rotate the **driving** body
by `180° / Driving Gear Teeth Number` — half a tooth pitch — about its shaft axis with
`rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)`, the framework helper,
which takes the rotation axis and origin from the B′→I profile edge's **world** endpoints
(`[PB-MOVE-ROTATE]`). A driving valley then sits where the pinion tooth crosses the axial plane,
giving the interlocked meshing look. Both gears are patterned from a starting tooth in that plane, so
without the offset a driving tooth and a pinion tooth would both sit at the crossing and visually
collide.

This runs in Design before `moveToComponent` because a construction axis cannot be added in the
moved-out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation must use the edge's
world geometry while the body is still in Design.

The pinion additionally gets `_pinionMeshPhase(pinionTeeth)`, its own helper, which returns the
pinion's extra mesh rotation **in radians** as `_PINION_MESH_PHASE_TEETH · 2π / pinionTeeth`. At the
shipped `_PINION_MESH_PHASE_TEETH` of `0.0` that is **0**, so no extra rotation is applied, and S18
leaves the mid-face section unrotated precisely so that none is needed. **A zero angle is a no-op,
not a move**: `setToRotation(0, axis, origin)` builds the identity and Fusion refuses to move a body
by it with `RuntimeError: 3 : invalid transform`, measured on 2026-09-02 on the bevel pinion, and
`rotate_body_about_edge` absorbs it so no call site has to guard it.

Then move this gear's finished body into its own `{gearLabel} Gear` component with
`body.moveToComponent(gearOccurrence)`, which preserves world position and needs no activation.

The proof rotates a real tooth body about the shaft axis and asserts the azimuth it moved through:
half a tooth pitch on the driving gear, and exactly zero on the pinion.

<!-- proof-run: proofkit3d.RunSolidParallel(bevSolidCases, stepMeshRotation, assertMeshRotation) -->

Calls this step requires: `rotate_body_about_edge`, `moveToComponent`, `_pinionMeshPhase`,
`math.pi`.

**From:** `spec/bevelgear/instructions.md` L517–L529, L556–L559, L985, L1029–L1037;
`.claude/skills/generate-gear/PLAYBOOK.md` L800–L809, L829–L834

## S27 `[PROSE]` Cleanup

Call `hide_construction_geometry(bevelComponent)` from `.solids`. It recursively walks the Bevel Gear
component tree, deduping by `entityToken`, and hides every sketch, construction plane and
construction axis by setting `isLightBulbOn = False` (`[BEVEL-F-CLEANUP]`, `[PB-TREE-CLEANUP]`).
Construction planes and axes are **not** hidden by `isVisible` — that is a Fusion gotcha, and
`isVisible` hides sketches while `isLightBulbOn` hides construction geometry, so do not cross them
(`[PB-HIDE-AFTER-USE]`). Leave only the two finished gear bodies visible.

There is no sketch-only mode and no per-mode guard: bevel always builds solids. Do not re-implement
the walk.

The driving gear's half-tooth-pitch meshing rotation is performed in S26, in the Design component
before the body is moved out, and is **not** a cleanup step.

Nothing here settles a sketch's browser icon. `commands/_gear_command.py` calls
`geargen.settle_sketch_display` once after `generate()` returns, every gear command runs through that
one call, and **a generated module must not add its own** (`[PB-SETTLE-DISPLAY]`).

Calls this step requires: `hide_construction_geometry`.

<!-- check-step-calls: ignore isVisible settle_sketch_display -->

`isVisible` is named only to forbid it on construction geometry; `settle_sketch_display` only to
record that the shared command wrapper owns it and the module must not call it.

**From:** `spec/bevelgear/instructions.md` L1033–L1037;
`spec/bevelgear/fusion.md` L216–L221;
`.claude/skills/generate-gear/PLAYBOOK.md` L534–L555, L659–L671, L835–L837
