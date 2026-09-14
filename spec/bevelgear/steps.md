# Bevel Gear — compiled step list

The proof for these steps is `proof/bevelgear/geometry_test.go`,
`proof/bevelgear/variables_test.go`, `proof/bevelgear/sketches_test.go`,
`proof/bevelgear/solids_test.go`, `proof/bevelgear/spiral_test.go` and the generated
`proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `289442b650cf6943c65e4b714ef0e37f26c136e6` |
| `spec/bevelgear/fusion.md` | `754729fc9a33ad9dbf93816e0430e4b94d82e05b` |
| `spec/bevelgear/spiral-tooth-trace.md` | `c9ec08561ced7975aa0ed9ad6a330186259c0d08` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `abb1123b5910f10c5c96c1ad936a38691ea3e7fb` |

## 1 `[PROSE]` Declare the command dialog inputs

`BevelGearCommandInputsConfigurator.configure(cls, cmd)` adds **20** inputs to `cmd.commandInputs`,
in exactly the row order of the table below. Target Plane is added first so Fusion's auto-focus,
which takes the FIRST `SelectionCommandInput` and ignores a later `hasFocus`, lands there
(`[PB-AUTOFOCUS-FIRST]`). Center Point follows, then the pre-selected Parent Component, then the
numeric and boolean fields.

Every id, label, unit string, default and tooltip below is reproduced surface. Use them verbatim.

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

The 20 module-level id constants are named exactly `INPUT_ID_PLANE`, `INPUT_ID_CENTER_POINT`,
`INPUT_ID_PARENT`, `INPUT_ID_MODULE`, `INPUT_ID_SHAFT_ANGLE`, `INPUT_ID_DRIVING_TEETH`,
`INPUT_ID_PINION_TEETH`, `INPUT_ID_DRIVING_BASE_HEIGHT`, `INPUT_ID_PINION_BASE_HEIGHT`,
`INPUT_ID_BORE_ENABLE`, `INPUT_ID_DRIVING_BORE`, `INPUT_ID_PINION_BORE`, `INPUT_ID_FACE_WIDTH`,
`INPUT_ID_TOOTH_SPACING`, `INPUT_ID_SPIRAL_ANGLE`, `INPUT_ID_HAND`, `INPUT_ID_CUTTER_RADIUS`,
`INPUT_ID_TOE_EXTENSION`, `INPUT_ID_DRIVING_TOE_RADIUS`, `INPUT_ID_PINION_TOE_RADIUS`, holding the
table's id strings in row order. The only other module constants are `_HAND_RIGHT = 'Right'` and
`_HAND_LEFT = 'Left'`. There are **no** live Fusion user parameters and therefore no `PARAM_*`
strings: every value is precomputed in Python in internal cm and written into geometry numerically
(`[PB-PRECOMPUTED-MODE]`).

Each selection input takes its filters as named constants, never quoted literals
(`[PB-SELECTION-FILTER-ENUM]`), and its limits per the table (`[PB-SELECTION-DECL]`):
`addSelectionInput`, `addSelectionFilter`, `setSelectionLimits`. The Parent selection pre-selects
`get_design().rootComponent`. Numeric defaults are passed in internal units
(`[PB-DIALOG-DEFAULT-UNITS]`): `createByReal` with `to_cm(...)` for `mm` fields, and
`createByString` for the two angle fields so the expression engine parses them. The Hand dropdown
is a `DropDownStyles.TextListDropDownStyle` built with `addDropDownCommandInput`, with `Right`
added selected and `Left` added unselected.

**Conditional visibility.** Hand of Spiral and Cutter Radius are hidden whenever Mean Spiral Angle
is 0 and shown when it is above 0; Mean Spiral Angle itself is always visible, because it is how
the user reaches a positive value. There is no declarative show-if in the API, so this is
`commandInput.isVisible`. Add `@classmethod _updateSpiralInputVisibility(cls, inputs)`: it
evaluates the `spiralAngle` input's `.expression` with
`unitsManager.evaluateExpression(spiral.expression, 'rad')` — internal **radians**, and NOT the
input's `.value` — and sets `inputs.itemById(INPUT_ID_HAND).isVisible` and
`inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible` to `(value > 0)`. Guard it: return early if any
of the three inputs is `None`, and wrap the evaluation in `try/except`, leaving both inputs SHOWN
on failure, because a half-typed expression can raise mid-edit. `configure()` calls
`cls._updateSpiralInputVisibility(inputs)` as its LAST step so the initial state is right.
`@classmethod def handle_input_changed(cls, args)` calls
`cls._updateSpiralInputVisibility(args.inputs)` on every input change, and is bound by name from
`commands/bevelgear/entry.py` along with `configure`.

`configure` and `handle_input_changed` are methods the framework calls, not calls this module
makes, and the four private names below are this module's own rather than Autodesk's.

<!-- check-step-calls: ignore configure handle_input_changed -->
<!-- check-compile: ignore configure handle_input_changed _updateSpiralInputVisibility get_design to_cm -->

**From:** `spec/bevelgear/instructions.md` L27–L33 L100–L104 L140–L144 L145–L225 L267–L292
`.claude/skills/generate-gear/PLAYBOOK.md` L346–L349 L128–L143 L548–L559 L843–L858

## 2 `[GO]` Read every input and resolve every derived value and bound

`_readInputs(inputs)` reads and validates all 20 inputs in one pass, returns the 7-tuple
`(parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth, shaftAngle_deg)`
and stashes the rest on `self` as `self._drivingBaseHeight_cm`, `self._pinionBaseHeight_cm`,
`self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`, `self._faceWidth_cm`,
`self._toothSpacing_cm`, `self._spiralAngle_rad`, `self._hand`, `self._cutterRadius_cm`. Bevel
carries **no** `GenerationContext`: per-gear anchors travel in plain dicts `pinionCtx` /
`drivingCtx` and shared anchors are self-stashed. Do not introduce a context class.

**How each input is read.** Read each numeric and angle input by evaluating its expression:
`design.unitsManager.evaluateExpression(input.expression, units)` with `''`, `'mm'` or `'deg'`
(`[PB-EVAL-EXPRESSION]`). The value comes back in Fusion INTERNAL units — cm for length, radians
for angle — whatever the unit string says, so convert an angle with `math.degrees(...)` before any
degree range check. The boolean is read with `get_boolean`; the three selections with
`get_selection`; the dropdown through `inputs.itemById(INPUT_ID_HAND).selectedItem` and its
`.name`, defaulting to `_HAND_RIGHT` when none. Both tooth inputs are coerced with
`int(round(...))` before validation.

**Units, and the one trap.** The `mm` inputs and the two `deg` inputs come back ALREADY in internal
units — use them as-is, never `to_cm` them again. `Module` is read with unit `''`, so it comes back
as a raw number meaning MILLIMETRES. Every length derived from Module must therefore be
`to_cm`-converted before it touches geometry: Pitch Diameter `to_cm(Module * teeth)`, Cone Distance,
the dedendum `to_cm(1.25 * Module)`, the module-length extensions E, F, G, H, I, J, and the default
Face Width. Mixing a raw-mm Module-derived length with an already-cm `mm` input makes the gear come
out about ten times off and the Face Width bound meaningless.

**The formulas, in the order they resolve.**

- Pinion Gear Pitch Diameter `PPD = Module * Pinion Gear Teeth`; Driving `DPD = Module *
  Driving Gear Teeth`.
- Cone Distance `= sqrt(DPD**2 + PPD**2)`. It depends on the tooth counts only, never on the
  Shaft Angle. It is NOT the Pitch Cone Distance.
- Range checks first: `Module > 0`, each tooth count `>= 3`, Shaft Angle at least 30 deg.
- Maximum Shaft Angle. Let `coneLimit = degrees(acos(-min(DPD, PPD) / max(DPD, PPD)))`, the angle at
  which a pitch cone angle reaches 90 deg, `R * cos(gamma)` passes through zero and the along-shaft
  seed points backwards. The Maximum Shaft Angle is `min(coneLimit, 150)`, and the cone-angle half is
  EXCLUSIVE while the 150 deg half is INCLUSIVE: reject `Shaft Angle >= coneLimit` when
  `coneLimit <= 150`, and `Shaft Angle > 150` otherwise. Name the computed limit in the message.
  A 31/17 pair gives 123.26 deg; equal counts give 180 deg, which is no constraint at all.
- Cone angles: `tan gamma_p = sin(Sigma) * PPD / (DPD + PPD * cos(Sigma))`, `gamma_g = Sigma -
  gamma_p`. Use a two-argument arctangent so an obtuse Shaft Angle, where the denominator goes
  negative, still lands in the right quadrant.
- Pitch Cone Distance `R = (PPD / 2) / sin(gamma_p)`. This is the apex-to-heel length along the
  pitch cone, and it equals `Cone Distance / 2` only at Shaft Angle 90 deg. An equal 31/31 pair at
  30 deg has Cone Distance 43.84 mm against R 59.89 mm, and at 140 deg R is 16.49 mm.
- **Minimum Teeth**, per gear with that gear's own gamma: `teeth >= 5.27 * cos(gamma)`. The constant
  is `2 * (1.05 * 1.25 / 0.95 + 1.25) = 5.2632` rounded UP; do not round it down and do not replace
  it with the exact value. Check it on top of the blanket `teeth >= 3` and name the computed floor.
  At Shaft Angle 90 deg the floor is 3.72, i.e. 4 teeth. Run this BEFORE the base-height bounds: it
  is exactly the statement that the base-height window is non-empty.
- **Minimum Base Height** `= 1.05 * 1.25 * Module * sin(gamma)` and **Maximum Base Height**
  `= 0.95 * (r - 1.25 * Module * cos(gamma)) * tan(gamma)`, per gear with its own `r = PitchDia / 2`
  and gamma. Both are closed form and need no solved geometry, so resolve them here. The base height
  is measured from **Apex 2's plane**, not from the dedendum point: the heel corner reaches the shaft
  axis at `r * tan(gamma)`, and the Maximum sits `1.25 * Module * sin(gamma)` below that crossing
  before the 0.95 factor, so it is deliberately conservative.
- Apply them per gear in both directions: raise a fallback below the minimum, cap one above the
  maximum, reject a user value outside either end naming the bound it broke. The DRIVING side
  resolves first with fallback `Module * Driving Gear Teeth / 8`; the PINION's fallback is the
  driving side's RESOLVED value times `Pinion Gear Teeth / Driving Gear Teeth` — after the driving
  fallback and after the driving cap, never the raw driving input — and then the pinion's OWN
  Maximum Base Height is applied to it.
- Bore diameters: only consulted when Enable Bore is checked; 0 means auto, which is that gear's
  own `PitchDia / 4`.
- Tooth Spacing, Cutter Radius and the two Toe Radii are non-negative; Mean Spiral Angle is in
  `[0, 60)` degrees; Toe Extension is in `[0, 100]`.

The Maximum Face Width, the Toe Radius ceiling and the Toe Limit need the §2 figure and are resolved
in step 6.

`_readInputs` is this module's own helper, and so are the two base readers it borrows.

<!-- check-compile: ignore _readInputs get_boolean get_selection to_cm int round math -->

<!-- proof-run: proofkit.RunParallel(bvResolveCases, stepResolveInputs) -->

`stepResolveInputs` proves this step. It draws the heel chain of the §2 lattice — the shaft axis,
the drop that carries the pitch radius, the pitch line, the dedendum line and the base-height
offset that drives the heel edge — and holds every bound above to what that figure actually does:
the true crossing measured off the dedendum line's meeting with the shaft axis, the Maximum Base
Height's distance below it, the Minimum Base Height's margin past the dedendum corner, the tooth
floor's constant, both spellings of the Maximum Face Width, the Toe Limit, and the defaulted Toe
Radius landing the inner toe corner exactly at A's station.

**From:** `spec/bevelgear/instructions.md` L35–L144 L227–L265 L294–L315
`.claude/skills/generate-gear/PLAYBOOK.md` L854–L858 L103–L118

## 3 `[PROSE]` Build the occurrence tree

`generate(inputs)` reads ALL inputs first, then creates occurrences. Bevel registers no user
parameters, so nothing creates an occurrence until every selection has been read and the
selection-context-shift hazard does not bite here — keep the order anyway so it stays that way
(`[PB-SELECTION-STASH]`).

Create each occurrence with `parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` and
name `occurrence.component.name` (`[PB-OCCURRENCE-TREE]`). The tree is: `Bevel Gear` under the
user's Parent Component, then `Design` under `Bevel Gear`. The Design component owns every sketch,
construction plane, axis and feature. `self.bevelOccurrence` holds the top occurrence for cleanup;
`self.designOccurrence`, `self.designComponent` and `self.bevelComponent` hold the inner tree.
`deleteComponent()` calls `deleteMe()` on the top occurrence and is the entry point's error
rollback.

**Never activate any occurrence** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`): the Anchor
sketch is created on the user's EXTERNAL, root-owned target plane, and an activated occurrence
resolves that plane in its own local frame, collapsing the build onto world XY whatever the real
tilt. The sole exception is the spiral crown's scale step in step 17. All features run in the one
Design component, so no cross-sibling reference is ever needed (`[PB-NO-CROSS-SIBLING]`).

Bevel uses a **standalone generator**: `BevelGearGenerator.__init__(self, design)` stores
`self.design` and `self.bevelOccurrence = None`. It does not subclass `base.Generator` and does not
use `getOccurrence`, `addParameter`, `parameterName` or `createSketchObject`; those four are named
here only to say they are not used. From `base.py` import only `get_selection` and `get_boolean`.
Imports are explicit — no `import *`.

`generate` and `deleteComponent` are methods this module DEFINES for the entry point to call, not
calls it makes.

<!-- check-step-calls: ignore getOccurrence addParameter parameterName createSketchObject deleteMe generate deleteComponent -->
<!-- check-compile: ignore getOccurrence addParameter parameterName createSketchObject generate deleteComponent -->

**From:** `spec/bevelgear/instructions.md` L267–L292 L317–L343 L411–L421 L455–L463
`spec/bevelgear/fusion.md` L153–L160
`.claude/skills/generate-gear/PLAYBOOK.md` L802–L828 L648–L649

## 4 `[GO]` Draw the Anchor sketch

`_buildAnchorSketch(design, plane, center)` adds a sketch named `Anchor` **directly on the
user-selected target plane** with `sketches.add`, whether that selection is a `ConstructionPlane` or
a `PlanarFace`. Do not re-derive or offset it: a coplanar construction plane built inside the
sub-component resolves in the sub-component's own frame and silently loses the selected plane's
world orientation (`[PB-USE-SELECTED-PLANE]`).

Mark the centre by projecting the user's Center Point into the sketch with `sketch.project`.

Draw one line through the projected centre with `sketchCurves.sketchLines.addByTwoPoints`, seeding
its two endpoints at exactly **±0.5 cm** from the projected centre along the sketch-local X, so the
seeded length is 10 mm. Apply **BOTH** `geometricConstraints.addCoincident(projectedCenter,
anchorLine)`, which pins the centre onto the line, **and**
`geometricConstraints.addMidPoint(projectedCenter, anchorLine)`, which makes the centre bisect it.
Use both, not midpoint alone. Add an aligned distance dimension with
`sketchDimensions.addDistanceDimension` and **do not assign** `.parameter.value`: the dimension
simply locks the seeded 10 mm, and the value is arbitrary because nothing downstream reads it.
Then pin the direction with `geometricConstraints.addHorizontal(anchorLine)` — sketch-local per
`[PB-REFLINE-DIRECTION]`, which survives a tilted target plane where a world-axis lock would
mis-orient the line.

Stash the projected-centre `SketchPoint` on `self` as `self._anchorCenterPoint`: §2 re-projects
THIS point, not the user's raw selection. Gate the sketch at the end of the step:
`if not sketch.isFullyConstrained: raise ...` naming the sketch (`[PB-FULL-CONSTRAINT]`,
`[BEVEL-F-FULL-CONSTRAINT]`). With midpoint, length and Horizontal the line has zero DOF.

`_buildAnchorSketch` is this module's own helper.

<!-- check-compile: ignore _buildAnchorSketch -->

<!-- proof-run: proofkit.RunParallel(bvAnchorCases, stepAnchorSketch) -->

`stepAnchorSketch` proves this step, with the projected centre both on the sketch origin and off it.
Two bench differences are recorded in the proof file: the engine's Midpoint emits two rows, so the
point-on-line coincidence that Fusion also requires would report redundant and is left out there;
and an unsigned length plus an unsigned horizontal admit both senses of the line, so the bench
writes the pair as signed distances, which is one row each either way.

**From:** `spec/bevelgear/instructions.md` L465–L469 L389–L397
`spec/bevelgear/fusion.md` L19–L30
`.claude/skills/generate-gear/PLAYBOOK.md` L432–L448 L829–L839

## 5 `[GO]` Build the Gear Profiles plane

Create a construction plane through the Anchor Line with
`constructionPlanes.createInput()` then `setByAngle(anchorLine, ValueInput.createByString('90 deg'),
targetPlane)`, and name it `Gear Profiles Plane`. Pass the `SketchLine` DIRECTLY; never wrap it in
`Path.create` first (`[PB-CONSTRUCTION-PLANES]`). Build it off the ORIGINAL `targetPlane` as the
reference — this is the second place the target-plane orientation reaches the bodies, and
substituting a re-derived plane here also collapses the gear onto XY (`[PB-USE-SELECTED-PLANE]`).

Then add the sketch `Gear Profiles` on this plane with `sketches.add`.

Why the plane has to be built this way: because it is perpendicular to the target plane and contains
the Anchor Line, the direction perpendicular to the projected anchor line INSIDE this sketch **is**
the target-plane normal, which is what makes every §2 position a purely sketch-local computation
(`[BEVEL-F-APEX-LOCAL]`).

<!-- check-step-calls: ignore Path.create -->

<!-- proof-run: proofkit.RunParallel(bvPlaneCases, stepGearProfilesPlane) -->

`stepGearProfilesPlane` proves this step across five target-plane tilts, including world XY and a
vertical plane. It builds both planes in world space and checks that the Gear Profiles plane is
perpendicular to the target plane, contains the Anchor Line, and offers the target normal as its
in-plane perpendicular — and that for a tilted target plane that perpendicular is NOT world Z, which
is the collapse this rule exists to prevent.

**From:** `spec/bevelgear/instructions.md` L471–L473
`spec/bevelgear/fusion.md` L117–L144
`.claude/skills/generate-gear/PLAYBOOK.md` L766–L777 L829–L839

## 6 `[GO]` Draw the Gear Profiles sketch — the §2 lattice

This one sketch holds the whole lattice for both gears. It is built inside `_buildGearProfiles`,
which also stashes `self._gearProfilesPlane`, `self._gpSketch`, `self._apexSketchPoint`,
`self._apex2d`, `self._coneDistance_cm`, `self._gamma_p`, `self._gamma_g` and, at the end,
`self._faceWidthResolved_cm`.

**Three rules govern every line and every dimension in this sketch.**

1. **Every line here is a construction line** — `line.isConstruction = True` — the lattice lines, the
   toe lines M->N and O->P, and the short reference and connector lines M->C, N->A, O->D, P->B,
   A->G, B->I, C->K/K', D->L/L' alike. The solid features consume only the per-gear Profile
   sketches of step 11, never a §2 curve.
2. **Every length dimension here is `AlignedDimensionOrientation`**, passed as the `orientation`
   argument of `sketchDimensions.addDistanceDimension(pointOne, pointTwo, orientation, textPoint)`
   as an `adsk.fusion.DimensionOrientations` value. This figure has no axis-aligned line in it: the
   shaft axes sit at the Shaft Angle to each other and the whole lattice tilts with the target
   plane, so a Horizontal or Vertical orientation would dimension the line's PROJECTION onto a
   sketch axis rather than its length. Wherever a value is named below — the PPD/2 and DPD/2 drops,
   the two `Module * 1.25` dedendum lines, the Tooth Spacing dimension on K' / L', and the Toe
   Radius dimension on N->A' and P->B' — it means an aligned distance dimension of that value. The
   offset dimensions are a different call, `sketchDimensions.addOffsetDimension`, which takes no
   orientation.
3. **Every §2 line is created from raw `Point3D` coordinates and each end that meets an existing
   point takes exactly ONE `geometricConstraints.addCoincident`** — never passed an existing
   `SketchPoint` to share it (`[BEVEL-F-COINCIDENT-STYLE]`, a stricter delta to
   `[PB-SHARE-XOR-COINCIDENT]`). Sharing without a coincident leaves the sketch under-constrained
   and the gate fails on "Gear Profiles"; sharing AND coinciding is redundant and the solve fails
   outright with `VCS_SKETCH_SOLVING_FAILED`. This covers the short reference and connector lines
   whose BOTH ends already exist — C->K, D->L, C->K', D->L', M->C, N->A, O->D, P->B, B->I, A->G. No
   §2 line is exempt. And each named line is created **once** and reused; never redraw a segment to
   get a second reference to it (`[BEVEL-F-LINE-ONCE]`).

**Project the anchor geometry.** Project the ANCHOR SKETCH's centre `SketchPoint`, the one stashed
in step 4, with `sketch.project(entity)` — not the user's raw Center Point, which is a
cross-component reference that can resolve inconsistently. Write the call as `sketch.project` and do
**not** substitute `project2`: the compiled API reference declares `project2(entities, isLinked)` and
no `project`, so every gate here reports the call as unverified, and that report is expected rather
than a defect. The two are not interchangeable in any case — `project2` takes a list and returns
one. Project the anchor line as well. `sketch.project` does not FIX what it brings in
(`[PB-PROJECT-NOT-FIXED]`).

<!-- check-step-calls: ignore project2 -->

**The seed frame.** Let `c` be the projected centre and `d` the projected anchor line's 2-D unit
direction. The in-plane perpendicular is `perp = (-d.y, d.x)`, and the **sign of `perp` is chosen by
the target-plane normal**, read as `targetPlane.geometry.normal` for BOTH selection kinds — a
`BRepFace`'s `geometry` and a `ConstructionPlane`'s `geometry` are each a `core.Plane` carrying
`.normal`. That is a one-bit direction comparison and the single permitted world use in §2
(`[BEVEL-F-GROW-SIDE]`). Every §2 POSITION is then computed in sketch-local 2-D coordinates; never
from a world round-trip (`[BEVEL-F-APEX-LOCAL]`).

**Centre -> Apex.** Draw a construction line from `c` with
`sketchCurves.sketchLines.addByTwoPoints`, seeding its far end — the Apex — at
`c + perp * (R * cos(gamma_g) + <resolved Driving Gear Base Height>)`. Seed it at that distance and
**not** at the Driving Gear Pitch Diameter: the net closes this line at `R * cos(gamma_g)` above
point I plus the resolved driving base height, and for the default 31/31 pair at 90 deg the old
seed sat 11.6 mm past where the solve puts it (`[PB-SEED-NEAR]`). Pin the start with exactly one
`geometricConstraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)`, and apply
`geometricConstraints.addPerpendicular(centerToApex, projectedAnchorLine)`. Add **no** length
constraint on this line.

**Driving Gear Shaft Axis.** From the Apex toward the anchor line, i.e. along `-perp`. Seed its far
end at `apex - perp * (R * cos(gamma_g))`, which is `c + perp * <resolved Driving Gear Base
Height>` — measured from the APEX, not from `c`; the older `c - perp * length` puts B on the wrong
side of the figure entirely. Coincident its start to the Apex. Apply
`geometricConstraints.addParallel(drivingShaftAxis, centerToApex)`. Do **NOT** use
`geometricConstraints.addVertical`: a world-vertical lock is wrong on a tilted target plane and
mis-orients the figure. The end point is **B**. Do not dimension this line's length.

<!-- check-step-calls: ignore addVertical -->

**Pinion Gear Shaft Axis.** The driving direction rotated about the Apex by the Shaft Angle. The two
senses put point A on opposite sides and choosing wrong mirrors the gear onto the wrong side of the
target plane, so **form BOTH candidate A positions — rotated by +Shaft Angle and by -Shaft Angle —
and keep the one whose endpoint has the greater X in the Gear Profiles sketch.** Compare the two and
take the larger; do not rotate one fixed sense and flip it only when X comes out negative, because
when both are positive that shortcut keeps the wrong one. Call the kept unit direction `pinionDir`.
Coincident its start to the Apex. Apply
`sketchDimensions.addAngularDimension(pinionShaftAxis, drivingShaftAxis, textPoint)` set to the
Shaft Angle, and place the text point INSIDE the Sigma wedge so it measures Sigma and not
`180 - Sigma` (`[PB-ANGULAR-DIM]`) — e.g. at
`apex + normalize(pinionDir + drivingDir) * (PPD / 4)`. The end point is **A**. Do not dimension
this line's length.

**The two drops, closing at Apex 2.** From A, a construction line perpendicular to the Pinion Shaft
Axis, drawn toward the side Apex 2 lies on. ⚠️ Apex 2 sits in the interior wedge BETWEEN the two
shaft axes, so this drop points toward the OTHER shaft axis, toward B; pick the perpendicular sense
by the sign of its dot product with the A->B direction. `addPerpendicular` against the Pinion Shaft
Axis, and an aligned distance dimension of `Pinion Gear Pitch Diameter / 2`. Coincident its start to
A. From B, the twin: perpendicular to the Driving Shaft Axis, sense picked by the dot with the B->A
direction, `addPerpendicular` against the Driving Shaft Axis, aligned distance dimension of
`Driving Gear Pitch Diameter / 2`, start coincident to B. ⚠️ **Do NOT choose the driving drop's
sense by a "toward the anchor line" reference**: the Driving Shaft Axis is itself parallel to that
direction, so the dot is about zero and the test silently picks an arbitrary side. If the two drops
seed Apex 2 on opposite sides, the closing coincidence flips the whole frame to its mirror, A jumps
across, the pinion dedendum C collapses onto D, the revolved frustum is degenerate and the toe cut
finds no cone face.

Then `geometricConstraints.addCoincident` the two drops' end points. That point is **Apex 2**. At
Shaft Angle 90 deg, Apex, A, Apex 2 and B form a rectangle; at other angles the along-shaft lengths
adjust so the two perpendicular drops coincide.

**Seed the along-shaft lengths from the closed form** so the solver takes the right branch at any
Shaft Angle — these are seeds only, and the lengths stay undimensioned
(`[BEVEL-F-DRIVEN-DIMS]`): `|Apex->A| = R * cos(gamma_p)` and `|Apex->B| = R * cos(gamma_g)`. Both
cosines are positive for every Shaft Angle the range check admits, which is what the Maximum Shaft
Angle guarantees.

**The Pitch Line and the two dedendum lines.** Draw Apex to Apex 2 and coincident both ends; this is
the Pitch Line. From Apex 2 draw a construction line to either side, each `addPerpendicular` against
the Pitch Line with an aligned distance dimension of `Module * 1.25`. The one drawn TOWARD the
anchor line is the Driving Gear Dedendum ending at **D**; the one drawn AWAY from it is the Pinion
Gear Dedendum ending at **C**. Then draw Apex->C and Apex->D, coincident at both ends: these are the
Pinion and Driving **Root Axis**.

**The module-length extension chains.** From A, a line collinear with Apex->A extending one Module,
with `geometricConstraints.addCollinear` naming **Apex->A**, the line A's start sits on, and the
start coincident to A. No dimensional constraint. Its end is **E**. Draw C->E with both ends
coincident and `addPerpendicular(A->E, C->E)`. From B, the twin: collinear with Apex->B, end **F**,
and D->F with `addPerpendicular(B->F, D->F)`.

From E, a line collinear with **A->E** — never with Apex->A further up the same chain, even though
both describe the same infinite line; naming the axis raises `VCS_SKETCH_OVER_CONSTRAINTS`
(`[BEVEL-F-COLLINEAR-CHAIN]`, `[PB-COLLINEAR-CHAIN]`). One Module long, undimensioned, start
coincident to E. Its end is **G**. From C, a line one Module long, undimensioned, start coincident
to C, collinear with **Apex2->C**, the Pinion Dedendum line C is the endpoint of. Its end is **H**.
From F, collinear with **B->F**, end **I**; from D, collinear with **Apex2->D**, end **J**.

**The heel edges and their base-height offsets.** Connect G and H with a line, both ends coincident,
and **`addPerpendicular(E->G, H->G)`**. That perpendicular is what makes H->G parallel to the
A->Apex2 drop, which `addOffsetDimension` requires before it can be applied at all: its
documentation asks for a second entity parallel to the first, and it controls only the perpendicular
distance. Connect I and J likewise with `addPerpendicular(F->I, J->I)`.

Then `sketchDimensions.addOffsetDimension(bDrop, iToJ, textPoint)` between the **B->Apex2
perpendicular drop** — the DPD/2 drop, not the Apex->B shaft axis — and J->I, with
`.parameter.value` set to the RESOLVED Driving Gear Base Height from step 2. J->I is already
parallel by construction, so add **no** extra parallel constraint (`[PB-OFFSET-DIM]`). Then
`addOffsetDimension` between the **A->Apex2 drop** and G->H, set to the RESOLVED Pinion Gear Base
Height. Same rule: no parallel constraint.

**Pin the figure.** `geometricConstraints.addCoincident(pointI, projectedCenter)`.

**The tooth centres.** Draw a construction line away from the Apex starting at G along Apex->A; its
end is **K**. Pin K with TWO point-on-line coincidences —
`addCoincident(K, line Apex->A)` and `addCoincident(K, the Pinion Dedendum line Apex2->C extended)` —
rather than `addCollinear`: G and C are already fixed by now, so a collinear over-constrains and
Fusion errors. Draw C->K for reference. The driving twin gives **L**, pinned onto Apex->B and onto
Apex2->D, with D->L for reference.

**Tooth Spacing.** The §3 tooth is centred at **K'**, which is K shifted along the dedendum line by
Tooth Spacing, AWAY from the lower corner C, i.e. in the C->K direction beyond K. **When Tooth
Spacing is 0, build nothing here**: set `K' = K` and reuse the existing C->K line, because a
zero-length dimensioned line is degenerate and one segment gets one line
(`[BEVEL-F-LINE-ONCE]`). When Tooth Spacing is above 0, draw a construction line starting at K with
its far end seeded on the far side of K from C along the dedendum direction, pin its start with
`addCoincident(start, K)` and its far end with `addCoincident(K', the Pinion Dedendum line Apex2->C
extended)`, and add an aligned length dimension of Tooth Spacing; no collinear, for the same reason
as K. The far end is **K'**. Then draw the tooth-centre reference line **C->K'**. Build all of this
INSIDE this sketch so the end-of-step gate covers it. The driving twin gives **L'** and **D->L'**,
from the same single Tooth Spacing value. Only the tooth's centre moves: the virtual tooth number
and the drawn tooth size are unchanged.

**Resolve the Maximum Face Width here.** A, B, C, D, H and J now exist and are solved, so read
`pointA.geometry`, `pointB.geometry`, `pointC.geometry`, `pointD.geometry`, `pointH.geometry` and
`pointJ.geometry` — the SOLVED positions, never the pre-solve seeds (`[PB-SOLVED-GEOMETRY]`) — and
take `0.95 *` the smaller of the perpendicular distance from A to the line through C and H, and from
B to the line through D and J. Cap the auto Face Width default `Cone Distance / 6` to it and reject
a user value above it, naming the maximum. The SMALLER pitch diameter binds and it is not always the
pinion's: written with the pinion's diameter by name the bound is wrong whenever the driving gear
carries the smaller tooth count — on a Driving 17 / Pinion 31 pair at Module 1 the real bound is
3.883 mm against the pinion form's 13.591. Seeds diverge markedly for asymmetric counts, so a
seed-based bound is too loose on the binding side and the toe still crosses the axis, failing the
revolve with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`). Stash the result as
`self._faceWidthResolved_cm`.

Then resolve, per gear: the **Root Length** at Toe Extension 0 as
`Face Width * |Apex->Ded| / R` with `|Apex->Ded| = sqrt(R**2 + (1.25 * Module)**2)`; the **Toe
Radius**, where 0 means auto and auto is `r - Face Width / sin(gamma)`; the **Toe Radius Ceiling**
`(r - 1.25 * Module * cos(gamma)) * (1 - Face Width / R)`, which a user value must be strictly
below; and the **Toe Limit** `sqrt(R**2 + (1.25*Module)**2) - Toe Radius / sin(gamma_root)` with
`gamma_root = gamma - atan(1.25 * Module / R)`. Toe Extension 100 reaches
`0.99` of the way from the Toe Extension 0 root length to the SMALLER of the two gears' Toe Limits;
the 0.99 is there because AT the limit the toe face has zero length, the body carries no cone at its
toe end and the conical toe cut has no face to find. Reject a Toe Extension above 0 on a pair whose
Toe Limit has fallen below the Toe Extension 0 root length, naming the gear and its Toe Radius
Ceiling; Toe Extension 0 still resolves, and do not silently substitute a smaller Toe Radius.

**The toe lines.** Create M->N. Seed M near the midpoint of Apex->C and N by sliding from that
M-seed along the C->H direction far enough to reach the pinion's toe radius; do NOT seed M/N a Face
Width away from C/H, which starts N near H and does not converge (`[PB-SEED-NEAR]`). Apply exactly
three constraints: `addCoincident(M, Pinion Root Axis)`; `addParallel(M->N, C->H)`; and
`addOffsetDimension(C->H, M->N, textPoint)` with `.parameter.value` set to the Root Length
re-measured perpendicular to the pitch line, `Root Length * R / |Apex->C|` — an offset dimension
controls a perpendicular distance, so the root length is carried in that form, and at Toe Extension
0 that value is exactly the resolved Face Width. Place the text point in the gap between C->H and
M->N on the Apex side, e.g. `(M_seed + C) / 2` (`[PB-OFFSET-DIM]`). The line's start is **M** and
its end is **N**. Draw M->C.

**The front face A'->N, which is what holds N.** ⚠️ N is NOT pinned to line A->Apex2. It rides the
Pinion Gear Toe Radius. Draw a line from N to a new point **A'**, seeded at N's station on the shaft
axis. Apply `addCoincident(A', line Apex->A)` — A' is the only toe-end point that touches that axis,
and it is a foot, not a corner. Apply `addPerpendicular(N->A', line Apex->A)`, so the front face
stands square to the shaft and the revolve sweeps it into a flat annulus. Add an aligned distance
dimension on the whole line N->A' equal to the resolved Pinion Gear Toe Radius. ⚠️ Pinning N itself
to the Apex->A shaft axis stays forbidden: that puts N ON the axis of revolution and the later
conical split fails with `ASM_API_FAILED` for asymmetric tooth counts even though the symmetric case
survives. Those three rows plus the offset and `addCoincident(M, Pinion Root Axis)` fully constrain
M, N and A' — six freedoms, six constraints.

**A' replaces A as the hexagon's first vertex.** Draw the line A'->G, the hexagon's shaft-axis edge:
it starts at the front face's foot, not at A, and the two coincide exactly at Toe Extension 0 with a
defaulted Toe Radius.

**The driving side.** Create O->P as the mirror of M->N: seed O near the midpoint of Apex->D and P
slid along D->J toward the driving toe radius, then `addCoincident(O, Driving Root Axis)`,
`addParallel(O->P, D->J)` and `addOffsetDimension(D->J, O->P, textPoint)` with the same root-length
value and the text point in the gap on the Apex side of D->J. Draw O->D. Build the driving front
face **B'->P** exactly as the pinion's, substituting B for A, P for N and the Driving Gear Toe
Radius: the line P->B', `addCoincident(B', line Apex->B)`, `addPerpendicular(P->B', line Apex->B)`
and a length dimension on P->B'. Draw B'->I.

**Gate the sketch.** `if not sketch.isFullyConstrained: raise ...` naming "Gear Profiles"
(`[BEVEL-F-FULL-CONSTRAINT]`). Do not reach full constraint by dimensioning a driven length
(`[PB-NO-OVERCONSTRAIN]`).

<!-- check-compile: ignore _buildGearProfiles -->

<!-- proof-run: proofkit.RunParallel(bvSketchCases, stepGearProfiles) -->

`stepGearProfiles` proves this step across the Shaft Angle range, both ratio directions, the low
tooth-count floor, both sides of the base-height, Face Width, Tooth Spacing, Toe Extension and Toe
Radius branches, and — because §2 derives every direction relative to the projected anchor line —
across several anchor directions, an off-origin projected centre, and both grow sides. It reads
every hexagon vertex, both tooth centres, both drops and both driven shaft lengths back off solved
geometry, and holds the Maximum Face Width read from `.geometry` to its closed form. Three bench
arity differences are recorded in the proof file: the two heel-edge perpendiculars and the two toe
parallels are left out because the engine's offset constraint emits two rows and already carries
them, and the coincidence pinning I to the projected centre is written as one row because I is
already on the ray through the centre. Shaft Angle 30 deg is carried as a **declared refusal**: this
lattice's conditioning reads 2.8308e-05 against the engine's 4e-05 floor, which is the same reading
the spec records for two of the three independently written lattices.

**From:** `spec/bevelgear/instructions.md` L106–L138 L471–L576 L389–L409
`spec/bevelgear/fusion.md` L19–L30 L69–L115 L117–L151
`.claude/skills/generate-gear/PLAYBOOK.md` L432–L484 L582–L613 L626–L642 L708–L714

## 7 `[GO]` Build the per-gear tooth plane

Run steps 7 through 9 and 11 through 24 **once per gear, pinion first and driving second**, with
these substitutions:

| | Pinion | Driving |
|---|---|---|
| `gearLabel` | `Pinion` | `Driving` |
| tooth centre / reference line | K' / C->K' | L' / D->L' |
| pitch cone half-angle | `gamma_p` | `gamma_g` |
| hexagon vertices, in draw order | A' -> G -> H -> C -> M -> N -> A' | B' -> I -> J -> D -> O -> P -> B' |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A'->G | B'->I |
| toe cut edge | M->N | O->P |
| heel cut edge | C->H | D->J |
| §2 shaft construction line (NOT usable as the axis) | Apex->A | Apex->B |

Compute this gear's virtual (back-cone, Tredgold) tooth number from the closed form, **not** by
measuring Apex2->K'/L': virtual pitch radius `= (this gear's Pitch Diameter / 2) / cos(gamma)`, and
virtual tooth number `= floor(2 * virtualPitchRadius / Module)` as an int. Pin the cm-to-mm
conversion: the stashed pitch diameters are internal **cm** while Module is the raw **mm** value, so
compute `virtualPitchRadius_mm = (pitchDia_cm * 10 / 2) / cos(gamma)` and then
`virtualTeeth = floor(2 * virtualPitchRadius_mm / Module)`. Skipping the `* 10` makes the count
about ten times off. The virtual tooth number does not depend on Tooth Spacing.

Then create a construction plane that includes the tooth-centre reference line, named
`{gearLabel} Plane`, perpendicular to the Gear Profiles sketch plane. Use the framework helper
`plane_by_angle(designComponent, referenceLine, gearProfilesPlane, 90)`, which wraps
`constructionPlanes.createInput()` plus `setByAngle`. Pass the sketch line DIRECTLY; never wrap it
in `Path.create` (`[PB-CONSTRUCTION-PLANES]`).

<!-- check-compile: ignore plane_by_angle floor _buildVirtualSpurProfile -->

<!-- proof-run: proofkit.RunParallel(bvToothCases, stepToothPlane) -->

`stepToothPlane` proves this step for both gears across the same regime: it draws the tooth-centre
reference line at the coordinates §2 solves it to, builds the plane in world space on a tilted
target plane, and checks that the plane contains that line, passes through the dedendum corner and
stands perpendicular to the Gear Profiles plane.

**From:** `spec/bevelgear/instructions.md` L578–L590 L689–L702
`.claude/skills/generate-gear/PLAYBOOK.md` L766–L777

## 8 `[GO]` Draw the virtual spur tooth sketch

Using the `{gearLabel} Plane` and the tooth-centre point as the centre, draw a spur gear tooth
profile of this Module and the virtual tooth number from step 7, in a sketch named
`{gearLabel} Tooth`.

Bevel borrows the spur tooth generator rather than re-implementing it:
`from .spurgear import SpurGearInvoluteToothDesignGenerator`, used once per gear inside
`_buildVirtualSpurProfile`:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))
```

`VirtualSpurProxy` is the framework's, imported `from .spurproxy import VirtualSpurProxy`; bevel
defines no local proxy or value-wrapper class. It precomputes, in internal cm, exactly the parameter
keys the spur drawer reads through `parent.getParameter(name).value`, and its defaults already match
bevel: pressure angle 20 deg, which is NOT a bevel dialog input, and `InvoluteSteps` 15.

The **180 degree rotation is delivered through the `draw()` angle argument** — the drawer rotates
the whole tooth by it — and not by a post-hoc move or sketch rotation, which relies on spur's radial
flank-to-root pinning so the connecting lines rotate with the tooth.

**Read `proxy._lastToothEmbedded` back after `draw()` returns.** The drawer decides during `draw()`
whether the tooth is embedded — tip, root and flanks meeting with no connecting lines — and records
it on the proxy, which pre-initialises the slot to absorb that write. This flag is not optional
bookkeeping: it is the deterministic selector for the tooth loop's line count in step 13. Stash it
alongside the tooth sketch and plane that `_buildVirtualSpurProfile` writes back into the per-gear
dict.

After `draw()` returns do **NOT** hard-gate this sketch: log it if `not
toothSketch.isFullyConstrained`, never raise. The tooth sketches are exempt from the
full-constraint gate, and only because the drawer labels each of its four circles with along-path
sketch text, which holds a DOF of its own (`[PB-TEXT-HOLDS-DOF]`, `[BEVEL-F-FULL-CONSTRAINT]`).
The exemption covers the labels and nothing else; never read it as licence for loose geometry.

`getParameter`, `draw` and `_lastToothEmbedded` belong to the borrowed generator and its proxy
rather than to this module's own call surface.

<!-- check-step-calls: ignore getParameter -->
<!-- check-compile: ignore VirtualSpurProxy SpurGearInvoluteToothDesignGenerator getParameter draw _lastToothEmbedded math.radians -->

<!-- proof-run: proofkit.RunParallel(bvToothCases, stepVirtualSpurTooth) -->

`stepVirtualSpurTooth` proves this step for both gears. It holds the virtual tooth number to the
back-cone closed form, checks it stays above the real count, draws the tooth already turned half a
turn, grounds its local origin on the projected tooth centre, and asserts the CURVE COUNTS step 13
selects on: one loop of 2 NURBS, 2 arcs and 0 lines when embedded or 2 lines when not, plus the one
root disc. Both branches are reached, because embedding happens at HIGH tooth counts and the virtual
numbers straddle the threshold. The proof also records that the Fusion sketch's exemption comes from
the four labels alone: this engine has no sketch text, so the same geometry reaches DOF 0 here.

**From:** `spec/bevelgear/instructions.md` L423–L451 L578–L591 L374–L384
`spec/bevelgear/fusion.md` L31–L58
`.claude/skills/generate-gear/PLAYBOOK.md` L188–L194 L508–L523

## 9 `[GO]` Build the per-gear tooth axis

Create a construction axis named `{gearLabel} Tooth Axis` through the tooth-centre point, normal to
the plane the tooth profile was drawn on, with `constructionAxes.createInput()` then
`setByTwoPlanes(planeA, planeB)` (`[PB-CONSTRUCTION-AXES]`). `setByPerpendicularAtPoint` is not
usable here: it would need a `BRepFace` this build does not have.

The two planes are the **Gear Profiles plane** and a **helper plane built
`setByDistanceOnPath(<tooth-centre reference line>, 1.0)`**, which stands perpendicular to that line
at its far end, the tooth-centre point. Their intersection is the line through the tooth centre
normal to the tooth plane. Pass the sketch line directly, never through `Path.create`. Creating this
axis in the never-activated Design component is proven to work — `constructionAxes.add` via
`setByTwoPlanes` does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here — so keep the axis.

<!-- check-step-calls: ignore setByPerpendicularAtPoint -->

<!-- proof-run: proofkit.RunParallel(bvToothCases, stepToothAxis) -->

`stepToothAxis` proves this step for both gears: it builds the two planes' normals from the same
geometry the spec names and checks that their intersection runs normal to the tooth plane, passes
through the tooth centre, and stands perpendicular to the tooth-centre reference line.

**From:** `spec/bevelgear/instructions.md` L580–L581 L593
`.claude/skills/generate-gear/PLAYBOOK.md` L766–L790

## 10 `[PROSE]` Create the per-gear component

Create a new component as a child of the **Bevel Gear** component — the same component that owns
Design, not the user's Parent Component — named `{gearLabel} Gear`, i.e. `Pinion Gear` or
`Driving Gear`. Use `parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` and name
`occurrence.component.name`.

The finished bodies for this gear end up here, but the feature operations do not run here: Fusion
rejects cross-sibling sketch and project calls even when the target is activated or the entities are
wrapped in assembly-context proxies (`[PB-NO-CROSS-SIBLING]`), so everything is built in the Design
component and the finished bodies are relocated at the end of step 24. The visible end state is
identical.

**From:** `spec/bevelgear/instructions.md` L703
`.claude/skills/generate-gear/PLAYBOOK.md` L808–L825

## 11 `[GO]` Draw the per-gear Profile sketch

Open a **fresh sketch on the axial Gear Profiles plane** with `sketches.add`, named per the table in
step 7 — **one profile sketch per gear**, so that `sketch.profiles` holds exactly this one hexagon
loop. Do not draw both gears' hexagons in the shared Gear Profiles sketch: that would leave two
identically shaped loops to disambiguate.

Build the hexagon on FIXED vertices by the recreate-share-fix recipe (`[PB-PROJECT-NOT-FIXED]`),
which is what makes this sketch fully constrained without projecting §2 points:

1. Recreate the six §2 vertices as NEW points at their exact world-mapped positions —
   `sketch.sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))` — valid because §2 is
   fully constrained by now, which is what makes `worldGeometry` defined
   (`[PB-WORLDGEO-CONSTRAINED]`). `modelToSketchSpace` is a point-transforming METHOD, not a matrix
   (`[PB-SPACE-METHODS]`).
2. Draw the closed hexagon in the table's draw order as six `sketchCurves.sketchLines.addByTwoPoints`
   lines **sharing** those points.
3. Fix the lines and their endpoints **after** the lines exist: `edge.startSketchPoint.isFixed =
   True` and `edge.endSketchPoint.isFixed = True`. Order matters — fixing a bare point before it is
   consumed as a line endpoint does not leave the sketch fully constrained.

The hexagon's **first edge is this gear's shaft axis** for the revolve, the pattern, the bore plane
and the meshing rotation, so it has to be fixed well enough to carry a trustworthy world position: a
free edge resolves against a default world-XY frame and silently moves the body onto world XY, which
was observed on the driving gear.

Gate the sketch: `if not sketch.isFullyConstrained: raise ...` naming it
(`[BEVEL-F-FULL-CONSTRAINT]`).

<!-- proof-run: proofkit.RunParallel(bvToothCases, stepProfileSketch) -->

`stepProfileSketch` proves this step for both gears: exactly one closed region, of the hexagon's own
area and usable as a revolve profile; the first edge lying on the axis of revolution at both ends;
no vertex on the far side of that axis, which is what would abort the revolve; and the heel face and
the front face each standing square to the shaft.

**From:** `spec/bevelgear/instructions.md` L689–L707
`spec/bevelgear/fusion.md` L27–L30
`.claude/skills/generate-gear/PLAYBOOK.md` L449–L463 L576–L596

## 12 `[GO]` Revolve the Gear Body

This sketch holds exactly one hexagon loop, so take its single profile directly —
`sketch.profiles.item(0)`, with no filtering (`[PB-SINGLE-PROFILE]`) — and revolve it around the
**shaft-axis edge**, the hexagon's first edge from step 11 and NOT the §2 `Apex->A` / `Apex->B`
construction line, which lives in a different sketch and fails or misbuilds.

`revolveFeatures.createInput(profile, axis, operation)` then
`setAngleExtent(False, ValueInput.createByString('360 deg'))` then `add(input)` (`[PB-REVOLVE]`).
The result is this gear's **Gear Body**, the frustum. Because the toe edge is one edge of the
revolved profile, the body already carries the conical face that edge sweeps, and likewise the heel
edge's cone; both are reused as cutting tools in step 14.

The profile must not cross the axis of revolution or Fusion aborts with `ASM_WIRE_X_AXIS`; the
Maximum Face Width cap of step 6 and the strictly positive Toe Radius are what keep it on one side.

<!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepRevolveGearBody, assertRevolveGearBody) -->

`stepRevolveGearBody` proves this step as a **polygonal sweep**, because decad publishes a revolved
body's volume with a proven bound equal to the volume itself and a revolved body is therefore
Suspect at any tolerance. It builds the three bands the frustum's profile edges sweep — the root
cone out to the dedendum corner, the heel cone out to the heel end, and the toe-dish plug that
hollows the front face — lays them apart along the shaft axis and never joins them. It asserts the
frustum as their SIGNED sum against Pappus on the §2 hexagon, band by band against its own stations
and ring radii, and cone half-angle by cone half-angle: the heel band and the toe plug come out
parallel on the back-cone family, and the root band stands one dedendum angle off the pitch cone.
The cost is the union, which the proof file records: each band is separately watertight, but the
three are not shown closing into one solid.

**From:** `spec/bevelgear/instructions.md` L709 L757–L791 L822–L828
`.claude/skills/generate-gear/PLAYBOOK.md` L485–L491 L708–L714

## 13 `[GO]` Loft the uncut Tooth Body

Loft the **§2 Apex sketch point** — `centerToApex.endSketchPoint` from the Gear Profiles sketch, the
degenerate point section — to this gear's §3 tooth profile. `loftFeatures.createInput(operation)`,
then `loftSections.add(entity)` for the apex point and for the tooth profile in that order, then
`add(input)` (`[PB-LOFT]`). The result is this gear's **Tooth Body**.

Use the §2 Apex SKETCH point directly; do NOT create a construction point for it, because the Design
component is never active and `constructionPoints.add` would raise
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`).

Select the tooth cross-section loop with
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)`, where the line count
is **determined by** the `embedded` flag read back in step 8: `wantLines = 0 if embedded else 2`.
⚠️ Do **not** accept "0 **or** 2 lines". For a given gear only one of those is the real tooth, and
an unrelated loop — an inter-tooth or annular region between the drawn circles — can carry 2 NURBS
and 2 arcs with the OTHER line count; selecting it makes this loft fail with
`RuntimeError ... ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`, because the impostor loop cannot form a loft
tool body (`[PB-PROFILE-MATCH]`).

<!-- check-compile: ignore find_profile_by_curve_counts -->

<!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepLoftTooth, assertLoftTooth) -->

`stepLoftTooth` proves this step with two substitutions the proof file states: a section shrunk to a
tenth of the heel station stands in for the degenerate apex point, because decad has no point
section, and axis-perpendicular sections stand in for the back-cone tooth plane. It asserts the
tooth as the generalized cone the apex loft makes — the heel section's own measured area times the
heel station over three, less the truncated tip — and that the tooth's root seats on the frustum's
root cone at the heel while its tip stands proud of it.

**From:** `spec/bevelgear/instructions.md` L374–L384 L711 L792–L793
`.claude/skills/generate-gear/PLAYBOOK.md` L151–L156 L663–L672 L715–L719 L782–L790

## 14 `[GO]` Draw the spiral cone element and the 2-D tooth trace

Steps 14 through 18 are the **spiral branch** of the tooth-body hook
`_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld,
apexSketchPoint, toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane, gearLabel,
teethNumber, gamma)`, which `_createGearBody` calls once per gear on the freshly lofted uncut tooth,
before pattern, combine and bore. Its FIRST line is the gate
`if self._spiralAngle_rad <= 0: return cut_conical_ends(...)`: at Mean Spiral Angle 0 the hook
returns immediately with the straight tooth of step 19 and none of steps 14 through 18 runs at all.
`gamma` is this gear's pitch cone half-angle from §2.

**The caller's hand-off, pinned exactly.** `_createGearBody` builds the four world points from the
§2 sketch points' WORLD geometry and passes them positionally in the order `toeMid, heelMid,
toeConeWorld, heelConeWorld`:

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M->N** | **C->H** | **M** | **C** |
| Driving | **O->P** | **D->J** | **O** | **D** |

`toeMid` is the world MIDPOINT of the TOE edge and `heelMid` the world MIDPOINT of the HEEL edge —
two DIFFERENT edges. ⚠️ Do NOT pass the two endpoints of a single edge: M and N both sit at the toe,
so the span collapses to about zero or goes negative and the spiral inverts. ⚠️ `heelConeWorld` is
the dedendum corner C / D, on the root axis Apex->C / Apex->D, and **never H / J**, which lie one
Module further out on the Apex2->C / Apex2->D dedendum line and are off the root cone element;
using them skews the cone vector.

**A. The frame.** `axisDir` is the shaft axis direction from the two WORLD endpoints of
`shaftAxisEdge`, the in-sketch profile edge A'->G / B'->I, normalized. `coneVec =
normalize(heelConeWorld - apexWorld)`, the root cone element. `v = normalize(axisDir x coneVec)` is
the circumferential direction. `tpNormal = normalize(coneVec x v)` is the tangent-plane normal.
`distAlong(p) = (p - apexWorld) . coneVec`. Read every sketch quantity here in WORLD space, not in
sketch-local `.geometry` (`[PB-WORLD-FRAME]`).

⚠️ **The heel MUST be the outer end.** Before building `coneVec`, check the passed midpoints: if
`apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid` with `heelMid` **and**
`toeConeWorld` with `heelConeWorld`, then build `coneVec`. A negative span silently inverts the
entire spiral frame — the cutter-arc direction, the slice direction and the per-segment twist — and
the gear comes out completely wrong with no error.

From the midpoints, AFTER the swap guard: `R_toe = distAlong(toeMid)`, `R_heel =
distAlong(heelMid)`, `R_mean = (R_toe + R_heel) / 2`, `span = R_heel - R_toe`, now positive.

**B. The cutter arc's geometry**, in the tangent-plane 2-D frame with the apex at the origin,
x along `coneVec` so a point's x is its cone distance, and y along `v`. The cutter radius `r_c` is
the Cutter Radius input when non-zero, else `R_mean`. The hand sign is `+1` for `Right` and `-1` for
`Left`, **then negated for the pinion**, because the pair meshes with opposite hands. The centre is

```
Cx = R_mean - r_c * sin(psi)
Cy = handSign * r_c * cos(psi)
```

⚠️ The hand sign goes on the `cos` / `Cy` term, NOT on the `sin` / `Cx` term. Opposite hands are
mirror images across the cone element `y = 0`, which flips `Cy`; putting the sign on `Cx` mirrors
about `x = R_mean` instead, which is a different curve and gives the two gears unequal twist.

The arc's ends are taken a hair past the face so the kept arc clears the end trims:
`toe2d = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)` and
`heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)` with
`R_lo = R_toe - 0.06 * span` and `R_hi = R_heel + 0.06 * span`. The framework helper intersects the
apex circle of that radius with the cutter circle and keeps the solution nearest `(R_mean, 0)`, the
branch the mean point sits on.

**C. The sketches.** First draw a **cone-element construction line** from the apex to
`apex + R_heel * coneVec` in a sketch on the axial Gear Profiles plane, named
`{gearLabel} Cone Element`, with `sketchCurves.sketchLines.addByTwoPoints`. Then make the tangent
plane as that axial plane rotated 90 degrees about that line:
`plane_by_angle(designComponent, coneElementLine, axialPlane, 90)`, named `{gearLabel} Trace Plane`.
Add a sketch on it named **`{gearLabel} 2D Tooth Trace`**, and map 2-D coordinates to world with the
framework's `combine_point(apexWorld, px, coneVec, py, v)`. In it draw:

- the **cutter circle**, `sketchCurves.sketchCircles.addByCenterRadius` at `combine_point(...Cx,
  ...Cy)` with radius `r_c`, marked `isConstruction`, its centre pinned with
  `circle.centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`) and a diameter dimension of
  `2 * r_c` via `sketchDimensions.addDiameterDimension`;
- the **trace arc**, a three-point arc through `combine_point(toe2d)`, the mean point
  `combine_point(R_mean, 0)` and `combine_point(heel2d)`, with
  `sketchCurves.sketchArcs.addByThreePoints`, its centre made coincident to the cutter circle's
  centre with `geometricConstraints.addCoincident` and a radius dimension of `r_c` via
  `sketchDimensions.addRadialDimension`, so it is the genuine cutter circle and not a look-alike
  spline.

⚠️ Both dimension text points must be OFF-CENTRE (`[PB-RADIAL-DIM]`): use the mean point for the
arc's radius dimension and a point on the cutter circle such as `combine_point(Cx + r_c, Cy)` for
the circle's diameter dimension.

**Coordinates — this rule governs BOTH sketches.** The world `Point3D`s from `combine_point`, and
the raw apex and cone-end points of the Cone Element line, are passed **directly** into
`addByTwoPoints`, `addByCenterRadius` and `addByThreePoints`, where they are consumed as
**sketch-space** input, with **no `modelToSketchSpace` conversion**, even though `Sketch` offers
exactly that call and the points really are model-space coordinates. This is deliberate and
harmless because the trace sketch is construction and reference only — the twist is computed
analytically in step 16 and no downstream feature consumes it — and because the one thing that IS
consumed, the cone-element line, only places the Trace Plane, on which nothing but the
inspection-only trace sketch is built. If a later revision ever makes a feature consume the trace
sketch or the Trace Plane, the shortcut stops being safe and both sketches need
`modelToSketchSpace` on every point.

**D. There is no 3-D projection.** The 2-D cutter arc is the only trace geometry needed. There is no
`projectToSurface`, no root-cone-face search and no 3-D trace sketch: for unequal-ratio pairs the
arc wraps around the cone and the projection comes back as multiple disjoint fragments, so the
measured azimuth collapses to a fraction of the true sweep and the pinion comes out grossly
under-twisted. Do not reintroduce it.

These sketches are **exempt** from the full-constraint gate: the arc's endpoints are pinned by the
three-point construction rather than dimensioned, because dimensioning them over-constrains the
solve against the cone-element plane, so the sketch is deliberately left with free DOF. Do not gate
it (`[BEVEL-F-FULL-CONSTRAINT]`).

<!-- check-step-calls: ignore projectToSurface modelToSketchSpace -->
<!-- check-compile: ignore _transformToothBody _createGearBody cut_conical_ends circle_intersect_nearest combine_point plane_by_angle distAlong -->

<!-- proof-run: proofkit.RunParallel(bvSpiralSketchCases, stepSpiralTrace) -->

`stepSpiralTrace` proves this step for both gears, both hands, both ends of the Mean Spiral Angle's
range, both sides of the Cutter Radius branch and both ratio directions, with a Mean Spiral Angle of
0 case that takes the gate's early return and authors no trace at all. It checks the construction's
own invariants: the two endpoints at their apex-circle radii a hair past the face, the centre
exactly `r_c` from the mean point, the mean point on the cone element, the spiral angle realised AT
the mean point, both endpoints on the cutter circle, mirror symmetry under a hand flip with the
twist magnitude unchanged, and the roll ratio taken on the PITCH cone angle rather than the root
one, which would inflate it.

**From:** `spec/bevelgear/instructions.md` L595–L650
`spec/bevelgear/spiral-tooth-trace.md` L18–L27 L30–L63 L66–L87 L90–L145 L149–L182
`spec/bevelgear/fusion.md` L59–L67
`.claude/skills/generate-gear/PLAYBOOK.md` L430–L431 L442–L448 L643–L647

## 15 `[GO]` Slice the tooth into slabs and drop the apex scrap

**E. Slice.** Split the uncut apex-to-heel Tooth Body into cross-section slabs by planes
**perpendicular to the cone element**, spanning a touch past toe and heel, by a **fixed** scheme of
about eight planes whose count is not user-configurable. The first cut plane is the **parent
transverse tooth plane** — `parentToothPlane`, the `{gearLabel} Plane` of step 7, passed into the
hook — offset toward the apex by `span / 6`. The offset **sign is chosen per gear** so that it moves
toward the apex, because the parent plane's normal points opposite ways for the two gears: test
`(apex - planeOrigin) . normal`. Then a sequence stepped further toward the apex in `span / 6`
increments. Split with the framework helper
`slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)` where
`offsets = [sign * (k + 1) * span / 6 for k in range(8)]`; it splits piece by piece and keeps a
piece whole when a plane misses it.

⚠️ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in ONE
piece, the offset sign was wrong or `parentToothPlane` sits outside the tooth's span — retry the
whole cut once with the opposite sign. If it is still one piece, `raise` a clear self-diagnosing
error naming the gear, the final piece count, `span` and the sign tried (`[PB-EMPTY-RESULT]`,
`[PB-SELF-DIAGNOSING]`). Do **not** return an unsliced single-piece result: step F then drops that
one piece as the apex scrap, leaving `segments` empty, and the crown crashes with
`ValueError: max() iterable argument is empty` far from the cause.

**F. Order and drop the scrap.** Sort the segments by the `distAlong` of their centroid, read as
`body.physicalProperties.centerOfMass`. The first, apex-most one is the long apex-side scrap below
the toe: **remove it**. Drop it by re-slicing the list FIRST and deleting afterwards —
`segments = segments[1:]` before `removeFeatures.add(scrap)`, which is the timeline-visible removal
rather than a bare `deleteMe()` (`[PB-REMOVE-PIECES]`). After the drop, `segments` must be
non-empty; if it is empty the slice failed in E, so `raise` a clear error rather than proceeding
into the twist and the crown, which both assume at least one segment.

<!-- check-step-calls: ignore deleteMe -->
<!-- check-compile: ignore slice_body_by_offset_planes distAlong range max -->

<!-- proof-run: proofkit3d.RunSolidParallel(bvSpiralSolidCases, stepSliceSpiralSlabs, assertSliceSpiralSlabs) -->

`stepSliceSpiralSlabs` proves this step by building the slabs the slice would LEAVE, at the same
stations, and laying them apart along the shaft axis — there is no split-by-plane in this harness,
and the proof file records that substitution and its cost. It asserts that at least one cut plane
falls inside the tooth, so the slice really splits; that the cut planes step toward the apex in
`span / 6` increments along the cone element; that dropping the apex scrap leaves a non-empty list;
that each kept segment carries the volume its own two stations give; and that the scrap plus the
kept segments account for the whole tooth.

**From:** `spec/bevelgear/instructions.md` L652–L654 L818–L820
`.claude/skills/generate-gear/PLAYBOOK.md` L176–L177 L431 L753–L765

## 16 `[GO]` Twist each slab about the shaft axis

**G.** Rotate each segment about the **shaft axis** — `axisDir` through `apexWorld` — so the tooth
follows the trace, **centred on `R_mean` so the mid-face section stays unrotated**. That section then
meshes exactly like the straight tooth, which is what the pinion's zero mesh nudge depends on.

The total toe-to-heel twist comes from the conjugate crown-gear generation law, computed
**analytically** from the 2-D endpoints of step 14 — no projection, no curve sampling:

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

⚠️ `gamma` is this gear's **PITCH** cone angle, `self._gamma_p` for the pinion and `self._gamma_g`
for the driving gear, both from §2 — **NOT** `acos(coneVec . axisDir)`, which is the root cone angle,
about 14 degrees against the pitch cone's 29 for a 17-tooth pinion, and yields a twist about 1.6
times too large. ⚠️ The two members of a meshing pair legitimately get DIFFERENT twists: same cutter,
same Mean Spiral Angle, but different gamma, so `1 / sin(gamma)` differs — about 2.08 for a 17-tooth
pinion against 1.14 for a 31-tooth gear. That is why equal-teeth pairs always meshed while ratio
pairs failed under any method that gets the roll ratio wrong.

Each segment's rotation is a **linear share** keyed to the cone distance of its **HEEL FACE**, the
segment's farthest-along-the-element face, which is the exact section the step-18 loft samples:

```
ang = -handSign * total * (R_mean - R_heelFace(seg)) / span
```

⚠️ Key the twist on the segment's HEEL-FACE cone distance, NOT on its centroid: the loft samples the
heel face, so that face is what must land at the right azimuth, and centroid-keying leaves the
loft's mid-face section rotated by half a segment and the mid-faces overlap.

**Define a slab's heel face precisely: the face whose centroid has the GREATEST
`distAlong(face.centroid)`, searched across ALL of the slab's faces with NO surface-type filter**;
its toe-side face is the least-centroid one. ⚠️ Do **not** restrict the search to
`PlaneSurfaceType` or any other surface type: a sliced slab is bounded by a mix of the two planar
cut faces and ruled side faces, and a type filter can pick the wrong face or miss the cut face,
which makes the step-18 loft fail with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. The same
all-faces-by-centroid rule — max for the heel, min for the toe — is used everywhere a slab end face
is needed: here, in step 17 and in step 18.

Apply the rotation as a free move by `Matrix3D.setToRotation(ang, axisDir, apexWorld)` through
`moveFeatures.createInput2(bodyCollection)` then `defineAsFreeMove(matrix)` then `add(input)`
(`[PB-MOVE-ROTATE]`).

<!-- check-compile: ignore atan2 acos -->

<!-- proof-run: proofkit3d.RunSolidParallel(bvSpiralSolidCases, stepTwistSpiralSlabs, assertTwistSpiralSlabs) -->

`stepTwistSpiralSlabs` proves this step by PLACING each slab at the rotation the law gives and
measuring the azimuth it actually landed at against the same slab unrotated. It also asserts that
the span is positive, that the per-segment share is linear in the heel-face cone distance, that the
toe-to-heel total matches, and that a ratio pair's two gears come out with different twists — the
whole content of the roll ratio.

**From:** `spec/bevelgear/instructions.md` L656–L669 L818–L820
`spec/bevelgear/spiral-tooth-trace.md` L186–L214
`.claude/skills/generate-gear/PLAYBOOK.md` L430–L431 L791–L800

## 17 `[GO]` Crown the slabs lengthwise

**H.** Crown the tooth by scaling each segment **except the outermost, heel one** down by a
**monotonic** factor: full at the heel and growing smoothly toward the toe. For each segment compute
its heel-distance fraction `u = (R_heel - R_heelFace) / span`, where `R_heelFace` is found by the
step-16 all-faces-by-centroid rule but **RECOMPUTED here, AFTER the twist has moved the slabs** —
do not reuse pre-twist values — and `R_heel` and `span` are from step 14. `u` runs 0 at the held-full
heel to 1 at the toe. The **outermost segment is the one with the GREATEST post-twist heel-face
`distAlong`**: sort by that and skip the last. Then

```
factor = 1 - _CROWN_PER_RAD * (abs(total) / 2) * u
```

`total` is the full toe-to-heel twist from step 16, and `abs(total) / 2` is the per-end peak twist
magnitude, so the maximum relief — now at the toe — keeps the magnitude the old per-end peak had,
just relocated. `_CROWN_PER_RAD` is a tunable class constant with the value **0.5**; 0 disables the
crown, so set it to 0.5 and do not leave it unset. If a computed `factor` comes out at or below 0,
`raise` a self-diagnosing error naming the gear, the segment's `u` and the factor; never scale by a
non-positive factor.

⚠️ Do **NOT** key the relief on `abs(ang)`, the twist magnitude. That is symmetric about the
mid-face and maximal at BOTH ends, so with the heel slab held full the slab just inside the heel
becomes the most relieved one and dips below both its neighbours, a notch that reverses the
heel-to-toe taper. Measured: the heel-adjacent slab came out at 0.932 while the next slab inward was
0.972 and therefore taller. Key on the monotonic heel distance `u`.

Three further rules:

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). `scaleFeatures` is the ONE exception to
   never-activate: it needs the Design occurrence as the active edit target, so call
   `designOccurrence.activate()` — a method on the `Occurrence` — before the crown scales and
   restore afterwards in a `finally` with `design.activateRootComponent()`. ⚠️ Do **not** write
   `design.rootComponent.activate()` or `someComponent.activate()`: a `Component` has no `.activate`
   and raises `AttributeError`. Build the feature with
   `scaleFeatures.createInput(inputEntities, point, scaleFactor)` then `add(input)`.
2. **Skip the outermost, heel segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone of step 19 trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, not its centroid.** `scaleFeatures` shrinks
   uniformly toward its base point, so a base point at the heel face's centroid — mid tooth height —
   pulls the tooth's ROOT edge upward by `(1 - factor) * (half the tooth height)`, the tooth stops
   seating on the gear body's root cone, and the Combine-Join leaves a visible gap, clearly so for
   ratio pairs. Put the base point on the root instead: of the heel face's vertices
   (`heelFace.vertices`, each `.geometry` a world `Point3D`), take the **two with the smallest
   perpendicular distance to the shaft axis** — the line through `apexWorld` along `axisDir`, with
   perpendicular distance `|(p - apex) - ((p - apex) . axisDir) * axisDir|` — because those are the
   two ROOT corners and the tip corners are the farthest from the axis, and place the base sketch
   point at their MIDPOINT, mapped into the heel-face sketch with `sketch.modelToSketchSpace`. The
   heel face is a planar cut, so that midpoint lies on it. A uniform scale about a point keeps every
   line through that point invariant, so anchoring on the root keeps the root edge on the seating
   cone while the tip is relieved progressively toward the toe. Finding the heel face itself is
   unchanged — still the max-`distAlong`-centroid face of step 16; only the point ON it changes.

<!-- check-step-calls: ignore rootComponent.activate -->
<!-- check-compile: ignore _CROWN_PER_RAD designOccurrence abs sort -->

<!-- proof-run: proofkit3d.RunSolidParallel(bvSpiralSolidCases, stepCrownSpiralSlabs, assertCrownSpiralSlabs) -->

`stepCrownSpiralSlabs` proves this step by drawing each slab at the relieved section the scale would
leave, scaled about the ROOT rather than the centroid, which the proof file states as the
substitution for a scale feature this harness does not have. It asserts that every factor is
positive, that the relief shrinks monotonically from toe to held-full heel with no notch, that the
outermost segment's computed factor really is below 1 so holding it full is load-bearing, that
keying on the twist magnitude instead WOULD notch the heel-adjacent slab, and that each relieved
slab's root stays exactly on the seating cone.

**From:** `spec/bevelgear/instructions.md` L671–L683
`.claude/skills/generate-gear/PLAYBOOK.md` L576–L581 L782–L790

## 18 `[GO]` Loft the twisted crowned slabs into the curved tooth

**I.** ⚠️ **Re-sort the segments by their heel-face cone distance HERE, after the twist and the
crown** — do not reuse the pre-twist slice or centroid order of step 15. The twist rotates each slab
about the shaft axis, and for high-twist unequal-ratio pairs that rotation changes the slabs'
along-cone order enough to reorder adjacent slabs; lofting in the stale order assembles the
cross-sections out of sequence and the crowned tooth comes out distorted, so the two gears
interfere. For equal or low-twist pairs the two orders coincide, which is why equal-teeth gears mesh
even with the stale order while ratio pairs distort — the single thing that makes a pair like 31/17
fail while 31/31 looks fine.

So compute `order = sorted(segment indices, key = distAlong(slabHeelFace(seg).centroid))` NOW, and
loft a NewBody through, in that order: first the **toe-most segment's apex-side face** — the toe
segment is `order[0]`, and its toe face goes in first so the loft pushes past the toe cone and the
toe trim bites — then the **heel-facing face of every segment, iterated in `order`**, each being
that segment's farthest-along-the-element face by post-twist centroid, the last of which reaches
past the heel cone. Build it with `loftFeatures.createInput(operation)`, `loftSections.add(face)`
per section in that order, and `add(input)`. Name the resulting body **`{gearLabel} Spiral Tooth`**.
Then remove the segment scaffolding with `removeFeatures.add(...)`, since the loft has captured
their faces.

<!-- check-compile: ignore sorted slabHeelFace -->

<!-- proof-run: proofkit3d.RunSolidParallel(bvSpiralSolidCases, stepLoftSpiralTooth, assertLoftSpiralTooth) -->

`stepLoftSpiralTooth` proves this step. decad lofts two sections at a time, so the curved tooth is
built as the chain of pieces between consecutive faces rather than as one feature through all of
them, and the proof file records that the single lofted body is what the substitution costs. It
asserts that the post-twist heel-face cone distances are strictly increasing so the recomputed order
is well defined, that the toe-most segment is first, that the chain starts past the toe corner and
reaches the heel corner, and that the crowned tooth is lighter than the uncrowned one but not by
more than half, which a relief and not a collapse is.

**From:** `spec/bevelgear/instructions.md` L685 L818–L820
`.claude/skills/generate-gear/PLAYBOOK.md` L715–L719 L761–L765

## 19 `[GO]` Trim the tooth flush with the two conical cuts

Both branches of the tooth-body hook end here. At Mean Spiral Angle 0 the hook's first line returns
this directly on the straight lofted tooth; above 0, step 18's `{gearLabel} Spiral Tooth` is what
is trimmed. Either way the call is the framework's:

```python
cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)
```

Do **not** re-implement the cut machinery. **Two distinct bodies are involved and must not be
conflated:** the cutting TOOLS are `ConeSurfaceType` faces of the **Gear Body**, the
revolved-hexagon frustum of step 12 — the lofted Tooth Body has no cone faces, so searching IT finds
none — and the TARGET being split is the **Tooth Body**.

The helper implements the pinned behaviour: the **toe cut first**, its cone face identified by the
toe edge's world **MIDPOINT** best-first across the frustum's cone faces
(`[PB-FACE-BY-MIDPOINT]` — endpoints sit near the apex singularity, where
`getParameterAtPoint` returns no result), each candidate tried as the actual split tool through
`splitBodyFeatures.createInput(target, face, True)` and `add(input)` (`[PB-SPLIT-BODY]`) with the
first that splits into more than one piece kept; **keeper selection after each cut**, removing
apex-containing pieces and keeping the largest (`[PB-REMOVE-PIECES]`); then the **heel cut on the
keeper alone**, which is what makes it deterministically two split features for every gear ratio.
A heel cone that does not intersect the keeper at all — common on ratio pairs such as Module 1 with
Driving 31 and Pinion 43, where the heel cone never overshoots the tooth — is raised by the helper
as the typed `solids.NonIntersectError` and caught, and the keeper is returned whole. Every failure
is self-diagnosing with the per-face distance and error history (`[PB-SELF-DIAGNOSING]`), and each
cut's outcome is logged with `force_console=True`.

**The caller's obligations stay in the generator.** Pass `toeMid` as the toe edge's world midpoint,
`(M_world + N_world) / 2` for the pinion and `(O_world + P_world) / 2` for the driving gear;
`heelMid` as the heel edge's world midpoint, `(C_world + H_world) / 2` and
`(D_world + J_world) / 2` — the same edge-midpoint pairs as step 14's hand-off; `apexWorld` as the
§2 Apex sketch point's world geometry; and `gearBody` as the revolved frustum, the cone-face source.
The toe cut must split, and its failure propagates and crashes the build, which is correct because
an uncut tooth is unusable; only the heel cut is lenient, and only through the typed
`NonIntersectError`.

`apply_conical_cut`, `select_keeper`, `find_cone_faces_by_midpoint` and `surface_distance` are the
helper's own internals rather than calls this module makes.

<!-- check-step-calls: ignore apply_conical_cut select_keeper find_cone_faces_by_midpoint surface_distance getParameterAtPoint -->
<!-- check-compile: ignore cut_conical_ends apply_conical_cut select_keeper find_cone_faces_by_midpoint surface_distance NonIntersectError force_console -->

<!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepCutConicalEnds, assertCutConicalEnds) -->

`stepCutConicalEnds` performs NEITHER cut, and the proof file says why: both operands are Lofts
here, the tooth and each cone alike. It builds the tooth and the two cones and lays them apart, then
reads each cone's two ring radii and stations back out of the body's own bounds, volume and
centroid, reads the tooth's root and tip surfaces off the tooth, solves the stations where they
cross from those readings and checks them against the flush band: the toe cut lands on the tooth's
root exactly at the toe corner and the heel cut exactly at the dedendum corner. The cost is the
split — no division of the tooth, no keeper selection, no watertight result — but what it does show
is that each cut lands where the flush band requires and that the two ends land on DIFFERENT
surfaces of the tooth, which a planar cut could not do and which is the observable signature of a
conical cut face.

**From:** `spec/bevelgear/instructions.md` L339 L595–L597 L687 L713–L739 L794–L801
`.claude/skills/generate-gear/PLAYBOOK.md` L160–L173 L724–L765

## 20 `[GO]` Circular-pattern the tooth

Circular-pattern the remaining tooth piece around the **shaft-axis edge**, the same in-sketch
profile edge the revolve used and not the §2 construction line.
`circularPatternFeatures.createInput(bodyCollection, shaftAxisEdge)`, then pin all three inputs
explicitly (`[PB-CIRCULAR-PATTERN]`): `quantity = ValueInput.createByReal(<this gear's Teeth
Number>)`, `totalAngle = ValueInput.createByString('360 deg')`, `isSymmetric = False`. Then
`add(input)`.

Although the pitch diameter shrinks from heel toward apex, the ANGULAR spacing about the shaft axis
stays constant at `360 deg / N` for the entire face width: the radial taper is already produced by
the loft from the Apex to the heel-end tooth profile, so the pattern just rotates that one tapered
tooth into N evenly spaced copies.

The pattern's `bodies` collection already includes the seed body plus the copies, so do not re-add
the seed in step 21 (`[PB-PATTERN-BODIES]`); copy them into a fresh
`adsk.core.ObjectCollection.create()` first, because `pattern.bodies` is a `BRepBodies` and the
combine input rejects it.

<!-- proof-run: proofkit3d.RunSolid(bvSolidCases, stepCircularPattern, assertCircularPattern) -->

`stepCircularPattern` is the ONE bevel step that stays on the serial runner. The pattern increment
retires the seed tooth, so the seed cannot be measured after the step runs: its azimuth, radius,
height and volume are read during the build and handed to the assertion through package-level
variables, and two cases running at once would overwrite each other's readings and the proof would
report a wrong verdict rather than failing loudly. The proof file records that beside those
variables. The assertion checks the seed's own reach against the tooth's tip radius at the heel, and
then a sample of the copies — the second, the one opposite the seed and the last — each in a
document of its own, because decad verifies every PAIR of live bodies and for the tooth pairs of a
real gear the disjoint-or-overlap partition resolves neither way.

**From:** `spec/bevelgear/instructions.md` L741 L838–L862
`.claude/skills/generate-gear/PLAYBOOK.md` L684–L694

## 21 `[GO]` Combine-Join the teeth into the Gear Body

Join all the patterned tooth pieces with the Gear Body in a SINGLE Combine-Join: the Gear Body as
the target and the patterned tooth bodies as the tools.
`combineFeatures.createInput(gearBody, toolCollection)` with the operation set to
`adsk.fusion.FeatureOperations.JoinFeatureOperation`, then `add(input)`.

<!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepCombineJoin, assertCombineJoin) -->

`stepCombineJoin` performs no join: both operands are Lofts. It lays them apart and asserts the
join's two consequences from their own measured geometry — that a join leaves ONE lump where the
tooth's root is at or below the body's root cone, seated rather than floating, and that the joined
body reaches further out than the frustum where the tooth's tip stands proud of it — taken at the
toe, the middle and the heel of the band the join would cover. ⚠️ The proof sinks the tooth's root a
twentieth of the tooth height below the gear body's root cone, which is what makes "seated"
measurable as a strict inequality. **The generated module seats the tooth exactly on the cone and
must not sink it**: the sink belongs to the proof alone, and the proof also asserts the unsunk root
sits exactly on the cone. The cost is the stitch — the proof cannot show the evaluator making one
boundary out of two.

**From:** `spec/bevelgear/instructions.md` L743 L802–L810
`.claude/skills/generate-gear/PLAYBOOK.md` L684–L688

## 22 `[GO]` Draw the Bore sketch

Skip this step and step 23 entirely when Enable Bore is unchecked.

The bore diameter is this gear's Bore Diameter if non-zero, otherwise this gear's
`Pitch Diameter / 4`.

Build the bore plane normal to the shaft at its START:
`constructionPlanes.createInput()` then `setByDistanceOnPath(<shaft-axis edge>, ValueInput.createByReal(0.0))`
— pass the in-sketch profile edge, not the §2 construction line, and pass it directly rather than
through `Path.create` (`[PB-CONSTRUCTION-PLANES]`).

In a sketch named `{gearLabel} Bore`, sketch the bore circle centred at the sketch ORIGIN, since the
plane is rooted at the shaft start and the origin is therefore on the axis:
`sketchCurves.sketchCircles.addByCenterRadius`. **Fix the circle's centre** with
`circle.centerSketchPoint.isFixed = True` and add a diameter dimension of the bore diameter with
`sketchDimensions.addDiameterDimension` (`[PB-CIRCLE-CENTER]`). A circle created at (0,0,0) does NOT
reuse the sketch's own origin point — its centre is a free point that happens to sit there — and
making it coincident to `sketch.originPoint` instead has been observed to throw
`VCS_SKETCH_SOLVING_FAILED` on a plane of exactly this kind. Gate the sketch:
`if not sketch.isFullyConstrained: raise ...` (`[BEVEL-F-FULL-CONSTRAINT]`).

<!-- check-step-calls: ignore originPoint -->

<!-- proof-run: proofkit.RunParallel(bvBoreCases, stepBoreSketch) -->

`stepBoreSketch` proves this step on both gears, on both sides of the Enable Bore branch and on both
sides of the per-gear auto branch: exactly one region, of the bore disc's own area and usable as an
extrude profile, with the diameter below the gear's pitch diameter. The unchecked case records that
no sketch is drawn at all.

**From:** `spec/bevelgear/instructions.md` L745
`.claude/skills/generate-gear/PLAYBOOK.md` L442–L448 L766–L777

## 23 `[GO]` Cut the bore through the Gear Body

Extrude-cut the bore circle as a SYMMETRIC through-cut restricted to this Gear Body.
`extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`, then
`extentInput.setSymmetricExtent(ValueInput.createByReal(<2 * Cone Distance>), False)` — the second
argument `isFullLength=False` means the distance is the half-length PER SIDE, and there is no third
taper argument (`[PB-THROUGH-CUT]`) — then `participantBodies = [thisGearBody]`, then `add(input)`.
Twice the Cone Distance is generously past any face width.

<!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepBoreCut, assertBoreCut) -->

`stepBoreCut` builds the tool as a REAL extrude, which a symmetric extent produces as a prism, but
performs no cut: the target is the frustum, whose bands are Lofts. It lays the tool and a band apart
and asserts the cut from the tool's own measured geometry — its diameter, that its two ends sit
exactly `2 * Cone Distance` either side of the shaft edge's start, and that both clear the frustum,
which is what makes it a THROUGH cut — and computes the material it would remove from the frustum's
own profile clipped to the bore radius. The unchecked case returns the body untouched. The cost is
the pierced body: one lump with a hole and no enclosed void is not shown.

**From:** `spec/bevelgear/instructions.md` L745 L811–L817
`.claude/skills/generate-gear/PLAYBOOK.md` L720–L723

## 24 `[GO]` Rotate the driving gear into mesh

Do this here, in the Design component, BEFORE the body is moved out. Rotate the DRIVING body by
`180 deg / Driving Gear Teeth Number`, half a tooth pitch, about its own shaft axis, with the
framework helper `rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)`, which
takes the rotation axis and origin from the B->I profile edge's WORLD endpoints and applies a
free-move matrix (`[PB-MOVE-ROTATE]`). A driving valley then sits where the pinion tooth crosses the
axial plane, giving the interlocked meshing look: both gears are patterned from a starting tooth in
the axial plane, so without the offset a driving tooth and a pinion tooth would both sit at that
crossing and visually collide.

This runs in Design before `moveToComponent` because a construction axis cannot be added in the
moved-out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation has to use the edge's
world geometry while the body is still here.

The PINION additionally gets `_pinionMeshPhase(pinionTeeth)`, its extra rotation about its own shaft
axis in radians, `_PINION_MESH_PHASE_TEETH * 2 * math.pi / pinionTeeth`. `_PINION_MESH_PHASE_TEETH`
is 0, so the pinion's phase is zero and no move is made: a zero angle builds the identity matrix and
Fusion refuses to move a body by it with `RuntimeError: 3 : invalid transform`, which
`rotate_body_about_edge` absorbs by returning early rather than each call site guarding it.

<!-- check-compile: ignore rotate_body_about_edge _pinionMeshPhase _PINION_MESH_PHASE_TEETH math.pi -->

<!-- proof-run: proofkit3d.RunSolidParallel(bvSolidCases, stepMeshRotation, assertMeshRotation) -->

`stepMeshRotation` rotates the TOOTH rather than the joined gear body, and the proof file says why
it has to: the frustum alone is a solid of revolution about the very axis the rotation turns about,
so no reading off it could tell a rotated body from an unrotated one, and the joined body is what
step 21 cannot build. It asserts the azimuth moved by exactly half a tooth pitch on the driving
gear, that the volume did not change, that the body stayed on the shaft axis, and that the pinion's
phase really is zero so no move is made at all.

**From:** `spec/bevelgear/instructions.md` L345–L357 L747
`.claude/skills/generate-gear/PLAYBOOK.md` L791–L800

## 25 `[PROSE]` Move the finished bodies into the gear component

Relocate this gear's finished body from Design into the `{gearLabel} Gear` component of step 10 with
`body.moveToComponent(targetOccurrence)`, which preserves the world position and needs no activation
(`[PB-NO-CROSS-SIBLING]`).

**From:** `spec/bevelgear/instructions.md` L703 L747
`.claude/skills/generate-gear/PLAYBOOK.md` L820–L825

## 26 `[PROSE]` Hide the construction geometry

Call the framework's `hide_construction_geometry(bevelComponent)` from `.solids`. It recursively
walks the Bevel Gear component tree, deduping by `entityToken`, and hides every sketch, construction
plane and construction axis with `isLightBulbOn = False` — construction planes and axes are **not**
hidden by `isVisible` (`[PB-HIDE-AFTER-USE]`, `[PB-TREE-CLEANUP]`, `[BEVEL-F-CLEANUP]`). Do not
re-implement the walk. There is no sketch-only mode and no per-mode guard: bevel always builds
solids. Leave only the two finished gear bodies visible.

The driving gear's meshing rotation is NOT a cleanup step; it happens in step 24, in the Design
component, before the body is moved out.

Nothing here calls `settle_sketch_display`: `commands/_gear_command.py` calls it once after
`generate()` returns, every gear command runs through that one call, and a generator must not add
its own (`[PB-SETTLE-DISPLAY]`).

<!-- check-step-calls: ignore settle_sketch_display isVisible -->
<!-- check-compile: ignore hide_construction_geometry settle_sketch_display _hideConstructionGeometry -->

**From:** `spec/bevelgear/instructions.md` L341 L751–L755
`spec/bevelgear/fusion.md` L161–L166
`.claude/skills/generate-gear/PLAYBOOK.md` L650–L662 L826–L828
