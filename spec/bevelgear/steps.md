# Bevel gear — compiled step list

The proof for this step list is `proof/bevelgear/geometry_test.go`, `proof/bevelgear/tables_test.go`,
`proof/bevelgear/sketches_test.go`, `proof/bevelgear/solids_test.go`, `proof/bevelgear/spiral_test.go`
and the generated `proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `72e2f747b6dbb6a173c3dea0007bc125600b50c8` |
| `spec/bevelgear/fusion.md` | `efa49ddcd40c9d796c687097ff9fa620c1bda325` |
| `spec/bevelgear/spiral-tooth-trace.md` | `84474c55a77775fba98991437ef77a0fa812bd12` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9ee2dcbaed7b5480aa69e9295e8b61acaea081f3` |

## S01 `[PROSE]` Configure the command dialog

Add the **20** dialog inputs to `cmd.commandInputs` from the configurator's `configure`
classmethod, in exactly the display order of the table below. Target Plane is added first so it
wins Fusion's auto-focus ([PB-AUTOFOCUS-FIRST]: Fusion focuses the FIRST
`SelectionCommandInput` and ignores a later
`hasFocus` flag), Center Point follows so the user flows from plane to point, and the pre-selected
Parent Component comes third.

`configure` and `handle_input_changed` are bound **by name** from `commands/bevelgear/entry.py`, so
both are part of the reproduced surface. The class and its three classmethods carry exactly these
signatures:

```python
class BevelGearCommandInputsConfigurator:
    @classmethod
    def configure(cls, cmd): ...

    @classmethod
    def handle_input_changed(cls, args): ...

    @classmethod
    def _updateSpiralInputVisibility(cls, inputs): ...
```

<!-- check-step-calls: ignore configure handle_input_changed _updateSpiralInputVisibility -->

The three names above are methods this module DEFINES for the framework to call, not calls it makes,
so they are mentions rather than requirements.

**The exact input table. Reproduce every string in it verbatim.** The id strings, the labels and the
three selection tooltips are contract surface; a plausible re-spelling of an id is a defect that
nothing downstream can catch.

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

**The 20 module-level id constants, in row order, holding those id strings and no others:**
`INPUT_ID_PLANE`, `INPUT_ID_CENTER_POINT`, `INPUT_ID_PARENT`, `INPUT_ID_MODULE`,
`INPUT_ID_SHAFT_ANGLE`, `INPUT_ID_DRIVING_TEETH`, `INPUT_ID_PINION_TEETH`,
`INPUT_ID_DRIVING_BASE_HEIGHT`, `INPUT_ID_PINION_BASE_HEIGHT`, `INPUT_ID_BORE_ENABLE`,
`INPUT_ID_DRIVING_BORE`, `INPUT_ID_PINION_BORE`, `INPUT_ID_FACE_WIDTH`, `INPUT_ID_TOOTH_SPACING`,
`INPUT_ID_SPIRAL_ANGLE`, `INPUT_ID_HAND`, `INPUT_ID_CUTTER_RADIUS`, `INPUT_ID_TOE_EXTENSION`,
`INPUT_ID_DRIVING_TOE_RADIUS`, `INPUT_ID_PINION_TOE_RADIUS`. Two further module constants and no
more: `_HAND_RIGHT = 'Right'` and `_HAND_LEFT = 'Left'`.

**Calls.** Each selection input is added with
`inputs.addSelectionInput(INPUT_ID_PLANE, 'Target Plane', 'Plane the bottom of the driving gear sits flush against')`.
Each one then takes its filters, one call per filter, with
`planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)`.
Each one then takes its limits with `planeInput.setSelectionLimits(1, 1)`, exactly one, per
[PB-SELECTION-DECL]. Write every filter as the named constant rather than a quoted literal
([PB-SELECTION-FILTER-ENUM]); the constant is checked for typos at import and survives a renamed
filter, where a quoted literal fails silently by selecting nothing. The Parent input additionally
pre-selects the root component with `parentInput.addSelection(get_design().rootComponent)`.

Numeric inputs use
`inputs.addValueInput(INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))`; the
checkbox uses `inputs.addBoolValueInput(INPUT_ID_BORE_ENABLE, 'Enable Bore', True, '', True)`; the
dropdown uses
`inputs.addDropDownCommandInput(INPUT_ID_HAND, 'Hand of Spiral', adsk.core.DropDownStyles.TextListDropDownStyle)`
followed by `handInput.listItems.add(_HAND_RIGHT, True)` and
`handInput.listItems.add(_HAND_LEFT, False)`.

Every length default goes through `to_cm(0)` before it reaches `createByReal`.

**Every `mm` and `deg` default passed as `createByReal` is in Fusion INTERNAL units**
([PB-DIALOG-DEFAULT-UNITS]): a 0 mm default is `adsk.core.ValueInput.createByReal(to_cm(0))`, and the
90° Shaft Angle default is `adsk.core.ValueInput.createByString('90 deg')` so the expression engine
parses it. The unit string on `addValueInput` controls display and parsing only.

**Conditional visibility — the spiral-only inputs show only when ψ > 0.** The private helper of the
three signatures above does the work. It evaluates the `spiralAngle`
input's **`.expression`** with
`get_design().unitsManager.evaluateExpression(spiral.expression, 'rad')` — internal **radians**, and
it does NOT read the input's `.value` — then sets `inputs.itemById(INPUT_ID_HAND).isVisible` and
`inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible` to `(value > 0)`. Guard it: if any of the three
inputs is `None`, return early, and wrap the evaluation in `try`/`except`, because a half-typed
expression can raise mid-edit; on failure leave both inputs **shown**. `isVisible` hides only the
dialog row — the input still exists and is read normally — so hiding is purely cosmetic and cannot
affect generation. Mean Spiral Angle itself is the controller and is **always visible**.

`configure` calls `cls._updateSpiralInputVisibility(inputs)` as its LAST step, so the initial state
is correct at the default ψ = 35° (both shown). The reactive update runs through the
`handle_input_changed` classmethod, whose whole body is
`cls._updateSpiralInputVisibility(args.inputs)` — recompute on **every** input change, with no branch
on which input changed.

**From:** `spec/bevelgear/instructions.md` L27–33, L168–248, L297–322

## S02 `[PROSE]` Read and validate every input

`BevelGearGenerator.generate(inputs)` reads **all** inputs first, through one `_readInputs(inputs)`
pass, before anything creates an occurrence. Bevel registers **no** Fusion user parameters, so
nothing creates an occurrence until every selection is already read and the
selection-context-shift hazard does not bite here — keep the order anyway.

<!-- check-step-calls: ignore generate deleteComponent get_value _readInputs -->

`generate` and `deleteComponent` are the two entry points `commands/bevelgear/entry.py` binds by
name, `_readInputs` is this step's own helper, and `get_value` is named below only to forbid it on
the boolean input; none of the four is a call this step requires.

**Reading.** Selections come from `get_selection(inputs, INPUT_ID_PLANE)`. Every numeric and angle
input is read by evaluating its expression:
`get_design().unitsManager.evaluateExpression(inputs.itemById(INPUT_ID_MODULE).expression, '')`, with
`''`, `'mm'` or `'deg'` as the table gives. The boolean is read with `get_boolean(inputs, INPUT_ID_BORE_ENABLE)`.
Never call `get_value` on it: that helper reaches for `.expression`, a member
`BoolValueCommandInput` does not have, and Fusion raises `AttributeError` at generation time
([PB-INPUT-READ]). The dropdown is read as
`inputs.itemById(INPUT_ID_HAND).selectedItem`, taking its `.name` and defaulting to `_HAND_RIGHT`
when there is no selection. From `base.py` import only `get_selection` and `get_boolean`
([PB-PRECOMPUTED-MODE], [PB-EVAL-EXPRESSION]).

**Units — the one that has made gears come out ten times too big.** Every `'mm'` and `'deg'`
expression comes back in Fusion internal units (cm, radians) regardless of the unit string, so use
them as-is and do **not** convert them again. **`Module` is read with unit `''`, so it comes back as
a raw number meaning MILLIMETRES.** Every length derived from Module must therefore be converted
with `to_cm(...)` before it touches geometry: the pitch diameters, the Cone Distance, the dedendum,
the §2 construction seed lengths, and the default Face Width. Mixing a raw-mm Module-derived length
with an already-cm `'mm'` input makes the gear come out about ten times off and the Face Width bound
meaningless. Both tooth counts are coerced with `int(round(value))` before validation.

**Validation, in this order.** The order is load-bearing.

1. `module > 0`; both teeth `>= 3`; non-negative base heights, bore diameters, Face Width, Tooth
   Spacing and Cutter Radius; Toe Extension in `[0, 100]`; Mean Spiral Angle converted to degrees
   and checked against `[0, 60)`.
2. Shaft Angle, converted to degrees, at least 30° and below the **Maximum Shaft Angle**. Both tooth
   counts must be read and coerced first, because the limit depends on them, and the computed limit
   goes in the rejection message.
3. The two pitch cone angles, then per gear the **Minimum Teeth** floor.
4. Per gear the **Minimum Base Height** and **Maximum Base Height**, applied to that gear's base
   height.

The Minimum Teeth check comes before the base-height step because it is exactly the statement that
the base-height window is non-empty, so step 4 never has to describe what to do when the minimum
exceeds the maximum.

**The closed forms, all in millimetres with γ in radians.** `Σ` is the Shaft Angle, `m` the Module,
`N_g` and `N_p` the driving and pinion tooth counts.

```
PPD = m * N_p                                   Pinion Gear Pitch Diameter
DPD = m * N_g                                   Driving Gear Pitch Diameter
Cone Distance   = sqrt(PPD**2 + DPD**2)         the DIAGONAL, never R
tan γ_p = sin Σ * PPD / (DPD + PPD * cos Σ)
γ_g     = Σ − γ_p
R       = (PPD / 2) / sin γ_p                   the PITCH CONE DISTANCE

Maximum Shaft Angle = min( acos(−min(PPD, DPD) / max(PPD, DPD)) , 150° )
    the acos half is EXCLUSIVE (it is a hard singularity), the 150° half INCLUSIVE

Minimum Teeth  : N >= 5.27 * cos γ              per gear, with that gear's own γ
Minimum Base Height = 1.05 * 1.25 * m * sin γ
Maximum Base Height = 0.95 * (r − 1.25 * m * cos γ) * tan γ      r = that gear's pitch radius
```

**The Maximum Shaft Angle is a cone-angle singularity, not a flat ceiling.** A pitch cone angle
reaching 90° turns that gear's pitch cone inside out: `R * cos γ` — the along-shaft seed length §2
uses for Apex→A and Apex→B, and the denominator of the back-cone virtual radius — passes through
zero and changes sign, so the seed points backwards along the shaft and the virtual radius is
unbounded. A 31/17 pair gives `acos(−17/31) = 123.26°`; the flat 150° earlier revisions promised was
never reachable for it. Equal tooth counts give `acos(−1) = 180°`, which is no constraint, and the
150° cap is what binds there. **150° is a practical ceiling on the figure, not a measured one.**

**The base-height bounds are measured from Apex 2's plane, not from the dedendum point, and it is
easy to get wrong by one dedendum.** The base height is the offset between the A→Apex 2 drop and
G→H (and the B→Apex 2 drop and I→J). Walking out the dedendum line from Apex 2, the perpendicular
distance to the shaft axis falls at `cos γ` per unit while the along-shaft coordinate rises at
`sin γ`, so H reaches the axis when the base height reaches `r * tan γ` — that is the true crossing.
The bound above sits `1.25 * m * sin γ` BELOW it, because it starts from the dedendum point C rather
than the pitch point, and is therefore **deliberately conservative, not exact**: it refuses a band of
base heights that would in fact still build. Past the true crossing the hexagonal frustum profile
has crossed its own axis of revolution and the revolve fails with `ASM_WIRE_X_AXIS` ([PB-REVOLVE]),
and a heel edge that merely approaches the axis is already degenerate. If a future revision wants the
exact bound it is `0.95 * r * tan γ`; do not adopt it without re-running the low-tooth-count cases.

Worked case, Module 1, Driving 31, Pinion 31, Shaft Angle 30°: each γ is 15°, the bound is
`0.95 * (15.5 − 1.25 * cos 15°) * tan 15° = 3.638 mm`, the true crossing is
`15.5 * tan 15° = 4.153 mm`, and the driving fallback resolves to `3.875 mm`. **The default therefore
sits between the two: it is capped to 3.638 mm, but it would not have folded uncapped.**

**The Minimum Base Height is the other end of the same heel edge, and the reason a low tooth count
used to fail.** The offset closes H at `|C→H| = base height / sin γ − 1.25 * m` beyond C, so unless
the base height carries H past the dedendum's own along-shaft projection of `1.25 * m * sin γ`, H
lands BEHIND C and the edge C→H runs back inward instead of outward. The fallback
`m * N_g / 8` clears the raw projection exactly while `N_g > 10 * sin γ` — 7.07 at Shaft Angle 90°,
so an equal 8-tooth pair was the smallest that built and a 7-tooth pair failed. **Raising the
fallback to the Minimum Base Height is what lets small tooth counts build at all; do NOT instead
raise the teeth floor**, which would refuse gears that are perfectly buildable once the base height
is bounded.

**Minimum Teeth is where the two base-height bounds cross.** Below it no base height satisfies both
and the gear cannot be built at any setting. Requiring `Minimum Base Height <= Maximum Base Height`
and solving gives `N >= 5.27 * cos γ`; the constant is `2 * (1.05 * 1.25 / 0.95 + 1.25) = 5.2632`
rounded UP to 5.27 so the published floor stays at or above the exact crossing. Do not round it
down, and do not replace it with the exact value without re-running the low-tooth-count cases. At
Shaft Angle 90° the floor is 3.72, i.e. **4 teeth** — measured, an equal 4-tooth pair solves and a
3-tooth pair still fails on the heel edge. Keep the blanket `teeth >= 3` as the absolute floor and
apply this computed floor on top of it.

**Base-height resolution, per gear.** The driving fallback is `m * N_g / 8`; the pinion fallback is
the RESOLVED driving base height times `N_p / N_g`, meaning the value after the driving fallback and
after the driving Maximum Base Height capped it — never the raw driving input — and then the
pinion's OWN Maximum Base Height is applied to the result. The two gears have different pitch cone
angles whenever the tooth counts differ, so the driving cap does not imply the pinion's. A fallback
below the minimum is raised, a fallback above the maximum is capped, and a USER value outside either
end is rejected with a message naming the bound it broke.

**A configuration can satisfy every bound here and still be refused as near-singular, and that is
not a validation rule.** The sketch engine refuses a system whose conditioning falls below its
`4e-5` trust floor, and the §2 lattice approaches that floor at both ends of the Shaft Angle range.
**Do not write a Shaft Angle bound from a conditioning measurement.** Three independently written
lattices, each holding DOF 0 with nothing redundant and each asserting its solved cone angles against
the closed form to nine decimals, do not agree about which end is reachable: two refuse the default
pair at 30° (`2.83e-5` and `2.94e-5`), first clear at 35°, and pass 142° and 150°; the third passes
30° and refuses 142° and 150°. The remedy for a conditioning failure is to change how the lattice is
built, never to loosen the gate and never to narrow the advertised range on one net's evidence.

**The bore bound is NOT part of this pass.** Validate the two bore diameters here only as
non-negative numbers. Its heel term would resolve here, but its toe term needs the Root Length,
hence the resolved Face Width, hence solved §2 geometry, and the bound is the minimum of the two —
so the whole Maximum Bore Diameter resolves in S06.

**`_readInputs` returns a 7-tuple** `(parentComponent, targetPlane, centerPoint, module,
drivingTeeth, pinionTeeth, shaftAngle_deg)` and stashes the rest on `self`:
`self._drivingBaseHeight_cm`, `self._pinionBaseHeight_cm`, `self._boreEnable`,
`self._drivingBore_cm`, `self._pinionBore_cm`, `self._faceWidth_cm`, `self._toothSpacing_cm`,
`self._spiralAngle_rad`, `self._hand`, `self._cutterRadius_cm`, `self._toeExtension_pct`,
`self._drivingToeRadius_cm`, `self._pinionToeRadius_cm`. `self._toeExtension_pct` is the raw
unitless percentage the dialog returns, not a length, and needs no `to_cm`. `generate()` later
stashes `self._coneDistance_cm`, `self._gamma_p` and `self._gamma_g`.

**There is no `GenerationContext` here and no `base.Generator`.** Per-gear state travels in plain
per-gear dicts, shared anchors on `self`. That class-level shape IS the intended structure — do not
introduce a context class.

**From:** `spec/bevelgear/instructions.md` L35–167, L250–295, L324–349

## S03 `[PROSE]` Create the Bevel Gear and Design components

Create the **Bevel Gear** component as a child of the user's Parent Component with
`parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`, and name it by assigning
to `bevelOccurrence.component.name` the string `Bevel Gear`. Create the **Design** component the
same way as a child of Bevel Gear and name it `Design`. Design holds every sketch, construction
plane and construction axis the build uses; the two per-gear components are created later, in S10.

Stash `self.bevelOccurrence` for cleanup, and `self.designOccurrence`, `self.designComponent` and
`self.bevelComponent` for the inner tree.

**NEVER activate any occurrence** ([PB-NEVER-ACTIVATE], [BEVEL-F-NEVER-ACTIVATE]). The bevel reason
is specific: the Anchor Sketch is created on the user's **EXTERNAL**, root-owned target plane, and an
activated occurrence resolves that external plane in its own local frame, so the build collapses onto
world XY regardless of the real plane tilt. Every feature runs in the single Design component, so no
cross-sibling reference is ever needed ([PB-NO-CROSS-SIBLING]). The sole exception in the whole
module is the spiral crown's scale in S19, which names the call it needs there.

**From:** `spec/bevelgear/instructions.md` L19–24, L555–563; `spec/bevelgear/fusion.md` L199–212

## S04 `[GO]` Anchor Sketch

Create the sketch with `designComponent.sketches.add(targetPlane)` — **directly on the user-selected
target plane**, whether the selection is a `ConstructionPlane` or a `PlanarFace`. Do not re-derive or
offset it ([PB-USE-SELECTED-PLANE]); re-deriving collapses the gear onto XY. Name it `Anchor`.

Mark the centre by projecting the user-specified centre point into the sketch with
`sketch.project(centerPoint)`.

Draw the Anchor Line with
`sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(cx - 0.5, cy, 0), adsk.core.Point3D.create(cx + 0.5, cy, 0))`
— **seed its two endpoints at exactly ±0.5 cm from the projected centre** along the sketch-local X,
so the seeded length is 10 mm.

Apply **BOTH** `sketch.geometricConstraints.addCoincident(projectedCenter, anchorLine)` — the
"intersection", which pins the centre onto the line — **and**
`sketch.geometricConstraints.addMidPoint(projectedCenter, anchorLine)`, which makes the centre bisect
it. Use both, not the midpoint alone. Then add an aligned distance dimension
`sketch.sketchDimensions.addDistanceDimension(anchorLine.startSketchPoint, anchorLine.endSketchPoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
and **do NOT assign `.parameter.value`** — the dimension simply locks the seeded 10 mm, and the value
is arbitrary because this is only a reference line.

Finally pin its direction with `sketch.geometricConstraints.addHorizontal(anchorLine)` — sketch-local
per [PB-REFLINE-DIRECTION], which works on any tilted target plane where a world-axis lock would
mis-orient. The line's absolute direction is arbitrary, since §2 derives every direction relative to
it, but it must not be a free degree of freedom; with midpoint, length and Horizontal it has zero.

**Stash the projected-centre `SketchPoint`** on `self._anchorCenterPoint`, so §2 re-projects THIS
anchor-sketch point rather than the raw user-selected centre.

Gate the sketch at the end of this step: raise, naming `Anchor`, if `sketch.isFullyConstrained` is
false ([PB-FULL-CONSTRAINT], [BEVEL-F-FULL-CONSTRAINT]). A free DOF here is a generation defect, not
a warning.

**Proof.** `stepAnchorSketch` builds this sketch. The bench substitutes a REFERENCE point for the
projection, which is what the sketch engine's reference geometry is for. Two differences are recorded
at the proof: Fusion's `project` does not fix what it brings in ([PB-PROJECT-NOT-FIXED]) while the
bench point is externally locked; and Fusion's two calls, coincident plus midpoint, are one call
there, because the engine's midpoint already carries the point-on-line row and adding a second
statement of it comes back redundant. The 10 mm dimension is written as the engine's SIGNED
horizontal distance rather than an unsigned aligned one, because the unsigned form leaves the line
free to solve end-for-end and the ambiguity probe reports that pair; nothing downstream depends on
the direction, so signing it changes no geometry.

<!-- proof-run: proofkit.RunParallel(bevelAnchorCases, stepAnchorSketch) -->

**From:** `spec/bevelgear/instructions.md` L565–569; `spec/bevelgear/fusion.md` L19–38

## S05 `[PROSE]` Gear Profiles Plane

Create the plane with
`designComponent.constructionPlanes.createInput()` then
`planeInput.setByAngle(anchorLine, adsk.core.ValueInput.createByString('90 deg'), targetPlane)` and
`designComponent.constructionPlanes.add(planeInput)`. Name it `Gear Profiles Plane` and stash it as
`self._gearProfilesPlane`.

The plane includes the Anchor Line and stands at 90° to it: by default it would lie flush to the
anchor line's own plane, and this build needs it perpendicular. **Build it off the ORIGINAL
`targetPlane` as the reference** ([PB-USE-SELECTED-PLANE]) — this is the other place the target-plane
orientation reaches the bodies, and substituting a different plane here also collapses the gear onto
world XY.

Pass the `SketchLine` **directly** to `setByAngle`; never wrap it in `Path.create` first
([PB-CONSTRUCTION-PLANES]), which raises an internal validation error whenever the curve's owner
sketch is not trivially resolvable in a multi-component context.

**From:** `spec/bevelgear/instructions.md` L573

## S06 `[GO]` Gear Profiles sketch — the §2 lattice

Create the sketch with `designComponent.sketches.add(self._gearProfilesPlane)`, name it
`Gear Profiles`, and stash it as `self._gpSketch`. Everything below happens inside this one sketch,
which is therefore one timeline entry.

### Two blanket rules for every line and every dimension here

**Every line drawn in this sketch is a construction line** — set `line.isConstruction` to `True` on
the lattice lines, the toe lines M→N and O→P, and the short reference and connector lines
(M→C, N→A′, O→D, P→B′, A′→G, B′→I, C→K or C→K′, D→L or D→L′) alike. The solid features later consume
only the per-gear Profile sketches, never a §2 curve.

**Every length dimension here is `AlignedDimensionOrientation`.**
`sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
takes the orientation, and this figure has no axis-aligned line in it: the shaft axes sit at the
Shaft Angle to each other, the whole lattice tilts with the target plane, and the sketch is not
world-aligned. `HorizontalDimensionOrientation` or `VerticalDimensionOrientation` would each
dimension the line's PROJECTION onto a sketch axis instead of its length. Wherever a value below is
called a length dimension — the PPD/2 and DPD/2 drops, the two `1.25 * Module` dedendum lines, the
Tooth Spacing dimension on K′/L′, and the Toe Radius dimension on N→A′ and P→B′ — it means an
aligned distance dimension of that value. The offset dimensions are the different call
`sketch.sketchDimensions.addOffsetDimension(line, entityTwo, textPoint)`, which takes no orientation.

**Build every line the COINCIDENT way** ([BEVEL-F-COINCIDENT-STYLE], a stricter delta to
[PB-SHARE-XOR-COINCIDENT], which allows either style where §2 allows only one). When a line must
start at or connect to an already-existing point, create it from raw `Point3D` coordinates for BOTH
ends and pin each connecting endpoint with exactly one
`sketch.geometricConstraints.addCoincident(line.startSketchPoint, existingPoint)` — never pass the
existing `SketchPoint` into `addByTwoPoints` to share it. It is load-bearing both ways: sharing
without a coincident leaves the sketch under-constrained and the gate fails on `Gear Profiles`;
sharing AND coinciding is redundant and the solve fails outright with
`VCS_SKETCH_SOLVING_FAILED - failed to create offset`. **This covers the short reference and
connector lines too, the ones whose both endpoints already exist** — a regen that shared only those
came out about fourteen coincidents short and the gate failed. No §2 line is exempt.

**Each named line is created ONCE and later references reuse that object** ([BEVEL-F-LINE-ONCE]).
The extension lines A→E, B→F, E→G, F→I and the dedendum and closing lines C→H, D→J, G→H, I→J are
named construction lines: when a step below says "collinear to line A→E", it means the very line
drawn earlier, so the helper that creates an extension must RETURN it and you keep the reference.
Drawing a second line between the same two points to obtain a reference over-determines the coupled
net and the solve fails with `VCS_SKETCH_OVER_CONSTRAINTS`. A duplicate whose endpoints carry only
per-end coincidents has been observed to solve and even pass the gate, so do not rely on the solver
to catch one for you.

**The driven lengths are NOT dimensioned** ([BEVEL-F-DRIVEN-DIMS], [PB-NO-OVERCONSTRAIN]). The
along-shaft lengths Apex→A and Apex→B and the extension lines are driven by the closing, collinear
and perpendicular constraints. Undimensioned does not mean unpinned: each has a closed-form seed
below, and the seed is what picks the figure.

**Every seed below is load-bearing geometry, never a convergence hint** ([BEVEL-F-MIRROR-FIGURE]).
§2 holds **15** constraint sites whose side is decided by the seed alone, and Fusion offers no
constraint that pins any of them: every geometric constraint it has is unsigned or undirected,
`addAngularDimension` takes an unsigned value plus a text point for the quadrant,
`addOffsetDimension` is unsigned, and `addDistanceDimension` is a magnitude whose side comes from the
seed. On a 43/31 pair at Shaft Angle 75° with Tooth Spacing above zero there are 13 independent
binary choices, so **8192** distinct figures satisfy every constraint; for the default 31/31 pair at
90° with Tooth Spacing 0 it is 10 choices and 1024 figures, because at 90° the shaft-angle
dimension's two senses are the same line and the two Tooth Spacing sites do not exist. A flipped seed
solves cleanly and the wrong gear gets built rather than refused.

**Never add a Fusion constraint to pin a side.** Two mechanisms fix a side at all and both are
refused, recorded here so the question is not re-opened: `addSymmetry` on C and D against the Pitch
Line rules out the collapse but not the swap, and it replaces a dedendum's perpendicular plus length,
which is a net redesign the [PB-SKETCH-FIRST] waiver forbids a regen from making; and
`SketchPoint.isFixed` over-constrains, because the closure already determines every core point, and
it turns the parametric lattice into placed geometry.

<!-- check-step-calls: ignore addSymmetry project2 -->

`addSymmetry` and `SketchPoint.isFixed` are named only to record that they are refused, and
`project2` only to explain why `project` is kept. `addVertical` is named below only where it is
forbidden, and `addCollinear` is both required — on the four extension chains — and forbidden at K
and K′, so it stays a requirement.

### The closed forms this section is built from

Work in the sketch's own 2-D frame. Let `c` be the projected centre, `d` the projected anchor line's
unit direction, and `perp = (-d.y, d.x)` with its SIGN chosen so it points toward the target-plane
normal ([BEVEL-F-GROW-SIDE], [BEVEL-F-APEX-LOCAL]). Read that normal as `targetPlane.geometry.normal`
for BOTH selection kinds — a `BRepFace`'s `geometry` and a `ConstructionPlane`'s `geometry` are each
a `core.Plane` carrying `.normal`. That one-bit direction comparison is the ONLY permitted world use
in this section; **never compute a §2 POSITION from a world round-trip**, which is the single biggest
source of orientation bugs and is what collapsed the gear onto world XY.

```
m   = Module                         ded = 1.25 * m
r_p = PPD / 2                        r_g = DPD / 2
R   = (PPD / 2) / sin γ_p            |Apex→Ded| = sqrt(R**2 + ded**2)
bh_p, bh_g   = the RESOLVED pinion and driving base heights from S02
uB  = −perp                          the unit Apex→B direction
uA  = drivingDir rotated about the Apex by the Shaft Angle, the candidate with the greater X
uP  = the unit Apex2→C direction     uG = −uP

Apex   = c + perp * (R * cos γ_g + bh_g)
B      = Apex + uB * (R * cos γ_g)
A      = Apex + uA * (R * cos γ_p)
Apex 2 = Apex + pitchDir * R,  pitchDir the unit Apex→Apex2 direction
C      = Apex 2 + uP * ded           D = Apex 2 + uG * ded
E      = A + uA * (ded * sin γ_p)    F = B + uB * (ded * sin γ_g)
G      = A + uA * bh_p               I = B + uB * bh_g
H      = Apex 2 + uP * (bh_p / sin γ_p)
J      = Apex 2 + uG * (bh_g / sin γ_g)
K      = Apex 2 + uP * (r_p / cos γ_p)        L = Apex 2 + uG * (r_g / cos γ_g)
K′     = Apex 2 + uP * (r_p / cos γ_p + Tooth Spacing)
L′     = Apex 2 + uG * (r_g / cos γ_g + Tooth Spacing)
M      = Apex + unit(C − Apex) * (|Apex→Ded| − Root Length)
N      = M + uP * ((perpendicular distance from M to Apex→A − Pinion Toe Radius) / cos γ_p)
A′     = the foot of the perpendicular from N onto the Apex→A shaft axis
O      = Apex + unit(D − Apex) * (|Apex→Ded| − Root Length)
P      = O + uG * ((perpendicular distance from O to Apex→B − Driving Toe Radius) / cos γ_g)
B′     = the foot of the perpendicular from P onto the Apex→B shaft axis
```

**Read the two along-shaft seeds carefully, because swapping them is the easy mistake:
`|Apex→B| = R * cos γ_g` and `|Apex→A| = R * cos γ_p`** — each gear's own cone angle, with B on the
DRIVING side. Both cosines are positive for every Shaft Angle the range check
admits, which is what the Maximum Shaft Angle is there to guarantee.

### Building it

**Project the Anchor Sketch's centre point** — the one stashed in S04, NOT the raw user-selected
centre — with `sketch.project(self._anchorCenterPoint)`. Both happen to be coincident, but projecting
the anchor-sketch point keeps the chain inside the Design component; projecting the raw external
point is a cross-component reference and can resolve inconsistently.

**Write the call as `sketch.project(entity)` and do not substitute `project2`.** The compiled Fusion
API reference declares `project2(entities, isLinked)` and no `project`, so every gate in this repo
reports the call as unverified; that report is expected and is not a defect to fix here. `project` is
what the shipped add-ins call, it sits on the repo's reported-not-blocking list, and the two are not
interchangeable in any case, since `project2` takes a list and returns a list. Only a Fusion session
can settle whether `project` exists at runtime, and if it does not, the fix belongs in the spec.

**centre → Apex.** From the projected centre draw a construction line perpendicular to the projected
anchor line: `sketch.geometricConstraints.addPerpendicular(centerToApex, projectedAnchorLine)`. Its
far end is the **Apex**, seeded at `c + perp * (R * cos γ_g + bh_g)`. **Seed it at that distance and
NOT at the Driving Gear Pitch Diameter**, which is what earlier revisions said: the constraint net
closes this line at `R * cos γ_g` above point I plus the resolved driving base height, so for the
default 31/31 pair at Shaft Angle 90° the old seed sat 11.6 mm past where the solve puts it — 31 mm
seeded against 19.375 mm solved. Fusion converges from the far seed, so that was latent rather than
broken, but a seed that disagrees with its own closure by that margin is a seed waiting to pick the
wrong branch ([PB-SEED-NEAR]). Pin the start with exactly one
`sketch.geometricConstraints.addCoincident(centerToApex.startSketchPoint, projectedCenter)`. Do NOT
add a length constraint on this line. Stash `centerToApex.endSketchPoint` as
`self._apexSketchPoint`, and its 2-D position as `self._apex2d`.

**Apex → B, the Driving Gear Shaft Axis.** A construction line from the Apex pointing back toward the
anchor line, in the `−perp` direction. **Seed its far end at `Apex − perp * (R * cos γ_g)`, which is
`c + perp * bh_g`** — measure from the Apex, not from `c`. Earlier revisions said `c − perp * length`,
which puts B on the far side of the projected centre from the Apex, the wrong side of the figure
entirely; the closure at Apex 2 drives `|Apex→B|` to `R * cos γ_g`, so B solves to exactly one base
height above `c`. Pin its start with a coincident to the Apex, and make it parallel to the
centre→Apex line with `sketch.geometricConstraints.addParallel(drivingShaftAxis, centerToApex)`. **Do
NOT use `addVertical`**: that forces the line to the sketch's world-vertical, which is wrong on a
tilted target plane and over-constrains or mis-orients the figure. The far end is point **B**. Do not
dimension the length.

**Apex → A, the Pinion Gear Shaft Axis.** The driving-shaft direction rotated about the Apex by the
Shaft Angle. Rotating has two senses, one to each side, and they place A on opposite sides; choosing
wrong mirrors the whole gear onto the wrong side of the target plane. **Select the sense this way:
form BOTH candidate point-A positions — the driving direction rotated by +Shaft Angle and by
−Shaft Angle — and keep the candidate whose endpoint has the greater X in this sketch.** Compare the
two candidates' X and take the larger; do NOT rotate one fixed sense and flip it only when its X
comes out negative, because when both candidates have a positive X that shortcut keeps the wrong one,
which is exactly the side-flip to avoid. Call the chosen unit direction `pinionDir`.

Pin the start with a coincident to the Apex, and add
`sketch.sketchDimensions.addAngularDimension(drivingShaftAxis, pinionShaftAxis, textPoint)` equal to
the Shaft Angle. **Place the text point inside the Σ wedge so it measures Σ and not its supplement
180 − Σ** ([PB-ANGULAR-DIM]) — for example on the interior bisector of the two shaft directions,
with `drivingDir` the unit Apex→B direction:

```
textPoint = Apex + unit(pinionDir + drivingDir) * (PPD / 4)
```

The angular dimension fixes the angle MAGNITUDE only; it does not pin which side the pinion lies on,
and the text point does not prevent a frame flip. The far end is point **A**. Do not dimension the
length.

**A → Apex 2, the PPD/2 drop.** From A, a construction line perpendicular to the Pinion Gear Shaft
Axis, drawn toward the side where Apex 2 will lie. **Apex 2 sits in the interior wedge BETWEEN the
two shaft axes, so this drop points toward the OTHER — the Driving — shaft axis and point B, NOT
"toward the anchor line".** Pick the perpendicular sense by the sign of its dot product with the A→B
direction. Add `sketch.geometricConstraints.addPerpendicular(dropA, pinionShaftAxis)` and an aligned
length dimension of `PPD / 2`, which is the pinion's pitch radius at the heel and the perpendicular
distance from Apex 2 to the Pinion Gear Shaft Axis for any Shaft Angle. Pin its start with a
coincident to A.

**Naming convention, used throughout: "A→Apex 2" ALWAYS means this perpendicular drop line, never the
Apex→A shaft axis.** The two share point A but are different lines. The same holds for "B→Apex 2"
against the Apex→B shaft axis.

**B → Apex 2, the DPD/2 drop.** From B, perpendicular to the Driving Gear Shaft Axis, toward the
other — Pinion — shaft axis and point A; pick the sense by the sign of its dot product with the B→A
direction. **Do NOT choose this sense by a "toward the anchor line" reference**, i.e. the `−perp`
grow direction: the Driving Gear Shaft Axis is itself parallel to that direction, so the
perpendicular's dot with it is about zero — a degenerate test that silently selects an arbitrary and
usually wrong side. Both drops must aim at the SAME interior-wedge point: if this one seeds Apex 2 on
the wrong side of the driving shaft while the pinion's drop seeds it on the correct side, the
coincidence that closes them makes the solver **flip the whole figure to the mirror solution** — A,
C, D, G, H, K, M, N and A′ all land at negative X and the pair comes out mirrored about the driving
shaft axis. **An earlier revision said this flip is what collapses C onto D. That was wrong and it
misdirected the fix**: the C-onto-D collapse is its own pair of sites, the two Apex 2 → dedendum
perpendiculars below. Nothing in the build refuses the mirrored figure; the end-of-section seed gate
is what catches it. Add the perpendicular against the Driving Gear Shaft Axis, an aligned length
dimension of `DPD / 2`, and a coincident pinning the start to B.

**Close them.** `sketch.geometricConstraints.addCoincident(dropB.endSketchPoint, dropA.endSketchPoint)`.
Call that point **Apex 2**. At Shaft Angle 90° the four points Apex, A, Apex 2, B form a rectangle;
at other angles it is a non-rectangular quadrilateral, and the lengths of Apex→A and Apex→B adjust so
the two drops of length PPD/2 and DPD/2 coincide. The quadrilateral deliberately lies well above the
anchor line: the Apex's offset of `R * cos γ_g` plus the resolved driving base height keeps the whole
figure above that line across the supported Shaft Angle range.

**The Pitch Line.** A construction line from Apex to Apex 2, each end pinned with a coincident.

**The two dedendum lines.** From Apex 2, two construction lines each of length `1.25 * Module` and
each perpendicular to the Pitch Line, via
`sketch.geometricConstraints.addPerpendicular(pinionDedendum, pitchLine)` and an aligned length
dimension. The one drawn toward the anchor line is the **Driving Gear Dedendum**, ending at point
**D**; the one drawn away from it is the **Pinion Gear Dedendum**, ending at point **C**.

**Seed the two ends by dot product against the shaft axes, never by "towards or away from the anchor
line".** Let `u` be either unit perpendicular to the Pitch Line. The pinion dedendum direction is the
`u` with `u · unit(Apex→A) > 0`, and the driving direction is its negation, which satisfies
`(−u) · unit(Apex→B) > 0`. Those two dot products are exactly `sin γ_p` and `sin γ_g`, strictly
positive for every admitted configuration — unlike the anchor-line test, which reads about zero by
construction. Seed `C = Apex 2 + 1.25 * Module * pinionDedendumDirection` and
`D = Apex 2 + 1.25 * Module * drivingDedendumDirection`.

**These two sites are where the "C collapses onto D" symptom lives, and each is held by its seed
alone.** The perpendicular constrains the direction and the dimension the magnitude; neither picks a
side, so each end has two solutions and the solver takes the seeded one. Flip the pinion seed and C
solves exactly onto D; flip the driving seed and D solves onto C. The collapsed figure inverts that
gear — the toe ends up OUTSIDE the heel, the revolved frustum is degenerate, and the conical end-cut
finds no cone face at the toe midpoint, reporting `face dist = inf`.

**The two Root Axes.** Construction lines from the Apex to D and to C, coincident at both ends. These
are the Driving and Pinion Root Axes.

**E and F, the feet of the dedendum perpendiculars.** From A, a construction line collinear with
Apex→A, seeded at `E = A + unit(Apex→A) * (1.25 * Module * sin γ_p)`, with no dimensional constraint.
E is the foot of the perpendicular dropped from C onto the Pinion Gear Shaft Axis, which is what
`C→E ⊥ A→E` closes it on, so `|Apex→E| = R * cos γ_p + 1.25 * Module * sin γ_p`. Earlier revisions
seeded this line one Module long: the correct side but not the solved position, and a seed that is
not the solved position cannot be gated. Add
`sketch.geometricConstraints.addCollinear(lineAE, pinionShaftAxis)`, and pin A→E's start to the end
of Apex→A with a coincident. Then draw C→E, pin both its ends, and add
`sketch.geometricConstraints.addPerpendicular(lineCE, lineAE)`.

F is the driving twin: from B, `sketch.geometricConstraints.addCollinear(lineBF, drivingShaftAxis)`,
seeded at
`F = B + unit(Apex→B) * (1.25 * Module * sin γ_g)` with no dimension, closed by `D→F ⊥ B→F`, so
`|Apex→F| = R * cos γ_g + 1.25 * Module * sin γ_g`. Draw D→F, pin both ends, add the perpendicular
against B→F.

**G and H, the pinion heel.** From E, a construction line collinear to **line A→E** — the collinear
names A→E, **never the Apex→A shaft axis further up the chain**, even though both describe the same
infinite line ([PB-COLLINEAR-CHAIN], [BEVEL-F-COLLINEAR-CHAIN]); naming the axis raises
`VCS_SKETCH_OVER_CONSTRAINTS`. So the call is
`sketch.geometricConstraints.addCollinear(lineEG, lineAE)`. Seed its far end at `G = A + unit(Apex→A) * bh_p`, i.e.
`|E→G| = bh_p − 1.25 * Module * sin γ_p`, with no dimension. That length is strictly positive because
the Minimum Base Height keeps every resolved base height above `1.25 * Module * sin γ` with a 1.05
margin. Pin E to the new line's start. The far end is **G**.

From C, a line seeded at `H = Apex 2 + unit(Apex2→C) * (bh_p / sin γ_p)`, i.e.
`|C→H| = bh_p / sin γ_p − 1.25 * Module`, with no dimension, positive by the same Minimum Base
Height. Pin C to its start. Make C→H collinear with **line Apex2→C**, the Pinion Dedendum line C is
the endpoint of, through `sketch.geometricConstraints.addCollinear(lineCH, pinionDedendum)`. The far
end is **H**.

**These two seeds pick the side of the pinion base-height offset below, which is unsigned.** Flip
them and G sits one base height on the Apex side of A instead of beyond it, H follows, and the
pinion's heel end folds back inside the figure.

Connect G and H with a line, coincident at both ends, and **add
`sketch.geometricConstraints.addPerpendicular(lineEG, lineGH)`.**

**That perpendicular is required in Fusion and is deliberately absent from the proof, and the reason
is a difference between the two engines rather than a choice.** `addOffsetDimension` in Fusion is a
distance dimension whose documentation requires the second entity to be a line parallel to the
first, and it controls only the perpendicular distance — so the parallelism has to exist before the
base-height offset can be applied at all, and this perpendicular is what supplies it: E→G runs along
the pinion shaft, so making H→G perpendicular to it makes H→G parallel to the A→Apex 2 drop.
Perpendicular plus offset is two equations for two freedoms and nothing is redundant. The proof
harness's offset emits TWO residual rows, holding both endpoints of the target line at the same
signed perpendicular distance, so it carries the parallelism itself, and adding the perpendicular
there makes the lattice overconstrained at DOF 0 with the two base-height offsets named as the
redundant pair.

**I and J, the driving heel.** From F, a construction line collinear to **line B→F** — never the
Apex→B shaft axis — seeded at `I = B + unit(Apex→B) * bh_g`, i.e.
`|F→I| = bh_g − 1.25 * Module * sin γ_g`, with no dimension. Pin F to its start; the far end is **I**.
From D, a line seeded at `J = Apex 2 + unit(Apex2→D) * (bh_g / sin γ_g)`, i.e.
`|D→J| = bh_g / sin γ_g − 1.25 * Module`, with no dimension, collinear with **line Apex2→D**. Pin D to
its start; the far end is **J**.

**The driving pair's seeds carry the whole figure, because the closure below hangs everything off
I.** Flip the driving base-height offset's side and the entire lattice drops by twice the resolved
Driving Gear Base Height, gear and pinion together, with every relative length still correct — which
is why nothing downstream refuses it.

Connect I and J with a line, coincident at both ends, and add
`sketch.geometricConstraints.addPerpendicular(lineFI, lineIJ)` — the driving twin of the G→H case,
required in Fusion and absent from the proof for the same reason.

**The two base-height offsets.** Create
`sketch.sketchDimensions.addOffsetDimension(dropB, lineIJ, textPoint)` between the **B→Apex 2
perpendicular drop line** — the DPD/2 drop, NOT the Apex→B shaft axis — and I→J, and set its
`.parameter.value` to the resolved Driving Gear Base Height. I→J is ALREADY parallel to the drop by
construction, since J→I is perpendicular to F→I which runs along the driving shaft, so add **no**
extra parallel constraint ([PB-OFFSET-DIM]). The value is the one AFTER the driving gear's Maximum
Base Height has been applied — a fallback capped to it, a user value already rejected if it exceeded
it — because the offset set here is what drives the heel edge D→J toward the shaft axis.
**`addOffsetDimension` is unsigned and does not pick which side of the drop I→J lands on**; the I and
J seeds are the only thing that does.

Create the pinion twin, `addOffsetDimension(dropA, lineGH, textPoint)`, between the **A→Apex 2
drop** and G→H, already parallel by construction, with no parallel constraint, and set its
`.parameter.value` to the resolved Pinion Gear Base Height from S02.

**Close the figure: constrain point I with the projected centre point**, with one
`sketch.geometricConstraints.addCoincident(pointI, projectedCenter)`.

**A′ and the hexagon's shaft-axis edge.** Draw a line from A′ to G and pin its endpoints. **This line
is what CREATES A′** — nothing above it does — so draw it with its start seeded at the closed form
`A′ = Apex + unit(Apex→A) * (the along-shaft coordinate of N)`, the foot of the perpendicular from N
onto the pinion shaft axis. The front face N→A′ further below is what PINS A′ to that axis; until
then A′ is a free endpoint sitting at its seed. **Draw the line here rather than after the front
face**, so the hexagon's edges are created in the walk order A′ → G → H → C → M → N that the Profile
sketch's first-edge rule depends on.

**K and L.** Draw a construction line away from the Apex, starting at G, extending along Apex→A, and
call its end **K**. Then **pin K with two point-on-line coincidents** —
`sketch.geometricConstraints.addCoincident(pointK, pinionShaftAxis)` and
`sketch.geometricConstraints.addCoincident(pointK, pinionDedendumLine)` — rather than `addCollinear`
on the connecting lines. By the time K is added, G and C are already fixed, so a collinear here
over-constrains the sketch and Fusion errors; the two point-on-line coincidents locate K exactly, at
the intersection of the two lines, without over-constraining. Draw a construction line from C to K
for reference. Build L the same way from I along Apex→B, pinned onto Apex→B and onto the Driving
Dedendum line Apex2→D, with a reference line from D to L.

**The tooth centres K′ and L′ (Tooth Spacing).** The §3 spur tooth is centred not at K but at K′,
obtained by shifting K outward along the dedendum line by Tooth Spacing, AWAY from the lower corner
C — that is, in the C→K direction beyond K.

**When Tooth Spacing is 0, the default, build nothing here: set K′ ≡ K, L′ ≡ L and reuse the existing
C→K and D→L reference lines.** A zero-length dimensioned line would be degenerate, and one segment
gets one line ([BEVEL-F-LINE-ONCE]).

When Tooth Spacing is greater than 0, draw a construction line starting at K with its far end seeded
at `K′ = Apex 2 + unit(Apex2→C) * (the pinion's virtual pitch radius + Tooth Spacing)`, which is K
plus Tooth Spacing along Apex2→C, on the far side of K from C. **"Virtual pitch radius" here is the
EXACT back-cone radius `(PPD / 2) / cos γ_p` that S08 defines, and never a radius rebuilt from a
tooth count** — that is what `|Apex 2 → K|` measures, because the dedendum line Apex2→C is
perpendicular to the Pitch Line, which meets the pinion shaft axis at γ_p, so walking `r_p / cos γ_p`
along it from Apex 2 lands exactly on the axis, at K. Reading the term as a rounded count times half
a Module puts the seed 0.4203 mm short on the shipped default geometry — 31 teeth, Module 1, Shaft
Angle 90° — which is 420 times the seed gate's tolerance. Pin the far end the same way K is pinned:
a coincident from the line's start to K, and a point-on-line coincident from K′ to the Pinion
Dedendum line Apex2→C extended. Then add an aligned **length dimension on this line equal to Tooth
Spacing**; do not use `addCollinear`, for the same over-constraint reason as K. **That length
dimension is unsigned, so the point-on-line pin plus the length admit K′ one Tooth Spacing on the C
side of K just as readily — the two candidates sit `2 × Tooth Spacing` apart — and this seed is the
only thing that rules the wrong one out.** A flipped K′ tightens the mesh by the clearance the input
asked to add and builds a gear that looks right. Build K′ HERE, inside this sketch, before the
end-of-step gate, so the gate covers it. Finally draw the tooth-centre reference line **C→K′** for
§3 to use in place of C→K. Only the tooth's centre moves: the virtual tooth number and the drawn
tooth size are unchanged.

Build L′ exactly as K′, substituting L for K, D for C and the Driving Dedendum line Apex2→D for the
pinion's, with the reference line **D→L′**, the same single Tooth Spacing value, and the seed
`L′ = Apex 2 + unit(Apex2→D) * ((DPD / 2) / cos γ_g + Tooth Spacing)`.

### Resolve the two bounds that could not resolve earlier

At this point A, B, C, D, H and J all exist **and are solved**, so resolve the **Maximum Face Width**
and apply it before using Face Width below.

```
Maximum Face Width = 0.95 * min(
    perpendicular distance from A to the line through C and H,
    perpendicular distance from B to the line through D and J )

Face Width = the user's value if non-zero, else min(Cone Distance / 6, Maximum Face Width)
             a user value above the Maximum Face Width is REJECTED, naming the maximum
```

**Compute both distances from the points' SOLVED sketch geometry** — `pointA.geometry`,
`pointB.geometry`, `pointC.geometry`, `pointD.geometry`, `pointH.geometry`, `pointJ.geometry` — and
NOT from the pre-solve seed coordinates ([PB-SOLVED-GEOMETRY]). By now the constraint network has
located all six, so `.geometry` is exact; seeds diverge substantially for asymmetric tooth counts or
non-90° shaft angles, making a seed-based bound too loose on the binding side, and the toe then still
crosses the axis with the cap defeated.

**Compute BOTH distances and take the minimum.** The pinion side is normally the binding one, its
smaller pitch radius giving the smaller distance, but either gear can be the smaller. At Shaft Angle
90° this limit equals `0.95 * min(PPD, DPD)**2 / (2 * Cone Distance)` — the SMALLER pitch diameter,
never the pinion's by name. Written with the pinion's diameter it is wrong whenever the driving gear
carries the smaller tooth count: on a Driving 17 / Pinion 31 pair at Module 1 the real bound is
3.883 mm and the pinion form gives 13.591, so the `Cone Distance / 6` default exceeds it — and the
gear fails to generate — for any gear ratio above roughly √2.

Rationale, which is not to be dropped when regenerating: the toe line M→N is C→H offset toward the
Apex, and its mirror O→P is D→J offset toward the Apex. **Before the Toe Radius existed, N was pinned
to line A→Apex 2**, so the offset drove it straight down that drop: when the offset reached the
perpendicular distance from A to line C→H, N landed exactly on A, and any larger value drove N PAST
A, across the gear's own shaft axis. That is the crossing this cap is measured from, and the cap is
kept because Face Width still sets where the toe end starts. **N and P no longer ride that drop**, so
the cap no longer describes where they end up; what stops the profile crossing its axis now is that
the Toe Radius is strictly positive and only the front face's foot touches the axis. The frustum
profile is revolved about that axis, so a profile that has crossed it self-intersects the axis of
revolution and Fusion aborts with `ASM_WIRE_X_AXIS` ([PB-REVOLVE]). The 0.95 keeps N clearly off A,
since a near-coincident N ≈ A degenerates the toe edge even before it strictly crosses.

With the Face Width resolved the **Root Length** follows, so this is also where each gear's
**Maximum Bore Diameter** resolves and is applied — **skip it entirely when Enable Bore is
unchecked**.

```
Root Length at Toe Extension 0 = Face Width * |Apex→Ded| / R
Toe Radius (per gear), when the input is 0 = r − Face Width / sin γ
Toe Radius Ceiling (per gear)             = (r − 1.25 * m * cos γ) * (1 − Face Width / R)
γ_root (per gear)                         = γ − atan(1.25 * m / R)
Toe Limit (per gear)                      = |Apex→Ded| − Toe Radius / sin γ_root
Root Length = Root Length at 0
            + (Toe Extension / 100) * 0.99 * (min(pinion Toe Limit, driving Toe Limit)
                                              − Root Length at 0)

r_heel = r − (that gear's RESOLVED Base Height) / tan γ
r_toe  = (|Apex→Ded| − Root Length) * sin γ_root
Maximum Bore Diameter = 2 * 0.95 * min(r_heel, r_toe)

Bore Diameter = the user's value if non-zero, else min(that gear's Pitch Diameter / 4,
                                                      Maximum Bore Diameter)
                a user value above the Maximum Bore Diameter is REJECTED, naming the maximum
```

**The bore is a through cut on the shaft axis, so it removes every ring of material inside its own
radius and reaches the two ends of the body first**: past the heel term it takes the ENTIRE flat back
face — the disc the back-face edge G→H sweeps — and past the toe term it takes the whole toe dish and
bites into the root cone. Either way the revolved frustum comes out of the Bore step with no end face
on that side. `r_toe` is where M lands, one Root Length back along the root element from the dedendum
point; **at Toe Extension 0 it is exactly that gear's Toe Radius Ceiling**, and it shrinks as the Toe
Extension climbs, so a large Toe Extension tightens this bound.

**The flat FRONT face is deliberately NOT protected.** Its radius is the Toe Radius, which is not on
the body's outer envelope — the toe dish leans inside the root cone — so a bore wider than the Toe
Radius only exits through the toe cone instead of through that face, and the frustum stays whole.

Worked case, Module 1, Driving 31, Pinion 31, Shaft Angle 35°: each γ is 17.5°; the driving fallback
`3.875 mm` clears its own Maximum Base Height of 4.286 mm and stands. So
`r_heel = 15.5 − 3.875 / tan 17.5° = 3.210 mm`, while `r_toe` is above 12 mm and the heel term binds.
The auto bore radius is `Pitch Diameter / 8 = 3.875 mm`, outside `r_heel`, so without this bound the
auto bore alone deletes the back face of both gears. The shipped default pair at Shaft Angle 90° is
unaffected — it is the low-γ end of the Shaft Angle range that is not.

**Toe Extension 100 stops at 0.99 of the way to the smaller of the two gears' Toe Limits, not at the
Toe Limit itself.** The smaller limit wins because the pair shares one root length; the other gear
simply stops short of its own X. The 0.99 is there because AT the limit the toe face has zero length,
so the revolved body carries NO cone at its toe end and the conical end-cut in S14 — whose toe cut
must split or the build fails — has no `ConeSurfaceType` face to find. The last percent is worth well
under a tenth of a millimetre of root length on every case in the proof's table, so the reach given
up is nil and the failure avoided is total. **Do not drop this factor when regenerating.**

**A defaulted Toe Radius can leave no room at all, and that is a real configuration rather than a
defect.** On a driving gear with a large pitch cone angle the inner toe corner already sits at a
LARGER radius than the outer one — the toe dish leans toward the heel rather than away from it — so X
falls behind the toe corner and the Toe Limit comes out below the Toe Extension 0 root length.
Measured over gear ratio against Shaft Angle it is a diagonal band that crosses 90° for every ratio
from about 2.75 up, and Module does not move its boundary. **Reject a Toe Extension above 0 on such a
pair**, with a message naming the gear and the Toe Radius Ceiling it needs to come below; Toe
Extension 0 still resolves, so the gear stays buildable exactly as before. **Do NOT silently
substitute a smaller Toe Radius**: that would change the toe end of a gear whose inputs asked for no
change. A user-supplied Toe Radius must be strictly below that gear's Toe Radius Ceiling; reject it
naming the ceiling.

### The toe lines and the two front faces

**Create line M→N. Seed BOTH ends at their closed-form solved positions, not near them**
([PB-SEED-NEAR]). Seed M on Apex→C at the fraction `1 − Root Length / |Apex→C|` from the Apex. Then
seed N by sliding from that M seed along the C→H direction by exactly

```
(M seed's perpendicular distance from the Pinion Gear Shaft Axis − Pinion Gear Toe Radius) / cos γ_p
```

**A seed that merely lands somewhere plausible is not enough here, and a wrong one builds the wrong
gear rather than failing to converge.** N's position is fixed by the toe line together with a LENGTH
dimension on the front face, and a length is unsigned: the toe line meets the Toe Radius on BOTH
sides of the shaft axis, so the solver takes whichever side the seed starts on. Seeded below the axis
it converges happily onto the mirror, N comes out on the far side, the revolved hexagon crosses its
own axis of revolution, and Fusion aborts with `ASM_WIRE_X_AXIS` at the revolve rather than at the
seed that caused it.

**Two earlier seeding rules are now known to do exactly that, so do not reinstate either:** sliding
from the M seed by the Root Length, and sliding by the distance from the M seed to A. Both were
written for the scheme that pinned N to the A→Apex 2 drop, where A was N's real target. Measured on
the shipped default pair, Module 1 with 31/31 teeth at Shaft Angle 90° and a Toe Extension of 50%,
the Root Length slide puts the N seed at a perpendicular distance of **−0.27 mm** from the shaft axis
— past it — against a solved N at **+5.17 mm**, and Fusion refuses the revolve. The slide above puts
it at 5.17 mm exactly. Do not seed M and N just Face Width away from C and H either, which starts N
near H, far from its constraint target.

Then apply **exactly these constraints**; all three are required, and the front face below is what
holds N off the shaft axis, which is what the pre-Toe-Radius scheme used the A→Apex 2 pin for:

- `sketch.geometricConstraints.addCoincident(pointM, pinionRootAxis)` — M lies on the Apex→C root
  axis;
- `sketch.geometricConstraints.addParallel(lineMN, lineCH)` — the toe line is parallel to C→H;
- `sketch.sketchDimensions.addOffsetDimension(lineCH, lineMN, textPoint)` with its `.parameter.value`
  set to the Root Length re-measured perpendicular to the pitch line, i.e.
  `Root Length * R / |Apex→C|`. An offset dimension controls a perpendicular distance, so it carries
  the root length in that form; at Toe Extension 0 the value is exactly the resolved Face Width,
  which is what this dimension has always been. Place the text point in the gap between C→H and M→N
  on the Apex side — the midpoint of the M seed and point C reads cleanly ([PB-OFFSET-DIM]).

**The toe's side relative to the heel is held by the M seed and by nothing else.** An earlier
revision said it follows from the §2 frame being built correctly, in particular from the Apex 2
drops; that was wrong. `addOffsetDimension` is unsigned, so a correctly built frame still admits M→N
one root length on the FAR side of C→H, where the toe lands outside the heel and the revolved frustum
is degenerate. The text point does not control it either.

Let the line's start be **M** and its end **N**. Draw a line from M to C.

**The front face N→A′, which is what holds N. N is NOT pinned to line A→Apex 2.** Earlier revisions
pinned it there, which fixed its station at A's and made the Maximum Face Width the value at which N
reached A. It now rides the **Pinion Gear Toe Radius**:

- draw a line from N to a new point **A′**, seeding A′ at N's station on the shaft axis;
- `sketch.geometricConstraints.addCoincident(pointAprime, pinionShaftAxis)` — A′ lies on the
  **Apex→A shaft axis**; it is a FOOT, not a corner, and it is the only toe-end point that touches
  that axis;
- `sketch.geometricConstraints.addPerpendicular(lineNAprime, pinionShaftAxis)` — the front face
  stands square to the shaft, so the revolve sweeps it into a flat annulus;
- an aligned distance dimension on the whole line N→A′ equal to the resolved Pinion Gear Toe Radius.

**Pinning N itself to the Apex→A shaft axis remains forbidden** — that would put N ON the axis of
revolution, and the later conical split fails with `ASM_API_FAILED` for asymmetric tooth counts even
though the symmetric 45° case happens to survive. A′ sits on the axis; N never does, because the Toe
Radius is strictly positive. Those three rows plus the offset and the coincident on M fully constrain
M, N and A′ — six freedoms, six constraints, the same arity the old drop pin and the old N→A
connector had between them.

**A′ replaces A as the hexagon's first vertex.** At Toe Extension 0 with a defaulted Toe Radius the
two coincide exactly, so nothing moves; a positive Toe Extension walks A′ along the shaft axis toward
the Apex and the shaft edge grows by that much.

**Create line O→P, the mirror on the driving side.** Seed O on Apex→D at the fraction
`1 − Root Length / |Apex→D|`, then P slid from that O seed along D→J by
`(O seed's perpendicular distance from the Driving Gear Shaft Axis − Driving Gear Toe Radius) / cos γ_g`.
The warning above applies unchanged. Then the same three constraints:
`addCoincident(pointO, drivingRootAxis)`, `addParallel(lineOP, lineDJ)`, and
`addOffsetDimension(lineDJ, lineOP, textPoint)` with its `.parameter.value` set to the same
perpendicular form of the Root Length, its text point in the gap on the Apex side of D→J. Let the
start be **O** and the end **P**. Draw a line from O to D.

Build the driving front face **P→B′** exactly as the pinion's N→A′, substituting B for A, P for N and
the Driving Gear Toe Radius for the pinion's: the line P→B′, a coincident pinning B′ onto the Apex→B
shaft axis, a perpendicular between P→B′ and the Apex→B shaft axis, and an aligned length dimension
on P→B′. P is never pinned to the shaft axis; only B′ touches it. Then draw a line from B′ to I.

### End of the section: two gates

**First, the full-constraint gate.** Raise, naming `Gear Profiles`, if `sketch.isFullyConstrained` is
false ([BEVEL-F-FULL-CONSTRAINT]). Do NOT reach full constraint by dimensioning the driven lines —
Apex→A, Apex→B and the extension lines are determined by the perpendicular, collinear and closing
constraints ([PB-NO-OVERCONSTRAIN]). Once the Anchor Line's direction is fixed, this lattice is fully
determined by its existing net.

**Second, gate the solved figure against its own seeds** ([BEVEL-F-SEED-HELD]). After the
full-constraint gate passes, compare every named point's solved `.geometry` against the closed-form
position this section seeded it at, and **raise naming the FIRST point that has moved**, with its
solved position and its seeded one. Check them in the order this section creates them —
**Apex, B, A, Apex 2, C, D, E, F, G, H, I, J, K, K′, M, N, A′, L, L′, O, P, B′** — so the message
names the earliest site that flipped rather than a downstream symptom. **Tolerance 0.001 mm**, which
is 1e-4 cm in internal units: two orders below the tenth of a millimetre of root length this spec
already treats as negligible reach, and orders above any residue a solve can leave on a figure whose
dimensions this module sets exactly. Compare in the sketch's own 2-D frame, against the same seed
values computed above, with no world round-trip. **The list is 22 points only when Tooth Spacing is
above zero; at Tooth Spacing 0 — the default — K′ ≡ K and L′ ≡ L are not built at all, so drop those
two and compare 20**; comparing a K′ that was never created is the one way this gate can raise on a
correct figure.

**This gate is the only measure that catches all 8192 figures**, and it names the point that moved
instead of leaving Fusion to report an opaque failure later. **NEVER treat the revolve's
`ASM_WIRE_X_AXIS` as the tripwire**: several of the flips build a valid-looking gear on the wrong
side and reach no error at all, and the shipped `ASM_WIRE_X_AXIS` failure was one flip out of the 15
arriving at the revolve rather than at its own site.

### Per-gear context

Build the two per-gear dicts here, `pinionCtx` and `drivingCtx`, with these ten keys and these
spellings exactly: `label` (`'Pinion'` or `'Driving'`), `teeth`, `gamma`, `pitchDiameter_cm`,
`toothCenterPoint` (the K′ / L′ `SketchPoint`), `toothCenterRefLine` (the C→K′ / D→L′ `SketchLine`),
`hexVertices` (the six vertices in draw order, A′, G, H, C, M, N and B′, I, J, D, O, P),
`toeEdgePoints` (M and N / O and P, **in that order**), `heelEdgePoints` (C and H / D and J, **in
that order**, so the FIRST element is the dedendum corner C/D and NEVER H/J), and `boreDiameter_cm`
(already resolved AND already bounded, so no reader re-applies the `/ 4` auto value or re-derives
it). Eight further keys are written back later: `toothPlane`, `toothSketch`, `toothEmbedded`,
`toothAxis`, `gearOccurrence`, `profileSketch`, `shaftAxisEdge`, `gearBody`. **Never rename a key,
split the dict, wrap it in a class, or carry one of these values under a different shape.**

Also stash the resolved `self._faceWidthResolved_cm`, `self._drivingToeRadiusResolved_cm` and
`self._pinionToeRadiusResolved_cm`.

**Proof.** `stepGearProfiles` builds this whole lattice and gates it. Read its two arity differences
at the proof: the bench's offset constraint emits two rows where Fusion's emits one, so the two
`E→G ⊥ H→G` and `F→I ⊥ J→I` perpendiculars are omitted there, and by the identical argument so are
the two `addParallel(M→N, C→H)` and `addParallel(O→P, D→J)` calls — a substitution this spec states
for the base-height pair and not for the toe pair, though it follows from the same arity. Every
perpendicular and the one parallel are written there as the SIGNED angle they are, because the bench
offers a signed angle where Fusion offers none and this proof's whole claim is that it signs every
site Fusion leaves unsigned. **The proof cannot catch a wrong seed**: it seeds at the closed form,
which is the rule, so it proves the constraints solve FROM a correct seed and never that the module's
seed is correct — which is why the seed gate has to live inside the generated module. The Shaft Angle
30° case is carried as a **declared refusal**: this net's conditioning reads 2.83e-05 against the
engine's 4e-05 floor, which is a property of the net and not of the geometry, and the case stays in
the table rather than the range being narrowed around it.

<!-- proof-run: proofkit.RunParallel(bevelLatticeCases, stepGearProfiles) -->

**From:** `spec/bevelgear/instructions.md` L106–127, L129–159, L571–698, L351–404, L1104–1131;
`spec/bevelgear/fusion.md` L69–197

## S07 `[PROSE]` `{gearLabel} Plane` — the tooth plane

Create a construction plane that includes the tooth-centre reference line C→K′ (pinion) or D→L′
(driving) and stands perpendicular to the Gear Profiles sketch plane, using the framework helper
`plane_by_angle(designComponent, toothCenterRefLine, self._gearProfilesPlane, 90)` from `.solids`.
Name it `{gearLabel} Plane` — `Pinion Plane` or `Driving Plane` — and write it back into that gear's
dict under `toothPlane`.

Pass the sketch line **directly**; never wrap it in `Path.create` first ([PB-CONSTRUCTION-PLANES]).

Run this and the next two steps **once per gear, pinion first and driving second**, with that gear's
own tooth centre, reference line and pitch cone angle: the pinion uses K′, C→K′ and γ_p, the driving
gear L′, D→L′ and γ_g.

**From:** `spec/bevelgear/instructions.md` L700–707, L728

## S08 `[GO]` `{gearLabel} Tooth` sketch — the virtual spur tooth

Create the sketch on that gear's `{gearLabel} Plane`, named `{gearLabel} Tooth` — `Pinion Tooth` or
`Driving Tooth` — and draw the spur tooth profile into it with the tooth-centre point K′ / L′ as its
anchor.

### The virtual tooth number and the four circle radii

```
virtualPitchRadius_mm = (pitchDia_cm * 10 / 2) / cos γ        the *10 converts cm to mm
virtualTeeth          = 2 * virtualPitchRadius_mm / Module    equivalently  teeth / cos γ
rootSink_mm           = 0.05 * 2.25 * Module

circle | radius
pitch  | virtualPitchRadius
base   | virtualPitchRadius * cos(20°)
tip    | virtualPitchRadius + Module
root   | virtualPitchRadius − 1.25 * Module − rootSink
```

**Units — pin the cm→mm conversion.** The stashed pitch diameters are internal **cm** while Module is
the raw **mm** value, so the `* 10` above is required. Skipping it makes the virtual tooth count
about ten times off.

**The virtual tooth number is a REAL number and is NEVER rounded** — not floored, not ceiled, not
cast to an int. The Tredgold construction puts the equivalent spur gear's pitch radius exactly at the
back-cone distance `r / cos γ` with this gear's own module, and `z_v = z / cos γ` is a real number in
every published form of it (NPTEL Machine Design II ch. 13 eq. 13.1–13.2; Osakue et al., *FME
Transactions* 49(3), 2021, §2.2; the KHK gear technical reference eq. 11.6). Rounding it rebuilds
every drawn circle from the rounded count, which draws the tooth smaller than the back cone places it
and shortens the working addendum: on the shipped default — 31 teeth, Module 1, Shaft Angle 90°,
γ = 45° — the exact virtual pitch radius is 21.9203 mm, a floored count of 43 draws 21.5 mm, and the
addendum the tooth works over falls to 0.5797 mm against a nominal 1.0 module.

**The real count reaches the spur drawer only as an angular half-thickness.** The drawer reads
`ToothNumber` as a float and uses it in one place, `π / (2 * toothNumber)`, the angle it rotates the
flank to so the pitch crossing lands there. With `z_v = 2 * r_v / Module` that angle gives a tooth
thickness of `π * Module / 2` at the pitch circle, the standard tooth thickness — the same thickness
the spur gear of this module carries. An INTEGER count drawn at the exact radius gives
`π * r_v / round(z_v)` instead, which misses nominal by a different amount on each member of an
unequal pair, so the two teeth of one pair no longer carry the same thickness.

**The root sink.** Draw the root circle one root sink INSIDE the dedendum corner rather than at it.
At the dedendum corner exactly, the tooth's root arc touches the gear body's root cone only where the
arc crosses the tooth's own centreline: the tooth is drawn on the back-cone plane, so only a point on
that centreline rides the cone its own polar radius names, and the arc's two corners stand outside it
— by 0.002 module on the default pair and 0.027 module on a 4/4 pair, the largest of any pair this
spec admits. The sink pushes the whole arc inside, so the Combine-Join in S23 meets the gear body
across the root rather than along one line.

### Drawing it

Bevel **borrows the spur tooth generator**, `from .spurgear import SpurGearInvoluteToothDesignGenerator`,
used only here, once per gear, with the framework proxy
`from .spurproxy import VirtualSpurProxy` — import it; do NOT define a local copy, and define no
local value-wrapper class either.

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth, rootSink_mm=rootSink)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))
```

The proxy precomputes, in internal cm, exactly the parameter keys the drawer reads via
`parent.getParameter(name).value` — `Module`, `ToothNumber`, `PressureAngle`, the four circle
diameters and radii, and `InvoluteSteps` — each wrapped in a `.value` carrier. Its defaults match
bevel: pressure angle 20°, which is NOT a bevel dialog input, and 15 involute steps.

**Draw the tooth ALREADY ROTATED 180°, by passing `angle=math.radians(180)` to `draw`**; the
generator rotates the whole tooth by that angle. Do not draw it flat and rotate the sketch afterward
— the 180° delivered through the `draw()` angle relies on spur's radial flank-to-root pinning so the
connecting lines rotate with the tooth.

<!-- check-step-calls: ignore getParameter -->

`getParameter` is the interface the borrowed drawer reads the proxy through, not a call this module
makes.

**Read `proxy._lastToothEmbedded` back after `draw()` returns** and write it into that gear's dict as
`toothEmbedded`. The spur generator decides during `draw()` whether the tooth is embedded — tip, root
and flanks meeting with no connecting lines — and records it on the proxy, which pre-initialises the
slot to absorb that write. **This flag is not optional bookkeeping**: it is the deterministic
selector for the tooth loop's line count in S13, and skipping it to accept either count grabs an
unrelated loop and the apex-to-tooth loft dies with `LOFT_NO_TOOLBODY`.

**After `draw()` returns do NOT hard-gate this sketch: log it if `sketch.isFullyConstrained` is
false, and never raise.** The tooth-profile sketches are the one exemption to the full-constraint
gate, and **only because they are labelled**: `drawCircles` labels each of the four circles with
along-path sketch text, sketch text holds a DOF ([PB-SKETCH-TEXT], [PB-TEXT-HOLDS-DOF]), so a tooth
sketch whose geometry is completely determined still reads `False` purely because it is labelled.
Bevel's own four sketches carry no text, which is why they gate normally.

**The exemption covers the labels and nothing else. Never read it as licence for loose geometry.**
Its earlier wording claimed the embedded tooth kept a free radial DOF because the flank-to-root stubs
are omitted, that the residual DOF was benign since the profile is consumed immediately by the loft,
and that fixing it would risk the whole spur family. All three were wrong: each stub is DOF-neutral,
`proof/spurgear/sketches_test.go` proves the embedded scheme reaches DOF 0 without them, the residual
DOF was the tooth-top arc's CENTRE — which `addByCenterStartEnd` copies rather than shares — and the
fix was one coincident constraint in the shared generator that all eleven bench cases pass. That
wrong reasoning let a deformed tooth ship: measured in Fusion on 2026-09-02 on a default 31/31 pair,
the pinion's tooth-top arc came out at 0.5743 mm and the driving gear's at 17.0204 mm where both
should have been the 22.5 mm tip radius, from two sketches with byte-identical constraint counts and
dimension values. With the fix both read 22.5 mm exactly, a centre gap of 0.000000 mm, and an
identical 3.5475 mm² profile.

Write the sketch back into that gear's dict as `toothSketch`.

**Proof.** `stepVirtualSpurTooth` draws the four circles and the tooth loop from the same shared
involute math, and asserts what BEVEL supplies: the exact virtual pitch radius and count, the four
radii, the root sink, the embedded flag, the tooth centre's station and radius against the shaft
axis, and the curve counts the profile finder then selects on. The tooth's own involute constraint
scheme belongs to the spur family and is proved in `proof/spurgear`, so the flank samples are placed
and fixed there rather than restating a scheme another proof owns. The tooth-top and root arcs are
built with a centre point of their own pinned back onto the tooth centre, which is faithful —
`addByCenterStartEnd` copies the centre it is handed — and the centre gap and both arc radii are
asserted, which is the reading the Fusion defect above was found on.

<!-- proof-run: proofkit.RunParallel(bevelToothCases, stepVirtualSpurTooth) -->

**From:** `spec/bevelgear/instructions.md` L517–551, L700–730, L1056–1068;
`spec/bevelgear/fusion.md` L19–58

## S09 `[PROSE]` `{gearLabel} Tooth Axis`

Create a construction axis through the tooth-centre point, normal to the plane the tooth profile was
drawn on, with `designComponent.constructionAxes.createInput()` then
`axisInput.setByTwoPlanes(self._gearProfilesPlane, helperPlane)` and
`designComponent.constructionAxes.add(axisInput)` ([PB-CONSTRUCTION-AXES]). Name it
`{gearLabel} Tooth Axis` and write it into that gear's dict as `toothAxis`.

The two planes are the **Gear Profiles plane** and a **helper plane** built with
`helperInput.setByDistanceOnPath(toothCenterRefLine, adsk.core.ValueInput.createByReal(1.0))`, which
is perpendicular to that line at its far end, the tooth-centre point; their intersection is the line
through the tooth centre normal to the tooth plane. Pass the sketch line directly, never through
`Path.create`.

<!-- check-step-calls: ignore setByPerpendicularAtPoint -->

`setByPerpendicularAtPoint` is named only to say why it is not used: it would need a `BRepFace` this
build does not have.

Creating this axis in the never-activated Design component is proven to work — `constructionAxes.add`
via `setByTwoPlanes` does not hit [PB-CONSTRUCTION-NEEDS-ACTIVE] here — so keep the axis.
`toothAxis` is the one dict entry no later step reads back; Cleanup hides the axis by entity kind
rather than through the dict. It is listed so that a regen which stashes the axis is not read as
having invented a key.

**From:** `spec/bevelgear/instructions.md` L732, L377

## S10 `[PROSE]` `{gearLabel} Gear` component

Create a new component as a child of the **Bevel Gear** component — the same component that owns
Design, NOT the user's Parent Component; this intentionally overrides the looser "child of Parent
Component" phrasing so the pair nests cleanly inside Bevel Gear — with
`self.bevelOccurrence.component.occurrences.addNewComponent(adsk.core.Matrix3D.create())`, named
`{gearLabel} Gear`, i.e. `Pinion Gear` or `Driving Gear`. Write it into that gear's dict as
`gearOccurrence`.

The finished bodies for this gear end up here, but the feature operations do not run here: Fusion
rejects cross-sibling sketch and project calls even when the target is activated or the entities are
wrapped in assembly-context proxies ([PB-NO-CROSS-SIBLING]), so every feature runs in the Design
component and the finished bodies are moved here at the end, in S26. The visible end state is
identical.

Run S10 through S25 once per gear, **pinion first and driving second**, with these substitutions:

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A′ → G → H → C → M → N → A′ | B′ → I → J → D → O → P → B′ |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge, the hexagon's FIRST edge | A′→G | B′→I |
| toe cut edge | M→N | O→P |
| heel cut edge | C→H | D→J |
| teeth, bore and pitch-diameter inputs | Pinion Gear … | Driving Gear … |
| §2 shaft construction line, NOT usable as the axis | Apex→A | Apex→B |

The profile and the body are built **interleaved per gear** — pinion profile, pinion body, driving
profile, driving body — and never both profiles followed by both bodies.

**From:** `spec/bevelgear/instructions.md` L828–842, L420–428

## S11 `[GO]` `{gearLabel} Profile` sketch

Open a **fresh sketch on the axial Gear Profiles plane**, `designComponent.sketches.add(self._gearProfilesPlane)`,
named per the table — **one profile sketch per gear**, so `sketch.profiles` holds exactly this one
hexagon loop. Drawing both gears' hexagons in the shared §2 sketch would leave two identically shaped
loops to disambiguate. Write it into that gear's dict as `profileSketch`.

Build the hexagon on fixed vertices by the recreate-share-fix recipe ([PB-PROJECT-NOT-FIXED]):

1. Recreate the six §2 vertices as new points at their exact world-mapped positions —
   `sketch.sketchPoints.add(sketch.modelToSketchSpace(vertex.worldGeometry))` for each of
   `hexVertices` — which is valid because §2 is fully constrained by now.
2. Draw the closed hexagon in the table's draw order as six
   `sketch.sketchCurves.sketchLines.addByTwoPoints(pointOne, pointTwo)` calls **sharing** those
   points.
3. **Then** fix the lines and their endpoints, by setting `line.startSketchPoint.isFixed` and
   `line.endSketchPoint.isFixed` to `True` for each line — **after** the lines exist, not before.

**The order in step 3 is load-bearing.** Setting `isFixed` on a bare point before it is consumed as a
line endpoint does NOT leave the sketch fully constrained. And projecting the §2 points instead of
recreating them would bring them in associatively, leaving the sketch under-constrained even though
every projected point already has a correct position — a defect that is silent until the property is
read.

**The hexagon's FIRST edge is the gear's shaft axis** for the revolve, the pattern, the bore plane
AND the meshing-rotation axis, so it must be fixed well enough to carry a trustworthy world position:
fixed endpoints give that edge a well-defined `worldGeometry` ([PB-WORLDGEO-CONSTRAINED]), while a
free edge resolves against a default or world-XY frame and silently moves the body onto world XY —
observed on the driving gear, the pinion looking fine only because it never read the edge's world
geometry. Write that first edge into the dict as `shaftAxisEdge`.

**The shaft axis every body operation below uses is this sketch's first edge, NOT the §2 Apex→A or
Apex→B construction line.** The edge is collinear with the shaft axis but lives in the SAME sketch as
the profile, which is what Fusion's revolve, pattern and path builders accept; reusing the §2
construction line, which lives in a different sketch, fails or misbuilds.

Gate the sketch: raise, naming it, if `sketch.isFullyConstrained` is false
([BEVEL-F-FULL-CONSTRAINT]).

**Proof.** `stepGearProfileHexagon` recreates the six vertices, draws the closed hexagon sharing
them, fixes the endpoints after the lines exist, and asserts what the next steps select on: exactly
one region, six edges and every one of them a line, the region valid and not self-intersecting —
which is the [PB-REVOLVE] crossing the Maximum Face Width cap exists to prevent — the first edge's
two endpoints at radius zero from the shaft axis, the toe corner strictly off it at the resolved Toe
Radius, and the region's own area against the closed form the revolve's Pappus reading then rests on.

<!-- proof-run: proofkit.RunParallel(bevelHexCases, stepGearProfileHexagon) -->

**From:** `spec/bevelgear/instructions.md` L830–846

## S12 `[GO]` Revolve the Gear Body

This sketch holds exactly one hexagon loop, so take its single profile with `sketch.profiles.item(0)`
directly ([PB-SINGLE-PROFILE]) — do not invent a search that filters by loop or curve type, which has
spuriously rejected a valid all-line loop and made the revolve fail with "could not find profile".

Revolve it about the shaft-axis edge:
`designComponent.features.revolveFeatures.createInput(profile, shaftAxisEdge, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
then `revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))`, then
`designComponent.features.revolveFeatures.add(revolveInput)` ([PB-REVOLVE]). The result is the **Gear
Body**, the frustum; write it into that gear's dict as `gearBody`.

Because the toe edge is one edge of the revolved profile, the body already carries the conical face
that edge sweeps around the axis — that face is reused as the cutting tool in S14, and likewise the
heel edge's cone.

**The profile must not cross the axis of revolution.** If it does, Fusion aborts with
`ASM_WIRE_X_AXIS … the profile crosses the axis of revolution`. The Maximum Face Width and the
Maximum Base Height resolved in S06 are what keep it on one side; reproduce both caps exactly.

**Proof.** `stepRevolveGearBody` substitutes a **polygonal sweep**, because decad publishes a
revolved body's volume with a proven bound equal to the volume itself, so a revolved body is Suspect
at any tolerance and cannot pass the harness gate. It builds the three bands the frustum's profile
edges sweep — the root cone out to the dedendum corner, the heel cone out to the heel end, and the
toe-dish plug that hollows the front face — lays them apart and never joins them, and asserts the
frustum as their SIGNED SUM against Pappus on the §2 hexagon, band by band against its own stations
and ring radii, and cone half-angle by cone half-angle: the heel band and the toe plug come out
parallel on the back-cone family at `90° − γ`, and the root band at this gear's own root cone angle,
one dedendum angle inside its pitch cone. **The cost is the union**: the proof does not show the
three bands closing into one watertight solid, only that each is separately watertight and that
together they have the right volume, stations and angles.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelSolidCases, stepRevolveGearBody, assertRevolveGearBody) -->

**From:** `spec/bevelgear/instructions.md` L848, L896–953

## S13 `[GO]` Loft the uncut tooth body

Select the tooth cross-section loop with
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)` from `.utilities`,
where **`wantLines = 0 if toothEmbedded else 2`**, reading `toothEmbedded` back from that gear's
dict.

**The line count is DETERMINED by the embedded flag, never guessed and never accepted either way.**
For a given gear only ONE of those counts is the real tooth; an UNRELATED loop — an inter-tooth or
annular region between the drawn circles — can also carry 2 NURBS and 2 arcs with the OTHER line
count, and selecting it makes this loft fail with
`RuntimeError ... ASM_RBI_INTERNAL / LOFT_NO_TOOLBODY`, because the impostor loop cannot form a loft
tool body. Embedded means tip, root and flanks meet with no connecting lines, four curves;
non-embedded means two connecting lines, six curves ([PB-PROFILE-MATCH]).

Loft the **§2 Apex sketch point** — `self._apexSketchPoint`, the `centerToApex.endSketchPoint` from
the Gear Profiles sketch, as the degenerate point section — to that profile:
`designComponent.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
then `loftInput.loftSections.add(self._apexSketchPoint)` and `loftInput.loftSections.add(toothProfile)`
in that order, then `designComponent.features.loftFeatures.add(loftInput)` ([PB-LOFT]). The result is
the **Tooth Body**.

**Use the §2 Apex SKETCH point directly — do NOT create a construction point for it**
([PB-CONSTRUCTION-NEEDS-ACTIVE]; the Design component is never active, and construction geometry
needs an active component where sketch geometry does not).

**Proof.** `stepLoftTooth` substitutes a shrunken section for the degenerate apex point, and nothing
else. **The tooth plane is not substituted**: the proof builds the real back-cone plane, tilted out
of the axis-perpendicular by γ, and takes the apex's perpendicular distance to the section as
`s_K * cos γ` rather than `s_K` — which at Tooth Spacing 0 is exactly the Pitch Cone Distance. **The
cost is the point section**: the loft's degenerate end is not built, and what the volume and the two
cone slopes prove is the taper it has to produce. The tooth section itself is chorded, because
decad's Loft pairs only line, arc and circle segments and the drawn flanks are splines; every vertex
of the chorded polygon is a point the drawn tooth passes through exactly, and the volume is asserted
against that polygon rather than against a smooth tooth.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelSolidCases, stepLoftTooth, assertLoftTooth) -->

**From:** `spec/bevelgear/instructions.md` L463–473, L850, L931–935

## S14 `[GO]` The tooth-body hook, and the straight tooth's conical end cuts

`_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld, apexSketchPoint,
toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane, gearLabel, teethNumber, gamma)` is
the single tooth-body hook `_createGearBody` calls after lofting the uncut apex-to-heel tooth.
`gamma` is this gear's pitch-cone half-angle, γ_p for the pinion and γ_g for the driving gear, and it
is forwarded to the spiral build's twist law in S18.

**Its first line is the gate:** when `self._spiralAngle_rad` is at or below zero, return
`cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)`
immediately, the framework helper from `.solids`. Straight bevels are byte-for-byte the prior
behaviour. Everything in S15 through S21 runs **only** when ψ is above zero, in place of the two
conical trims.

**Caller obligations, which stay in `_createGearBody` and are the single biggest spiral-regen
hazard.** The §2 lattice gives each gear a **toe edge**, the inner face-width edge nearer the apex,
and a **heel edge**, the outer edge at the back cone:

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

From those §2 sketch points' **world** geometry, compute and pass positionally, in the order
`toeMid, heelMid, toeConeWorld, heelConeWorld`:

- `toeMid` = the world **midpoint of the TOE edge**, `(M_world + N_world) / 2` for the pinion and
  `(O_world + P_world) / 2` for the driving gear;
- `heelMid` = the world **midpoint of the HEEL edge**, `(C_world + H_world) / 2` and
  `(D_world + J_world) / 2`;
- `toeConeWorld` = the toe edge's inner endpoint, **M** or **O**, which lies on the root cone element
  at the inner end — M is pinned onto Apex→C in §2, O onto Apex→D;
- `heelConeWorld` = the **dedendum corner C or D**, the heel edge's FIRST endpoint, which is the
  outer end of that SAME root cone element.

**Two scrambles to avoid; a fresh regen has made both.** Do NOT pass the two endpoints of a single
edge as `toeMid` and `heelMid` — M and N both sit at the toe, so the span collapses to about zero or
goes negative and the spiral inverts. And `heelConeWorld` is **never H or J**: those lie on the
Apex2→C and Apex2→D dedendum lines, one Module beyond C and D and OFF the root cone element, and
using them skews the cone direction.

`apexWorld` is the §2 Apex sketch point's world geometry, and `gearBody` is the revolved frustum, the
cone-face source.

### The straight tooth's trim

`cut_conical_ends` implements the pinned cut behaviour; do **NOT** re-implement the cut machinery.
**Two distinct bodies are involved and must not be conflated:** the cutting TOOLS are
`ConeSurfaceType` faces of the **Gear Body**, the revolved-hexagon frustum — the lofted Tooth Body
has no cone faces, so searching IT for the cone face finds none — and the TARGET being split is the
**Tooth Body**.

The helper performs the **toe cut first**, its cone face identified by the toe edge's world
**MIDPOINT** best-first across the frustum's cone faces ([PB-FACE-BY-MIDPOINT]; endpoints sit near
the apex singularity where `getParameterAtPoint` returns no result), trying each candidate as the
actual split tool and keeping the first that splits into more than one piece; then keeper selection
after each cut, removing apex-containing pieces and keeping the largest ([PB-REMOVE-PIECES]); then
the **heel cut on the keeper alone** — removing the apex tip first is what makes it deterministically
two split features for every gear ratio. A heel cone that does not intersect the keeper at all,
common on ratio pairs such as Module 1 with driving 31 and pinion 43, where the heel cone never
overshoots the tooth, is raised as the typed `solids.NonIntersectError` and caught, and the keeper is
returned whole. **The toe cut must split** — its failure propagates and crashes the build, which is
correct, since an uncut tooth is unusable; only the heel cut is lenient, and only through that typed
error. Every failure is self-diagnosing with the per-face distance and error history
([PB-SELF-DIAGNOSING]), and each cut's outcome is logged with `force_console=True`.

**Proof.** `stepConicalEndCuts` performs **neither cut**: both operands are Lofts, the tooth and each
cone alike, and decad refuses a boolean on a Loft operand. It builds the tooth and the two cones and
lays them apart, reads each cone's apex and half-angle off the cone and each of the tooth's two
surfaces off the tooth, and solves the stations where they cross from those readings. The step
asserts exactly three things: each cut lands where the flush band requires — the toe cone meets the
gear body's own root cone at M, the heel cone meets it at C; each cone's half-angle equals this
gear's back-cone half-angle `90° − γ`, at the same slope tolerance the revolve's bands use; and each
cone crosses the tooth's TIP inboard of where it crosses the tooth's ROOT, so the trimmed end is
shorter at the tip than at the root. **It does not assert that a cut meets tip and root at different
stations, and carries no message for that case**: both cones have their apex on the shaft axis, so a
cone of wall slope `k` and apex station `a` crosses a tooth surface of slope `m` at
`a * k / (m + k)`, positive for every `m > 0` and different for the two surfaces whenever the tooth
has height, so every cut crosses both surfaces in every configuration and that assertion passes on
any figure this spec can build. **The cost is the split**: the proof does not show the evaluator
dividing the tooth, selecting the keeper, or leaving a watertight body. Two further limits are
recorded beside the assertions: a cone and a tilted plane read identically in everything this step
measures, and what makes the face conical — that it is a surface of revolution about the shaft axis —
is had by construction rather than measured; and the flush-band check cannot see the half-angle,
because any band through M crosses the root ray at M whatever slope it has.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelSolidCases, stepConicalEndCuts, assertConicalEndCuts) -->

**From:** `spec/bevelgear/instructions.md` L434–447, L736–753, L852–878, L936–953, L999–1029

## S15 `[GO]` Spiral: the cone element, the trace plane and the 2-D tooth trace

Everything from here to S21 runs only when ψ is above zero.

### The frame and its gate

Build a world frame from the geometry already constructed for this gear:

```
axisDir   = the SHAFT AXIS direction, from the two WORLD endpoints of shaftAxisEdge
            (the in-sketch profile edge A′→G / B′→I), normalized
coneVec   = normalize(heelConeWorld − apexWorld)   the ROOT cone element Apex→C / Apex→D
v         = normalize(axisDir × coneVec)           the CIRCUMFERENTIAL direction
tpNormal  = normalize(coneVec × v)                 the tangent-plane normal
distAlong(p) = (p − apexWorld) · coneVec           a point's CONE DISTANCE
```

`tpNormal` completes the frame and **nothing consumes it**: step D of the spiral build removed the
projection that once used it, so it is computed and left unread.

**The heel MUST be the outer end so `coneVec` points outward and the span is positive.** Before
building `coneVec`, check the passed midpoints and **fix a swapped toe and heel**: if
`apexWorld.distanceTo(heelMid)` is less than `apexWorld.distanceTo(toeMid)`, swap `toeMid` with
`heelMid` AND `toeConeWorld` with `heelConeWorld`, then build `coneVec` from the apex to
`heelConeWorld`. A negative span **silently inverts the entire spiral frame** — it flips the cutter
arc's direction, the slice direction, and the per-segment twist — and the gear comes out completely
wrong with no error at all. The inversion can also originate upstream in §2 or §3 mislabelling the
edges; this guard catches it at the frame.

Then, from the midpoints **after** the swap guard:

```
R_toe  = distAlong(toeMid)        R_heel = distAlong(heelMid)
R_mean = (R_toe + R_heel) / 2     span   = R_heel − R_toe     the face width, now POSITIVE
```

### The cutter-arc geometry

Work in the tangent-plane 2-D frame with the origin at the apex, **x = coneVec** so a point's x is
its cone distance, and **y = v**, circumferential.

```
r_c      = Cutter Radius if non-zero, else R_mean          the auto default
handSign = +1 for 'Right' else −1, then NEGATED for the pinion
Cx = R_mean − r_c * sin ψ
Cy = handSign * r_c * cos ψ
R_lo = R_toe − 0.06 * span        R_hi = R_heel + 0.06 * span
toe2d  = circle_intersect_nearest(R_lo, Cx, Cy, r_c, R_mean, 0)
heel2d = circle_intersect_nearest(R_hi, Cx, Cy, r_c, R_mean, 0)
```

**The hand sign goes on the `cos` and `Cy` term, NOT on the `sin` and `Cx` term.** This was a real
bug. Opposite hand mirrors the cutter centre **across the cone element, y = 0**, which flips `Cy`.
Putting `handSign` on `Cx` mirrors about `x = R_mean` instead — a different curve that gives the two
gears **unequal twist**, where for equal teeth the driving and pinion traces must come out as exact
mirror images.

`circle_intersect_nearest` from `.solids` intersects the apex circle of radius R with the cutter
circle and keeps the solution nearest `(R_mean, 0)`, the branch the mean point sits on; a
non-overlapping pair clamps to tangency. The ends are taken a hair PAST the face so the kept arc
reaches cleanly past the end-trims.

### Building the two sketches

First draw a **cone-element construction line** from the apex to `apex + R_heel * coneVec` in a
sketch on the axial Gear Profiles plane, named `{gear} Cone Element`. Then make the tangent plane by
rotating that axial plane **90°** about the cone-element line, with
`plane_by_angle(designComponent, coneElementLine, self._gearProfilesPlane, 90)`; name it
`{gear} Trace Plane`. Add a sketch on it named **`{gear} 2D Tooth Trace`**.

In that sketch, with `combine_point(apexWorld, px, coneVec, py, v)` from `.solids` mapping 2-D
coordinates to world points, draw:

- the **cutter circle**, centre at the mapped `(Cx, Cy)`, radius `r_c`, marked `isConstruction`, with
  its centre pinned by setting `circle.centerSketchPoint.isFixed` to `True` ([PB-CIRCLE-CENTER]) and a
  diameter dimension of `2 * r_c` placed at a point ON the circle such as the mapped `(Cx + r_c, Cy)`;
- the **trace arc**, a three-point arc
  `sketch.sketchCurves.sketchArcs.addByThreePoints(startPoint, midPoint, endPoint)` through the
  mapped `toe2d`, the mapped `(R_mean, 0)` and the mapped `heel2d`, with its centre coincident to the
  cutter circle's centre and a radius dimension
  `sketch.sketchDimensions.addRadialDimension(traceArc, textPoint)` of `r_c`, so it is the genuine
  cutter circle and not a look-alike spline. Its text point is the mean point, off-centre and on the
  curve ([PB-RADIAL-DIM]; a text point AT the centre is rejected outright).

**Coordinates — this rule governs the `{gear} Cone Element` sketch as well as the trace sketch.** The
world `Point3D`s from `combine_point`, and the raw apex and cone-end points of the cone-element line,
are passed **directly** into the sketch calls, where they are consumed as **sketch-space** input:
**no `modelToSketchSpace` conversion is applied**, even though `Sketch` offers exactly that call and
the points really are model-space coordinates.

This is deliberate, and why it is harmless is not the obvious reason. The trace sketch is
construction and reference only: **no downstream feature ever consumes it** — the twist is computed
analytically in S18 from the 2-D endpoints, and the sketch exists only so the genuine cutter arc is
inspectable before cleanup hides it. The cone-element line is the one that needs the extra sentence,
because it IS consumed, by `plane_by_angle`, which rotates about it to make the Trace Plane. An
unconverted cone-element line does place that plane somewhere other than the true tangent plane. That
still reaches no feature, because the only thing built on the Trace Plane is the inspection-only
trace sketch, and the chain ends there. **If a later revision ever makes any feature consume the
trace sketch or the Trace Plane, this shortcut stops being safe and both sketches need
`modelToSketchSpace` on every point.**

This sketch is **deliberately left with free DOF** — the arc's endpoints are pinned by the three-point
construction, not by endpoint dimensions, because dimensioning them over-constrains the solve against
the cone-element plane. It is therefore **exempt from the full-constraint gate**, along with the
`{gear} Cone Element` sketch and the `{gear} Trace Plane`; do NOT gate any of them. The gate applies
only to the bevel's own permanent sketches — Anchor, Gear Profiles, the two per-gear Profile sketches
and the Bore sketch — for both straight and spiral builds.

**Proof.** `stepSpiralTrace` builds the cutter circle and the trace arc in the flat crown frame and
asserts the trace invariants: the arc's radius is `r_c` everywhere because it is one circle, the
centre is exactly `r_c` from the mean point, the two ends sit on the apex circles they were
intersected with, the angle between the arc's tangent at the mean point and the cone element IS ψ,
flipping the hand mirrors the whole construction across the cone element and changes nothing else,
and at ψ = 0 the centre sits straight north of the mean point so the arc is tangent to the element
there and only there. The bench gate is not optional, so the proof **pins the two endpoints** where
Fusion leaves them free and lets the radius follow; the radius Fusion writes as a dimension is
asserted on the built arc instead, which is the same claim. The proof works in the flat frame the
twist law reads, so the Trace Plane's own placement — the thing the unconverted points move — is
outside what it can check.

<!-- proof-run: proofkit.RunParallel(bevelTraceCases, stepSpiralTrace) -->

**From:** `spec/bevelgear/instructions.md` L734–789; `spec/bevelgear/spiral-tooth-trace.md` L32–185

## S16 `[GO]` Spiral: slice the straight tooth into slabs

Split the uncut apex-to-heel `toothBody` into cross-section slabs by planes **parallel to the parent
transverse tooth plane** — `parentToothPlane`, the `{gearLabel} Plane` from S07, passed into the hook
— through a **fixed** scheme of **exactly 8 planes**. The count is not user-configurable.

```
first cut plane = parentToothPlane offset toward the apex by span/6
offsets         = [sign * (k + 1) * span / 6  for k in 0…7]
```

The offset **sign is chosen per gear** so it moves toward the apex: the parent plane's normal points
opposite ways for the two gears, so pick `sign` such that `sign * normal` points apex-ward, testing
`(apexWorld − planeOrigin) · normal`.

**Where the eight land: the first sits `span/6` inside the HEEL and none of them lies past it** — the
parent plane is already the heel end, so there is no heel overshoot to give — **the sixth lands at
the toe, and the last two sit `span/6` and `2 * span/6` PAST the toe**. The two segments beyond the
toe are what the toe cone trims away in S21. The first sits a hair more than `span/6` inside the heel
and the sixth a fraction of a millimetre inside the toe, because `R_heel` and `R_toe` are read at the
two edge midpoints rather than on the root element.

**The slice planes are NOT perpendicular to the cone element.** The parent plane carries the
tooth-centre line C→K′ / D→L′, which is the back-cone line and so perpendicular to the Pitch Line, so
**the parent plane's normal runs along the PITCH element** — while `coneVec` is the **ROOT** element.
The two differ by the dedendum angle

```
δ_f = atan(1.25 * Module / R)  =  atan(2.5 * sin γ_p / N_p)  =  atan(2.5 * sin γ_g / N_g)
```

Module cancels, so δ_f depends only on the tooth counts and the Shaft Angle and is **the same for
both members**: 3.26° on the default 31/31 pair at Shaft Angle 90°, growing as the tooth counts fall.
The parallel family is what the build requires rather than what it happens to use:
`slice_body_by_offset_planes` offsets the parent plane with `setByOffset`, which produces PARALLEL
planes; the sign test reads the parent plane's own normal, which is meaningful only for that plane's
own offsets; and the tooth is lofted from the Apex to the profile drawn in the parent plane, so **the
heel-most slab's heel face IS the parent plane** and a consistent family has to contain it.

**A build that follows "perpendicular to the cone element" instead is wrong and silent**: it tilts
every cut face by δ_f and nothing in the pipeline fails — parallel planes cut a cone in similar
sections whatever their orientation, so the loft still reproduces the taper; the piece count, the
retry gate below and the conical trims of S21 are all indifferent to slab orientation; the proof
builds its own slabs and never sees the module's plane; and the runtime gate only counts pieces. What
moves is the geometry: on the default pair at Module 4 a face corner lands
`1.125 * Module * tan δ_f` = 0.26 mm along the cone from where the parallel cut puts it, and the
step-G twist keyed across one face mismatches by up to 0.0078 rad.

Split the body with
`slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)` from `.solids`,
which splits piece by piece and keeps a piece whole when a plane misses it.

**The slice MUST actually split the tooth.** After the cut loop, if the body is still in ONE piece —
no plane cut it — the offset sign was wrong or `parentToothPlane` sits outside the tooth's span:
**retry the whole cut once with the opposite sign**. If it is STILL one piece, **raise a clear
self-diagnosing error** naming the gear, the final piece count, the span, and the sign tried
([PB-EMPTY-RESULT], [PB-SELF-DIAGNOSING]). Do **NOT** return an unsliced single-piece result: S17
then drops that one piece as the apex scrap, leaving `segments` **empty**, and the crown in S19
crashes with `ValueError: max() iterable argument is empty` far from the cause.

**Proof.** `stepSliceToothSlabs` builds the nine pieces directly rather than cutting them out of one
tooth, and lays them apart, because decad performs no boolean on a Loft operand. **The cost is the
division**: the proof does not show the evaluator dividing one tooth into these pieces. What it does
show is that the nine pieces are exactly the whole tooth, that each of the eight planes lands where
this step puts it, that the first is `span/6` inside the heel with none past it, that the sixth lands
at the toe and the last two sit one and two sixths past it, that the slice split at all, and that the
angle between the pitch element and the root element IS the dedendum angle and carries no Module —
the same value for both members of the pair. The apex-most piece takes the same shrunken stand-in for
its degenerate point end that S13's loft does.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelSpiralCases, stepSliceToothSlabs, assertSliceToothSlabs) -->

**From:** `spec/bevelgear/instructions.md` L791, L995–997

## S17 `[GO]` Spiral: order the segments and drop the apex scrap

Sort the segments by the `distAlong` of their centroid, reading it from
`segment.physicalProperties.centerOfMass`. The first — the apex-most — is the long **apex-side
scrap** below the toe; **remove it** and keep the rest as the working `segments`.

**Drop the scrap by re-slicing the list, and only THEN delete it**: take `segments = segments[1:]`
before calling `designComponent.features.removeFeatures.add(scrap)`, which is the timeline-visible
removal ([PB-REMOVE-PIECES]) rather than a bare delete.

After dropping the scrap, **`segments` must be non-empty**, at least one cross-section. If it is
empty the slice failed in S16 — **raise a clear error** rather than proceeding into the twist and the
crown, which both assume at least one segment ([PB-EMPTY-RESULT]).

**Proof.** `stepDropApexScrap` builds the nine pieces, sorts them by the cone distance of their own
centroid, and returns the eight the drop keeps. It asserts that the piece dropped is the apex-most
one, that it is the LONG one rather than another `span/6` slab, that eight remain and that the list
is not empty, and that the eight are the tooth less its scrap by volume. The same lay-apart cost as
S16 applies.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelSpiralCases, stepDropApexScrap, assertDropApexScrap) -->

**From:** `spec/bevelgear/instructions.md` L793

## S18 `[GO]` Spiral: twist each segment about the shaft axis

Rotate each segment about the **shaft axis**, `axisDir` through `apexWorld`, so the tooth follows the
trace, **centred on `R_mean` so the mid-face section stays unrotated** — that section then meshes
exactly like the straight tooth, which is critical, because the pinion's zero mesh nudge depends on
it.

The total toe-to-heel twist comes from the **conjugate crown-gear generation law**, the standard
Gleason and Litvin model: a spiral bevel is generated by an imaginary flat **crown gear**, and the
work gear's shaft rotation relates to the developed crown-plane azimuth by the **roll ratio
`1 / sin γ`**, the generating crown gear having `N / sin γ` teeth. Compute it **analytically — no
projection and no curve sampling**:

```
phi_crown = atan2(heel2d[1], heel2d[0]) − atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
ang       = −handSign * total * (R_mean − R_heelFace(seg)) / span
```

`phi_crown` is the angle the cutter arc's toe and heel endpoints subtend at the apex in the flat 2-D
crown frame — exactly the `toe2d` and `heel2d` pairs from S15. `handSign` sets the direction and
`total` is the magnitude.

**Use the PITCH cone angle γ from §2 — `self._gamma_p` or `self._gamma_g`, the `gamma` argument the
hook receives — and NOT `acos(coneVec · axisDir)`**, which is the ROOT or dedendum cone angle,
smaller than γ by δ_f — 24.7° against the pitch 28.7° for a 17-tooth pinion meshing a 31-tooth gear —
and yields a twist about 1.15 times too large.

**The two members of a meshing pair legitimately get different twists**: same cutter, same spiral
angle ψ, but γ differs, so `1 / sin γ` differs — about 2.08 for a 17-tooth pinion against about 1.14
for a 31-tooth gear, a ratio near 1.83. This is WHY equal-teeth pairs, with equal γ, always meshed
while ratio pairs failed under any method that gets `1 / sin γ` wrong. **Do NOT measure the twist off
a projected 3-D cone trace**: `projectToSurface` wraps the arc around the cone for ratio pairs and
returns it as multiple disjoint fragments, so the measured azimuth collapses to a fraction of the
true sweep, the pinion comes out grossly under-twisted, and the pair interferes. The analytic law
here is exact, deterministic, and cannot wrap.

<!-- check-step-calls: ignore projectToSurface -->

`projectToSurface` is named only to forbid it.

**Each segment's rotation angle is a linear share keyed to the cone distance of its HEEL FACE**, the
segment's farthest-along-the-element face, which is the exact section the later loft samples.
**Define a slab's heel face precisely: the face whose centroid has the GREATEST cone distance,
searched across ALL of the slab's faces with NO surface-type filter**;
its toe-side face is the least-centroid one. **Do NOT restrict the search to `PlaneSurfaceType` or
any surface type** — a sliced slab is bounded by a mix of the two planar cut faces and ruled side
faces, and a type filter can pick the wrong face or miss the cut face, which makes the S20 loft fail
with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this same all-faces-by-centroid rule
everywhere a slab end face is needed — the twist key here, the crown base in S19, and the loft
sections in S20.

**Key the twist on the segment's HEEL-FACE cone distance, NOT on its centroid.** The loft samples
each segment's heel face, so that face is what must land at the right azimuth; centroid-keying leaves
the loft's mid-face section rotated by half a segment and the sections overlap at mid-face.

Apply the rotation as a free move: build the matrix with
`adsk.core.Matrix3D.setToRotation(ang, axisVector, apexWorld)`, then
`designComponent.features.moveFeatures.createInput2(bodyCollection)`,
`moveInput.defineAsFreeMove(matrix)` and `designComponent.features.moveFeatures.add(moveInput)`
([PB-MOVE-ROTATE]).

**Proof.** `stepTwistSegments` rotates each built slab about the shaft axis by its own share and
asserts the law itself: the total against `abs(phi_crown) / sin γ`; that the root cone angle really
is γ less the dedendum angle and that using it would give a LARGER total, so the case can tell the
two apart; that the segment whose heel face sits at the mean cone distance takes no rotation at all;
that each segment's angle is the linear share keyed on the heel face; that keying on the centroid
gives a different angle, so the case can tell those apart too; and that a rigid rotation about the
shaft axis changes no volume. The proof's local frame keeps the shaft axis and the root cone element
as separate lines, which is what makes those readings mean anything.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelSpiralCases, stepTwistSegments, assertTwistSegments) -->

**From:** `spec/bevelgear/instructions.md` L795–808; `spec/bevelgear/spiral-tooth-trace.md` L188–217

## S19 `[GO]` Spiral: the lengthwise crown

Crown the tooth by scaling each segment **except the outermost, heel one** down by a **monotonic**
factor — full at the heel and growing smoothly toward the toe — about a sketch point on the **ROOT
edge of its heel face**.

For each segment compute its **heel-distance fraction**

```
u      = (R_heel − R_heelFace) / span
factor = 1 − _CROWN_PER_RAD * (abs(total) / 2) * u
```

with `R_heelFace` the `distAlong` of that segment's heel face, found by the same
all-faces-by-centroid rule as S18 but **RECOMPUTED HERE, AFTER the twist has moved the slabs** — do
not reuse pre-twist values — and `R_heel` and `span` from S15.

**`_CROWN_PER_RAD` is a tunable class constant with default `0.5`**, where 0 disables the crown. Set
it to 0.5; do not leave it unset or zero.

`abs(total) / 2` is the per-end peak twist magnitude, so the maximum relief — now at the **toe** —
keeps the same magnitude the old per-end peak had, just relocated. This makes relief **grow
monotonically from the full heel to the toe**, so slab heights stay **strictly ordered heel to toe**
and the natural cone taper is never reversed.

**`u` runs 0 at the held-full heel to 1 at the toe, and PAST 1 on the two segments beyond the toe.**
Those two segments' heel faces sit `6 * span/6` and `7 * span/6` in from the parent plane, so the
toe-most one reads about `7/6` ≈ 1.18 before the twist and a little more, about 1.21, after this
recompute, at the default Spiral Angle 35° on the default 31/31 pair. **Do not treat `8/6` = 1.33 as
a ceiling**: that figure is the last plane's offset, and that plane is a toe face rather than any
segment's heel face, so nothing ever evaluates `u` there — but the twist moves the heel faces, so the
recomputed `u` climbs with the Spiral Angle and is **measured at 1.351 at Spiral Angle 55°**, which
the `[0, 60)` range admits. Nothing reads an upper bound on `u`; the crown factor stays positive for
every value it takes, and what the slab count actually rests on is the structural fact that the last
cut plane is never a heel face. The heel segment reads a few hundredths rather than exactly 0,
because `R_heel` is read at the heel edge's midpoint rather than on the parent plane; that segment is
skipped anyway.

**"Outermost heel segment" = the one with the GREATEST post-twist heel-face `distAlong`** — sort the
segments by their recomputed heel-face distance and skip the last.

If a computed `factor` comes out at or below 0, at extreme twist, **raise a self-diagnosing error**
naming the gear, the segment's `u` and the factor. **Never scale by a non-positive factor.**

**Do NOT key the relief on `abs(ang)`**, the twist magnitude, i.e. the distance from mid-face. That
is **symmetric** about mid-face, maximal at BOTH ends, so — because the heel slab is held full — the
slab JUST INSIDE the heel becomes the **most** relieved one and dips below both its neighbours, a
notch that reverses the heel-to-toe taper. This was the observed bug: the heel-adjacent slab came out
at factor `0.932` while the next slab inward was `0.972`, taller. Key the relief on the monotonic
heel-distance `u`, never on `abs(ang)`.

### Three gotchas

1. **The scale base must be a sketch point** — a point added in a sketch on the heel face, or a BRep
   vertex — per [PB-CONSTRUCTION-NEEDS-ACTIVE]. `scaleFeatures` is the ONE exception to
   never-activate: it needs the Design occurrence as the **active** edit target, so call
   `self.designOccurrence.activate()` before the crown scales and restore afterwards, in a `finally`,
   with `self.design.activateRootComponent()`. **Do NOT write `design.rootComponent.activate()` or
   `someComponent.activate()`** — a `Component` has **no** `activate` method and raises
   `AttributeError`. Only `Occurrence` has it; the root is re-activated through `Design`.
2. **Skip the outermost heel segment.** Its heel face is the loft's heel end and must stay full so
   the heel cone in S21 trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, NOT its centroid, or the crowned tooth lifts off
   the gear base.** `scaleFeatures` shrinks UNIFORMLY toward the base point, so a base point at the
   heel-face **centroid**, mid tooth-height, pulls the tooth's **root** edge upward by
   `(1 − factor) * (half the tooth height)`: the tooth no longer seats on the gear body's root cone,
   floats above the base, and the Combine-Join leaves a gap — clearly visible for ratio pairs such as
   Module 2 with driving 19 and pinion 13, which is the symptom that exposed this. Put the base point
   on the **root** instead: of the heel face's vertices, each `vertex.geometry` a world `Point3D`,
   take the **two with the smallest perpendicular distance to the shaft axis** — the line through
   `apexWorld` along `axisDir`, the distance being
   `|(p − apex) − ((p − apex) · axisDir) * axisDir|` — which are the two **root corners**, the tip
   corners being the farthest from the axis, and place the base sketch point at their **midpoint**,
   mapped into the heel-face sketch with `sketch.modelToSketchSpace(worldPoint)`. The heel face is a
   planar cut, so that midpoint lies on it. A uniform scale about a point keeps every line and plane
   through that point invariant, so anchoring on the root keeps the root edge on the seating cone
   while the tip is relieved progressively toward the toe — which is exactly the lengthwise crown
   intended. Finding the heel face itself is unchanged, still the max-`distAlong`-centroid face; only
   the point ON it changes.

Apply the scale with
`designComponent.features.scaleFeatures.createInput(bodyCollection, baseSketchPoint, adsk.core.ValueInput.createByReal(factor))`
then `designComponent.features.scaleFeatures.add(scaleInput)`.

**Proof.** `stepCrownSegments` builds each segment at its crowned size about the same base point,
because decad has no scale at all — its transform type is a rigid motion by construction and admits
none. **The cost is the scale itself**: the result is the same solid, but the proof does not show the
evaluator performing it. What it asserts is the crown formula, that exactly one segment is held full
and it is the outermost by post-twist heel-face distance, that every factor is strictly positive,
that the base point is the heel face's root-edge midpoint and is a fixed point of the scale so the
root edge keeps its distance from the shaft axis, that anchoring on the heel-face CENTROID instead
would lift that root edge off the seating cone, that the crowned tip is relieved, and that the relief
is **monotonic** from the held heel to the toe, which is the reading that tells the correct keying
from the `abs(ang)` one.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelSpiralCases, stepCrownSegments, assertCrownSegments) -->

**From:** `spec/bevelgear/instructions.md` L810–822

## S20 `[GO]` Spiral: loft the curved tooth

**Re-sort the segments by their heel-face cone distance HERE, AFTER the twist and the crown — do NOT
reuse the pre-twist slice or centroid order from S17.** The twist rotates each slab about the shaft
axis, and for high-twist **unequal-ratio** pairs that rotation changes the slabs' along-cone order
enough to **reorder adjacent slabs**; lofting in the stale pre-twist order then assembles the
cross-sections out of sequence and the crowned tooth comes out distorted, so the two gears interfere.
For equal or low-twist pairs the two orders coincide, which is why equal-teeth gears mesh even with
the stale order while unequal ratios distort — this is the single thing that makes a ratio pair like
31/17 fail while 31/31 looks fine.

So compute the order **now**, from the post-twist heel faces:

```
order = the segment indices sorted by the cone distance of that segment's heel-face centroid
```

and loft a new body through, in that order:

1. first the **toe-most segment's apex-side, toe-facing face** — the toe segment is `order[0]`, and
   its toe face is added first to push the loft past the toe cone so the toe trim bites;
2. then the **heel-facing face of every segment, iterated in `order`**, each segment's
   farthest-along-the-element face by post-twist centroid, the last of which reaches past the heel
   cone.

Build it with `designComponent.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
one `loftInput.loftSections.add(face)` per section in that order, and
`designComponent.features.loftFeatures.add(loftInput)`. Name the resulting body
**`{gear} Spiral Tooth`**. Then remove the segment scaffolding — the loft has captured their faces —
with `designComponent.features.removeFeatures.add(segment)` per segment.

**Proof.** `stepSpiralLoft` builds the chain of consecutive pairs the multi-section loft passes
through, because decad's Loft takes two sections. **The cost is the single body**: the sections are
the same and in the same order, but the proof does not show one body closing over all of them. It
asserts that the first section is the toe-most segment's TOE face, that every section after it lies
strictly farther along the element so the chain is a single sweep rather than a folded one, that each
link has the volume its two sections and their crown factors give, and it records for each case
whether the twist left the slab order alone or changed it.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelSpiralCases, stepSpiralLoft, assertSpiralLoft) -->

**From:** `spec/bevelgear/instructions.md` L824

## S21 `[GO]` Spiral: flush trim

Return
`cut_conical_ends(designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)` —
the same toe-then-heel two-cone trim the straight tooth takes in S14 — so the curved tooth's ends sit
**flush** on the gear base.

The toe and heel **mesh phasing** is NOT handled here: it is handled outside this hook by the
mesh-rotate step, S25. The pinion's extra phase is 0 by default, because the mid-face section is
unrotated and already meshes.

**Proof.** `stepSpiralFlushTrim` makes the same substitution as S14's — neither cut is performed, the
curved tooth and the two cones are laid apart, and the stations are solved from their own measured
geometry — with the same cost, the split. What this step adds is that the body being trimmed is the
CURVED one: it asserts each cone's half-angle and its meeting point on the root cone as before, and
additionally that the curved tooth's toe section reaches past the toe and its heel section reaches
the parent plane, which is what makes the trimmed ends come out flush.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelSpiralCases, stepSpiralFlushTrim, assertSpiralFlushTrim) -->

**From:** `spec/bevelgear/instructions.md` L826, L995–997

## S22 `[GO]` Circular-pattern the tooth

Circular-pattern the remaining tooth piece around the **shaft-axis edge** — the same in-sketch
profile edge the revolve used, never the §2 construction line:
`designComponent.features.circularPatternFeatures.createInput(bodyCollection, shaftAxisEdge)`, then
pin all three inputs explicitly ([PB-CIRCULAR-PATTERN]) —
`patternInput.quantity = adsk.core.ValueInput.createByReal(teethNumber)`,
`patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')`, and
`patternInput.isSymmetric = False` — then
`designComponent.features.circularPatternFeatures.add(patternInput)`. The number of copies equals
this gear's Teeth Number.

Although the pitch diameter shrinks from heel toward apex, the **angular** spacing around the shaft
axis stays constant at `360° / N` for the entire face width: the radial taper is already produced by
the loft from the Apex to the heel-end tooth profile, so the pattern just rotates that single tapered
tooth into N evenly spaced copies.

The pattern returns the original plus the copies, so **do not re-add the seed**, and **copy them into
an `ObjectCollection` first** — `pattern.bodies` is a `BRepBodies`, which the combine input rejects —
by looping `pattern.bodies.item(i)` into a fresh `adsk.core.ObjectCollection.create()`
([PB-PATTERN-BODIES]).

**Proof.** `stepCircularPattern` is **the one serial step in this package**, and the proof records
why beside the variables that cause it: the increment retires the seed tooth, so its azimuth, radius,
height and volume have to be read during the build and handed to the assertion, and that hand-off
leaves the case. Two cases sharing one set of seed readings overwrite each other, and it is not a
hazard that announces itself — the two gear sides differ enough in volume that the overwrite was
caught when it happened, and a pair of cases whose seeds measured alike would have passed on each
other's numbers instead. Two substitutions are declared there. The tooth is built on an
axis-perpendicular section rather than the tilted back-cone plane, the tilt being what S13 proves and
irrelevant to the angular spacing. And the whole ring of N copies is not built: decad's
disjoint/overlap partition proof does not resolve a full ring at the real tooth pitch — measured on
the default pair at Module 4 it comes back Sound at 8 and at 16 copies and Suspect at 12, at 20 and
at the real 31 — so the step builds the seed and the ONE adjacent copy the increment puts beside it,
which is the ring's tightest pair, and asserts the ring arithmetically. **The cost is the ring**: the
proof shows one increment rather than N of them.

<!-- proof-run: proofkit3d.RunSolid(bevelPatternCases, stepCircularPattern, assertCircularPattern) -->

**From:** `spec/bevelgear/instructions.md` L880, L1078–1102

## S23 `[GO]` Combine-Join the teeth onto the Gear Body

Join all patterned tooth pieces with the Gear Body in a single Combine-Join, the Gear Body as the
target and the patterned tooth bodies as the tools:
`designComponent.features.combineFeatures.createInput(gearBody, toolBodies)` with
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`, then
`designComponent.features.combineFeatures.add(combineInput)`.

**Proof.** `stepCombineJoin` performs no join: the operands are laid apart and the join's two
consequences are asserted from their own measured geometry — a join leaves ONE lump when the tooth's
root is below the body's root cone, seated rather than floating, and the joined body reaches further
out than the frustum when the tooth's tip stands proud of it. Both readings are taken at the toe, the
middle and the heel of the band the join would cover. **The cost is the stitch**: the proof cannot
show the evaluator making one boundary out of two. The generated module draws its root circle one
root sink inside the dedendum corner (S08), so the root arc lies inside the root cone across its
whole width and the join overlaps along the whole root rather than along the centreline alone; **the
proof applies that same sink — it is one figure, not a proof-only offset** — and it **reads the root
arc's OUTERMOST point, not the tooth's centreline**, because the centreline sits inside both root
corners and a reading taken there passes a tooth whose corners float outside the cone, which is
exactly the defect the sink exists to remove.

**Fusion has made this stitch once, loaded 2026-09-16**, from the build the root sink was introduced
on: two configurations built with no error, the shipped default of 31 teeth on both gears at Module 1
and Shaft Angle 90°, and a 16 driving / 12 pinion pair at Module 4. The default is also the
configuration where the sink drops the root circle below the base circle, so its tooth is drawn
NON-embedded and the spur drawer adds the two flank-to-root lines, 0.0405 mm each; neither that
profile nor a Combine-Join at a sunk root had been through Fusion before that load. So the stitch
this substitution cannot show has been seen once, on those two configurations, and on nothing else in
the table. **The heel tip radius was not measured on that load**, so the tip is still checked only
where the proof checks it: no case reads a tip radius off a joined body, because no case joins. That
measurement is still outstanding.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelSolidCases, stepCombineJoin, assertCombineJoin) -->

**From:** `spec/bevelgear/instructions.md` L882, L954–987

## S24 `[GO]` Bore

**Skip this step entirely when Enable Bore is unchecked** — no bore is cut on either gear and the
per-gear bore diameter inputs are ignored.

Cut a cylindrical through bore through the Gear Body along the shaft axis. The diameter is this
gear's `boreDiameter_cm` from its dict, **already resolved and already bounded** by that gear's
Maximum Bore Diameter in S06 — an auto value capped to it, a user value above it already rejected.
**Take that resolved number; do not re-derive it here**, or the cap is lost and the bore deletes the
body's back face.

Build the bore plane normal to the shaft at its start:
`designComponent.constructionPlanes.createInput()` then
`planeInput.setByDistanceOnPath(shaftAxisEdge, adsk.core.ValueInput.createByReal(0.0))` and
`designComponent.constructionPlanes.add(planeInput)`. Pass the in-sketch edge, not the §2
construction line.

In a sketch named `{gearLabel} Bore` on that plane, sketch the bore circle centred at the sketch
origin — the plane is rooted at the shaft start, so the origin is on the axis — with
`sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), boreRadius)`.
**Fix the circle's centre and add a diameter dimension set to the bore diameter**
([PB-CIRCLE-CENTER]): set `circle.centerSketchPoint.isFixed` to `True` and add
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` with its text point off-centre, on
or near the curve ([PB-RADIAL-DIM]). Do **NOT** `addCoincident` the centre to the sketch origin,
which has been observed to throw `VCS_SKETCH_SOLVING_FAILED` on a `setByDistanceOnPath` plane.

Extrude-cut it as a symmetric through-cut restricted to this Gear Body:
`designComponent.features.extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
`extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(2 * coneDistance), False)`.
The second argument is `isFullLength=False`, so `2 * Cone Distance` is the half-length **per side**,
generously past any face width; do not pass a third taper argument ([PB-THROUGH-CUT]). Restrict the
cut with `extrudeInput.participantBodies = [gearBody]`, then
`designComponent.features.extrudeFeatures.add(extrudeInput)`.

**Cone Distance here is the DIAGONAL `sqrt(PPD**2 + DPD**2)`, not the Pitch Cone Distance `R`.** The
two are different lengths and both are used in this gear: they coincide as `Cone Distance = 2 * R`
exactly when the Shaft Angle is 90°, for any pair of tooth counts, and diverge everywhere else — an
equal 31/31 pair at Shaft Angle 30° has `Cone Distance = 43.84 mm` against `R = 59.89 mm`, and at
140° `R = 16.49 mm`.

Gate the Bore sketch: raise, naming it, if `sketch.isFullyConstrained` is false.

**Proof.** `stepBoreCut` builds the TOOL as a real extrude, which a symmetric extent produces as a
prism, but performs no cut: the target is the frustum, whose bands are Lofts. The tool and the bands
are laid apart, and the cut is asserted from the tool's own measured geometry — its diameter, that
its two ends sit exactly `2 * Cone Distance` either side of the shaft edge's start, and that both
clear the frustum, which is what makes it a THROUGH cut — and the material it would remove is
computed from the frustum's own profile clipped to the bore radius. It also asserts the Maximum Bore
Diameter itself against its two terms, and records where a bore wider than the Toe Radius exits,
which the bound deliberately allows. **The cost is the pierced body**: one lump with a hole and no
enclosed void is not shown.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelBoreCases, stepBoreCut, assertBoreCut) -->

**From:** `spec/bevelgear/instructions.md` L133–138, L884, L988–994

## S25 `[GO]` Meshing rotation

**Driving gear only, and here, in the Design component, before the body is moved out.** Rotate the
driving body by `180° / Driving Gear Teeth Number` — half a tooth pitch — about its shaft axis, with
`rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)` from `.solids`, which takes
the rotation axis and origin from the B′→I profile edge's **world** endpoints ([PB-MOVE-ROTATE]). A
driving valley then sits where the pinion tooth crosses the axial plane, giving the interlocked
meshing look.

Rationale: both gears are patterned from a starting tooth in the axial plane, so without the offset a
driving tooth and a pinion tooth would both sit at the axial-plane crossing and visually collide.
This runs in Design before the move because a construction axis cannot be added in the moved-out gear
component ([PB-CONSTRUCTION-NEEDS-ACTIVE]), so the rotation must use the edge's world geometry while
still there.

The pinion additionally gets `_pinionMeshPhase(pinionTeeth)`, its own extra rotation about its own
shaft axis in **radians**, `_PINION_MESH_PHASE_TEETH * 2π / pinionTeeth`, which is **0 by default**
and 0 for every straight bevel.

**A zero angle is a no-op, not a move.** `setToRotation(0, axis, origin)` builds the identity and
Fusion refuses to move a body by it, raising `RuntimeError: 3 : invalid transform` — measured
2026-09-02 on the bevel pinion, whose mesh phase is 0 by default. Any caller that COMPUTES an angle
can legitimately arrive at zero, so the framework helper absorbs it and returns early; do not guard
it again at the call site.

**Proof.** `stepMeshRotation` places the body by the half-pitch rotation and asserts the angle is
half a tooth pitch, that the volume is unchanged, that the centroid lands where the rotation puts it,
and that its distance from the shaft axis is unchanged, so the body turns and does not travel. The
zero-phase case is asserted as a no-op: nothing is placed and the body stays where it was built.
**The cost is the before reading**: half a tooth pitch is less than a whole one, so a body and its
rotated self overlap and decad refuses to classify that pair however far apart the two are laid,
because the operands are the same recipe and share their face planes — so one side of the comparison
is the closed form the section and the two stations give, which is exactly what the body was built
from.

<!-- proof-run: proofkit3d.RunSolidParallel(bevelMeshCases, stepMeshRotation, assertMeshRotation) -->

**From:** `spec/bevelgear/instructions.md` L446, L886

## S26 `[PROSE]` Move the finished bodies into the gear component

Relocate this gear's finished body into its own `{gearLabel} Gear` component with
`gearBody.moveToComponent(gearOccurrence)`, which preserves the world position and needs no
activation ([PB-NO-CROSS-SIBLING]). Every feature ran in the Design component; this is what puts the
visible result where the browser tree says it is.

**From:** `spec/bevelgear/instructions.md` L842

## S27 `[PROSE]` Cleanup

Call `hide_construction_geometry(self.bevelComponent)` from `.solids` — call it, do not re-implement
the walk. It recursively walks the Bevel Gear component tree, deduping by `entityToken`, and hides
every sketch, construction plane and construction axis by setting `isLightBulbOn` to `False`.
**Construction planes and axes are NOT hidden by `isVisible`** — that is a Fusion gotcha;
`isVisible = False` hides sketches and `isLightBulbOn = False` hides construction planes and axes, so
do not cross them ([PB-HIDE-AFTER-USE], [PB-TREE-CLEANUP], [BEVEL-F-CLEANUP]). There is no sketch-only
mode and no per-mode guard: bevel always builds solids. Leave only the two finished gear bodies
visible.

The driving gear's half-tooth-pitch meshing rotation is NOT a cleanup step; it happened in S25, in
the Design component, before the body was moved out.

`deleteComponent()` is the error rollback the entry point calls on an exception: it calls
`deleteMe()` on the top occurrence.

**Do not add a call to settle the browser's constraint icons.** Fusion shows a stale constraint icon
for every sketch a generator authors, until something makes it settle, so a finished, fully
constrained sketch still shows the unconstrained icon ([PB-SETTLE-DISPLAY]). The icon is not
evidence — measured in Fusion on 2026-09-12, `isFullyConstrained` returned `True` for the Anchor,
Gear Profiles and both per-gear Profile sketches while all four icons showed otherwise. The shared
command wrapper calls `geargen.settle_sketch_display` once after `generate()` returns, so **nothing
about this belongs in this module**.

**From:** `spec/bevelgear/instructions.md` L890–894; `spec/bevelgear/fusion.md` L205–212
