# Spur Gear — compiled step list

One step is one entry in the Fusion timeline. A whole sketch is one step however much geometry
goes into it, and so is each construction plane, construction axis, extrude, chamfer, pattern,
combine and fillet. Two steps carry no timeline entry at all and say so.

`[GO]` marks a step the proof exercises, and names the build function that realises it in
`spurgear_test.go`. Everything three-dimensional is `[PROSE]`: the proof engine models 2D sketches
only, so an extrude or a fillet cannot be exercised there.

## Provenance

| Source | Blob hash |
|---|---|
| `spec/spurgear/instructions.md` | `c805d1026ebf3159e61b4a130dfb4e87f053ea44` |
| `spec/spurgear/fusion.md` | `5708cb98645e094a992e1cc5a79c9e24094ab0b8` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9297782f07cf4a68372a500eaa3b0e35a9d27091` |

## Steps at a glance

| Step | Kind | Timeline entry | Proof function |
|---|---|---|---|
| S1 Read Inputs And Register Parameters | `[PROSE]` | none | — |
| S2 Normalize The Target Plane | `[PROSE]` | construction plane (conditional) | — |
| S3 Tools Sketch | `[GO]` | sketch | `stepToolsSketch` |
| S4 Extrusion End Plane | `[PROSE]` | construction plane | — |
| S5 Gear Profile Sketch | `[GO]` | sketch | `stepGearProfileSketch` |
| S6 Sketch-Only Short-Circuit | `[PROSE]` | none | — |
| S7 Extrude The Tooth | `[PROSE]` | extrude | — |
| S8 Chamfer The Tooth | `[PROSE]` | chamfer (conditional) | — |
| S9 Extrude The Body | `[PROSE]` | extrude | — |
| S10 Gear Center Axis | `[PROSE]` | construction axis | — |
| S11 Pattern The Teeth | `[PROSE]` | circular pattern | — |
| S12 Combine The Teeth Into The Body | `[PROSE]` | combine | — |
| S13 Root Fillets | `[PROSE]` | fillet (conditional) | — |
| S14 Bore Profile Sketch | `[GO]` | sketch (conditional) | `stepBoreProfileSketch` |
| S15 Cut The Bore | `[PROSE]` | extrude-cut (conditional) | — |
| S16 Cleanup | `[PROSE]` | none | — |

---

## S1 `[PROSE]` Read Inputs And Register Parameters

This step adds nothing to the timeline. It runs before every other step and decides what they
build, so the list starts here.

Read the three selection inputs **first** and stash the entities on `self`: `self.parentComponent`
from `parentComponent`, `self.plane` from `plane`, `self.anchorPoint` from `anchorPoint`. Only then
touch anything that creates the occurrence. The reason is a context shift — the first
`parentComponent.occurrences.addNewComponent(...)`, whether called directly through
`getOccurrence()` or transitively through the first `addParameter()` or `parameterName()`, moves
Fusion's active component to the new occurrence, and a `SelectionCommandInput` holding an entity
that lives in another component can drop its selection when that happens. Numeric and boolean
inputs are unaffected. Read a selection with `get_selection(inputs, id)`, a numeric or
string-value input with `get_value(inputs, id, units)`, and a checkbox with
`get_boolean(inputs, id)` — the read helper is fixed by the `add*Input` the dialog used, and
calling `get_value` on a checkbox raises `AttributeError` because `BoolValueCommandInput` has no
`expression`.

The dialog itself is built by `SpurGearCommandInputsConfigurator.configure(cmd)`, in this exact
display order: Target Plane, Anchor Point, Module, Tooth Number, Pressure Angle, Bore Diameter,
Thickness, Apply chamfer to teeth, Generate sketches but do not build body, Parent Component
last. That order is not the `processInputs` read order and must not be rearranged to match it.
Each selection input carries `addSelectionFilter(...)` per filter and
`setSelectionLimits(1, 1)`: Target Plane takes `ConstructionPlanes` and `PlanarFaces`, Anchor Point
takes `ConstructionPoints` and `SketchPoints`, Parent Component takes `Occurrences` and
`RootComponents` and pre-selects `get_design().rootComponent`. Bore Diameter is
`addStringValueInput(id, name, '0 mm')` so it accepts expressions; the rest are
`addValueInput(id, name, unit, ValueInput.createByReal(default))` with the default in Fusion
internal units regardless of the display unit — Pressure Angle is `'deg'` with
`ValueInput.createByReal(math.radians(20))`, Thickness is `'mm'` with
`ValueInput.createByReal(to_cm(10))`, Module and Tooth Number are unitless
`ValueInput.createByReal(1)` and `ValueInput.createByReal(17)`, chamfer is
`ValueInput.createByReal(0)`, and SketchOnly is `addBoolValueInput(...)`.

> **API mismatch (recorded, not corrected).** `[PB-SELECTION-FILTER-ENUM]` says the filter
> arguments are int enum constants and that passing the string `'ConstructionPlanes'` raises a
> `TypeError`. The API index disagrees on both points: `addSelectionFilter` has signature
> `addSelectionFilter(filter: str) -> bool`, and every member of the filter set is a string
> constant equal to its own name (`ConstructionPlanes = 'ConstructionPlanes'`). Writing
> `adsk.core.SelectionCommandInput.ConstructionPlanes` is still correct — it *is* that string — but
> the stated failure mode does not exist.

Register the parameters next, all under the `SpurGear<N>_` prefix, using
`addParameter(name, ValueInput, units, comment)` which reaches
`design.userParameters.add(name, value, units, comment)`. The input-sourced ones come first
(`Module` unitless, `ToothNumber` unitless, `PressureAngle` in `'rad'`, `BoreDiameter`,
`Thickness`, `ChamferTooth`, `SketchOnly` as a real 1 or 0), then the
`addExtraPrimaryParameters(inputs)` hook — a no-op on the spur base, the seam a subclass registers
its own primary parameters through — and then the derived ones as live expression strings via
`ValueInput.createByString(...)`: `PitchCircleDiameter = Module * ToothNumber`,
`PitchCircleRadius`, `BaseCircleDiameter = PitchCircleDiameter * cos(PressureAngle)`,
`BaseCircleRadius`, `RootCircleDiameter = PitchCircleDiameter - 2.5 * Module`, `RootCircleRadius`,
`TipCircleDiameter = PitchCircleDiameter + 2 * Module`, `TipCircleRadius`, `InvoluteSteps` 15,
`FilletClearance` 0.9, and `FilletRadius = (ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>`
where `<factor>` is what `filletHelixFactorExpression()` returns (`'1'` for spur). Two
registrations need care. `Module` is registered **unitless**, not `'mm'`, so `generateName`
renders `M=1` and the `mm`-registered expressions above read it as a bare factor.
`ToothSpaceAngleAtRoot` is pre-computed in Python as `π / ToothNumber − 2 · (tan(PressureAngle) −
PressureAngle)` and registered with `ValueInput.createByReal(...)` **unitless**, because Fusion's
expression engine will not subtract a radian value from the unitless output of `tan()`, and
because the next parameter, `ToothSpaceArcAtRoot = RootCircleRadius * ToothSpaceAngleAtRoot`,
is only accepted as a length when that factor is dimensionless.

Finally name the component: `component.name = self.generateName()`, which returns
`'Spur Gear (M={}, Tooth={}, Thickness={})'` filled from the `.expression` of `Module`,
`ToothNumber` and `Thickness` — the expressions, not the values, so units show.

**From:** `spec/spurgear/instructions.md` 36–88 (Variables), 90–178 (exact input ids, dialog
order, filters, defaults), 232–298 (method contract, `generateName`, `addExtraPrimaryParameters`,
`filletHelixFactorExpression`), 370–379 (Generation Order);
`.claude/skills/generate-gear/PLAYBOOK.md` 53–60, 103–143 (`[PB-INPUT-READ]`,
`[PB-GET-VALUE-CONTRACT]`, `[PB-DIALOG-DEFAULT-UNITS]`, `[PB-SELECTION-DECL]`), 196–218
(`processInputs` pattern), 230–240, 474–479 (`[PB-SELECTION-FILTER-ENUM]`), 556–557
(`[PB-SELECTION-STASH]`).

---

## S2 `[PROSE]` Normalize The Target Plane

Conditional. If `self.plane` is already a `ConstructionPlane` this step is skipped and adds no
timeline entry. Otherwise — the user picked a planar face — build a coplanar construction plane
and use it for everything downstream, so profile detection never has to filter out the selected
face's own profile.

`component.constructionPlanes.createInput()` →
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))` →
`component.constructionPlanes.add(planeInput)`, and replace `self.plane` with the result. The
offset argument is a `ValueInput`, never a bare number: `setByOffset(plane, 0)` is a runtime
`TypeError`. The normalized plane is one of the entities S16 switches the light bulb off on, so
keep a handle to it. Both `self.plane` and `ctx.plane` must end up holding it — subclasses read
`self.plane` directly.

**From:** `spec/spurgear/instructions.md` 39 (Target Plane), 215–230 (`ctx.plane`, `self.plane`),
383–385 (step 1); `.claude/skills/generate-gear/PLAYBOOK.md` 230–240 (generate orchestration),
674–685 (`[PB-CONSTRUCTION-PLANES]`).

---

## S3 `[GO]` Tools Sketch

Proof function: `stepToolsSketch`.

One sketch, named `Tools`, on the target plane, created with
`self.createSketchObject('Tools', self.plane)` which reaches
`component.sketches.add(planarEntity)`. It draws no geometry of its own. Its whole purpose is to
own one entity: the projection of the user's Anchor Point, `sketch.project(self.anchorPoint)`,
kept as `ctx.anchorPoint`. That projection is the canonical handle — every later sketch projects
*it* in again rather than the user's original entity, so the chain of projections keeps the whole
gear tracking the anchor if the anchor moves.

Leave the sketch visible. Later sketches project from it and `sketch.project(...)` has failed on
invisible sketches in this repo's history, and S15 re-projects from it after the body is complete,
which is why S16 rather than this step is where `isVisible = False` goes.

> **API mismatch (recorded, not corrected).** The spec and `[SPUR-F-ANCHOR-CHAIN]` name
> `sketch.project(...)`. `grep '"name":"project"'` over the API index returns zero lines; the
> `Sketch` class in the index carries `project2(entities: list[core.Base], isLinked: bool) ->
> list[SketchEntity]`, `projectCutEdges(body)` and `projectToSurface(...)`, and no `project`. If
> the index is current then this call does not exist, and its replacement takes a **list** and a
> second `isLinked` argument and returns a **list**, so `ctx.anchorPoint` would be the first
> element rather than the return value itself.

> **Spec gap (recorded, not corrected).** Sketch Discipline says every sketch here is fully
> constrained with exactly one exception, the Bore Profile sketch's stray point. The Tools sketch
> is a second exception nobody declared: it contains a single projected point and nothing else,
> and `[PB-PROJECT-NOT-FIXED]` says a projected point is associative but still carries free
> degrees of freedom, so `toolsSketch.isFullyConstrained` is false. The proof models the
> projection as reference geometry — locked from outside, which is what a projection *means* — and
> under that reading the sketch is sound. Under Fusion's reading it is not, and the spec has to
> pick one.

**From:** `spec/spurgear/instructions.md` 41 (Anchor Point), 180–213 (Sketch Discipline), 215–230
(`ctx.anchorPoint`), 232–298 (`prepareTools` owns steps 1–2), 387–389 (step 2, first paragraph);
`spec/spurgear/fusion.md` 19–24 (`[SPUR-F-ANCHOR-CHAIN]`);
`.claude/skills/generate-gear/PLAYBOOK.md` 92–96 (`createSketchObject`), 430–444
(`[PB-PROJECT-NOT-FIXED]`), 558–570 (`[PB-HIDE-AFTER-USE]`).

---

## S4 `[PROSE]` Extrusion End Plane

One construction plane, named `Extrusion End Plane`, offset from the target plane by `Thickness`:
`component.constructionPlanes.createInput()` →
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))` →
`component.constructionPlanes.add(planeInput)`. Again the offset is a `ValueInput`, and again the
number is a snapshot of the parameter's current value, not a live expression.

Its only job is to be the `to-entity` both the tooth extrude (S7) and the body extrude (S9) end
on, so the two bodies share one well-defined end face. Store it as `ctx.extrusionEndPlane`. Leave
it visible while those extrudes run; S16 hides it with `isLightBulbOn = False`, because
`isVisible = False` has no effect on a construction plane.

**From:** `spec/spurgear/instructions.md` 56 (Thickness), 215–230 (`ctx.extrusionEndPlane`),
385 (the `ValueInput` rule extends to this plane), 391 (step 2, second paragraph);
`spec/spurgear/fusion.md` 201–206 (`[SPUR-F-SNAPSHOT]`);
`.claude/skills/generate-gear/PLAYBOOK.md` 220–228 (`[PB-NUMERIC-SNAPSHOT]`), 558–570
(`[PB-HIDE-AFTER-USE]`), 674–685 (`[PB-CONSTRUCTION-PLANES]`).

---

## S5 `[GO]` Gear Profile Sketch

Proof function: `stepGearProfileSketch`.

One sketch, named `Gear Profile`, on the target plane. Everything below happens inside it, so it
is one timeline entry: the four circles, the involute tooth, the ribs, the spine and the
anchoring. It is created by `buildSketches(ctx)` — the method boundary helical overrides and calls
`super()` on — which then runs `SpurGearInvoluteToothDesignGenerator(sketch, self).draw(ctx.anchorPoint, angle=0)`
and afterwards copies `ctx.toothProfileIsEmbedded = self._lastToothEmbedded`.

`draw(anchorPoint, angle=0)` does exactly four things in order: `drawCircles()`, `drawTooth(angle)`,
the anchoring, and then — only when `angle != 0` — setting the confirming angular dimension's
value. `drawTooth` rotates by the `angle` that arrives as `draw`'s argument, never by the
constructor-stored `self.toothAngle`; helical and herringbone construct the generator with the
default `angle=0` and pass the helix angle to `draw`, so reading the stored field would draw a
flat tooth and leave the loft with no twist.

**The constructor.** It adds the movable local origin: a fresh `SketchPoint` at (0, 0, 0) via
`sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`, stored as the field
`self.anchorPoint`. Not `sketch.originPoint`, which is immutable and cannot be
coincident-constrained to anything brought in from elsewhere. Every point below is placed relative
to this one, which is what lets the single coincidence at the end drag the whole drawing onto the
user's anchor as a unit.

**`drawCircles`.** Four circles, all centred on the local origin, added in this order: Root
(**solid**, radius `RootCircleRadius`), Tip (**construction**, `TipCircleRadius`), Base
(**construction**, `BaseCircleRadius`), Pitch (**construction**, `PitchCircleRadius`). Each is
`sketch.sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)` — the local origin
`SketchPoint` passed **directly** as the centre so all four share it, never
`localOrigin.geometry` followed by a coincident, which double-binds the point and kills the solver.
Each gets a driving diameter dimension,
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`, with the text point off the
centre — a text point at the centre is rejected, since there is no radial direction there. Each
circle is also labelled with along-path text: `sketch.sketchTexts.createInput2(text, height)` →
`textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`
→ `sketch.sketchTexts.add(textInput)`. The string is
`'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` with the radii read as internal cm
`.value`, and `size = TipCircleRadius − RootCircleRadius` is passed as the text height too.

> **API mismatch (recorded, not corrected).** `[PB-SKETCH-TEXT]` and this step name
> `sketchTexts.createInput2(text, height)` with a float height. The index shows `SketchTexts`
> carrying only `createInput3(expression: str, height: core.ValueInput) -> SketchTextInput`;
> `createInput2` appears on `ChamferFeatures` and `MoveFeatures` and nowhere else. If the index is
> current, this call does not exist under that name, and the height is a `ValueInput` rather than
> a float.

**`drawTooth(angle)`.** Sample the involute flank with `calculateInvolutePoint(BaseCircleRadius, r)`
at `steps = InvoluteSteps` radii, endpoint-inclusive:
`r = BaseCircleRadius + (TipCircleRadius − BaseCircleRadius) · i / (steps − 1)` for
`i = 0 … steps−1`, so the first sample sits exactly on the base circle and the last exactly on the
tip circle. Do not clamp the start up to the root circle — the embedded case is detected below
from where the flank start lands, not by trimming the sampling. A sample whose radius is inside
the base circle returns `None` and is dropped. The point math itself is
`alpha = acos(baseRadius / intersectionRadius)`, `t = tan(alpha)`,
`x = baseRadius · (cos t + t · sin t)`, `y = baseRadius · (sin t − t · cos t)` — the curve
parameter is `tan(alpha)`, not `inv(alpha) = tan(alpha) − alpha`, which is the common substitution
and produces a mis-parameterised flank.

Then, in order: mirror every sample across +X (negate y), because the standard parametric involute
spirals the wrong way for a left flank and drawn as-is gives a tooth wider at the tip than at the
root; rotate all of them by
`rotate_angle = π / (2 · ToothNumber) − atan2(−py, px)` where `(px, py) =
calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`, which lands the pitch-circle crossing
at half a tooth width above +X and is computed analytically rather than interpolated from the
samples, so few samples still place the tooth correctly; mirror the result across X to get the
right flank; and rotate **both** flanks by the requested `angle`, along with the tooth-top point
and the rib midpoint seeds. The tooth is drawn at its final angular position. Leaving it at +X and
letting the spine's angular dimension swing it there afterwards is what puts Fusion on the ~180°-off
solver branch.

Draw each flank as `sketch.sketchCurves.sketchFittedSplines.add(fitPointCollection)`, where the
collection is an `adsk.core.ObjectCollection.create()` of `SketchPoint`s.

**Tooth-top arc.** Materialize a tooth-top `SketchPoint` at
`(TipCircleRadius · cos(angle), TipCircleRadius · sin(angle))` and constrain it with
`sketch.geometricConstraints.addCoincident(toothTopPoint, tipCircle)`. Then
`sketch.sketchCurves.sketchArcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)`,
passing the local origin and both splines' end `SketchPoint`s directly so the arc shares all
three. Add **no** diameter dimension. A free centre plus a diameter fixes the arc's size but not
which way it curves, and the inward-bulging arc through the same two ends satisfies the same
numbers, so the sketch reaches DOF 0 with two answers and the solver picks by seed. Sharing the
centre removes the choice, and it is also why the last rib below carries no perpendicular.

> **Spec contradiction (recorded, not corrected).** Sketch Discipline lists the tooth-top arc
> among the diameter dimensions that "must be driving", which reads as an instruction to
> dimension it. `[SPUR-F-TOOTHTOP-ARC]` step 3 says to add no diameter dimension at all, and gives
> the failure that follows from adding one. The proof follows `fusion.md`.

**Spine, +X reference and the angular pin.** The spine is a construction line from the local origin
to the tooth-top point: `sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, toothTopPoint)`
with both existing `SketchPoint`s passed directly and `line.isConstruction = True`. No separate
start-coincident, and no constraint putting its end on the arc. Build the +X reference line for
**every** angle including 0: add a far endpoint at `(TipCircleRadius, 0)`, pin it with two signed
dimensions from the local origin —
`sketch.sketchDimensions.addDistanceDimension(localOrigin, refEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
at `TipCircleRadius` and the same with
`adsk.fusion.DimensionOrientations.VerticalDimensionOrientation` at 0 — draw the line, mark it
construction, and add
`sketch.sketchDimensions.addAngularDimension(referenceLine, spine, textPoint)` in that argument
order, with the text point on the bisector `(R·cos(angle/2), R·sin(angle/2))` for a small R so
Fusion selects the angle rather than its supplement. A plain `addHorizontal` on the spine is not a
substitute at `angle = 0`: horizontal fixes the direction but says nothing about the sense, so the
tooth top can settle at either end of the tip circle and the tooth comes out 180° around.

**Ribs.** One construction line per fit-point index, for **all** N indices including the first
(base-circle) and last (tip) pairs — the fit points carry no other constraint, so a missing rib
leaves one free and the sketch never closes. Build each rib in exactly this order, because a
different order over-constrains it:

1. `sketch.sketchCurves.sketchLines.addByTwoPoints(leftSpline.fitPoints.item(i), rightSpline.fitPoints.item(i))`,
   passing the fit-point `SketchPoint`s directly; mark it construction.
2. Dimension it with a **signed** `addDistanceDimension` — vertical orientation at the measured Δy
   for `angle = 0`. An aligned dimension gives only the length, which the left and right flanks
   satisfy equally well swapped over, and the tooth comes out mirrored.
3. Add a fresh midpoint `SketchPoint`, seeded **already on the spine**: with
   `t = fitX·cos(angle) + fitY·sin(angle)`, at `(t·cos(angle), t·sin(angle))`. Not at the rib's
   true 2-D midpoint, and not at `(fitX, 0)` for a rotated tooth.
4. `sketch.geometricConstraints.addCoincident(midpoint, spine)` — onto the spine first.
5. `sketch.geometricConstraints.addMidPoint(midpoint, rib)` — then the rib's midpoint.
6. `sketch.geometricConstraints.addPerpendicular(spine, rib)` — then perpendicular. **Skip this on
   the last rib**, whose two ends the tooth-top arc already holds at equal radius either side of
   the spine; keeping it throws `VCS_SKETCH_OVER_CONSTRAINTS`.

Then chain the midpoints: a signed `addDistanceDimension` from each rib's midpoint to the
previous one along the spine direction (horizontal for `angle = 0`), with the **first** link
running from the local origin to rib 0's midpoint. Without that first link the whole chain slides
along the spine as a unit and the sketch keeps one degree of freedom.

> **Spec gap (recorded, not corrected).** For a rotated tooth `[SPUR-F-RIBS]` says to use
> "whichever of the horizontal/vertical pair is better conditioned for that angle" and leaves the
> rule at that; the chain dimension is pinned only for `angle = 0`. Two builds can differ here and
> both claim to follow the spec. The proof resolves it with one predicate — when `|cos(angle)| ≥
> |sin(angle)|` the spine is nearer horizontal, so the chain is dimensioned horizontally and the
> ribs vertically, and otherwise the other way round — and proves it at 0°, 30°, 90° and 145°.

**Flank-to-root.** Compare the distance from the local origin to the left flank's first fit point
against `RootCircleRadius`, strictly: `embedded = firstRadius < RootCircleRadius`, raw values, no
tolerance. Exact equality counts as **not** embedded and draws a zero-length stub, which is the
ill-conditioned region the bench proof is there to flag; do not soften it to `<=`.

When not embedded, draw one short radial line per side with
`sketch.sketchCurves.sketchLines.addByTwoPoints(rootEndGeometry, flankStartFitPoint)` — the
spline's start `SketchPoint` passed directly — and place the root end with **exactly two** signed
dimensions from the local origin, `addDistanceDimension(..., HorizontalDimensionOrientation, ...)`
at its Δx and the vertical one at its Δy, and no others. Placing it with "root end on the root
circle" plus "local origin on the line" instead admits the far intersection as well: the line
through the flank start and the centre meets the root circle again on the other side, the stub
becomes a line straight across the gear, and the sketch reaches DOF 0 with both answers live. The
tooth loop then has **6 curves** (2 splines, 2 stubs, 2 arcs). When embedded, no stub is drawn and
the loop has **4** (2 splines, 2 arcs). Record which by writing
`self.parent._lastToothEmbedded` from inside `drawTooth` — the tooth generator cannot reach `ctx`
— which `SpurGearGenerator.__init__` must pre-initialise to `False`.

**Anchoring.** The last structural act of `draw()`: project the Tools-sketch anchor into this
sketch, `sketch.project(ctx.anchorPoint)`, and add
`sketch.geometricConstraints.addCoincident(projectedPoint, localOrigin)` between that fresh
projection and the tooth generator's local origin — not `sketch.originPoint`. Because everything
above is placed relative to the local origin, this one constraint slides the entire drawing onto
the anchor. It happens inside `draw()`, not in `buildSketches` afterwards, because helical and
herringbone call `draw()` directly on their twisted loft sketch and rely on that single call to
anchor it.

Then, and only when `angle != 0`, set the confirming rotation as the very last action of all:
`spineAngularDimension.parameter.value = angle`. Drawing the tooth pre-rotated and setting this
value are not alternatives — the pre-rotation puts the geometry on the intended solver branch and
the value-set locks it there.

**What the proof does not model.** The along-path circle labels: sketch text is annotation, it
bounds no region and carries no constraint. Everything else in this step is modelled, including
the projections, which the proof represents as reference geometry — a point positioned from
outside the sketch, which is what a projection is.

**From:** `spec/spurgear/instructions.md` 180–213 (Sketch Discipline), 215–230
(`ctx.gearProfileSketch`, `ctx.toothProfileIsEmbedded`), 268–285 (`buildSketches` boundary),
300–341 (tooth-generator surface and the exact involute math), 393–402 (step 3), 404–440 (step 4),
442–446 (step 5); `spec/spurgear/fusion.md` 26–43 (`[SPUR-F-LOCAL-ORIGIN]`,
`[SPUR-F-SHARED-ADJACENCY]`), 47–60 (`[SPUR-F-ROTATE-CONFIRM]`), 69–87
(`[SPUR-F-TOOTHTOP-ARC]`), 89–112 (`[SPUR-F-SPINE]`), 114–147 (`[SPUR-F-RIBS]`), 149–183
(`[SPUR-F-FLANK-ROOT]`); `.claude/skills/generate-gear/PLAYBOOK.md` 413–450
(`[PB-FULL-CONSTRAINT]`, `[PB-CIRCLE-CENTER]`, `[PB-NO-OVERCONSTRAIN]`), 458–473
(`[PB-API-SPELLING]`, `[PB-SKETCHCURVES]`), 517–535 (`[PB-SEED-NEAR]`,
`[PB-SHARE-XOR-COINCIDENT]`, `[PB-DRIVING-DIM]`), 547–555 (`[PB-ANGULAR-DIM]`,
`[PB-RADIAL-DIM]`), 581–591 (`[PB-SKETCH-TEXT]`).

---

## S6 `[PROSE]` Sketch-Only Short-Circuit

No timeline entry — a branch, listed because it decides whether S7 through S15 run at all.

If the `SketchOnly` parameter reads true (`getParameterAsBoolean`), set the Gear Profile sketch's
`isVisible = True` and return from `buildMainGearBody` immediately. No tooth extrude, no body, no
pattern, no fillet, no bore. `buildBore` still gets called from `generate()` and guards itself
(S14), and `cleanup` still runs unconditionally (S16) with its own per-mode split.

**From:** `spec/spurgear/instructions.md` 60 (the input), 232–266 (call graph; `cleanup` is
unconditional and last), 448–450 (step 6); `spec/spurgear/fusion.md` 187–197
(`[SPUR-F-CLEANUP]`).

---

## S7 `[PROSE]` Extrude The Tooth

One extrude. Find the single tooth cross-section with
`find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`
— the framework helper, which rejects loops whose counts do not match and raises when nothing
does. Do not re-implement the search.

`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`
→ `extrudeInput.setOneSideExtent(adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False), adsk.fusion.ExtentDirections.PositiveExtentDirection)`
→ `component.features.extrudeFeatures.add(extrudeInput)`. Name the feature `Extrude tooth` and
store the resulting body as `ctx.toothBody`.

This step belongs to `buildTooth(ctx)`, which **must** call `self.chamferTooth(ctx)` as its last
action. Helical overrides `buildTooth` to loft instead of extrude and herringbone to loft and
mirror, and both still end by calling `chamferTooth`, so `buildMainGearBody` must not chamfer
separately.

**From:** `spec/spurgear/instructions.md` 215–230 (`ctx.toothBody`), 268–285 (`buildTooth`
boundary), 432–440 (the 6-curve / 4-curve loop), 452–456 (step 7);
`.claude/skills/generate-gear/PLAYBOOK.md` 151–155 (`find_profile_by_curve_counts`), 571–580
(`[PB-PROFILE-MATCH]`).

---

## S8 `[PROSE]` Chamfer The Tooth

Conditional on `ChamferTooth > 0`; otherwise no timeline entry.

Find the front face with a **single conjunction predicate**: walk `ctx.toothBody.faces` and take
the first face for which **both** `face.edges.count == self.chamferWantEdges()` (6 on the spur
base) **and** `sketchPlane.isCoPlanarTo(face.geometry)`, with
`sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`. Both of the same face — this is not
an edge-count match with a coplanarity tiebreak. If no face satisfies both, raise; do not fall back
to a partial match. An embedded profile yields a 4-edge front face while `chamferWantEdges()` stays
6, so chamfering an embedded spur tooth raises here. That is a known and accepted limitation:
users turn chamfer off for such gears.

Then collect the edges. Walk the front face's edges and add each to an
`adsk.core.ObjectCollection.create()`, skipping any edge whose
`edge.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType` and whose
`edge.geometry.radius` equals `RootCircleRadius` within `0.001` cm. Identify the root arc by that
radius match, not by picking the smallest arc: it is the only edge lying on the root circle, so the
match is exact. Skipping it is the point of the step — chamfering the root arc would eat into the
neighbouring tooth. The two flanks, the tooth-top arc and both flank-to-root lines are chamfered.

`component.features.chamferFeatures.createInput2()` →
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(chamferValue), False)`
→ `component.features.chamferFeatures.add(chamferInput)`.

> **API note (recorded, not corrected).** The index gives the third parameter of
> `addEqualDistanceChamferEdgeSet` as `isTangentChain`, while `[PB-FILLET-CHAMFER]` calls it
> `isFlipped`. The value passed is `False` either way, so nothing changes here, but the playbook's
> name for the argument is not the API's.

Subclasses override only `chamferWantEdges()` and change nothing else about this step.

**From:** `spec/spurgear/instructions.md` 58 (the input), 268–285 (`chamferWantEdges` hook),
458–464 (step 8); `.claude/skills/generate-gear/PLAYBOOK.md` 480–486 (`[PB-FILLET-CHAMFER]`),
571–580 (`[PB-PROFILE-MATCH]`, curve-type constant names).

---

## S9 `[PROSE]` Extrude The Body

One extrude. Find the gear-body profile with `find_profile_by_curve_counts(sketch, arcs=2)`, then
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`
→ `extrudeInput.setOneSideExtent(adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False), adsk.fusion.ExtentDirections.PositiveExtentDirection)`
→ `component.features.extrudeFeatures.add(extrudeInput)`. Name the feature `Extrude body`, name
the body `Gear Body`, and store it as `ctx.gearBody`.

While iterating `extrude.bodies.item(0).faces`, capture `ctx.extrusionExtent` — the far end cap,
which S15 cuts the bore down to. Classify by `face.geometry.surfaceType`: among the faces whose
type is `adsk.core.SurfaceTypes.PlaneSurfaceType`, take the one where
`sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry)`,
using the plane-geometry API rather than a hand-rolled dot product. The near cap is ruled out by
being coplanar; the cylindrical face is not planar at all. Raise if it is not found.

> **Spec error (recorded, not corrected).** The step describes the profile as "the annular loop
> bounded by exactly 2 arcs (the root circle and the tip circle)". The tip circle is construction
> geometry (S5), so it bounds no profile, and the loop is not annular — it is a disc. The two arcs
> are the two fragments the tooth's flank-to-root lines (or, in the embedded case, its flanks)
> split the **root circle** into. The call itself, `find_profile_by_curve_counts(sketch, arcs=2)`,
> is right; only the explanation of which two arcs is wrong. The proof checks this directly: it
> draws no root arc by hand, lets region detection split the root circle, and asserts that the
> tooth region comes back bounded by the 6 (or 4) entities S7 expects and the body region by the
> root circle alone.

**From:** `spec/spurgear/instructions.md` 215–230 (`ctx.gearBody`, `ctx.extrusionExtent`),
466–475 (step 9); `.claude/skills/generate-gear/PLAYBOOK.md` 151–155
(`find_profile_by_curve_counts`), 410 (`[PB-ADSK-MODULES]`, `SurfaceTypes` is `adsk.core`),
639–660 (`[PB-FACE-BY-MIDPOINT]`, face classification by surface type).

---

## S10 `[PROSE]` Gear Center Axis

One construction axis, built from the same face walk as S9. Take any face whose
`face.geometry.surfaceType` is `adsk.core.SurfaceTypes.CylinderSurfaceType`, then
`component.constructionAxes.createInput()` → `axisInput.setByCircularFace(cylindricalFace)` →
`component.constructionAxes.add(axisInput)`. Name it `Gear Center`, set `isLightBulbOn = False`,
and store it as `ctx.centerAxis`. Raise if no cylindrical face is found.

**From:** `spec/spurgear/instructions.md` 215–230 (`ctx.centerAxis`), 470–473 (step 9, the axis
bullet); `.claude/skills/generate-gear/PLAYBOOK.md` 558–570 (`[PB-HIDE-AFTER-USE]`), 686–689
(`[PB-CONSTRUCTION-AXES]`).

---

## S11 `[PROSE]` Pattern The Teeth

One circular pattern. `component.features.circularPatternFeatures.createInput(inputEntities, ctx.centerAxis)`
→ `patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)` →
`patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')` →
`patternInput.isSymmetric = False` →
`component.features.circularPatternFeatures.add(patternInput)`. Pin all three explicitly rather
than trusting Fusion's defaults to stay equal to them.

> **Spec gap (recorded, not corrected).** The step says to circular-pattern `ctx.toothBody` and
> does not say how it reaches the input. `createInput`'s first parameter is
> `inputEntities: core.ObjectCollection`, so the single body has to be wrapped —
> `adsk.core.ObjectCollection.create()` then `.add(ctx.toothBody)`. The spec spells out the
> matching wrap on the combine side (`[PB-PATTERN-BODIES]`) but not this one.

**From:** `spec/spurgear/instructions.md` 268–285 (`patternTeeth` boundary), 477–479 (step 10);
`.claude/skills/generate-gear/PLAYBOOK.md` 597–602 (`[PB-CIRCULAR-PATTERN]`).

---

## S12 `[PROSE]` Combine The Teeth Into The Body

One combine, join. Copy `pattern.bodies` into a fresh `adsk.core.ObjectCollection.create()` one
item at a time with `collection.add(pattern.bodies.item(i))` — `pattern.bodies` is a `BRepBodies`
and `createInput` rejects it — and pass that collection as-is. It already contains the seed tooth
along with the copies, so do not re-add `ctx.toothBody`.

`component.features.combineFeatures.createInput(ctx.gearBody, toolBodies)` →
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation` →
`component.features.combineFeatures.add(combineInput)`.

**From:** `spec/spurgear/instructions.md` 479–481 (step 10, the combine and the bodies note);
`.claude/skills/generate-gear/PLAYBOOK.md` 592–596 (`[PB-PATTERN-BODIES]`).

---

## S13 `[PROSE]` Root Fillets

Conditional on `FilletRadius > 0`, and further conditional on the edge collection coming back
non-empty; otherwise no timeline entry.

Round the inside corner where each valley floor meets a tooth flank — the one that runs the full
thickness parallel to the gear axis, where bending stress concentrates. Not the front and back rim,
which is a cosmetic rounding nobody asked for here. Two things make the edge pick fiddly.

First, after the pattern and combine the root cylinder is usually split into one patch per valley
rather than one continuous surface, so collect **every** cylindrical face whose radius equals
`RootCircleRadius`, not the first one found.

Second, on each such face keep only the axial straight edges. Filter to
`edge.geometry.curveType == adsk.core.Curve3DTypes.Line3DCurveType`, take each line's direction
from its **geometry endpoints** — `edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)` —
normalize, and keep it when `abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01`. Use exactly
that tolerance: a tighter test drops valid axial edges that are slightly off from tessellation and
leaves root fillets missing. Do not read the direction with `edge.evaluator.getTangent(0)` —
parameter 0 is not guaranteed to lie inside the edge's parameter range and Fusion raises
`RuntimeError: invalid argument parameter`.

If the collection ends up empty, return silently without creating the feature. An empty edge set
must not reach `filletFeatures.add`. Otherwise
`component.features.filletFeatures.createInput()` →
`filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), False)`
→ `component.features.filletFeatures.add(filletInput)`. `isTangentChain` must be `False`: the
collected edges are exactly the axial root corners, and tangent-chaining would let Fusion pull in
tangent-adjacent edges and round more than the intended corner. The radius is read as the
`FilletRadius` parameter's numeric `.value`; `filletHelixFactorExpression()` is **not** consulted
here — it was spliced into that parameter's expression back in S1.

> **API mismatch (recorded, not corrected).** The spec and `[PB-FILLET-CHAMFER]` both insist the
> edge set goes on the fillet input **itself**, and explicitly forbid routing it through
> `filletInput.edgeSetInputs` on the grounds that reaching for that collection fails. The index
> says the opposite: `FilletFeatureInput` has no `addConstantRadiusEdgeSet` — its members are
> `isRollingBallCorner`, `targetBaseFeature` and `edgeSetInputs` — and
> `addConstantRadiusEdgeSet(entities: core.ObjectCollection, radius: core.ValueInput, isTangentChain: bool)`
> is owned by `FilletEdgeSetInputs`, which is exactly what `filletInput.edgeSetInputs` returns. If
> the index is current, the forbidden route is the only one that exists.

**From:** `spec/spurgear/instructions.md` 86–88 (Fillet Clearance, Fillet Radius), 268–285
(`createFillets` reads only the parameter value), 483–492 (step 11);
`.claude/skills/generate-gear/PLAYBOOK.md` 412 (`[PB-EMPTY-RESULT]`), 480–486
(`[PB-FILLET-CHAMFER]`), 571–580 (`[PB-PROFILE-MATCH]`, curve-type constant names).

---

## S14 `[GO]` Bore Profile Sketch

Proof function: `stepBoreProfileSketch`.

Conditional. `buildBore(ctx)` runs unconditionally from `generate()`, after `buildMainGearBody`,
so it must early-return in **two** cases: when `SketchOnly` is set, and when `BoreDiameter <= 0`.
The `SketchOnly` guard is the essential one — in that mode `buildMainGearBody` returned before
`buildBody`, so `ctx.gearBody` and `ctx.extrusionExtent` are still `None` and the cut would
dereference them. Do not lean on the bore diameter being 0 in sketch-only mode; the user may have
set both.

Otherwise one sketch, named `Bore Profile`, on the target plane. Draw the bore circle by
instantiating the tooth generator on it — `SpurGearInvoluteToothDesignGenerator(boreSketch, self)`
— and calling `drawBore(ctx.anchorPoint, boreDiameter)`, which projects the anchor in with
`sketch.project(ctx.anchorPoint)`, draws a non-construction
`sketch.sketchCurves.sketchCircles.addByCenterRadius(projectedAnchor, boreDiameter / 2)` centred on
that projection, gives it a driving
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`, and returns the circle.

The sketch carries one stray point: the tooth generator's constructor always adds its local origin
`SketchPoint` at (0, 0, 0), and `drawBore` never touches it. Keep it — this is faithful behaviour,
not something to suppress. It is placed, not free: the constructor puts it at the sketch origin and
nothing moves it, which is why it is the declared exception to the full-constraint rule. The proof
models it the same way, as a point locked from outside.

**From:** `spec/spurgear/instructions.md` 54 (Bore Diameter), 208–210 (the declared exception),
232–266 (`buildBore` runs last, unconditionally), 319–324 (`drawBore` surface), 494–498 (step 12);
`spec/spurgear/fusion.md` 19–31 (`[SPUR-F-ANCHOR-CHAIN]`, `[SPUR-F-LOCAL-ORIGIN]`);
`.claude/skills/generate-gear/PLAYBOOK.md` 430–444 (`[PB-PROJECT-NOT-FIXED]`), 534–535
(`[PB-DRIVING-DIM]`), 551–555 (`[PB-RADIAL-DIM]`).

---

## S15 `[PROSE]` Cut The Bore

One extrude-cut, under the same two guards as S14.

`component.features.extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)`
→ `extrudeInput.participantBodies = [ctx.gearBody]` →
`extrudeInput.setOneSideExtent(adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False), adsk.fusion.ExtentDirections.PositiveExtentDirection)`
→ `component.features.extrudeFeatures.add(extrudeInput)`. Cutting to the far end-cap face rather
than a distance is what guarantees the bore goes all the way through whatever `Thickness` is.

**From:** `spec/spurgear/instructions.md` 215–230 (`ctx.extrusionExtent`), 494–498 (step 12, last
sentences); `.claude/skills/generate-gear/PLAYBOOK.md` 220–228 (`[PB-NUMERIC-SNAPSHOT]`).

---

## S16 `[PROSE]` Cleanup

No timeline entry — visibility flags only. `cleanup(ctx)` is the very last action of `generate()`,
after `buildBore`, and it is called **unconditionally** in both modes. The mode split lives inside
it, not around the call. Position matters: `buildBore` re-projects `ctx.anchorPoint` out of the
Tools sketch, and projection fails once that sketch is hidden, so the Tools sketch has to stay
visible through S15.

Construction geometry, **always, in both modes**: `isLightBulbOn = False` on the Extrusion End
Plane, on the `Gear Center` axis, and on the normalized target plane if S2 created one. Sketches,
**only on the full-build path**: `isVisible = False` on Tools, Gear Profile and Bore Profile.
Sketch-only mode leaves Tools and Gear Profile visible, which is the entire point of that mode.
Guard each entity individually — the `Gear Center` axis and the Bore Profile sketch do not exist in
sketch-only mode. Never cross the two properties: `isVisible` hides sketches and has no effect on
construction geometry; `isLightBulbOn` hides construction planes and axes.

**From:** `spec/spurgear/instructions.md` 204–205 (Sketch Discipline), 256–266 (`cleanup` is last
and unconditional), 448–450 (step 6, sketch-only leaves sketches visible);
`spec/spurgear/fusion.md` 187–197 (`[SPUR-F-CLEANUP]`);
`.claude/skills/generate-gear/PLAYBOOK.md` 558–570 (`[PB-HIDE-AFTER-USE]`).
