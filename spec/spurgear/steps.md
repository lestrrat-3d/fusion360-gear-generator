# Spur Gear — build steps

One step per Fusion timeline entry. A step tagged `[GO]` is one the proof exercises and names the
proof function that realises it; everything else is `[PROSE]`, because the proof engine models 2D
sketches only and every remaining step is 3D or is a visibility toggle.

The proof is `.tmp/spurgear_test.go`, package `spurgear_test`.

## Sources

| Source | Blob hash |
|---|---|
| `spec/spurgear/instructions.md` | `7fe74f0e8de83703225ad8aadccc2d261425dedf` |
| `spec/spurgear/fusion.md` | `5708cb98645e094a992e1cc5a79c9e24094ab0b8` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9297782f07cf4a68372a500eaa3b0e35a9d27091` |

Marks used below: **⚑ API** flags a call the spec names that the Fusion API index does not carry, or
carries with different arguments. The step keeps the spec's wording and states the index's finding
next to it; it does not quietly substitute one for the other.

---

## S1 `[PROSE]` Read the Dialog, Create the Gear Component, Register Parameters

Read the three selection inputs first — `parentComponent`, `plane`, `anchorPoint` — with
`get_selection(inputs, id)`, and stash the entities on `self` (`self.parentComponent`, `self.plane`,
`self.anchorPoint`). Nothing before this may touch the design: the first
`parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` — reached directly through
`getOccurrence()` or transitively through `parameterName()` / `addParameter()` — shifts Fusion's
active component context, and a `SelectionCommandInput` holding an entity that lives in another
component can drop its selection when that happens. Numeric and boolean inputs are immune.

Then create the occurrence and name the component with `generateName()`:
`'Spur Gear (M={}, Tooth={}, Thickness={})'` filled from the `Module`, `ToothNumber` and `Thickness`
parameters' `.expression` strings, not their `.value`.

Register the input-sourced parameters through `design.userParameters.add(name, ValueInput, units, comment)`
(via `base.Generator.addParameter`), reading each input with the helper that matches how it was
declared: `get_value(inputs, id, units)` for the value inputs, `get_boolean(inputs, id)` for the
checkbox. `Module` is registered **unitless** (`''`), not `'mm'`. `PressureAngle` is stored in
radians. `SketchOnly` is stored as a real 1/0 and read back with `getParameterAsBoolean`.

Call the `addExtraPrimaryParameters(self, inputs)` hook here — a no-op on the spur base, the seam a
subclass uses — **between** the input-sourced parameters and the derived ones.

Register the derived parameters as live expressions with `adsk.core.ValueInput.createByString(...)`:
`PitchCircleDiameter = Module * ToothNumber`, `PitchCircleRadius`, `BaseCircleDiameter = PitchCircleDiameter * cos(PressureAngle)`,
`BaseCircleRadius`, `RootCircleDiameter = PitchCircleDiameter - 2.5 * Module`, `RootCircleRadius`,
`TipCircleDiameter = PitchCircleDiameter + 2 * Module`, `TipCircleRadius`, `InvoluteSteps = 15`,
`FilletClearance = 0.9`, `ToothSpaceArcAtRoot = RootCircleRadius * ToothSpaceAngleAtRoot`, and
`FilletRadius = (ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>` where `<factor>` is
`filletHelixFactorExpression()` (spur base: `'1'`) spliced in as the last factor.

`ToothSpaceAngleAtRoot` is the one that cannot be an expression: Fusion refuses to subtract a
radian-valued `PressureAngle` from the unitless output of `tan()`. Compute
`π / ToothNumber − 2 · (tan(PressureAngle) − PressureAngle)` in Python and register it with
`adsk.core.ValueInput.createByReal(...)`, **unitless** (`''`) and not `'rad'`, so the `mm` product
below it is accepted.

Dialog surface, added by `SpurGearCommandInputsConfigurator.configure(cls, cmd)` in exactly this
order — `plane`, `anchorPoint`, `module`, `toothNumber`, `pressureAngle`, `boreDiameter`,
`thickness`, `chamferTooth`, `sketchOnly`, `parentComponent` last. Selections use
`cmd.commandInputs.addSelectionInput(id, name, commandPrompt)` +
`selectionInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)` (and
`PlanarFaces`, `ConstructionPoints`, `SketchPoints`, `Occurrences`, `RootComponents`) +
`selectionInput.setSelectionLimits(1, 1)`. Value inputs use
`cmd.commandInputs.addValueInput(id, name, unitType, adsk.core.ValueInput.createByReal(default))`
with the default in internal units — `math.radians(20)` for the `'deg'` pressure angle, `to_cm(10)`
for the `'mm'` thickness. Bore Diameter is
`cmd.commandInputs.addStringValueInput(id, name, '0 mm')`; the checkbox is
`cmd.commandInputs.addBoolValueInput(id, name, True, '', False)`.

⚑ API — `addSelectionFilter` is indexed as `addSelectionFilter(filter: str) -> bool`, a **string**
parameter, while `[PB-SELECTION-FILTER-ENUM]` requires the `adsk.core.SelectionCommandInput.<Member>`
enum constant and says a string raises `TypeError`. The members do exist in the index as enum members
of `SelectionCommandInput`. The step keeps the playbook's enum form.

**From:** `spec/spurgear/instructions.md` 36–88, 90–178, 217–232, 234–301, 372–381;
`.claude/skills/generate-gear/PLAYBOOK.md` 53–74, 75–102, 103–127, 128–137, 138–147, 196–219,
230–241, 474–479.

---

## S2 `[PROSE]` Normalize the Target Plane

If the user's Target Plane selection is already a `ConstructionPlane`, use it. Otherwise (a planar
face, say) build a coplanar construction plane from it and use that instead, so downstream profile
detection never has to filter out the selected face's own profile:

`component.constructionPlanes.createInput()` →
`planeInput.setByOffset(selectedPlane, adsk.core.ValueInput.createByReal(0))` →
`component.constructionPlanes.add(planeInput)`.

The offset argument is a `ValueInput`, never a bare number; `setByOffset(plane, 0)` is a runtime
`TypeError`. The result is held on both `self.plane` and `ctx.plane`. If a plane was created here,
step S16 switches its light bulb off.

**From:** `spec/spurgear/instructions.md` 385–387, 217–232;
`.claude/skills/generate-gear/PLAYBOOK.md` 230–241, 674–685.

---

## S3 `[GO]` Tools Sketch

Proof: `stepToolsSketch`.

Create the sketch on the target plane with `base.Generator.createSketchObject('Tools', self.plane)`
(which wraps `component.sketches.add(plane)` and sets `sketch.name`), then project the user's Anchor
Point into it and keep the resulting `SketchPoint` as `ctx.anchorPoint`. That projection is the
canonical handle: the Gear Profile and Bore Profile sketches each project *it* in again, chaining
back to the user's original anchor entity so the whole gear follows if the anchor moves.

`sketch.project(self.anchorPoint)`

The sketch draws nothing else. Leave `sketch.isVisible = True` while the later sketches are still
projecting from it — projection fails on a hidden sketch, which is why S16 runs after the bore and
not before it.

⚑ API — the index has no `Sketch.project`. What it carries is
`sketch.project2(entities: list[core.Base], isLinked: bool) -> list[SketchEntity]`. The spec names
`project`, `[SPUR-F-ANCHOR-CHAIN]` names `sketch.project(...)`, and `[PB-PROJECT-NOT-FIXED]` names it
again; the step keeps that name.

In the proof, the projection is a `CreateReferencePoint` — coordinates placed by the layer above and
never moved by the solver, which is exactly what a projection is. The proof carries the anchor's
sketch-local position as a case parameter so the off-origin anchor is exercised too.

**From:** `spec/spurgear/instructions.md` 389–391, 217–232, 192–194;
`spec/spurgear/fusion.md` 19–24; `.claude/skills/generate-gear/PLAYBOOK.md` 91–96, 430–444, 558–570.

---

## S4 `[PROSE]` Extrusion End Plane

Create the offset construction plane `Extrusion End Plane` at distance `Thickness` from the target
plane. Both the tooth extrude (S7) and the body extrude (S9) end on it, so the two land on one
well-defined face.

`component.constructionPlanes.createInput()` →
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))` →
`component.constructionPlanes.add(planeInput)` → name it `Extrusion End Plane` → store on
`ctx.extrusionEndPlane`.

The offset is a `ValueInput` carrying the parameter's current numeric value in cm, not a live
expression. Leave the plane visible while the extrudes run; S16 sets `isLightBulbOn = False`.

**From:** `spec/spurgear/instructions.md` 393, 385–387, 217–232, 213–215;
`spec/spurgear/fusion.md` 201–206; `.claude/skills/generate-gear/PLAYBOOK.md` 220–228, 674–685.

---

## S5 `[GO]` Gear Profile Sketch

Proof: `stepGearProfileSketch`.

One timeline entry covers the spec's steps 3, 4 and 5 together, because the tooth generator's
`draw(anchorPoint, angle=0)` runs all three inside this one sketch:
`drawCircles()`, then `drawTooth(angle)`, then the anchoring, then — for `angle != 0` only, and as
its very last action — the confirming angular dimension's value-set.

Create the sketch with `createSketchObject('Gear Profile', self.plane)` and construct
`SpurGearInvoluteToothDesignGenerator(sketch, self)` on it. The constructor adds the sketch's own
movable local origin: a fresh `SketchPoint` at (0, 0, 0) stored on the field `self.anchorPoint`, not
`sketch.originPoint` (which is immutable and cannot be coincident-constrained to anything projected
in). Everything below is drawn relative to that point.

`sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`

**Circles.** Four, in this order, each centred by passing the local-origin `SketchPoint` **directly**
as the centre so all four share it — never `localOrigin.geometry` followed by a coincident:

1. Root Circle — solid, radius `RootCircleRadius`.
2. Tip Circle — construction, `TipCircleRadius`.
3. Base Circle — construction, `BaseCircleRadius`.
4. Pitch Circle — construction, `PitchCircleRadius`.

`sketch.sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)`,
`circle.isConstruction = True` for the last three, and a driving diameter dimension on each:
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` with `textPoint` off-centre
(a point at the centre is rejected) and `isDriving` left at its default — never passed as `True`.
Set the value with `dimension.parameter.value = <number>`, a numeric snapshot of the parameter.

Each circle also carries an along-path label. The string is
`'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` from the radii's internal `.value` in cm,
with `size = TipCircleRadius − RootCircleRadius`, and that same `size` is the text height:

```
textInput = sketch.sketchTexts.createInput2(text, height)
textInput.setAsAlongPath(curve, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
sketch.sketchTexts.add(textInput)
```

⚑ API — the index has no `SketchTexts.createInput2`. What it carries is
`sketchTexts.createInput3(expression: str, height: core.ValueInput) -> SketchTextInput`, whose height
is a `ValueInput` and not a float in cm. `setAsAlongPath(path, isAbovePath, horizontalAlignment, characterSpacing)`
and `sketchTexts.add(input)` both check out. The step keeps the spec's `createInput2`.

**Flanks.** Sample `InvoluteSteps` points along the involute of the base circle, endpoint-inclusive:
sample `i` sits at `r = BaseCircleRadius + (TipCircleRadius − BaseCircleRadius) · i / (steps − 1)`, so
the first is exactly on the base circle and the last exactly on the tip circle. Do not clamp the
start to `max(BaseCircleRadius, RootCircleRadius)`. Each sample is
`calculateInvolutePoint(BaseCircleRadius, r)`:

```
alpha = acos(baseRadius / intersectionRadius)
t     = tan(alpha)
x = baseRadius * (cos(t) + t * sin(t))
y = baseRadius * (sin(t) - t * cos(t))
```

The parameter is `tan(alpha)`, **not** `inv(alpha) = tan(alpha) − alpha`. A sample whose radius is
below the base circle returns `None` and is dropped.

Then, in this order: mirror the samples across +X (negate y), because the standard parametric
involute spirals the wrong way for a left flank and drawn as-is gives a tooth wider at the tip than
at the root; rotate by `rotate_angle = π / (2 · ToothNumber) − atan2(−py, px)` where
`(px, py) = calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`, which lands the pitch
crossing where the left flank's belongs; mirror that result across X to get the right flank; and
finally rotate **both** flanks by the `angle` argument that came in from `draw()` at call time. Not
by `self.toothAngle` — helical and herringbone construct the generator with the default `angle=0` and
pass the helix angle to `draw()`, so reading the stored field draws a flat tooth and the loft has no
twist.

Draw each flank as a fitted spline through its point collection:
`sketch.sketchCurves.sketchFittedSplines.add(pointCollection)` over an
`adsk.core.ObjectCollection.create()` of `adsk.core.Point3D.create(x, y, 0)`.

**Tooth-top arc.** Materialize a tooth-top `SketchPoint` at
`(TipCircleRadius · cos(angle), TipCircleRadius · sin(angle))` and constrain it coincident to the tip
circle, then create the arc sharing the local origin and both flank splines' end points:

`sketch.geometricConstraints.addCoincident(toothTopPoint, tipCircle)`
`sketch.sketchCurves.sketchArcs.addByCenterStartEnd(localOrigin, rightFlank.endSketchPoint, leftFlank.endSketchPoint)`

Add **no** diameter dimension on it. The shared centre plus the two shared ends already determine the
arc, and a free centre with a diameter dimension would leave an inward-bulging answer available.
Sharing the centre also makes the last rib's perpendicular redundant, which is why the rib loop below
skips it.

**Spine and the +X reference.** The spine is a construction line from the local origin to the
tooth-top point, both passed as existing `SketchPoint`s. No separate start-coincident, and no
constraint tying the spine's end to the arc.

`sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, toothTopPoint)`, then
`line.isConstruction = True`.

The +X reference is built for **every** angle including 0. Add a far endpoint at
`(TipCircleRadius, 0)` and pin it with two signed dimensions from the local origin — not with a
coincident onto the tip circle, which has two answers and touches the circle where the numbers go
unstable:

`sketch.sketchDimensions.addDistanceDimension(localOrigin, xEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)` at `TipCircleRadius`
`sketch.sketchDimensions.addDistanceDimension(localOrigin, xEnd, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, textPoint)` at `0`

Draw the reference line from the origin to that endpoint, mark it construction, and add the angular
dimension **from the reference to the spine, in that argument order**, with its text point on the
bisector `(R·cos(angle/2), R·sin(angle/2))` so Fusion picks `angle` rather than its supplement:

`sketch.sketchDimensions.addAngularDimension(referenceLine, spine, textPoint)`

A plain `addHorizontal` on the spine is not a substitute at `angle = 0`: horizontal fixes the line's
direction but not which way it points, and the tooth comes out 180° round. The proof confirms this —
built with `NewHorizontal` on the spine instead, the engine's ambiguity probe reports two
configurations for the same geometry.

**Ribs.** One per fit-point index, for **all N indices** including the first (base circle) and the
last (tip). The fit-points carry no other constraint, so an omitted endpoint rib leaves that point
free. Per rib, in exactly this order:

1. `sketch.sketchCurves.sketchLines.addByTwoPoints(leftSpline.fitPoints.item(i), rightSpline.fitPoints.item(i))`, then `line.isConstruction = True`.
2. A **signed** dimension across it — `addDistanceDimension(left, right, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, textPoint)` at the measured Δy for `angle = 0`, or whichever of the horizontal/vertical pair is better conditioned for a rotated tooth. An aligned dimension gives only the length, which the two flanks satisfy equally well swapped over, and the tooth comes out mirrored.
3. `sketch.sketchPoints.add(adsk.core.Point3D.create(t·cos(angle), t·sin(angle), 0))` with `t = fitX·cos(angle) + fitY·sin(angle)` — the foot of the left fit point on the spine, not the rib's true 2-D midpoint.
4. `sketch.geometricConstraints.addCoincident(midpoint, spine)`
5. `sketch.geometricConstraints.addMidPoint(midpoint, rib)`
6. `sketch.geometricConstraints.addPerpendicular(spine, rib)` — **skipped for the last rib**, where the tooth-top arc already holds the two flank tips at equal radius either side of the spine.

Then dimension each midpoint from the previous one with a signed dimension along the spine direction
(horizontal for `angle = 0`), starting the chain at the local origin. Without that first
origin-to-first-rib link the chain slides along the spine as a unit and the sketch never closes; the
proof confirms it, reporting DOF 1 when the link is dropped.

**Close the tooth at the root.** With `firstRadius` the distance from the local origin to the left
flank's first fit point, `embedded = firstRadius < RootCircleRadius`, compared raw with no tolerance
— exact equality counts as **not** embedded and draws a zero-length stub. When not embedded, draw a
short radial line each side from the root circle up to the flank start, sharing the spline's start
point as the far end:

`sketch.sketchCurves.sketchLines.addByTwoPoints(rootEndGeometry, flankSpline.startSketchPoint)`

and place the root end with exactly two signed dimensions from the local origin, one horizontal at
its Δx and one vertical at its Δy, and nothing else. "Root end on the root circle" plus "local origin
on the line" is satisfied by two points — the line carries on through the centre and meets the circle
again on the far side — and the stub becomes a line straight across the gear. The signed offsets rule
that out. The loop is then **6 curves** (2 splines + 2 lines + 2 arcs); embedded it is **4**
(2 splines + 2 arcs). Record which was drawn by writing `self.parent._lastToothEmbedded`, which
`buildSketches` copies to `ctx.toothProfileIsEmbedded`.

**Anchor the sketch.** Project `ctx.anchorPoint` into this sketch and make the local origin
coincident with the projection. Every piece of geometry above is placed relative to that origin, so
this drags the whole tooth profile onto the user's anchor as a unit. It happens inside `draw()`, not
in `buildSketches` afterwards — helical and herringbone call `draw()` directly on their own twisted
loft sketch and rely on that one call to anchor it.

`sketch.project(ctx.anchorPoint)` (⚑ API, as S3)
`sketch.geometricConstraints.addCoincident(localOrigin, projectedAnchor)`

Finally, for `angle != 0` only and after the whole constraint network exists,
`spineAngularDimension.parameter.value = angle`. The pre-rotation put the geometry on the right
solver branch; this confirms and locks it. Both are required.

The proof sweeps module 1 at 12 and 17 teeth, module 2 at 20, module 3 at 15, all at 20° with 15
involute steps, plus a fine module, both embedded branches, the near-degenerate transition either
side of it, rotations of 30°, 90°, 180°, −45° and 120°, an anchor away from the sketch origin, and 8
and 24 involute steps. It also checks that the two loops S7 and S9 look for are the two the sketch
holds.

**From:** `spec/spurgear/instructions.md` 180–215, 302–343, 395–404, 406–442, 444–448;
`spec/spurgear/fusion.md` 19–43, 47–60, 69–87, 90–112, 114–147, 149–183;
`.claude/skills/generate-gear/PLAYBOOK.md` 413–450, 458–465, 466–473, 525–533, 534–535, 547–550,
551–555, 581–591.

---

## S6 `[PROSE]` Sketch-Only Short-Circuit

If `SketchOnly` is true, set `ctx.gearProfileSketch.isVisible = True` and stop: no tooth extrude, no
body, no pattern, no fillet, no chamfer, no bore. `buildMainGearBody` returns here.

This is control flow, not a timeline entry, but it decides how many entries the rest of the build
produces at all. `cleanup(ctx)` still runs afterwards (S16) — in this mode it hides the construction
planes and axes but leaves the sketches visible, which is the point of the mode.

**From:** `spec/spurgear/instructions.md` 450–452, 243–268; `spec/spurgear/fusion.md` 187–197.

---

## S7 `[PROSE]` Extrude the Tooth

Find the tooth cross-section in the Gear Profile sketch by curve counts, not by index:

`find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`

from `.utilities` — do not re-implement the loop search; the helper rejects a mismatched loop and
raises rather than falling back to a wrong profile. The two arcs are the tooth top and the root arc
Fusion derives by splitting the root circle where the flank-to-root lines meet it.

Extrude it from the target plane to the Extrusion End Plane as a **New Body**:

`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`
`extrudeInput.setOneSideExtent(extentDefinition, adsk.fusion.ExtentDirections.PositiveExtentDirection)`
`component.features.extrudeFeatures.add(extrudeInput)`

Name the feature `Extrude tooth` and store the body on `ctx.toothBody`. `buildTooth` then calls
`self.chamferTooth(ctx)` as its last action — the chamfer is triggered from inside `buildTooth`, so
`buildMainGearBody` must not chamfer separately.

**From:** `spec/spurgear/instructions.md` 454–458, 243–268, 217–232;
`.claude/skills/generate-gear/PLAYBOOK.md` 151–158, 571–580.

---

## S8 `[PROSE]` Chamfer the Tooth's Front Face

Runs only when `ChamferTooth > 0`.

Find the front face with a **single conjunction predicate** over `ctx.toothBody.faces`: the first
face for which **both** `face.edges.count == self.chamferWantEdges()` (6 on the spur base) **and**
`sketchPlane.isCoPlanarTo(face.geometry)`, with
`sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`. Both of the same face — not an
edge-count match with a coplanarity tiebreak. If no face satisfies both, raise; no partial-match
fallback. An embedded profile gives a 4-edge front face while `chamferWantEdges()` stays 6, so
chamfering an embedded spur tooth raises — an accepted limitation, and users turn the chamfer off for
those gears.

Walk that face's edges and add each to the edge set **except** any edge whose
`edge.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType` and whose `edge.geometry.radius`
equals `RootCircleRadius` within 0.001 cm. That is the root arc, and chamfering it would eat into the
neighbouring tooth. Everything else — the two flanks, the tooth top, the two flank-to-root lines —
gets chamfered.

`component.features.chamferFeatures.createInput2()`
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(chamferValue), False)`
`component.features.chamferFeatures.add(chamferInput)`

⚑ API — the index gives
`addEqualDistanceChamferEdgeSet(edges: core.ObjectCollection, distance: core.ValueInput, isTangentChain: bool) -> bool`.
The third argument is `isTangentChain`; `[PB-FILLET-CHAMFER]` calls it `isFlipped`. The value `False`
the spec passes is right either way, but the playbook's name for it is not.

**From:** `spec/spurgear/instructions.md` 460–466, 270–286;
`.claude/skills/generate-gear/PLAYBOOK.md` 480–486, 571–580.

---

## S9 `[PROSE]` Extrude the Gear Body

Find the gear body loop — the one bounded by exactly two arcs — with
`find_profile_by_curve_counts(sketch, arcs=2)`, and extrude it from the target plane to the Extrusion
End Plane as a **New Body**, same call shape as S7. Name the feature `Extrude body` and the body
`Gear Body`; store it on `ctx.gearBody`.

⚑ Spec — step 9 calls this "the annular loop bounded by exactly 2 arcs (the root circle and the tip
circle)". The tip circle is construction geometry (S5) and cannot bound a profile, and the region is
a disk of root diameter, not an annulus. The two arcs are the two pieces the root circle is split
into where the flank-to-root lines meet it. The curve count the code searches on is right; the
parenthetical naming which arcs they are is not. The proof's region check reads the same disk.

While iterating `extrude.bodies.item(0).faces`, classify each face by
`face.geometry.surfaceType` and capture `ctx.extrusionExtent`: among the
`adsk.core.SurfaceTypes.PlaneSurfaceType` faces, the one where
`sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry)` — the
far end cap, which S15 cuts the bore down to. The near cap is coplanar with the sketch plane, so
`isCoPlanarTo` rules it out. Raise if it is not found.

**From:** `spec/spurgear/instructions.md` 468–477, 217–232;
`.claude/skills/generate-gear/PLAYBOOK.md` 151–158, 405–412, 639–660.

---

## S10 `[PROSE]` Gear Center Construction Axis

From the same face walk as S9: any face whose `surfaceType` is
`adsk.core.SurfaceTypes.CylinderSurfaceType` gives the gear's axis.

`component.constructionAxes.createInput()`
`axisInput.setByCircularFace(cylindricalFace)`
`component.constructionAxes.add(axisInput)`

Name it `Gear Center`, set `isLightBulbOn = False`, and store it on `ctx.centerAxis`. Raise if no
cylindrical face is found.

**From:** `spec/spurgear/instructions.md` 468–477, 217–232;
`.claude/skills/generate-gear/PLAYBOOK.md` 558–570, 686–689.

---

## S11 `[PROSE]` Circular-Pattern the Teeth

Pattern `ctx.toothBody` about the `Gear Center` axis, quantity = `ToothNumber`. Pin all three inputs
explicitly rather than trusting Fusion's defaults:

`component.features.circularPatternFeatures.createInput(bodyCollection, ctx.centerAxis)`
`patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)`
`patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')`
`patternInput.isSymmetric = False`
`component.features.circularPatternFeatures.add(patternInput)`

`createInput` takes an `adsk.core.ObjectCollection` of bodies, so the tooth body goes into a fresh
`adsk.core.ObjectCollection.create()` first.

**From:** `spec/spurgear/instructions.md` 479–483;
`.claude/skills/generate-gear/PLAYBOOK.md` 592–602.

---

## S12 `[PROSE]` Combine the Teeth into the Body

One Combine-Join of the patterned teeth into `Gear Body`.

`CircularPatternFeature.bodies` already holds the seed tooth plus the copies, so feed it as-is — do
not re-add the seed. It is a `BRepBodies` and `createInput` rejects that, so copy
`pattern.bodies.item(i)` into a fresh `adsk.core.ObjectCollection.create()` and pass that.

`component.features.combineFeatures.createInput(ctx.gearBody, toolBodies)`
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`
`component.features.combineFeatures.add(combineInput)`

`patternTeeth` then calls `self.createFillets(ctx)` as its last action.

**From:** `spec/spurgear/instructions.md` 479–483, 243–268;
`.claude/skills/generate-gear/PLAYBOOK.md` 592–596.

---

## S13 `[PROSE]` Root Fillets

Runs only when `FilletRadius > 0`, reading the registered parameter's numeric `.value` —
`filletHelixFactorExpression()` is not consulted here; it was spliced into the parameter's expression
back in S1.

Round the inside corner where each valley floor meets a tooth flank — the one that runs the full
thickness of the gear parallel to its axis, where bending stress concentrates. Two things make the
edge pick fiddly:

- After the pattern-and-combine the root cylinder is usually one patch per valley, not one continuous
  surface. Collect **every** cylindrical face whose radius equals `RootCircleRadius`, not just the
  first.
- On each such face keep only the axial straight edges. Filter to
  `adsk.core.Curve3DTypes.Line3DCurveType` first, take each line's direction from its geometry
  endpoints — `edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)` — normalize, and keep it
  when `abs(abs(dot(direction, axisNormal)) - 1.0) < 0.01`. Use exactly that tolerance; a tighter
  `> 0.999` drops valid axial edges that tessellation left slightly off and the root fillets go
  missing. Do **not** read the direction via `edge.evaluator.getTangent(0)` — parameter 0 is not
  guaranteed to lie in the edge's parameter range and Fusion raises
  `RuntimeError: invalid argument parameter`. The circular edges wrapping the front and back end caps
  are rims, not root corners; drop them.

If the edge collection comes out empty, return without creating the feature — silently, no error. An
empty edge set must never reach `add`.

`component.features.filletFeatures.createInput()`
`filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), False)`
`component.features.filletFeatures.add(filletInput)`

`isTangentChain` must be `False`: the collected edges are exactly the axial root corners, and
tangent-chaining would let Fusion pull in tangent-adjacent edges and round more than the intended
corner.

⚑ API — the index has no `addConstantRadiusEdgeSet` on `FilletFeatureInput`. The method is owned by
`FilletEdgeSetInputs` and `FilletEdgeSets`, with signature
`addConstantRadiusEdgeSet(entities: core.ObjectCollection, radius: core.ValueInput, isTangentChain: bool)`,
and `FilletFeatureInput` does carry an `edgeSetInputs` property returning `FilletEdgeSetInputs`. Both
the spec ("add the edge set on the input **itself**") and `[PB-FILLET-CHAMFER]` ("There is no
`filletInput.edgeSetInputs`; reaching for it raises `AttributeError`") say the opposite of the index.
The step keeps the spec's form; the contradiction is a defect to settle in Fusion.

**From:** `spec/spurgear/instructions.md` 485–494, 279–287;
`.claude/skills/generate-gear/PLAYBOOK.md` 412, 480–486, 571–580.

---

## S14 `[GO]` Bore Profile Sketch

Proof: `stepBoreProfileSketch`.

`buildBore` runs unconditionally from `generate()`, after `buildMainGearBody`, so it early-returns in
two cases: **SketchOnly** set, and **Bore Diameter ≤ 0**. The SketchOnly guard is not optional — in
that mode `buildBody` never ran, so `ctx.gearBody` and `ctx.extrusionExtent` are `None` and the cut
below would dereference them. Do not lean on the bore diameter being 0 in sketch-only mode; the user
may have set both.

Otherwise create a separate `Bore Profile` sketch on the target plane and draw the circle by
instantiating the tooth generator on it — `SpurGearInvoluteToothDesignGenerator(boreSketch, self)` —
and calling `drawBore(ctx.anchorPoint, boreDiameter)`. That projects the anchor in, draws a
construction-less circle of that diameter centred on the projection, gives it a driving diameter
dimension, and returns the circle.

`sketch.project(ctx.anchorPoint)` (⚑ API, as S3)
`sketch.sketchCurves.sketchCircles.addByCenterRadius(projectedAnchor, boreDiameter / 2)`
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`

The constructor still adds its local-origin `SketchPoint` at (0, 0, 0), so the sketch carries one
stray unused point. That is faithful behaviour and is not to be suppressed. The spec treats it as the
one exception to the full-constraint rule; the proof models it as reference geometry — placed from
outside, never moved — and the sketch comes out fully constrained with it in, so the exception turns
out not to be needed.

The proof runs one case with Bore Diameter 0, which reports as unmodelled rather than as a pass:
there is no sketch to prove when the step does not run.

**From:** `spec/spurgear/instructions.md` 496–500, 210–212, 321–326, 243–268;
`spec/spurgear/fusion.md` 19–24, 26–31;
`.claude/skills/generate-gear/PLAYBOOK.md` 430–444, 551–555.

---

## S15 `[PROSE]` Extrude-Cut the Bore

Cut the bore profile from the target plane down to `ctx.extrusionExtent`, the far end-cap face
captured in S9. Going to that face rather than a distance guarantees the hole goes all the way
through whatever `Thickness` is.

`component.features.extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)`
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)`
`extrudeInput.setOneSideExtent(extentDefinition, adsk.fusion.ExtentDirections.PositiveExtentDirection)`
`extrudeInput.participantBodies = [ctx.gearBody]`
`component.features.extrudeFeatures.add(extrudeInput)`

`participantBodies` restricts the cut to the gear body.

**From:** `spec/spurgear/instructions.md` 496–500, 217–232;
`.claude/skills/generate-gear/PLAYBOOK.md` 628–631.

---

## S16 `[PROSE]` Cleanup

The very last action of `generate()` — after `buildBore`, not inside `buildMainGearBody` — and called
**unconditionally** in both modes. Placement matters: `buildBore` re-projects `ctx.anchorPoint` out of
the Tools sketch, and projection fails once that sketch is hidden, so the Tools sketch has to stay
visible through the bore.

Split by entity kind and by mode, and never cross the two properties:

- **Always, in both modes** — `isLightBulbOn = False` on every construction plane and axis this build
  created: `ctx.extrusionEndPlane`, `ctx.centerAxis`, and the normalized target plane if S2 made one.
  `isVisible = False` does not hide construction geometry.
- **Full build only** — `isVisible = False` on the Tools, Gear Profile and Bore Profile sketches.
  Sketch-only mode leaves Tools and Gear Profile visible for inspection.

Guard each entity individually: the `Gear Center` axis and the Bore Profile sketch do not exist in
sketch-only mode. Guard the hiding inside `cleanup`, never the call to it.

**From:** `spec/spurgear/instructions.md` 203–205, 243–268, 450–452;
`spec/spurgear/fusion.md` 187–197; `.claude/skills/generate-gear/PLAYBOOK.md` 558–570.
