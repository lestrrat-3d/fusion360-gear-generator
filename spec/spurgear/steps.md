# Spur Gear — compiled step list

One step is one entry in the Fusion timeline. A whole sketch is one step however much geometry
goes into it, and so is each construction plane, construction axis, extrude, chamfer, pattern,
combine and fillet. S1–S3 are the dialog-and-parameter prologue and S18 the cleanup epilogue;
neither leaves a timeline entry, but the build is not reproducible without them, so they are
steps here too.

`[GO]` marks a step the proof in `.tmp/spurgear_test.go` exercises. Only the three sketches carry
it: the proof engine models 2D sketches, so everything 3D is `[PROSE]`.

## Sources

| Source | Blob hash |
|---|---|
| `spec/spurgear/instructions.md` | `6ce986c609c1d086e5158592f439e6d1c62309e3` |
| `spec/spurgear/fusion.md` | `3e8b0b338e80a1199eb7eb95f9f004a6f0bb747d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `802a53d6e2be4743227920a9d58c3dbe132553dc` |

## Defects marked inline

Four places where the spec names an API call the signature will not accept, or names one
inconsistently. Each is marked `⚠ DEFECT` at its step and written as the spec writes it, not
silently corrected.

---

## S1 `[PROSE]` Add the dialog inputs

`SpurGearCommandInputsConfigurator.configure(cls, cmd)` — a `@classmethod` on a plain class — adds
ten inputs to `cmd.commandInputs` in **exactly this display order**. The order is the order the
Variables section lists them, not a grouping by input type; Target Plane and Anchor Point are the
first two and Parent Component is last. This is independent of the `processInputs` *read* order in
S2, which is different for a different reason.

| # | Dialog input | id | how it is added |
|---|---|---|---|
| 1 | Target Plane | `plane` | `addSelectionInput('plane', …)` + filters `ConstructionPlanes`, `PlanarFaces` |
| 2 | Anchor Point | `anchorPoint` | `addSelectionInput('anchorPoint', …)` + filters `ConstructionPoints`, `SketchPoints` |
| 3 | Module | `module` | `addValueInput('module', …, '', ValueInput.createByReal(1))` |
| 4 | Tooth Number | `toothNumber` | `addValueInput('toothNumber', …, '', ValueInput.createByReal(17))` |
| 5 | Pressure Angle | `pressureAngle` | `addValueInput('pressureAngle', …, 'deg', ValueInput.createByReal(math.radians(20)))` |
| 6 | Bore Diameter | `boreDiameter` | `addStringValueInput('boreDiameter', …, '0 mm')` |
| 7 | Thickness | `thickness` | `addValueInput('thickness', …, 'mm', ValueInput.createByReal(to_cm(10)))` |
| 8 | Apply chamfer to teeth | `chamferTooth` | `addValueInput('chamferTooth', …, 'mm', ValueInput.createByReal(0))` |
| 9 | Generate sketches, but do not build body | `sketchOnly` | `addBoolValueInput('sketchOnly', …, True)` |
| 10 | Parent Component | `parentComponent` | `addSelectionInput('parentComponent', …)` + filters `Occurrences`, `RootComponents` |

`addValueInput` defaults are in Fusion **internal** units — cm for length, radians for angle —
whatever the display unit string says. That is why Pressure Angle's default is
`math.radians(20)` and Thickness's is `to_cm(10)`.

Each of the three selection inputs takes its filters as enum constants, never strings
(`selectionInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)` and the
matching `PlanarFaces`, `ConstructionPoints`, `SketchPoints`, `Occurrences`, `RootComponents`),
and each is limited to exactly one with `selectionInput.setSelectionLimits(1, 1)`. Parent
Component pre-selects the root: `parentInput.addSelection(get_design().rootComponent)`.

Every id above, and every parameter name in S2, is exported as a module-level constant
(`INPUT_ID_PLANE`, `PARAM_MODULE`, … the full roster is in the spec) because helical and
herringbone import them by name. A subclass extends this step by subclassing the configurator and
appending after `super().configure(cmd)`, which necessarily puts its extra input below Parent
Component.

**From:** `spec/spurgear/instructions.md` 20–22, 36–62, 90–178; `PLAYBOOK.md` 53–60, 128–144,
332–334, 474–479.

---

## S2 `[PROSE]` Read the inputs and register the user parameters

`SpurGearGenerator.processInputs(inputs)`. Read order is load-bearing and has nothing to do with
S1's display order: as soon as anything creates the occurrence, a `SelectionCommandInput` holding
an entity that lives in another component can drop its selection. So:

1. `get_selection(inputs, 'parentComponent')` → resolve `Occurrence.component` vs `Component` into
   `self.parentComponent`; raise on the wrong count or type.
2. `get_selection(inputs, 'plane')` → `self.plane`; `get_selection(inputs, 'anchorPoint')` →
   `self.anchorPoint`. Nothing has touched the design yet.
3. Only now register parameters, which creates the occurrence transitively through
   `parameterName()` / `addParameter()`.

Read each input with the helper matching how it was declared: `get_value(inputs, id, units)` for
the value and string-value inputs, `get_boolean(inputs, 'sketchOnly')` for the checkbox. Calling
`get_value` on the bool input raises `AttributeError` — `BoolValueCommandInput` has no
`expression`. `get_value` always returns a `ValueInput` ready to hand straight to
`addParameter`; it raises on a bad expression rather than returning `None`.

Input-sourced parameters, via `addParameter(name, ValueInput, units, comment)` →
`design.userParameters.add(parameterName(name), value, units, comment)`: `Module` (units `''`,
**not** `'mm'`, so `generateName` renders `M=1` and the derived `mm` expressions read it as a bare
factor), `ToothNumber`, `PressureAngle` (`'rad'`), `BoreDiameter`, `Thickness`, `ChamferTooth`,
and `SketchOnly` as a real-valued 1/0 (the framework reads booleans back numerically through
`getParameterAsBoolean`).

Then call the `addExtraPrimaryParameters(self, inputs)` hook — a no-op on the spur base, the seam
where a subclass registers its own primary parameter. It must sit **between** the input-sourced
parameters and the derived ones.

Derived parameters, as live expression strings via `ValueInput.createByString(...)`:

| Parameter | Expression |
|---|---|
| `PitchCircleDiameter` | `Module * ToothNumber` |
| `PitchCircleRadius` | `PitchCircleDiameter / 2` |
| `BaseCircleDiameter` | `PitchCircleDiameter * cos(PressureAngle)` |
| `BaseCircleRadius` | `BaseCircleDiameter / 2` |
| `RootCircleDiameter` | `PitchCircleDiameter - 2.5 * Module` |
| `RootCircleRadius` | `RootCircleDiameter / 2` |
| `TipCircleDiameter` | `PitchCircleDiameter + 2 * Module` |
| `TipCircleRadius` | `TipCircleDiameter / 2` |
| `InvoluteSteps` | `15` |
| `ToothSpaceAngleAtRoot` | pre-computed in Python, `ValueInput.createByReal(π/ToothNumber − 2·(tan(PressureAngle) − PressureAngle))`, registered **unitless** |
| `ToothSpaceArcAtRoot` | `RootCircleRadius * ToothSpaceAngleAtRoot`, registered `'mm'` |
| `FilletClearance` | `0.9` |
| `FilletRadius` | `(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>` |

`ToothSpaceAngleAtRoot` is pre-computed because Fusion's expression engine refuses to subtract a
radian-valued term from the unitless output of `tan()`, and registered unitless because the next
parameter multiplies it by a length — as `'rad'` the product reads `mm·rad` and Fusion rejects the
dependent parameter with `RuntimeError: Invalid expression`. Radians are dimensionless, so this is
correct as well as necessary.

`<factor>` in `FilletRadius` is the string returned by the overridable
`filletHelixFactorExpression()` — `'1'` for spur, `'cos(<prefix>_HelixAngle)'` for helical. It is
consumed here and **nowhere else**; `createFillets` in S15 reads only the resulting parameter's
numeric `.value`.

Every dimension and feature input downstream is a numeric snapshot of these parameters at
generation time, not a live link. Editing a `<prefix>_…` parameter does not change an existing
gear; re-run the dialog.

**From:** `spec/spurgear/instructions.md` 36–88, 147–150, 217–232, 283–295, 372–381;
`spec/spurgear/fusion.md` 204–209; `PLAYBOOK.md` 84–98, 103–126, 196–228.

---

## S3 `[PROSE]` Create the gear occurrence and name the component

`generate(inputs)` calls `processInputs`, then `component = self.getComponent()` and
`component.name = self.generateName()`. `getOccurrence()` lazily creates the child under
`self.parentComponent` with `parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`,
and on first creation builds the `ParamNamePrefix` (`SpurGear_<component id without dashes>`, from
`prefixBase()` returning `'SpurGear'`) and a `ComponentCleaner`.

`generateName()` returns
`'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression, thickness.expression)`
— the parameters' `.expression` strings, not `.value`, so units show through:
`Spur Gear (M=1, Tooth=17, Thickness=10 mm)`.

Then `ctx = self.newContext()`, a `SpurGearGenerationContext` whose fields are all `cast(None)` at
this point except `toothProfileIsEmbedded`, which starts `False`. `SpurGearGenerator.__init__` must
also pre-initialise `self._lastToothEmbedded = False`, `self.toolsSketch = None` and
`self.boreSketch = None`.

**From:** `spec/spurgear/instructions.md` 11, 20–33, 217–232, 296–300; `spec/spurgear/fusion.md`
182–186; `PLAYBOOK.md` 84–91, 230–240.

---

## S4 `[PROSE]` Normalize the Target Plane

If the selected plane is already a `ConstructionPlane`, keep it. Otherwise — the user picked a
planar face — build a coplanar construction plane and use that everywhere downstream, so
profile detection never has to filter out the selected face's own profile:

```
planeInput = component.constructionPlanes.createInput()
planeInput.setByOffset(selectedPlane, adsk.core.ValueInput.createByReal(0))
plane = component.constructionPlanes.add(planeInput)
```

The offset is a `ValueInput`, never a bare number — `setByOffset(plane, 0)` is a runtime
`TypeError`. Keep the normalized plane on both `self.plane` and `ctx.plane`; subclasses read
`self.plane` directly. If one was created here, S18 switches its light bulb off.

**From:** `spec/spurgear/instructions.md` 39, 217–232, 385–387; `PLAYBOOK.md` 230–240, 674–685.

---

## S5 `[GO]` Tools Sketch

**Proof function: `stepToolsSketch`.**

`self.toolsSketch = self.createSketchObject('Tools', ctx.plane)` — one sketch on the target plane
that draws no geometry of its own. Its only content is the projection of the user's Anchor Point:

```
projected = toolsSketch.project(self.anchorPoint)
ctx.anchorPoint = projected.item(0)
```

That projection is the **canonical handle**. Every later sketch projects *this* in again, forming
a chain back to the user's original anchor entity, so the whole gear moves if the anchor moves.

The sketch stays visible while the later sketches project from it — `sketch.project(...)` has
failed on invisible sketches in this repo's history — and only S18 sets `toolsSketch.isVisible =
False`, after the bore has re-projected from it.

⚠ DEFECT — the spec claims every sketch here is fully constrained "with no exceptions"
(instructions.md 210–212), but this sketch holds one projected point and nothing else, and
`[PB-PROJECT-NOT-FIXED]` says a projection carries free degrees of freedom. In Fusion this sketch
reports under-constrained. The proof models the projection as a locked reference point, which is
the only reading under which the claim holds.

**From:** `spec/spurgear/instructions.md` 41, 180–215, 222–223, 389–391;
`spec/spurgear/fusion.md` 19–24; `PLAYBOOK.md` 92–96, 430–444, 558–570.

---

## S6 `[PROSE]` Extrusion End Plane

An offset construction plane at distance `Thickness` from the target plane, the `to-entity` target
for both the tooth extrude (S9) and the body extrude (S11), so the two end on the same
well-defined face:

```
planeInput = component.constructionPlanes.createInput()
planeInput.setByOffset(ctx.plane, adsk.core.ValueInput.createByReal(thickness))
ctx.extrusionEndPlane = component.constructionPlanes.add(planeInput)
ctx.extrusionEndPlane.name = 'Extrusion End Plane'
```

`thickness` is the `Thickness` parameter's numeric `.value`, in cm. Leave the plane visible while
those extrudes run; S18 hides it with `isLightBulbOn = False`. `isVisible = False` does not hide a
construction plane.

**From:** `spec/spurgear/instructions.md` 224, 387, 389–393; `PLAYBOOK.md` 558–570, 674–685.

---

## S7 `[GO]` Gear Profile Sketch

**Proof function: `stepGearProfileSketch`.**

`ctx.gearProfileSketch = self.createSketchObject('Gear Profile', ctx.plane)`, then
`SpurGearInvoluteToothDesignGenerator(gearProfileSketch, self).draw(ctx.anchorPoint)`. That single
`draw` call does everything below — the circles, the tooth, the anchoring — and the anchoring must
stay inside it, because helical and herringbone build their twisted loft profile by calling
`draw(loftSketch…, angle=helixAngle)` and rely on that one call to constrain the sketch. After it
returns, `buildSketches` copies the embedded flag across:
`ctx.toothProfileIsEmbedded = self._lastToothEmbedded`.

All of it is one timeline entry.

### 7a — the local origin

The generator's **constructor** adds a fresh `SketchPoint` at (0, 0, 0) — `sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`
— stored as the field `self.anchorPoint`. Not `sketch.originPoint`, which is immutable and cannot
be coincident-constrained to anything projected in. Everything below is drawn relative to it.

### 7b — `drawCircles()`

Four circles, in this order, every one centred by passing the local-origin `SketchPoint`
**directly** so all four share it:

| Order | Circle | Radius | Solid? |
|---|---|---|---|
| 1 | Root | `RootCircleRadius` | solid |
| 2 | Tip | `TipCircleRadius` | construction |
| 3 | Base | `BaseCircleRadius` | construction |
| 4 | Pitch | `PitchCircleRadius` | construction |

```
circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)
circle.isConstruction = True                       # tip, base, pitch only
sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
```

Do not pass `localOrigin.geometry` and then add a centre coincident: share the point or coincident
a fresh one, never both. Every diameter dimension is driving — never pass the trailing
`isDriven=True`. Its `textPoint` must be off the centre, or Fusion rejects it with "some input
arguments are invalid"; a point at radius `r` from the centre works.

Each circle is then labelled along its own path:

```
textInput = sketch.sketchTexts.createInput2(label, size)
textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
sketch.sketchTexts.add(textInput)
```

with `label = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` and
`size = TipCircleRadius − RootCircleRadius`, every number the radii's internal `.value` in cm.
The same `size` is the text height.

### 7c — `drawTooth(angle)`

`drawTooth` rotates by the `angle` argument that flows in from `draw()` at call time, **never** by
the constructor-stored `self.toothAngle`. Helical constructs the generator with the default
`angle=0` and then calls `draw(ctx.anchorPoint, angle=helixAngle)`; reading the stored field would
draw a flat tooth and leave the loft with no twist.

1. **Sample the flank.** With `steps = InvoluteSteps`, sample `i = 0 … steps−1` at
   `r = BaseCircleRadius + (TipCircleRadius − BaseCircleRadius)·i/(steps−1)` — endpoint-inclusive,
   so the first sample is exactly on the base circle and the last exactly on the tip circle. Each
   sample is `calculateInvolutePoint(BaseCircleRadius, r)`:

   ```
   alpha = acos(baseRadius / intersectionRadius)
   t     = tan(alpha)            # tan(alpha), NOT inv(alpha) = tan(alpha) − alpha
   x = baseRadius * (cos(t) + t * sin(t))
   y = baseRadius * (sin(t) - t * cos(t))
   ```

   It returns `None` when `intersectionRadius < baseRadius`; drop those samples. Do not clamp the
   start to `max(base, root)` — the flank is sampled from the base circle even when the base
   circle sits inside the root circle.
2. **Mirror across +X** (negate y). The standard parametric involute spirals the wrong way for a
   left flank — its angular position grows with radius, giving a tooth wider at the tip than at
   the root.
3. **Find the rotation analytically.** With `(px, py) = calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`,
   `rotate_angle = π/(2·ToothNumber) − atan2(−py, px)`. The `−py` is step 2's mirror applied to the
   analytic point; `atan2(py, px)` is the wrong sign. Do not interpolate between sampled points.
4. **Rotate by `rotate_angle`** → the left flank. Mirror that across X → the right flank. Then
   rotate **both** by the requested `angle`, here in the Python point math, and seed the tooth-top
   point and every rib midpoint at their rotated positions too. Draw the tooth at its final
   angular position; do not leave it at +X and swing it into place with the angular dimension
   afterwards. For `angle = 0` this is a rotation by zero.
5. **Two fitted splines** through the two point collections:
   `sketch.sketchCurves.sketchFittedSplines.add(pointCollection)`, one per flank.
6. **Tooth-top point and arc.** A `SketchPoint` at
   `(TipCircleRadius·cos(angle), TipCircleRadius·sin(angle))`, constrained
   `sketch.geometricConstraints.addCoincident(toothTopPoint, tipCircle)`. Then
   `sketch.sketchCurves.sketchArcs.addByCenterStartEnd(localOrigin, rightFlank.endSketchPoint, leftFlank.endSketchPoint)`
   — the local origin and both flank end points passed directly, so the arc shares all three and
   needs no coincidences. **No diameter dimension.** A free centre plus a diameter reaches DOF 0
   with two answers: the arc can bulge inward, back through the tooth. Sharing the centre removes
   the choice, and it is also what makes the last rib's perpendicular redundant in 7c.8.
7. **Spine, +X reference, angular pin.**
   `spine = sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, toothTopPoint)`, both
   existing points passed directly, marked `isConstruction = True`. No separate start-coincident to
   the origin, and no constraint putting the spine's end on the arc. Then, for **every** angle
   including 0:
   - a far endpoint at `(TipCircleRadius, 0)` pinned by two signed dimensions from the local
     origin — `sketch.sketchDimensions.addDistanceDimension(localOrigin, refEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
     at `TipCircleRadius`, and the same with `VerticalDimensionOrientation` at `0`. Not
     `addCoincident(refEnd, tipCircle)`: a point on a circle has two answers, and pinning x at the
     tip radius touches the circle where the numbers go unstable;
   - the reference line from the origin to it, `isConstruction = True`;
   - `sketch.sketchDimensions.addAngularDimension(reference, spine, textPoint)` — reference first,
     spine second — with its text on the bisector `(R·cos(angle/2), R·sin(angle/2))` so Fusion
     picks `angle` and not its supplement.

   Not `addHorizontal` on the spine for the `angle = 0` case: horizontal fixes the direction but
   not which way it points, and the tooth comes out 180° around.
8. **Ribs.** One construction line per fit-point index, for **all N indices** including the first
   (base circle) and the last (tip) — the fit points carry no other constraint, so an omitted
   endpoint rib leaves one free. Per rib, in exactly this order:
   1. `rib = sketch.sketchCurves.sketchLines.addByTwoPoints(leftSpline.fitPoints.item(i), rightSpline.fitPoints.item(i))`,
      the two fit points passed directly; `rib.isConstruction = True`.
   2. A **signed** dimension, never an aligned one — aligned gives only the length, which the two
      flanks satisfy equally well swapped over. The rib takes the axis **across** the spine and
      the midpoint chain the one **along** it: `VerticalDimensionOrientation` for the rib and
      `HorizontalDimensionOrientation` for the chain when `|cos(angle)| >= |sin(angle)|`, swapped
      otherwise. At `angle = 0` that is vertical rib / horizontal chain; a tooth at 90° fails
      without the swap.
   3. A fresh `SketchPoint` for the midpoint, seeded **already on the spine**: with
      `t = fitX·cos(angle) + fitY·sin(angle)`, at `(t·cos(angle), t·sin(angle))`. Not the rib's
      true 2-D midpoint, and not `(fitX, 0)` for a rotated tooth.
   4. `sketch.geometricConstraints.addCoincident(midpoint, spine)` — onto the spine first.
   5. `sketch.geometricConstraints.addMidPoint(midpoint, rib)`.
   6. `sketch.geometricConstraints.addPerpendicular(spine, rib)` — **skipped for the last rib**.
      That rib joins the two flank tips, which the tooth-top arc's shared centre already holds at
      equal radius either side of the spine, so Fusion rejects it with
      `VCS_SKETCH_OVER_CONSTRAINTS`.

   Then the midpoint chain: a **signed** dimension along the spine direction from each midpoint to
   the previous one, with the **first taken from the local origin**. Without that first link the
   whole chain slides along the spine as a unit and the sketch never fully constrains.
9. **Close the tooth at the root.** With `firstRadius` the distance from the local origin to the
   left flank's first fit point, `embedded = firstRadius < RootCircleRadius`, compared raw with no
   tolerance. The comparison is strict: exact equality counts as **non**-embedded and draws a
   zero-length stub. Do not soften it to `<=`.
   - **Not embedded** — a short radial line per side,
     `sketch.sketchCurves.sketchLines.addByTwoPoints(rootEndGeometry, flankSpline.startSketchPoint)`,
     the flank's start point passed directly. Place the root end with **exactly two** signed
     dimensions from the local origin, `HorizontalDimensionOrientation` at its Δx and
     `VerticalDimensionOrientation` at its Δy, and nothing else. Not "root end on the root circle"
     plus "origin on the line": the line through the flank start and the centre meets the root
     circle again on the far side, both answers satisfy those two, and the stub becomes a line
     across the gear. Tooth loop: **6 curves** (2 splines + 2 stubs + 2 arcs).
   - **Embedded** — no stub. Tooth loop: **4 curves** (2 splines + 2 arcs). It happens above
     `2.5/(1 − cos(PressureAngle))` teeth: 41.5 at 20°, 78.5 at 14.5°, 26.7 at 25°.

   The generator has no `ctx`, so it records the result on its parent:
   `self.parent._lastToothEmbedded = True/False`.

### 7d — anchor the sketch

Still inside `draw()`, after `drawTooth`: project the Tools-sketch anchor in and constrain the
local origin to it.

```
projected = sketch.project(anchorPoint)
sketch.geometricConstraints.addCoincident(self.anchorPoint, projected.item(0))
```

`self.anchorPoint` is the local origin from 7a, not `sketch.originPoint`. Every piece of geometry
above is placed relative to that point, so this one constraint drags the whole tooth profile onto
the user's anchor as a unit.

### 7e — confirm the rotation

As the **very last action**, after the entire constraint network exists:
`if angle != 0: spineAngularDimension.parameter.value = angle`. Drawing rotated (7c.4) and
confirming here are two required actions, not alternatives. The pre-rotation puts the geometry on
the correct solver branch; this locks it. At `angle = 0` the dimension still exists — it is what
says which way the spine points — and there is simply nothing to set.

**From:** `spec/spurgear/instructions.md` 180–215, 224, 229, 271–277, 302–343, 395–449;
`spec/spurgear/fusion.md` 19–43, 47–60, 69–186; `PLAYBOOK.md` 413–450, 458–473, 525–535,
547–555, 581–591.

---

## S8 `[PROSE]` Sketch-only short circuit

If `SketchOnly` is true, set `ctx.gearProfileSketch.isVisible = True` and stop —
`buildMainGearBody` returns before `buildTooth`, so S9–S15 do not run. `generate` still calls
`buildBore` (which returns immediately, S16) and `cleanup` (S18). Nothing else in the build is
guarded on this flag; the guard lives here and inside `cleanup`.

**From:** `spec/spurgear/instructions.md` 60, 242–268, 451–453; `spec/spurgear/fusion.md` 190–200.

---

## S9 `[PROSE]` Extrude the tooth

`buildTooth(ctx)` owns this step, and **must call `self.chamferTooth(ctx)` as its last action** —
subclasses override `buildTooth` to loft instead of extrude and still end by calling it, so
`buildMainGearBody` must not chamfer separately.

Find the single tooth cross-section with the framework helper, never a hand-rolled loop search:

```
profile = find_profile_by_curve_counts(sketch, nurbs=2, arcs=2,
                                       lines=0 if ctx.toothProfileIsEmbedded else 2)
```

2 NURBS (the flanks), 2 arcs (the tooth top and the root arc between the stubs), and the 2 stubs
unless the profile is embedded. The helper rejects loops whose counts don't match and raises when
nothing does.

```
extrudeInput = component.features.extrudeFeatures.createInput(
    profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
extrude = component.features.extrudeFeatures.add(extrudeInput)
extrude.name = 'Extrude tooth'
ctx.toothBody = extrude.bodies.item(0)
```

⚠ DEFECT — the spec names `ToEntityExtentDefinition` and `PositiveExtentDirection` but never names
the call that consumes them. `setOneSideExtent(extent, direction)` is inferred from the API index;
S11 at least gives `ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`, this step gives
nothing.

**From:** `spec/spurgear/instructions.md` 226, 253–255, 271–279, 434–443, 455–459;
`PLAYBOOK.md` 151–155, 571–580.

---

## S10 `[PROSE]` Chamfer the tooth (optional)

Skipped when `ChamferTooth` is 0. Find the front face with a **single conjunction predicate**:
walk `ctx.toothBody.faces` and take the first face where **both** `face.edges.count ==
self.chamferWantEdges()` (6 on the spur base) **and** `sketchPlane.isCoPlanarTo(face.geometry)`,
with `sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`. Both of the same face — this
is not an edge-count match with a coplanarity tiebreak. If no face satisfies both, raise; no
partial-match fallback.

Then walk that face's edges and add every one to the collection **except** the root arc, which is
identified **by radius, not by relative size**: skip any edge whose
`edge.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType` and whose
`edge.geometry.radius` equals `RootCircleRadius` (the registered parameter's `.value`, tolerance
`0.001` cm). Chamfering the root arc would eat into the neighbouring tooth. Everything else — both
flanks, the tooth-top arc, both stubs — is chamfered.

```
chamferInput = component.features.chamferFeatures.createInput2()
chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, <ChamferTooth value>, False)
component.features.chamferFeatures.add(chamferInput)
```

⚠ DEFECT — `addEqualDistanceChamferEdgeSet(edges: ObjectCollection, distance: ValueInput,
isTangentChain: bool)` wants a `ValueInput` for the distance; the spec passes the parameter's bare
numeric value. Separately, `[PB-FILLET-CHAMFER]` calls the third argument `isFlipped`; the API
index calls it `isTangentChain`. Written above as the spec writes it.

Known and accepted: an embedded profile yields a 4-edge front face while `chamferWantEdges()`
stays 6, so chamfering an embedded spur tooth raises the front-face-not-found error. Helical and
herringbone override only `chamferWantEdges()`.

**From:** `spec/spurgear/instructions.md` 58, 280–283, 461–467; `PLAYBOOK.md` 480–486, 571–580.

---

## S11 `[PROSE]` Extrude the body

`buildBody(ctx)`. The gear body profile is the solid disc inside the root circle, whose boundary is
**exactly 2 arcs** — the two pieces the tooth cuts the root circle into. It is not an annulus, and
the tip circle is not part of it: the tip circle is construction geometry and construction geometry
bounds no profile.

```
profile = find_profile_by_curve_counts(sketch, arcs=2)
extrudeInput = component.features.extrudeFeatures.createInput(
    profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
extrude = component.features.extrudeFeatures.add(extrudeInput)
extrude.name = 'Extrude body'
ctx.gearBody = extrude.bodies.item(0)
ctx.gearBody.name = 'Gear Body'
```

While iterating `extrude.bodies.item(0).faces`, classify by `face.geometry.surfaceType` and capture
`ctx.extrusionExtent`: among `adsk.core.SurfaceTypes.PlaneSurfaceType` faces, the one where
`sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry)` —
the far end cap. The near cap is coplanar with the sketch plane, so `isCoPlanarTo` rules it out.
Raise if it is not found. Use the plane-geometry API, not a hand-rolled dot product.

⚠ DEFECT — the spec justifies the 2-arc count as "the two pieces the tooth's **flank-to-root
lines** cut the root circle into". In the embedded case there are no flank-to-root lines; it is the
flank splines that cut the circle. The count still holds, the reason given does not, and the spec
never states the embedded variant here the way S9 does.

**From:** `spec/spurgear/instructions.md` 226–228, 256, 469–478; `PLAYBOOK.md` 151–155, 571–580.

---

## S12 `[PROSE]` `Gear Center` construction axis

From the same face walk as S11: any face whose `surfaceType` is
`adsk.core.SurfaceTypes.CylinderSurfaceType`.

```
axisInput = component.constructionAxes.createInput()
axisInput.setByCircularFace(cylindricalFace)
ctx.centerAxis = component.constructionAxes.add(axisInput)
ctx.centerAxis.name = 'Gear Center'
ctx.centerAxis.isLightBulbOn = False
```

Raise if no cylindrical face is found.

**From:** `spec/spurgear/instructions.md` 227, 473–478; `PLAYBOOK.md` 686–689.

---

## S13 `[PROSE]` Circular-pattern the teeth

`patternTeeth(ctx)` owns this and S14, and **must call `self.createFillets(ctx)` as its last
action**.

```
bodies = adsk.core.ObjectCollection.create()
bodies.add(ctx.toothBody)
patternInput = component.features.circularPatternFeatures.createInput(bodies, ctx.centerAxis)
patternInput.quantity = adsk.core.ValueInput.createByReal(<ToothNumber value>)
patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
patternInput.isSymmetric = False
pattern = component.features.circularPatternFeatures.add(patternInput)
```

All three of quantity, `totalAngle` and `isSymmetric` are pinned explicitly; do not rely on
Fusion's defaults staying equal to them.

**From:** `spec/spurgear/instructions.md` 256, 480–484; `PLAYBOOK.md` 597–602.

---

## S14 `[PROSE]` Combine the patterned teeth into the body

One Combine-Join. `pattern.bodies` already includes the original tooth body, so feed it as-is and
do not re-add the seed — but copy it into an `ObjectCollection` first, because `pattern.bodies` is
a `BRepBodies` and `createInput(targetBody: BRepBody, toolBodies: core.ObjectCollection)` rejects
it.

```
tools = adsk.core.ObjectCollection.create()
for i in range(pattern.bodies.count):
    tools.add(pattern.bodies.item(i))
combineInput = component.features.combineFeatures.createInput(ctx.gearBody, tools)
combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
component.features.combineFeatures.add(combineInput)
```

**From:** `spec/spurgear/instructions.md` 482–484; `PLAYBOOK.md` 592–596.

---

## S15 `[PROSE]` Root fillets (optional)

`createFillets(ctx)`, called from the end of S13's method. Round the inside corner where the root
valley floor meets each tooth flank — the one that runs the full thickness parallel to the gear
axis, where bending stress concentrates. Not the front/back rim.

Two things make the edge pick fiddly:

- After the pattern-and-combine the root cylinder is usually split into one patch per valley.
  Collect **every** cylindrical face whose radius equals `RootCircleRadius`, not just the first.
- On each patch, filter to `adsk.core.Curve3DTypes.Line3DCurveType` edges, take each line's
  direction from its **geometry endpoints** —
  `edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)`, then `normalize()` — and keep it
  when `abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01`. Use exactly that tolerance; a
  tighter `> 0.999` drops valid axial edges that are slightly off from tessellation. The circular
  edges wrapping the end caps are rims, not root corners. Do **not** read the direction via
  `edge.evaluator.getTangent(0)` — parameter 0 is not guaranteed to lie in the edge's range and
  Fusion raises `RuntimeError: invalid argument parameter`.

If the collection ends up empty, return silently — no fillet feature, no error. An empty edge set
must never reach `add`.

```
filletInput = component.features.filletFeatures.createInput()
filletInput.addConstantRadiusEdgeSet(edges, <FilletRadius value>, False)
component.features.filletFeatures.add(filletInput)
```

The edge set goes on the input **itself**; there is no `filletInput.edgeSetInputs` and reaching for
it raises `AttributeError`. That is the chamfer-side shape (S10). `isTangentChain` must be `False`
— the collected edges are exactly the axial root corners, and tangent-chaining would let Fusion
round more than that.

⚠ DEFECT — `addConstantRadiusEdgeSet(entities: ObjectCollection, radius: ValueInput,
isTangentChain: bool)` wants a `ValueInput`; the spec passes `FilletRadius`'s bare numeric `.value`
(instructions.md 287 confirms `.value` is what `createFillets` reads). Written above as the spec
writes it.

**From:** `spec/spurgear/instructions.md` 86–88, 256, 283–287, 486–495; `PLAYBOOK.md` 480–486.

---

## S16 `[GO]` Bore Profile Sketch

**Proof function: `stepBoreProfileSketch`.**

`buildBore(ctx)` runs unconditionally from `generate()`, after `buildMainGearBody`, so it must
early-return in **two** cases: when `SketchOnly` is set, and when `BoreDiameter <= 0`. The
SketchOnly guard is essential — in that mode `buildBody` never ran, so `ctx.gearBody` and
`ctx.extrusionExtent` are `None` and the cut would dereference them. Do not lean on the bore
diameter being 0 in sketch-only mode; the user may have set both.

Otherwise: `self.boreSketch = self.createSketchObject('Bore Profile', ctx.plane)`, then draw the
circle **by instantiating the tooth generator on that sketch** —
`SpurGearInvoluteToothDesignGenerator(boreSketch, self)` — and calling
`drawBore(ctx.anchorPoint, boreDiameter)`, which:

```
projected = sketch.project(anchorPoint)
circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(projected.item(0), diameter / 2)
sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
sketch.geometricConstraints.addCoincident(toothGen.anchorPoint, projected.item(0))
```

The circle is not construction and its diameter dimension is driving. `diameter` is in cm.

The constructor always adds its local-origin `SketchPoint` at (0,0,0), so this sketch carries one
stray point nothing else uses — faithful behaviour, do not suppress it. **Ground it on the same
projection**, which is the last line above. Not on `boreSketch.originPoint`: that pins the point to
the plane rather than to the gear, and constraining to `originPoint` has been observed to throw
`VCS_SKETCH_SOLVING_FAILED`. Ungrounded it is free in two directions and the sketch never reaches
`isFullyConstrained`.

**From:** `spec/spurgear/instructions.md` 54, 210–212, 258–268, 321–326, 497–501;
`spec/spurgear/fusion.md` 19–31; `PLAYBOOK.md` 423–429, 430–444.

---

## S17 `[PROSE]` Extrude-cut the bore

```
profile = boreSketch.profiles.item(0)
extrudeInput = component.features.extrudeFeatures.createInput(
    profile, adsk.fusion.FeatureOperations.CutFeatureOperation)
extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
extrudeInput.participantBodies = [ctx.gearBody]
component.features.extrudeFeatures.add(extrudeInput)
```

From the target plane to `ctx.extrusionExtent`, the far end-cap face captured in S11. Going to that
face rather than a distance guarantees the bore pierces the gear whatever `Thickness` is. The cut
affects only `ctx.gearBody`.

**From:** `spec/spurgear/instructions.md` 228, 497–501; `PLAYBOOK.md` 451–457.

---

## S18 `[PROSE]` Cleanup

`cleanup(ctx)` is the **very last action** of `generate()`, after `buildBore`, and is called
**unconditionally** in both modes. The placement matters: `buildBore` re-projects
`ctx.anchorPoint` out of the Tools sketch, and projection fails once that sketch is hidden. Do not
move it up into `buildMainGearBody`, and do not guard the call — guard the sketch hiding inside it.

Split by entity kind and by mode:

- **Construction planes and axes — always, in both modes**, so no stray plane floats after a
  sketch-only run: `ctx.extrusionEndPlane.isLightBulbOn = False`, `ctx.centerAxis.isLightBulbOn =
  False`, and the normalized target plane from S4 if one was created.
- **Sketches — only on the full-build path**: `self.toolsSketch.isVisible = False`,
  `ctx.gearProfileSketch.isVisible = False`, `self.boreSketch.isVisible = False`. Sketch-only mode
  leaves Tools and Gear Profile visible; inspecting them is the point of that mode.

Guard each entity individually — the `Gear Center` axis and the Bore Profile sketch do not exist in
sketch-only mode. `isVisible = False` hides sketches, `isLightBulbOn = False` hides construction
planes and axes; never cross them.

**From:** `spec/spurgear/instructions.md` 202–205, 258–268, 451–453; `spec/spurgear/fusion.md`
190–200; `PLAYBOOK.md` 558–570.
