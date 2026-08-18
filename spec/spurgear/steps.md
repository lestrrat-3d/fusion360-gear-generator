# Spur Gear — compiled step list

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/spurgear/instructions.md` | `f5ffe3451454bb3b187b1318e47b92281d9f0bb0` |
| `spec/spurgear/fusion.md` | `3e8b0b338e80a1199eb7eb95f9f004a6f0bb747d` |
| `spec/helicalgear/fusion.md` | `83fac920272341e3c4584f16031478a69b7472e7` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `e2d9d754012ed9e0e0306cd1327c36caac690f61` |

The proof is `sketches_test.go`, `solids_test.go` and `geometry_test.go` in package `spurgear_test`.

## 0.1 `[PROSE]` Module surface — classes, constants and the call graph

`lib/geargen/spurgear.py` defines exactly four classes, and every name below is public API because
`helicalgear.py` and `herringbonegear.py` subclass them by name and `commands/spurgear/entry.py`
binds two of them.

- `SpurGearCommandInputsConfigurator` — plain class, one `@classmethod configure(cls, cmd)` that
  adds the dialog inputs of step 0.2. A subclass adds its own input by calling
  `super().configure(cmd)` first and appending after it.
- `SpurGearGenerationContext(GenerationContext)` — data carrier. Its `__init__` declares, each
  `cast(None)`-initialised: `plane`, `anchorPoint`, `extrusionEndPlane`, `gearProfileSketch`,
  `toothBody`, `gearBody`, `centerAxis`, `extrusionExtent`, and `toothProfileIsEmbedded`, which
  starts `False`.
- `SpurGearInvoluteToothDesignGenerator` — plain class, constructed `(sketch, parent, angle=0)`.
  Stores `self.toothAngle = angle` as an incidental field, adds its movable local origin
  `self.anchorPoint` as a fresh `SketchPoint` at (0, 0, 0), and exposes `drawCircles`, `drawTooth`,
  `drawBore`, `draw`, `calculateInvolutePoint`, `getParameter` and `getParameterValue`.
- `SpurGearGenerator(Generator)` — the orchestrator. `__init__` pre-initialises
  `self._lastToothEmbedded = False`, `self.toolsSketch = None` and `self.boreSketch = None`.
  `prefixBase()` returns `'SpurGear'`.

`configure`, `prefixBase` and `generate` are named above as methods this module *defines* for the
framework and its subclasses to call. Nothing inside the module calls them, so naming them here
describes the surface rather than requiring a call. `find_circle_by_radius` is one of the two ways
the spec allows a drawing step to reach a circle drawn by `drawCircles` — the other is a direct
reference kept from that method — so naming it does not require the implementation to take that
route.

<!-- check-step-calls: ignore configure prefixBase generate find_circle_by_radius -->

Imports are explicit — no `import *`: `math`, `adsk.core`, `adsk.fusion`, `futil`, `to_cm` and
`get_design` from `.misc`, `Generator`, `GenerationContext`, `get_value`, `get_boolean` and
`get_selection` from `.base`, `get_normal`, `find_profile_by_curve_counts` and
`find_circle_by_radius` from `.utilities`.

Module-level constants, all exported and imported by name elsewhere: input ids `INPUT_ID_PARENT`,
`INPUT_ID_PLANE`, `INPUT_ID_ANCHOR_POINT`, `INPUT_ID_MODULE`, `INPUT_ID_TOOTH_NUMBER`,
`INPUT_ID_PRESSURE_ANGLE`, `INPUT_ID_BORE_DIAMETER`, `INPUT_ID_THICKNESS`,
`INPUT_ID_CHAMFER_TOOTH`, `INPUT_ID_SKETCH_ONLY`; parameter names `PARAM_MODULE`,
`PARAM_TOOTH_NUMBER`, `PARAM_PRESSURE_ANGLE`, `PARAM_BORE_DIAMETER`, `PARAM_THICKNESS`,
`PARAM_CHAMFER_TOOTH`, `PARAM_SKETCH_ONLY`, `PARAM_PITCH_DIAMETER`, `PARAM_PITCH_RADIUS`,
`PARAM_BASE_DIAMETER`, `PARAM_BASE_RADIUS`, `PARAM_ROOT_DIAMETER`, `PARAM_ROOT_RADIUS`,
`PARAM_TIP_DIAMETER`, `PARAM_TIP_RADIUS`, `PARAM_INVOLUTE_STEPS`, `PARAM_TOOTH_SPACE_ANGLE`,
`PARAM_TOOTH_SPACE_ARC`, `PARAM_FILLET_CLEARANCE`, `PARAM_FILLET_RADIUS`.

The call graph is fixed, because subclasses override at its boundaries and call `super()` at
specific points. Do not merge or reorder these methods:

```
generate(inputs)
  → processInputs(inputs)                       # step 0.3
  → component.name = generateName()
  → normalize self.plane to a ConstructionPlane # step 1
  → ctx = newContext()
  → prepareTools(ctx)                           # steps 2 and 3
  → buildMainGearBody(ctx)
        → buildSketches(ctx)                    # step 4
        → if SketchOnly: show the Gear Profile sketch and stop   # step 5
          else:
            → buildTooth(ctx)                   # steps 6 and 7 (buildTooth ends by calling chamferTooth)
            → buildBody(ctx)                    # step 8
            → patternTeeth(ctx)                 # steps 9, 10 and 11 (patternTeeth ends by calling createFillets)
  → buildBore(ctx)                              # steps 12 and 13
  → cleanup(ctx)                                # step 14, unconditional, last
```

`buildTooth` MUST call `self.chamferTooth(ctx)` as its last action, and `buildMainGearBody` must
not chamfer separately. `patternTeeth` MUST call `self.createFillets(ctx)`. `cleanup(ctx)` is the
last action of `generate()`, after `buildBore`, and is called unconditionally — the SketchOnly
distinction lives inside it, because `buildBore` re-projects `ctx.anchorPoint` from the Tools
sketch and projection fails once that sketch is hidden.

Three overridable hooks exist on the generator and are consumed at different points:
`chamferWantEdges()` returns `6` and is read by `chamferTooth`; `filletHelixFactorExpression()`
returns the string `'1'` and is read ONLY by `registerDerivedParameters`, spliced in as the last
factor of the live `FilletRadius` expression; `addExtraPrimaryParameters(self, inputs)` is a no-op
that `processInputs` calls between the input-sourced and the derived parameters.

`generateName()` returns
`'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression, thickness.expression)`
— the parameters' `.expression` strings, not their `.value`.

`bevelgear.py` borrows the tooth generator with a `VirtualSpurProxy` parent, so inside
`drawCircles`, `drawTooth`, `draw` and the helpers they call, parameters may be read ONLY from the
keys `Module`, `ToothNumber`, `PressureAngle`, `PitchCircleDiameter`, `PitchCircleRadius`,
`BaseCircleDiameter`, `BaseCircleRadius`, `RootCircleDiameter`, `RootCircleRadius`,
`TipCircleDiameter`, `TipCircleRadius` and `InvoluteSteps`. Any other key raises `KeyError` there.

**From:** `spec/spurgear/instructions.md` L9–11, L13–33, L152–178, L253–268, L270–336, L338–346,
L381–406; `.claude/skills/generate-gear/PLAYBOOK.md` L17–41, L42–74, L75–101, L242–266, L744–761

Fusion API calls: none — this step declares structure only.

## 0.2 `[PROSE]` Command dialog inputs

`SpurGearCommandInputsConfigurator.configure(cls, cmd)` adds the inputs to `cmd.commandInputs` in
exactly this order. The order is the dialog's display order and is NOT the `processInputs` read
order; putting the selections last because they are read first has the rule backwards.

1. Target Plane, id `plane` — `addSelectionInput('plane', 'Target Plane', ...)`, filters
   `adsk.core.SelectionCommandInput.ConstructionPlanes` and
   `adsk.core.SelectionCommandInput.PlanarFaces`, then `setSelectionLimits(1, 1)`. It is added
   first so it owns the dialog's initial focus.
2. Anchor Point, id `anchorPoint` — `addSelectionInput(...)`, filters
   `adsk.core.SelectionCommandInput.ConstructionPoints` and
   `adsk.core.SelectionCommandInput.SketchPoints`, then `setSelectionLimits(1, 1)`.
3. Module, id `module` — `addValueInput('module', 'Module', '', adsk.core.ValueInput.createByReal(1))`.
4. Tooth Number, id `toothNumber` — `addValueInput('toothNumber', 'Tooth Number', '', adsk.core.ValueInput.createByReal(17))`.
5. Pressure Angle, id `pressureAngle` — `addValueInput('pressureAngle', 'Pressure Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(20)))`.
   The display unit is degrees and the default is in radians, because a `createByReal` default is
   always in Fusion's internal units.
6. Bore Diameter, id `boreDiameter` — `addStringValueInput('boreDiameter', 'Bore Diameter', '0 mm')`,
   a string input so it accepts expressions.
7. Thickness, id `thickness` — `addValueInput('thickness', 'Thickness', 'mm', adsk.core.ValueInput.createByReal(to_cm(10)))`.
   Display unit mm, default in cm.
8. Apply chamfer to teeth, id `chamferTooth` — `addValueInput('chamferTooth', 'Apply chamfer to teeth', 'mm', adsk.core.ValueInput.createByReal(0))`.
9. Generate sketches, but do not build body, id `sketchOnly` — `addBoolValueInput(...)`, a checkbox.
10. Parent Component, id `parentComponent` — `addSelectionInput(...)`, filters
    `adsk.core.SelectionCommandInput.Occurrences` and
    `adsk.core.SelectionCommandInput.RootComponents`, `setSelectionLimits(1, 1)`, pre-selecting
    `get_design().rootComponent`. Last, because its default is right for most uses.

Filters are enum constants, never quoted strings.

**From:** `spec/spurgear/instructions.md` L37–62, L90–146, L171–178;
`.claude/skills/generate-gear/PLAYBOOK.md` L53–61, L128–136, L138–143, L332–334, L474–479

Fusion API calls: `cmd.commandInputs.addSelectionInput(...)`, `selectionInput.addSelectionFilter(...)`,
`selectionInput.setSelectionLimits(1, 1)`, `cmd.commandInputs.addValueInput(...)`,
`cmd.commandInputs.addStringValueInput(...)`, `cmd.commandInputs.addBoolValueInput(...)`,
`adsk.core.ValueInput.createByReal(...)`, `math.radians(20)`, `to_cm(10)`, `get_design()`

## 0.3 `[PROSE]` processInputs — read the dialog and register the parameters

Read order is load-bearing and is the opposite of the display order. Registering any parameter
creates the occurrence, and creating the occurrence shifts Fusion's active component context, which
can drop a `SelectionCommandInput`'s entity. So:

1. `get_selection(inputs, INPUT_ID_PARENT)` first, resolving an `Occurrence` to its `.component`
   into `self.parentComponent`; raise on the wrong count or type.
2. `get_selection(inputs, INPUT_ID_PLANE)` into `self.plane` and
   `get_selection(inputs, INPUT_ID_ANCHOR_POINT)` into `self.anchorPoint`. Nothing that touches the
   design has run yet.
3. Register the input-sourced parameters, each read with the helper matching the input's declared
   type: `get_value(inputs, INPUT_ID_MODULE, '')` → `Module` in units `''`; `ToothNumber` in `''`;
   `PressureAngle` in `'rad'`; `BoreDiameter` in `'mm'`; `Thickness` in `'mm'`; `ChamferTooth` in
   `'mm'`. `SketchOnly` is read with `get_boolean(inputs, INPUT_ID_SKETCH_ONLY)` and registered as
   the real number 1 or 0 — never `get_value`, which reads `.expression` and raises
   `AttributeError` on a `BoolValueCommandInput`.
   **`Module` is registered unitless (`''`), NOT `'mm'`**, so `generateName` renders `M=1` and the
   `mm`-registered expressions below read it as a bare factor.
4. Call the `addExtraPrimaryParameters(inputs)` hook.
5. Register the derived parameters as live expressions with
   `adsk.core.ValueInput.createByString(...)`, each name prefixed by `parameterName(...)`:
   - `PitchCircleDiameter` = `Module * ToothNumber`, mm
   - `PitchCircleRadius` = `PitchCircleDiameter / 2`, mm
   - `BaseCircleDiameter` = `PitchCircleDiameter * cos(PressureAngle)`, mm
   - `BaseCircleRadius` = `BaseCircleDiameter / 2`, mm
   - `RootCircleDiameter` = `PitchCircleDiameter - 2.5 * Module`, mm
   - `RootCircleRadius` = `RootCircleDiameter / 2`, mm
   - `TipCircleDiameter` = `PitchCircleDiameter + 2 * Module`, mm
   - `TipCircleRadius` = `TipCircleDiameter / 2`, mm
   - `InvoluteSteps` = 15, unitless
   - `FilletClearance` = 0.9, unitless
6. `ToothSpaceAngleAtRoot` cannot be a live expression: Fusion refuses to subtract a radian-valued
   `PressureAngle` from the unitless output of `tan()`. Compute it in Python as
   `math.pi / toothNumber - 2 * (math.tan(pressureAngle) - pressureAngle)` and register it with
   `adsk.core.ValueInput.createByReal(...)` in units **`''`**, not `'rad'` — the next parameter
   multiplies it by a length, and `mm·rad` is rejected as `RuntimeError: Invalid expression`.
7. `ToothSpaceArcAtRoot` = `RootCircleRadius * ToothSpaceAngleAtRoot`, mm, live.
8. `FilletRadius` = `(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>`, mm, live, where
   `<factor>` is whatever `filletHelixFactorExpression()` returns — `'1'` for the spur base. This
   is the only place that hook is read.

Every dimension and feature input downstream is set from a parameter's current numeric `.value`,
not as a live link: editing a `SpurGear<N>_…` parameter does not change an existing gear.

**From:** `spec/spurgear/instructions.md` L35, L37–89, L147–151, L318–331, L408–417;
`.claude/skills/generate-gear/PLAYBOOK.md` L103–126, L196–228, L556–557;
`spec/spurgear/fusion.md` L202–209

Fusion API calls: `get_selection(inputs, id)`, `get_value(inputs, id, units)`,
`get_boolean(inputs, id)`, `self.addParameter(name, valueInput, units, comment)`,
`self.parameterName(name)`, `design.userParameters.add(name, valueInput, units, comment)`,
`adsk.core.ValueInput.createByReal(...)`, `adsk.core.ValueInput.createByString(...)`,
`math.tan(pressureAngle)`, `math.pi`

`parameterName` stays a requirement. `addParameter` does apply the prefix on the generator's
behalf, but that covers only the parameter being created; the derived parameters are registered as
live expression strings that name OTHER parameters, and there is no way to build
`'<prefix>_PitchCircleDiameter / 2'` without calling `parameterName` for the operand. So step 5
above calls it directly, and the module must. (`design.userParameters.add(...)` is the opposite
case — it is what `addParameter` does underneath, named here to say where a registered parameter
ends up, not as a call this module makes itself.)

## 1 `[PROSE]` Normalize the Target Plane

If `self.plane` is not already a `ConstructionPlane` — the user may have picked a planar face —
create a coplanar construction plane and replace `self.plane` with it, so profile detection later
is not confused by the selected face's own profile. Keep a handle to it: step 14 switches its light
bulb off, and only if it was created here.

The offset argument is a `ValueInput`, not a bare number:
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))`. Never pass the bare
number `setByOffset(plane, 0)`, which raises `TypeError` at runtime.

`ctx.plane` and `self.plane` both hold the normalized plane; subclasses read `self.plane` directly,
so keep both available.

This step is prose because the proof has no construction planes: neither the sketch engine nor
`decad` models a datum that a later feature ends on, and a coplanar offset of zero changes no
geometry the proof could measure.

**From:** `spec/spurgear/instructions.md` L39, L257, L266–268, L419–423;
`.claude/skills/generate-gear/PLAYBOOK.md` L230–240, L674–685

Fusion API calls: `component.constructionPlanes.createInput()`,
`planeInput.setByOffset(planarEntity, offset)`, `component.constructionPlanes.add(planeInput)`,
`adsk.core.ValueInput.createByReal(0)`

## 2 `[GO]` Tools Sketch

One timeline entry: a sketch named `Tools` on the target plane, created with
`self.createSketchObject('Tools', self.plane)` and made visible while later sketches project from
it. It draws no geometry of its own. Project the user's Anchor Point into it and keep the first
projected entity as `ctx.anchorPoint`; that projection is the canonical handle every later sketch
re-projects from, so the whole gear tracks the anchor if the user moves it. Store the sketch on
`self.toolsSketch` so step 14 can hide it.

Leave it visible through step 13: `buildBore` projects from it again, and projection fails on a
hidden sketch.

The proof function is `stepToolsSketch`. It models the projection with the sketch engine's
externally-locked reference point, because a Fusion projection is a reference that the sketch may
not move, and asserts what the spec pins here — that the sketch holds that one point and no curves.

**From:** `spec/spurgear/instructions.md` L41, L228–230, L258, L285, L425–427, L481–485;
`spec/spurgear/fusion.md` L19–24;
`.claude/skills/generate-gear/PLAYBOOK.md` L92–96, L376–385, L430–444, L558–570

Fusion API calls: `self.createSketchObject('Tools', self.plane)`,
`toolsSketch.project(self.anchorPoint)`, `toolsSketch.isVisible`

## 3 `[PROSE]` Extrusion End Plane

One timeline entry: a construction plane named `Extrusion End Plane`, offset from the target plane
by `Thickness`, created in `prepareTools` alongside the Tools sketch. Its only purpose is to be the
to-entity target of the tooth and body extrudes, so both end on the same well-defined face. Store
it as `ctx.extrusionEndPlane`; step 14 switches its light bulb off.

The offset is a `ValueInput` carrying the `Thickness` parameter's current numeric value:
`adsk.core.ValueInput.createByReal(thickness)`.

Prose for the same reason as step 1 — the proof has no construction planes. What the plane
determines, the extrude extent, is asserted instead in steps 6 and 8, which check that each body
spans exactly `Thickness` from the sketch plane.

**From:** `spec/spurgear/instructions.md` L259, L285, L423, L429;
`spec/spurgear/fusion.md` L202–209;
`.claude/skills/generate-gear/PLAYBOOK.md` L674–685

Fusion API calls: `component.constructionPlanes.createInput()`,
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))`,
`component.constructionPlanes.add(planeInput)`, `endPlane.isLightBulbOn`

## 4 `[GO]` Gear Profile Sketch

One timeline entry, and the whole constraint scheme. `buildSketches(ctx)` creates a sketch named
`Gear Profile` on the target plane, stores it as `ctx.gearProfileSketch`, constructs
`SpurGearInvoluteToothDesignGenerator(gearProfileSketch, self)` and calls
`draw(ctx.anchorPoint, angle=0)`, then copies `ctx.toothProfileIsEmbedded = self._lastToothEmbedded`.
`draw` performs, in order, `drawCircles()`, `drawTooth(angle)`, the anchoring, and — only when
`angle != 0` — the confirming angular dimension's value-set as its very last action.

**The constructor** adds the movable local origin: a fresh `SketchPoint` at (0, 0, 0) held as
`self.anchorPoint`. It is not `sketch.originPoint`, which is immutable and cannot be
coincident-constrained to anything projected in.

**`drawCircles()`** draws, in this order, the Root circle at `RootCircleRadius` as solid geometry
and then the Tip, Base and Pitch circles at their radii as construction geometry. Every circle is
created by passing the local-origin `SketchPoint` **directly** as the centre —
`sketch.sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)` — so all four share that
one point; never pass `localOrigin.geometry` and then add a centre coincident. Each gets a driving
diameter dimension, its text point off-centre. Each is labelled with along-path sketch text whose
string is `'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` using the radii's internal cm
`.value`, with `size = TipCircleRadius - RootCircleRadius`, and that same `size` passed as the text
height.

**`drawTooth(angle)`** rotates by the `angle` argument that arrives from `draw()` at call time,
never by the constructor-stored `self.toothAngle`; helical constructs the generator at 0 and passes
its helix angle to `draw`, so using the stored field would draw a flat tooth.

1. Sample `InvoluteSteps` points along the flank, endpoint-inclusive: sample `i` sits at radius
   `BaseCircleRadius + (TipCircleRadius - BaseCircleRadius) * i / (steps - 1)`, so the first is
   exactly on the base circle and the last exactly on the tip circle. Do not clamp the start to the
   root circle. Each sample is `calculateInvolutePoint(BaseCircleRadius, r)`; drop any that returns
   `None`.
   `calculateInvolutePoint(baseRadius, intersectionRadius)` returns `None` when
   `intersectionRadius < baseRadius`, and otherwise `alpha = math.acos(baseRadius / intersectionRadius)`,
   `t = math.tan(alpha)`, `x = baseRadius * (math.cos(t) + t * math.sin(t))`,
   `y = baseRadius * (math.sin(t) - t * math.cos(t))`. The curve parameter is `tan(alpha)`, NOT
   `inv(alpha) = tan(alpha) - alpha`.
2. Mirror the samples across +X — negate y — before rotating. The standard parametric involute
   spirals the wrong way for a left flank and would give a tooth wider at the tip than at the root.
3. With `(px, py) = calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`,
   `rotate_angle = math.pi / (2 * toothNumber) - math.atan2(-py, px)`. The `-py` is the mirror
   applied to the analytic point; `atan2(py, px)` is wrong.
4. Rotate the mirrored samples by `rotate_angle` to get the left flank, mirror that across X for
   the right flank, then rotate BOTH flanks — and the tooth-top point and every rib midpoint seed —
   by `angle`. Draw the tooth at its final angular position; do not draw it flat and swing it into
   place with the dimension.
5. Draw each flank as a `SketchFittedSpline` through its point collection.
6. Tooth-top arc. Add a tooth-top `SketchPoint` at
   `(TipCircleRadius * math.cos(angle), TipCircleRadius * math.sin(angle))` and constrain it
   coincident to the tip circle. Create the arc with
   `sketch.sketchCurves.sketchArcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)`,
   passing the local origin and the two splines' end `SketchPoint`s directly so the arc shares all
   three. Add **no** diameter dimension: a free centre plus a diameter leaves an arc that can bulge
   inward through the tooth at the same radius through the same two ends.
7. Spine and +X reference. Draw the spine as a construction line
   `addByTwoPoints(localOrigin, toothTopPoint)`, sharing both existing points; add no start
   coincident and do not constrain its end onto the arc. Then, for **every** angle including 0:
   add a far endpoint at `(TipCircleRadius, 0)` and pin it with two signed dimensions from the local
   origin — a horizontal one at `TipCircleRadius` and a vertical one at 0, never a coincident to
   the tip circle; draw the reference line from the origin to it and mark it construction; add the
   angular dimension `addAngularDimension(reference, spine, textPoint)` in that argument order,
   with the text point on the bisector `(R * math.cos(angle / 2), R * math.sin(angle / 2))` for a
   small R so Fusion picks the angle and not its supplement. A plain `addHorizontal` on the spine
   is not a substitute at angle 0: it fixes the direction but not which way the spine points.
8. Ribs — one per fit-point index, for **all** N indices including the first and the last, built in
   exactly this order:
   1. `addByTwoPoints(leftSpline.fitPoints.item(i), rightSpline.fitPoints.item(i))`, passing the
      two fit points directly; mark construction.
   2. One **signed** dimension across the spine at the measured delta — vertical when
      `abs(math.cos(angle)) >= abs(math.sin(angle))`, horizontal otherwise. An aligned dimension
      gives only the length, which the two flanks satisfy equally well swapped over.
   3. A fresh midpoint `SketchPoint`, seeded already on the spine: with
      `t = fitX * math.cos(angle) + fitY * math.sin(angle)`, at
      `(t * math.cos(angle), t * math.sin(angle))`. Not the rib's true 2-D midpoint, and not
      `(fitX, 0)` for a rotated tooth.
   4. `addCoincident(midpoint, spine)`.
   5. `addMidPoint(midpoint, rib)`.
   6. `addPerpendicular(spine, rib)` — **skipped for the last rib only**, because the tooth-top arc
      already holds the two tips at equal radius either side of the spine and Fusion rejects the
      duplicate with `VCS_SKETCH_OVER_CONSTRAINTS`.

   Then chain the midpoints with one **signed** dimension each along the spine direction —
   horizontal when the rib dimension was vertical, vertical when it was horizontal — measuring from
   the previous midpoint, and **from the local origin for the first rib**. Without that
   origin-to-first dimension the chain slides along the spine as a unit and the sketch never fully
   constrains.
9. Close the tooth at the root. Let `firstRadius` be the distance from the local origin to the left
   flank's first fit point. `embedded = firstRadius < RootCircleRadius`, a strict comparison of raw
   values with no tolerance; exact equality counts as NOT embedded and draws a zero-length stub.
   Write the result to `self.parent._lastToothEmbedded`.
   When not embedded, draw a short radial line on each side as
   `addByTwoPoints(rootEndGeometry, flankStartFitPoint)`, passing the spline's start `SketchPoint`
   directly, and place the root end with exactly two signed dimensions from the local origin, a
   horizontal one at its delta-x and a vertical one at its delta-y — and nothing else. "Root end on
   the root circle" plus "origin on the line" is satisfied by two points, the second of them across
   the gear, and the sketch then reaches DOF 0 with a long line straight through the centre.

**The anchoring** happens inside `draw()`, not in `buildSketches` after it returns: project
`ctx.anchorPoint` into the Gear Profile sketch and add a coincidence between that fresh projection
and the local origin — the generator's `self.anchorPoint`, not `sketch.originPoint`. Every other
piece of geometry is placed relative to the local origin, so this one constraint drags the whole
tooth onto the anchor.

**The confirming rotation** is the last action of `draw()`, after the anchoring:
`if angle != 0: spineAngularDimension.parameter.value = angle`.

The sketch closes exactly two regions and their curve counts are a contract step 6 and step 8 match
on: the tooth section, 6 curves when a stub was drawn and 4 when the profile is embedded, and the
disc inside the root circle, 2 arcs. Both exist only because the tooth meets the root circle and
cuts it in two.

The proof function is `stepGearProfileSketch`. It builds this scheme in the sketch engine and gates
it on the engine's full verdict — DOF 0, no redundant or conflicting constraint, valid profiles, a
system that is not near-singular, and no discrete ambiguity — across the regime the spec declares:
several module and tooth-number pairs, the whole signed angle range including both quarter turns
and the half turn, a three-sample rib count beside the standard fifteen, and both routes into the
embedded shape with a case either side of each threshold. It reads the solved drawing back and
asserts the spine really sits at `angle`, that the tooth region carries the contracted curve count
and two spline flanks, that its boundary takes only part of the root circle, and that the disc
region's area is the root circle's.

**From:** `spec/spurgear/instructions.md` L180–251, L260, L265, L308–310, L338–380, L431–440,
L442–479, L481–485;
`spec/spurgear/fusion.md` L17–43, L45–60, L62–67, L69–87, L89–112, L114–149, L151–186;
`.claude/skills/generate-gear/PLAYBOOK.md` L336–403, L413–450, L458–473, L502–533, L534–535,
L547–555, L581–591

Fusion API calls: `self.createSketchObject('Gear Profile', self.plane)`,
`sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`,
`sketch.sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)`,
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`,
`sketch.sketchTexts.createInput2(text, height)`,
`textInput.setAsAlongPath(curve, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`,
`sketch.sketchTexts.add(textInput)`, `adsk.core.ObjectCollection.create()`,
`sketch.sketchCurves.sketchFittedSplines.add(fitPoints)`,
`sketch.sketchCurves.sketchArcs.addByCenterStartEnd(localOrigin, rightEnd, leftEnd)`,
`sketch.sketchCurves.sketchLines.addByTwoPoints(pointOne, pointTwo)`,
`sketch.geometricConstraints.addCoincident(point, entity)`,
`sketch.geometricConstraints.addMidPoint(point, curve)`,
`sketch.geometricConstraints.addPerpendicular(lineOne, lineTwo)`,
`sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, orientation, textPoint)`,
`adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation`,
`adsk.fusion.DimensionOrientations.VerticalDimensionOrientation`,
`sketch.sketchDimensions.addAngularDimension(reference, spine, textPoint)`,
`gearProfileSketch.project(ctx.anchorPoint)`, `find_circle_by_radius(sketch, radius)`,
`math.acos(x)`, `math.tan(alpha)`, `math.atan2(-py, px)`, `math.cos(angle)`, `math.sin(angle)`

## 5 `[PROSE]` Sketch-Only Short-Circuit

If the `SketchOnly` parameter reads true — `self.getParameterAsBoolean(PARAM_SKETCH_ONLY)` — set
`ctx.gearProfileSketch.isVisible = True` and return from `buildMainGearBody` without building
anything: no tooth extrude, no body extrude, no pattern, no combine, no fillet. `buildBore` and
`cleanup` still run, and `cleanup` leaves the sketches visible in this mode.

Prose because the branch produces no timeline entry of its own; it decides whether steps 6 to 11
happen. Its consequences are proved where they land: step 13's guard case, and step 14's per-mode
split.

**From:** `spec/spurgear/instructions.md` L60, L147–151, L287–295, L487–489;
`spec/spurgear/fusion.md` L188–200

Fusion API calls: `self.getParameterAsBoolean(name)`, `ctx.gearProfileSketch.isVisible`

## 6 `[GO]` Extrude the Tooth

One timeline entry. Find the tooth cross-section with
`find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`
— the framework helper, not a re-implemented loop search; it raises when nothing matches and never
falls back to a wrong profile. Extrude it as a **New Body** from the target plane to
`ctx.extrusionEndPlane`, using `ToEntityExtentDefinition` in the positive extent direction. Name
the feature `Extrude tooth` and store the body as `ctx.toothBody`.

The proof function is `stepExtrudeTooth`. Its case table sweeps module, tooth number and thickness,
both sides of the embedded branch — which is what changes the curve count the search matches on —
and a coarse involute sample count. It asserts the body spans exactly `Thickness` from the sketch
plane, reaches out to the tip circle and no further, and reaches in to the root circle and no
further, which is the tooth section standing outside the disc.

Substitution: the proof extrudes the same tooth with each flank chorded through the same involute
samples, because `decad` refuses to record a profile boundary whose trim it cannot certify and the
sketch engine withholds certification from every edge of a sketch holding a spline, and because a
spline-walled loop is separately refused by the extruder. The curve-count contract this step
matches on is therefore asserted in step 4, on the sketch that really draws it.

**From:** `spec/spurgear/instructions.md` L218–226, L261, L265, L311–315, L491–495;
`.claude/skills/generate-gear/PLAYBOOK.md` L151–158, L571–580, L607–614

Fusion API calls: `find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=2)`,
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`component.features.extrudeFeatures.add(extrudeInput)`

## 7 `[PROSE]` Chamfer the Tooth

`buildTooth` calls `self.chamferTooth(ctx)` as its last action, so this is triggered from inside
step 6's method and never separately from `buildMainGearBody`. If `ChamferTooth` is not greater
than 0, return; otherwise it is one timeline entry.

Find the front face with a **single conjunction predicate**: walk `ctx.toothBody.faces` and take the
first face for which **both** `face.edges.count == self.chamferWantEdges()` — 6 for the spur base —
**and** `sketchPlane.isCoPlanarTo(face.geometry)`, with
`sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`. Both of the same face; this is not an
edge-count match with a coplanarity tiebreak. If no face satisfies both, raise; do not fall back to
a partial match. An embedded profile yields a 4-edge front face while `chamferWantEdges()` stays 6,
so chamfering an embedded spur tooth raises that error — a known, accepted limitation.

Then walk the front face's edges and add each to an `ObjectCollection`, **skipping** any edge whose
`edge.geometry.curveType` is `adsk.core.Curve3DTypes.Arc3DCurveType` and whose `edge.geometry.radius`
equals `RootCircleRadius` within 0.001 cm. That is the root arc, and chamfering it would eat into
the neighbouring tooth. Everything else on the face — the two flanks, the tooth-top arc and the two
flank-to-root lines — is chamfered. Apply with `chamferFeatures.createInput2()` then
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distance, False)` — the edge set
goes on the input's `chamferEdgeSets` collection, which is the mirror image of the fillet shape in
step 11.

Helical and herringbone override only `chamferWantEdges()`, whose value and caveat belong to their
own spec.

**This step is prose because no substitute survives the proof's gate, and the reason is recorded in
`solids_test.go` beside `stepExtrudeTooth`, which builds the body it would chamfer.** In short:
`decad` chamfers a prism's cap only as one or more complete cap loops, so the spec's
all-but-the-root-arc selection is refused outright; the complete-loop substitute builds, but every
size, thickness, chamfer distance and sample count tried leaves the resulting cap-blend body
reporting its volume beyond the verification tolerance, which the solid gate rejects and which
passing a weaker gate would only hide.

**From:** `spec/spurgear/instructions.md` L290, L311–318, L497–503;
`spec/helicalgear/fusion.md` L69–84;
`.claude/skills/generate-gear/PLAYBOOK.md` L480–486, L571–580

Fusion API calls: `self.chamferWantEdges()`, `ctx.gearProfileSketch.referencePlane`,
`sketchPlane.isCoPlanarTo(face.geometry)`, `adsk.core.ObjectCollection.create()`,
`adsk.core.Curve3DTypes.Arc3DCurveType`,
`component.features.chamferFeatures.createInput2()`,
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distance, False)`,
`component.features.chamferFeatures.add(chamferInput)`,
`adsk.core.ValueInput.createByReal(chamferDistance)`

## 8 `[GO]` Extrude the Body

One timeline entry. Find the gear body profile — the solid disc inside the root circle, bounded by
**exactly 2 arcs**, the two pieces the tooth cut the root circle into — with
`find_profile_by_curve_counts(sketch, arcs=2)`. It is not an annulus: the tip circle is construction
geometry and bounds no profile. Extrude it as a **New Body** from the target plane to
`ctx.extrusionEndPlane` in the positive extent direction, name the feature `Extrude body`, name the
body `Gear Body` and store it as `ctx.gearBody`.

While iterating `extrude.bodies.item(0).faces`, capture two references, classifying each face by
`face.geometry.surfaceType`:

- From any face whose type is `adsk.core.SurfaceTypes.CylinderSurfaceType`, build the `Gear Center`
  construction axis: `constructionAxes.createInput()` → `axisInput.setByCircularFace(face)` →
  `constructionAxes.add(axisInput)`. Name it `Gear Center`, set `isLightBulbOn = False`, store it
  as `ctx.centerAxis`.
- Among faces whose type is `adsk.core.SurfaceTypes.PlaneSurfaceType`, take the one where
  `sketchPlane.isParallelToPlane(face.geometry)` and **not** `sketchPlane.isCoPlanarTo(face.geometry)`
  — the far end cap, the target of step 13's cut — and store it as `ctx.extrusionExtent`. Use the
  plane-geometry API, not a hand-rolled dot product.

Raise if either reference is not found.

The proof function is `stepExtrudeBody`. It asserts the body's volume is the full root-circle disc's
and not an annulus, that it spans exactly `Thickness` from the sketch plane, that it offers the two
planar end caps this step's far-cap search picks between, and that it offers the cylindrical face the
`Gear Center` axis is built from.

Substitution: the proof draws the disc as one whole circle rather than as the two arcs the tooth
splits it into, for the recording reason given in step 6; the two-arc count is asserted in step 4 on
the sketch that draws it, and both descriptions bound the same disc.

**From:** `spec/spurgear/instructions.md` L218–226, L262–264, L291, L505–514;
`.claude/skills/generate-gear/PLAYBOOK.md` L151–158, L410, L571–580, L639–660, L686–689

Fusion API calls: `find_profile_by_curve_counts(sketch, arcs=2)`,
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`component.features.extrudeFeatures.add(extrudeInput)`,
`adsk.core.SurfaceTypes.CylinderSurfaceType`, `adsk.core.SurfaceTypes.PlaneSurfaceType`,
`component.constructionAxes.createInput()`, `axisInput.setByCircularFace(cylindricalFace)`,
`component.constructionAxes.add(axisInput)`,
`sketchPlane.isParallelToPlane(face.geometry)`, `sketchPlane.isCoPlanarTo(face.geometry)`

## 9 `[GO]` Pattern the Teeth

One timeline entry. Circular-pattern `ctx.toothBody` around the `Gear Center` axis. Put the tooth
body in an `ObjectCollection`, then `circularPatternFeatures.createInput(bodies, ctx.centerAxis)`
and pin all three inputs explicitly rather than trusting Fusion's defaults:
`patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)`,
`patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')` — a full turn, as a
string expression — and `patternInput.isSymmetric = False`. Then
`circularPatternFeatures.add(patternInput)`.

The proof function is `stepPatternTeeth`. It asserts every placement carries the seed's volume, sits
at exactly its share of the full turn measured from the seed, and stands outside the root circle —
and, by building the place one short of a full turn, that the turn really was divided into Tooth
Number parts and the pattern was not symmetric about the seed.

Substitution: only three of the placements are built. `decad` decides interference pairwise, and a
pair of chorded teeth a few places apart comes back undecidable, which the solid gate rejects — as
it should, since an undecided pair is not a proven-disjoint one. The seed, its immediate successor
and the place one short of a full turn are what the proof holds; the rest are the same operation at
the same spacing. The chorded flank substitution of step 6 applies here too.

**From:** `spec/spurgear/instructions.md` L263, L292, L516–520;
`.claude/skills/generate-gear/PLAYBOOK.md` L592–602

Fusion API calls: `adsk.core.ObjectCollection.create()`,
`component.features.circularPatternFeatures.createInput(inputEntities, axis)`,
`adsk.core.ValueInput.createByReal(toothNumber)`,
`adsk.core.ValueInput.createByString('360 deg')`,
`component.features.circularPatternFeatures.add(patternInput)`

## 10 `[GO]` Combine the Teeth into the Gear Body

One timeline entry: a single Combine-Join. Copy `pattern.bodies` item by item into a fresh
`adsk.core.ObjectCollection` — the collection already includes the seed tooth, so do not add it
again, and `combineFeatures.createInput` rejects the `BRepBodies` collection itself. Then
`combineFeatures.createInput(ctx.gearBody, toolBodies)` with the operation left as join, and
`combineFeatures.add(combineInput)`.

The proof function is `stepCombineTeeth`. It asserts the join adds exactly the volume of one tooth
standing outside the root circle, leaves a single lump, and extends the body out to the tip circle.

Two substitutions, both recorded beside the function. The tooth is drawn slightly sunk into the
disc so the join is a crossing rather than a contact, because `decad`'s boolean refuses a pair whose
facets meet in one plane without crossing — which is exactly what a tooth that only touches the root
circle presents; the sunk material is already inside the disc, so the volume the join adds is
unchanged and is what the assertion checks. And only one tooth is joined: a second join takes the
first result as an operand, and `decad` refuses a boolean whose faceted operand shares a cap plane
with its tool, which two bodies both running from the sketch plane to the end plane always do. That
leaves the mutual disjointness of the remaining teeth unproven here.

**From:** `spec/spurgear/instructions.md` L262, L292, L516–520;
`.claude/skills/generate-gear/PLAYBOOK.md` L592–596

Fusion API calls: `adsk.core.ObjectCollection.create()`,
`component.features.combineFeatures.createInput(targetBody, toolBodies)`,
`component.features.combineFeatures.add(combineInput)`,
`adsk.fusion.FeatureOperations.JoinFeatureOperation`

## 11 `[GO]` Root Fillets

`patternTeeth` calls `self.createFillets(ctx)` as its last action. If `FilletRadius` is not greater
than 0, return. Otherwise one timeline entry.

Round the inside corner where each root valley floor meets a tooth flank — the corner that runs the
full thickness parallel to the gear's main axis, where bending stress concentrates. Not the
front and back rim, which is cosmetic and unwanted here.

Collect **every** cylindrical face of `ctx.gearBody` whose radius equals `RootCircleRadius`, not
just the first: after the pattern and combine the root cylinder is usually split into one patch per
valley. On each such face, filter to edges whose `edge.geometry.curveType` is
`adsk.core.Curve3DTypes.Line3DCurveType`, take each line's direction from its geometry endpoints
with `edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)`, `normalize()` it, and keep it when
`abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01` — exactly that tolerance, because a
tighter test drops valid axial edges that tessellation has left slightly off. Never read the
direction with `edge.evaluator.getTangent(0)`: parameter 0 is not guaranteed to lie inside the
edge's range and Fusion raises `RuntimeError: invalid argument parameter`.

Apply with `filletFeatures.createInput()` then
`filletInput.addConstantRadiusEdgeSet(edges, radius, False)` — on the input **itself**, not through
an `edgeSetInputs` collection, which does not exist on a fillet input. `isTangentChain` must be
`False`, or Fusion pulls in tangent-adjacent edges and rounds more than the root corner.

If the edge collection is **empty**, return silently without creating the feature. An empty edge set
must not reach `filletFeatures.add`.

The proof function is `stepRootFillets`. Its table covers both sides of the embedded branch, a
coarse and a default size, and the empty-collection branch — modelled with a body that genuinely
has no axial root corner, a bare disc, where the selection comes back with no match and the body is
handed on untouched. Where the fillet does run it asserts the body grew, that it grew by less than
the two corners could possibly hold, and that the end caps did not move.

Substitution: the fillet is applied to a body extruded from the disc and one tooth drawn as a single
region, because that is the only shape in reach that HAS the valley corners — a tooth alone has
none, and `decad` will not fillet the boolean result step 10 leaves. The corners are selected the
way this step selects them, by direction parallel to the gear axis, with concavity separating the
two valley corners from the tooth's own convex ones. One case uses a coarse involute sample count,
because a finely chorded flank has segments shorter than the fillet's own setback near the root
crossing, which is an artefact of the chording — the real flank there is one smooth spline with no
corner at all.

**From:** `spec/spurgear/instructions.md` L86–88, L292, L318–323, L522–531;
`.claude/skills/generate-gear/PLAYBOOK.md` L480–486, L571–580

Fusion API calls: `adsk.core.SurfaceTypes.CylinderSurfaceType`,
`adsk.core.Curve3DTypes.Line3DCurveType`,
`edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)`, `direction.normalize()`,
`direction.dotProduct(axisNormal)`, `get_normal(self.plane)`,
`adsk.core.ObjectCollection.create()`,
`component.features.filletFeatures.createInput()`,
`filletInput.addConstantRadiusEdgeSet(edges, radius, False)`,
`component.features.filletFeatures.add(filletInput)`,
`adsk.core.ValueInput.createByReal(filletRadius)`

## 12 `[GO]` Bore Profile Sketch

`buildBore(ctx)` runs unconditionally from `generate()`, so it must return early in **two** cases
before anything here happens: when `SketchOnly` is set, and when `BoreDiameter` is not greater than
0. The SketchOnly guard is essential on its own — in that mode `buildMainGearBody` short-circuits
before `buildBody`, so `ctx.gearBody` and `ctx.extrusionExtent` were never set and the cut would
dereference `None`. Do not lean on the bore being 0 in sketch-only mode; the user may have set both.

Otherwise this is one timeline entry: a sketch named `Bore Profile` on the target plane, stored on
`self.boreSketch`. Draw the bore circle by instantiating the tooth generator on that sketch —
`SpurGearInvoluteToothDesignGenerator(boreSketch, self)` — and calling
`drawBore(ctx.anchorPoint, boreDiameter)`, which projects the anchor in, draws a non-construction
circle of that diameter centred on the projection, gives it a driving diameter dimension and returns
the circle.

The generator's constructor always adds its local-origin `SketchPoint` at (0, 0, 0), so this sketch
carries one stray unused point. That is faithful behaviour — do not suppress it — but it must be
grounded: add `addCoincident(toothGen.anchorPoint, projectedAnchor)` using the same projection
`drawBore` made. Not `boreSketch.originPoint`, which pins the point to the plane rather than to the
gear and has been observed to fail the solver. Without any grounding the point is free in two
directions and the sketch never reaches `isFullyConstrained`.

The proof function is `stepBoreProfileSketch`. Its table runs from no bore, where the step is not
reached at all, through a small bore, one close to the root circle, and a coarse-module bore. It
gates the sketch on the same full verdict step 4 uses, so a stray point left free would fail here,
and asserts the region drawn is the bore circle's.

**From:** `spec/spurgear/instructions.md` L54, L242–248, L293, L357–362, L533–537;
`spec/spurgear/fusion.md` L26–31;
`.claude/skills/generate-gear/PLAYBOOK.md` L413–429, L534–535

Fusion API calls: `self.createSketchObject('Bore Profile', self.plane)`,
`boreSketch.project(ctx.anchorPoint)`,
`sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`,
`sketch.sketchCurves.sketchCircles.addByCenterRadius(projectedAnchor, boreDiameter / 2)`,
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`,
`sketch.geometricConstraints.addCoincident(toothGen.anchorPoint, projectedAnchor)`

## 13 `[GO]` Bore Cut

One timeline entry. Extrude-cut the Bore Profile sketch's profile from the target plane to
`ctx.extrusionExtent` — the far end-cap face captured in step 8 — with the operation set to cut and
`extrudeInput.participantBodies = [ctx.gearBody]` so only the gear body is affected. Ending on the
far face is what guarantees the bore goes all the way through whatever `Thickness` is.

The proof function is `stepBoreCut`. Its table carries the non-positive-bore guard as a live case —
the step runs, cuts nothing and hands the gear body back unchanged — beside a small bore, one close
to the root circle, a coarse-module bore and a thin gear. It asserts the bored volume is the disc
less the bore exactly, that the body still spans the full thickness so the cut ran right through,
and that the wall it leaves sits at the bore radius.

The sketch-only guard has no case: in that mode `buildMainGearBody` returns before the gear body
exists, so there is no body for this step to build or for the proof to measure.

Substitution: the cut ends on a tool that spans the same range as the to-entity extent would, since
`decad` has no named face to end on; the reach the extent guarantees is asserted directly instead,
by checking the bored body still spans the full thickness.

**From:** `spec/spurgear/instructions.md` L264, L293, L533–537;
`.claude/skills/generate-gear/PLAYBOOK.md` L607–614, L628–631

Fusion API calls:
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`extrudeInput.participantBodies`,
`component.features.extrudeFeatures.add(extrudeInput)`

## 14 `[PROSE]` Cleanup

The last action of `generate()`, after `buildBore`, called unconditionally in both modes. It creates
no timeline entry.

Hide construction geometry with `isLightBulbOn = False` and sketches with `isVisible = False`; the
two are never crossed, because `isVisible` has no effect on a construction plane or axis. Guard each
entity individually, since not all of them exist in every run.

- **Always, in both modes**: switch off the light bulb on every construction plane and axis this
  build created — the `Extrusion End Plane`, the `Gear Center` axis, and the normalized target plane
  if step 1 created one. Sketch-only mode included, so no stray plane is left floating.
- **Only on the full-build path**: set `isVisible = False` on the Tools, Gear Profile and Bore
  Profile sketches, so only the finished gear body shows. In sketch-only mode they stay visible;
  inspecting them is the point of that mode.

**From:** `spec/spurgear/instructions.md` L239–241, L294–304, L427, L429, L489;
`spec/spurgear/fusion.md` L188–200;
`.claude/skills/generate-gear/PLAYBOOK.md` L558–570

Fusion API calls: `ctx.extrusionEndPlane.isLightBulbOn`, `ctx.centerAxis.isLightBulbOn`,
`self.toolsSketch.isVisible`, `ctx.gearProfileSketch.isVisible`, `self.boreSketch.isVisible`
