# Spur Gear — compiled step list

Compiled from `spec/spurgear/instructions.md`, `spec/spurgear/fusion.md` and
`.claude/skills/generate-gear/PLAYBOOK.md`. Each step below is one entry in the Fusion timeline,
or one `[PROSE]` preamble that produces no timeline entry. The runnable proof is the pair
`proof/spurgear/sketches_test.go` and `proof/spurgear/solids_test.go`; every `[GO]` step
names the function there that realises it.

## Provenance

| file | `git hash-object` |
|---|---|
| `.claude/skills/generate-gear/PLAYBOOK.md` | `e2d9d754012ed9e0e0306cd1327c36caac690f61` |
| `spec/helicalgear/fusion.md` | `83fac920272341e3c4584f16031478a69b7472e7` |
| `spec/spurgear/fusion.md` | `3e8b0b338e80a1199eb7eb95f9f004a6f0bb747d` |
| `spec/spurgear/instructions.md` | `6ce986c609c1d086e5158592f439e6d1c62309e3` |

## S1 `[PROSE]` Command Dialog Inputs

`SpurGearCommandInputsConfigurator.configure(cls, cmd)` is a `@classmethod` on a plain class. It
adds the inputs to `cmd.commandInputs` in exactly this display order, which is not the order they
are read later: Target Plane, Anchor Point, Module, Tooth Number, Pressure Angle, Bore Diameter,
Thickness, Apply chamfer to teeth, Generate sketches but do not build body, Parent Component last.
Do not regroup by input type. The two selection inputs coming first is also what gives Target Plane
the dialog's initial focus, since Fusion focuses the first selection input and ignores a later
focus flag.

| dialog input | id | how it is added |
|---|---|---|
| Target Plane | `plane` | selection; filters `ConstructionPlanes`, `PlanarFaces` |
| Anchor Point | `anchorPoint` | selection; filters `ConstructionPoints`, `SketchPoints` |
| Module | `module` | value, unit `''`, default `createByReal(1)` |
| Tooth Number | `toothNumber` | value, unit `''`, default `createByReal(17)` |
| Pressure Angle | `pressureAngle` | value, unit `'deg'`, default `createByReal(math.radians(20))` |
| Bore Diameter | `boreDiameter` | string value, default `'0 mm'` |
| Thickness | `thickness` | value, unit `'mm'`, default `createByReal(to_cm(10))` |
| Apply chamfer to teeth | `chamferTooth` | value, unit `'mm'`, default `createByReal(0)` |
| Generate sketches only | `sketchOnly` | bool checkbox, default false |
| Parent Component | `parentComponent` | selection; filters `Occurrences`, `RootComponents`; pre-selects the root component |

Selection inputs are built with `cmd.commandInputs.addSelectionInput(id, name, commandPrompt)`, then
`selectionInput.addSelectionFilter(filter)` once per filter, then
`selectionInput.setSelectionLimits(1, 1)` — exactly one selection each. The Parent Component input
additionally calls `selectionInput.addSelection(entity)` on `get_design().rootComponent`. Value
inputs are `cmd.commandInputs.addValueInput(id, name, unitType, initialValue)`, the expression field
is `cmd.commandInputs.addStringValueInput(id, name, initialValue)`, and the checkbox is
`cmd.commandInputs.addBoolValueInput(id, name, isCheckBox)`. Every numeric default is a
`adsk.core.ValueInput.createByReal(realValue)` in Fusion internal units — cm for length, radians for
angle — regardless of the displayed unit string.

A subclass adds its own input by subclassing this configurator and appending after
`super().configure(cmd)`, so its inputs necessarily land below Parent Component.

`configure` is named above as the method this module *defines*, not one it calls. The shared command
wiring in `commands/_gear_command.py` invokes it, and a subclass's `super()` chain reaches the
override; nothing inside this module calls either.

<!-- check-step-calls: ignore configure -->

Every id and parameter name in the table is exported as a module-level constant — `INPUT_ID_PARENT`,
`INPUT_ID_PLANE`, `INPUT_ID_ANCHOR_POINT`, `INPUT_ID_MODULE`, `INPUT_ID_TOOTH_NUMBER`,
`INPUT_ID_PRESSURE_ANGLE`, `INPUT_ID_BORE_DIAMETER`, `INPUT_ID_THICKNESS`, `INPUT_ID_CHAMFER_TOOTH`,
`INPUT_ID_SKETCH_ONLY`, and `PARAM_MODULE` through `PARAM_FILLET_RADIUS` for the parameters of S2.
Dependent modules import them by name, so renaming any of them is a breaking change.

**From:** `spec/spurgear/instructions.md` L20–22, L35–62, L90–150, L152–169, L171–178,
`.claude/skills/generate-gear/PLAYBOOK.md` L38–40, L53–60, L128–143, L332–334

## S2 `[PROSE]` Read Inputs and Register Parameters

`processInputs(inputs)` reads in an order fixed by the occurrence-context shift: creating the gear
occurrence can drop selections held by `SelectionCommandInput`s that point at entities in another
component, and anything that calls `parameterName(name)` or `addParameter(...)` creates that
occurrence transitively. So read and stash all three selections first.

`parameterName` is named only to say what trips that shift. It is inherited from `base.Generator`
and reached through `addParameter`, so this module makes no direct call to it.

<!-- check-step-calls: ignore parameterName -->

1. `get_selection(inputs, 'parentComponent')` → resolve an `Occurrence` to its component, keep as
   `self.parentComponent`; raise on the wrong count or type.
2. `get_selection(inputs, 'plane')` → `self.plane`, `get_selection(inputs, 'anchorPoint')` →
   `self.anchorPoint`.
3. For each numeric input `get_value(inputs, id, units)`, for the checkbox
   `get_boolean(inputs, 'sketchOnly')`; register each with `addParameter(name, value, units, comment)`
   under the `SpurGear<N>_` prefix. `SketchOnly` is stored as a real `1`/`0` and read back with
   `getParameterAsBoolean(name)`. **Never** call `get_value` on the bool input.
4. Call the `addExtraPrimaryParameters(inputs)` hook — a no-op on the spur base, the seam a subclass
   uses to register its own primary parameter before the derived ones reference it.
5. `registerDerivedParameters(...)` registers the calculated parameters as live expression strings
   through `adsk.core.ValueInput.createByString(stringValue)`:

| parameter | units | expression |
|---|---|---|
| `Module` | `''` | from the input — unitless, **not** `'mm'`, so `generateName` renders `M=1` |
| `PressureAngle` | `'rad'` | from the input |
| `PitchCircleDiameter` | `'mm'` | `Module * ToothNumber` |
| `PitchCircleRadius` | `'mm'` | `PitchCircleDiameter / 2` |
| `BaseCircleDiameter` | `'mm'` | `PitchCircleDiameter * cos(PressureAngle)` |
| `BaseCircleRadius` | `'mm'` | `BaseCircleDiameter / 2` |
| `RootCircleDiameter` | `'mm'` | `PitchCircleDiameter - 2.5 * Module` |
| `RootCircleRadius` | `'mm'` | `RootCircleDiameter / 2` |
| `TipCircleDiameter` | `'mm'` | `PitchCircleDiameter + 2 * Module` |
| `TipCircleRadius` | `'mm'` | `TipCircleDiameter / 2` |
| `InvoluteSteps` | `''` | `15` |
| `ToothSpaceAngleAtRoot` | `''` | pre-computed in Python: `pi / ToothNumber - 2 * (tan(PressureAngle) - PressureAngle)` |
| `ToothSpaceArcAtRoot` | `'mm'` | `RootCircleRadius * ToothSpaceAngleAtRoot` |
| `FilletClearance` | `''` | `0.9` |
| `FilletRadius` | `'mm'` | `(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>` |

`ToothSpaceAngleAtRoot` is registered **unitless**, not `'rad'`, even though it holds a radian
magnitude: the next parameter multiplies it by a length, and Fusion rejects a `mm·rad` product. The
`<factor>` in `FilletRadius` is the string `filletHelixFactorExpression()` returns — `'1'` for spur,
spliced in here and nowhere else. Every dimension and feature input downstream takes the numeric
`.value` of its parameter at generation time, not a live link.

**From:** `spec/spurgear/instructions.md` L37–88, L285–295, L372–381,
`spec/spurgear/fusion.md` L204–209,
`.claude/skills/generate-gear/PLAYBOOK.md` L103–126, L196–228

## S3 `[PROSE]` Create and Name the Gear Component

`generate(inputs)` calls `processInputs(inputs)`, then `getComponent()` — which lazily creates the
child occurrence under `self.parentComponent` with
`parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` and builds the
`SpurGear<N>_` parameter prefix — and sets `component.name` from
`generateName()`. That returns
`'Spur Gear (M={}, Tooth={}, Thickness={})'` formatted from the `Module`, `ToothNumber` and
`Thickness` parameters' `.expression` strings, not their `.value`, so units show through
(`Spur Gear (M=1, Tooth=17, Thickness=10 mm)`). `prefixBase()` returns `'SpurGear'`.

Three of those names belong to the framework side of the boundary rather than to this module's own
call graph. `generate` and `prefixBase` are members this module defines for `base.Generator` and the
command wiring to call, and `addNewComponent` is the call `base.Generator` makes inside its own
occurrence helper. The module contains no call to any of the three.

<!-- check-step-calls: ignore addNewComponent generate prefixBase -->

`newContext()` then builds the `SpurGearGenerationContext`, whose fields carry state between the
steps below and are read by name by subclasses: `ctx.plane` (S4), `ctx.anchorPoint` (S5),
`ctx.extrusionEndPlane` (S6), `ctx.gearProfileSketch` and `ctx.toothProfileIsEmbedded` (S7),
`ctx.toothBody` (S9), `ctx.gearBody`, `ctx.centerAxis` and `ctx.extrusionExtent` (S11–S12). Each
starts `cast(None)`-initialised, except `toothProfileIsEmbedded`, which starts `False`.

One invocation produces exactly one gear; there is no pairing. Never activate the occurrence — the
build runs on the non-activated component throughout.

Three modules bind to this one's surface and must keep working: `helicalgear.py` and
`herringbonegear.py` subclass all four classes and import `PARAM_MODULE`, `PARAM_TOOTH_NUMBER` and
`PARAM_THICKNESS`; `bevelgear.py` borrows the tooth generator alone, handing it a
`VirtualSpurProxy` as `parent`. That proxy serves only `Module`, `ToothNumber`, `PressureAngle`, the
Pitch/Base/Root/Tip diameters and radii, and `InvoluteSteps` — so `drawCircles`, `drawTooth`, `draw`
and their helpers may read no other parameter key, or the bevel build dies on a `KeyError`. The
proxy also carries the `_lastToothEmbedded` slot the tooth generator writes in S7 and bevel reads
back afterwards. Spur itself imports only the framework: `Generator`, `GenerationContext`,
`get_value`, `get_boolean`, `get_selection` from `.base`; `get_normal` and
`find_profile_by_curve_counts` from `.utilities`; `to_cm` and `get_design` from `.misc`.

**From:** `spec/spurgear/instructions.md` L9–11, L13–18, L23–25, L29–33, L217–232, L234–241,
L243–259, L270, L296–300, L345–370,
`.claude/skills/generate-gear/PLAYBOOK.md` L17–36, L62–74, L81–91, L230–240, L242–266, L715–722

## S4 `[PROSE]` Normalize the Target Plane

Timeline entry, and only when the user picked something that is not already a `ConstructionPlane` —
a planar face, typically. Build a coplanar construction plane on the gear's own component:
`component.constructionPlanes.createInput()`, then
`constructionPlaneInput.setByOffset(planarEntity, adsk.core.ValueInput.createByReal(0))`, then
`component.constructionPlanes.add(constructionPlaneInput)`. Replace `self.plane` and `ctx.plane`
with the result; keep both available, because subclasses read `self.plane` directly.

The offset argument is a `ValueInput`, never a bare number — `setByOffset(plane, 0)` is a runtime
`TypeError`. Normalizing keeps the selected face's own profile out of the later profile searches.
Remember the plane was created, so the cleanup step can switch its light bulb off.

**From:** `spec/spurgear/instructions.md` L39, L221, L231–232, L385–387,
`.claude/skills/generate-gear/PLAYBOOK.md` L674–685

## S5 `[GO]` Tools Sketch

Proof function: `stepToolsSketch`.

One timeline entry: a sketch named `Tools` on the target plane, created through
`createSketchObject('Tools', plane=self.plane)`. It draws no geometry of its own. Project the user's
Anchor Point into it — `toolsSketch.project(entity)` — and keep the single projected `SketchPoint`
as `ctx.anchorPoint`. This is the canonical handle: the Gear Profile and Bore Profile sketches each
re-project *this* point rather than the user's original entity, so the whole gear follows the anchor
if it moves.

A sketch created by `createSketchObject` starts hidden. Make this one visible before projecting and
leave it visible until the build is finished — projection has failed on hidden sketches in this
repo's history, and the bore step in S17 re-projects from it after `buildMainGearBody` has returned.

Proof note: the projection is modelled as reference geometry, which is what a Fusion projection is —
a point whose position is handed in from outside and which the solver never moves. Modelling it as
an ordinary point at fixed coordinates would prove a different sketch.

**From:** `spec/spurgear/instructions.md` L192–197, L222, L249, L261–268, L389–391,
`spec/spurgear/fusion.md` L19–24,
`.claude/skills/generate-gear/PLAYBOOK.md` L94–96, L430–444, L558–570

## S6 `[PROSE]` Extrusion End Plane

Timeline entry: a construction plane named `Extrusion End Plane`, offset from the target plane by
`Thickness`. `component.constructionPlanes.createInput()` →
`constructionPlaneInput.setByOffset(planarEntity, offset)` with the offset as
`adsk.core.ValueInput.createByReal(thickness)` — the numeric snapshot of the `Thickness` parameter,
in internal cm — → `component.constructionPlanes.add(constructionPlaneInput)`. Store it on
`ctx.extrusionEndPlane`.

Its only job is to be the `to-entity` target for the tooth extrude (S9) and the body extrude (S11),
so both end on the same well-defined face. Leave it visible while those extrudes run; the cleanup
step turns its light bulb off. `isVisible = False` does not hide a construction plane.

`prepareTools(ctx)` owns S5 and S6 together.

**From:** `spec/spurgear/instructions.md` L223, L249, L387, L393,
`spec/spurgear/fusion.md` L190–200,
`.claude/skills/generate-gear/PLAYBOOK.md` L558–570, L674–678

## S7 `[GO]` Gear Profile Sketch

Proof function: `stepGearProfileSketch`.

One timeline entry, however much geometry goes into it. `buildSketches(ctx)` creates the sketch —
`createSketchObject('Gear Profile', plane=self.plane)`, stored on `ctx.gearProfileSketch` — then
constructs `SpurGearInvoluteToothDesignGenerator(sketch, self, angle=0)` and calls
`draw(ctx.anchorPoint, angle=0)` on it. The constructor stores `self.toothAngle = angle` and adds
the sketch's movable **local origin**: a fresh `SketchPoint` at (0,0,0) through
`sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`, held on the field `self.anchorPoint`.
Never `sketch.originPoint` — that one is immutable and cannot be coincident-constrained to anything
projected in. `draw` then performs `drawCircles()`, `drawTooth(angle)`, the anchoring, and finally
the confirming angular value-set.

**drawCircles.** Four circles, every one centred by passing the local origin `SketchPoint`
**directly** as the centre — `sketch.sketchCurves.sketchCircles.addByCenterRadius(centerPoint, radius)`
— so all four share that one point; do not pass its `.geometry` and then add a centre coincident.

| circle | radius | geometry |
|---|---|---|
| Root Circle | `RootCircleRadius` | solid |
| Tip Circle | `TipCircleRadius` | construction (`circle.isConstruction = True`) |
| Base Circle | `BaseCircleRadius` | construction |
| Pitch Circle | `PitchCircleRadius` | construction |

Each gets a driving diameter dimension,
`sketch.sketchDimensions.addDiameterDimension(entity, textPoint)`, with the text point off the
centre — a text point *at* the centre is rejected. Never pass `isDriven=True`. Each circle is also
labelled with along-path text: `sketch.sketchTexts.createInput2(text, height)`, then
`textInput.setAsAlongPath(path, isAbovePath, horizontalAlignment, characterSpacing)` with
`adsk.core.HorizontalAlignments.CenterHorizontalAlignment`, then
`sketch.sketchTexts.add(input)`. The label string is
`'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` using the radii's internal `.value` in cm,
where `size = TipCircleRadius - RootCircleRadius`; that same `size` is the text height.

**drawTooth(angle).** Sample the involute flank endpoint-inclusively: with `steps = InvoluteSteps`,
sample `i` for `i = 0 … steps-1` sits at `r = BaseCircleRadius + (TipCircleRadius - BaseCircleRadius) * i / (steps - 1)`,
so the first sample is exactly on the base circle and the last exactly on the tip circle. Do not
clamp the start to the root circle. Each sample is `calculateInvolutePoint(baseRadius, r)`; drop any
that returns `None`. That helper is exactly

```
alpha = acos(baseRadius / intersectionRadius)
t     = tan(alpha)
x = baseRadius * (cos(t) + t * sin(t))
y = baseRadius * (sin(t) - t * cos(t))
```

— the parameter is `tan(alpha)`, not `inv(alpha) = tan(alpha) - alpha`, and the function returns
`None` when `intersectionRadius < baseRadius`.

Then, in this order: mirror every sample across +X (negate y), because the standard parametric
involute spirals the wrong way for a left flank and produces a tooth wider at the tip than at the
root; rotate by `rotate_angle = pi / (2 * ToothNumber) - atan2(-py, px)` where `(px, py)` is
`calculateInvolutePoint(baseRadius, pitchRadius)`, which lands the pitch crossing at half a tooth
width above +X analytically rather than by interpolating between samples; that gives the **left**
flank, and mirroring it across X gives the **right**. Finally rotate both collections by the
runtime `angle`, along with the tooth-top point and the rib-midpoint seeds. Rotate by the `angle`
argument that arrived from `draw()`, **not** by the stored `self.toothAngle` — helical and
herringbone construct the generator with the default 0 and pass the helix angle at call time, and
using the stored value would draw a flat tooth and a loft with no twist. At `angle = 0` this
rotation is a no-op.

Draw each flank as one fitted spline: collect the `adsk.core.Point3D.create(x, y, z)` samples into
`adsk.core.ObjectCollection.create()` with `points.add(item)`, then
`sketch.sketchCurves.sketchFittedSplines.add(fitPoints)`.

**Tooth-top arc.** Materialize a tooth-top `SketchPoint` at
`(TipCircleRadius * cos(angle), TipCircleRadius * sin(angle))` and constrain it onto the tip circle
with `sketch.geometricConstraints.addCoincident(point, entity)`. Then
`sketch.sketchCurves.sketchArcs.addByCenterStartEnd(centerPoint, startPoint, endPoint)` passing the
local origin as the centre and the two flank splines' end `SketchPoint`s directly, so the arc shares
all three. Add **no** diameter dimension: the shared centre plus the two shared ends already
determine the arc, and a free centre with a radius would leave an inward-bulging arc equally valid
at DOF 0.

**Spine, +X reference and the angular pin.** The spine is a construction line
`sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)` between the local origin and
the tooth-top point, both passed directly; add no separate start coincident, and do not constrain
its end onto the arc. Build the +X reference line for **every** angle, 0 included: a far endpoint at
`(TipCircleRadius, 0)` pinned by two signed dimensions from the local origin —
`sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, orientation, textPoint)` with
`adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation` at `TipCircleRadius` and with
`adsk.fusion.DimensionOrientations.VerticalDimensionOrientation` at `0` — then a construction line
from the origin to it. Dimension the angle from the reference to the spine, in that argument order,
`sketch.sketchDimensions.addAngularDimension(lineOne, lineTwo, textPoint)`, with the text point on
the bisector of the intended angle so Fusion picks that angle and not its supplement.
Never pin the spine with a plain `sketch.geometricConstraints.addHorizontal(line)` instead:
horizontal fixes the line's direction but not which way it points, so the tooth top can settle at
either end of the tip circle and the whole tooth comes out 180 degrees around.

**Ribs.** One construction line per fit-point index, for **all** N indices including the first
(base-circle) and last (tip) pairs — the fit points carry no other constraint, so an omitted
endpoint rib leaves the sketch under-constrained. Per rib, in exactly this order:

1. `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)` on
   `leftSpline.fitPoints.item(i)` and `rightSpline.fitPoints.item(i)`, passed directly; mark it
   construction.
2. One **signed** distance dimension across the rib — vertical for `angle = 0`, and in general
   vertical for the rib and horizontal for the midpoint chain when `abs(cos(angle)) >= abs(sin(angle))`,
   swapped otherwise. An aligned dimension gives only the length, which the two flanks satisfy
   equally well swapped over, and the tooth comes out mirrored.
3. A fresh midpoint `SketchPoint` created already **on** the spine: with
   `t = fitX * cos(angle) + fitY * sin(angle)`, seed it at `(t * cos(angle), t * sin(angle))`. Not
   the rib's true 2-D midpoint, and not `(fitX, 0)` for a rotated tooth.
4. `sketch.geometricConstraints.addCoincident(point, entity)` pinning that point onto the spine.
5. `sketch.geometricConstraints.addMidPoint(point, midPointCurve)` making it the rib's midpoint.
6. `sketch.geometricConstraints.addPerpendicular(lineOne, lineTwo)` between spine and rib — **skipped
   for the last rib only**, because the tooth-top arc already holds those two ends at equal radius
   either side of the spine and the perpendicular there throws `VCS_SKETCH_OVER_CONSTRAINTS`.

Then chain the midpoints with signed dimensions along the spine direction, starting from the local
origin to the first rib's midpoint. Without that first link the whole chain slides along the spine
as a unit and the sketch never fully constrains.

**Flank-to-root lines.** If the flank's first fit point lies **outside** the root circle, draw a
short radial line on each side: `sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`
from a fresh root-end point to the flank spline's start `SketchPoint`, passed directly. Place the
root end with exactly two signed dimensions from the local origin — horizontal at its Δx and
vertical at its Δy — and nothing else. "Root end on the root circle plus origin on the line" is
satisfied by two points, because that line meets the root circle again on the far side, and the stub
becomes a line straight across the gear. The tooth loop then holds **6 curves**: 2 splines, 2
flank-to-root lines, 2 arcs.

The embedded test is strict `<` on raw values: with `firstRadius` the distance from the local origin
to the left flank's first fit point, `embedded = firstRadius < RootCircleRadius`. Exact equality
counts as **not** embedded and draws a zero-length stub. When embedded — which happens above
`2.5 / (1 - cos(PressureAngle))` teeth, 41.5 at 20°, 78.5 at 14.5°, 26.7 at 25° — draw no stub and
the loop holds **4 curves**. The tooth generator cannot reach `ctx`, so it writes
`self.parent._lastToothEmbedded`, which `SpurGearGenerator.__init__` pre-initialises to `False` and
`buildSketches` copies into `ctx.toothProfileIsEmbedded`.

**Anchoring.** Inside `draw()`, after `drawCircles()` and `drawTooth(angle)`: project the Tools
anchor into this sketch — `gearProfileSketch.project(entity)` on `ctx.anchorPoint` — and
`sketch.geometricConstraints.addCoincident(point, entity)` between the local origin and that fresh
projection. Everything above is drawn relative to the local origin, so this single coincidence
drags the whole profile onto the user's anchor. It must live in `draw()`, not in `buildSketches`,
because helical and herringbone call `draw()` directly for their twisted sketch and rely on it.

**Last action of all**, after the entire constraint network exists, and only when `angle != 0`: set
`spineAngularDimension.parameter.value = angle`. Pre-rotating puts the geometry on the correct
solver branch and this value-set confirms it; drawing flat and relying on the dimension alone lets
Fusion pick the branch ~180 degrees away.

The sketch must end fully constrained — `sketch.isFullyConstrained` — with no redundant or
conflicting constraint.

Proof note: the proof runs this scheme over six parameter cases, covering both profile shapes and
`angle` values of 0, 30° and 90°, and gates each on the sketch engine's full verdict: DOF 0, no
redundancy, no conflict, valid profiles, well-conditioned, and no discrete ambiguity. The ambiguity
check is what settles the two branch traps above — the inward-bulging tooth-top arc and the
180-degree spine — because a scheme that admits either reaches DOF 0 and still fails.

**From:** `spec/spurgear/instructions.md` L26–28, L186–215, L224, L229, L272–275, L302–343,
L395–404, L406–443, L445–449,
`spec/spurgear/fusion.md` L26–43, L47–60, L69–87, L89–112, L114–149, L151–186,
`.claude/skills/generate-gear/PLAYBOOK.md` L336–403, L413–450, L466–473, L525–535, L547–555,
L581–591

## S8 `[PROSE]` Sketch-Only Short-Circuit

Not a timeline entry. If the `SketchOnly` parameter reads true through
`getParameterAsBoolean(name)`, `buildMainGearBody` sets `ctx.gearProfileSketch.isVisible = True` and
returns before S9. Steps S9 through S15 are skipped; S16 and S17 skip themselves on the same flag,
and S18 still runs.

**From:** `spec/spurgear/instructions.md` L60, L147–150, L252–256, L451–453

## S9 `[PROSE]` Extrude the Tooth

Timeline entry. `buildTooth(ctx)` finds the single tooth cross-section with the framework helper —
`find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`
— never a hand-rolled loop search; the helper raises when nothing matches instead of falling back to
a wrong profile.

Extrude it from the target plane to the Extrusion End Plane as a **new body**:
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
then `extrudeInput.setOneSideExtent(extent, direction)` with
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)` and
`adsk.fusion.ExtentDirections.PositiveExtentDirection`, then
`component.features.extrudeFeatures.add(input)`. Name the feature `Extrude tooth` and store
`extrude.bodies.item(0)` as `ctx.toothBody`. `buildTooth` must call `self.chamferTooth(ctx)` as its
last action — the chamfer is triggered from inside it, and `buildMainGearBody` must not chamfer
separately.

Not proved. The tooth section's boundary includes the two involute flanks, and the solid harness
refuses a free-form wall edge whose curvature sign it cannot certify — which it cannot for an
interpolating spline through 15 samples, or through 3. See the report's proof-limits note.

**From:** `spec/spurgear/instructions.md` L225, L229, L253–254, L276–279, L455–459,
`.claude/skills/generate-gear/PLAYBOOK.md` L151–158, L571–580

## S10 `[PROSE]` Chamfer the Tooth

Timeline entry, and only when the `ChamferTooth` parameter is greater than 0. `chamferTooth(ctx)`
finds the tooth's front face with a **single conjunction predicate**: walk `ctx.toothBody.faces` and
take the first face for which **both** `face.edges.count == chamferWantEdges()` — 6 on the spur base
— **and** `sketchPlane.isCoPlanarTo(plane)` holds for
`sketchPlane = ctx.gearProfileSketch.referencePlane.geometry` against `face.geometry`. Both
conditions of the same face; not an edge-count match with a coplanarity tiebreak. If no face
satisfies both, raise; do not fall back to a partial match.

Then walk that face's edges and collect every one into an `adsk.core.ObjectCollection.create()`
**except** the root arc, identified by radius: skip an edge whose
`edge.geometry.curveType` is `adsk.core.Curve3DTypes.Arc3DCurveType` and whose `edge.geometry.radius`
equals the `RootCircleRadius` parameter's `.value` within `0.001` cm. That radius match is exact —
the root arc is the only edge on the root circle — and more robust than "the smallest arc".
Chamfering it would eat into the neighbouring tooth. Everything else on the face is chamfered: both
flanks, the tooth-top arc, and both flank-to-root lines.

Apply with `component.features.chamferFeatures.createInput2()`, then
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distance, isTangentChain)` with
the `ChamferTooth` value and `False`, then `component.features.chamferFeatures.add(input)`.

Known and accepted: an embedded profile yields a 4-edge front face while `chamferWantEdges()` stays
6, so chamfering an embedded spur tooth raises the front-face-not-found error. Users turn the
chamfer off for such gears. Helical and herringbone override only `chamferWantEdges()`, to 4; that
value and its own flagged caveat belong to their spec.

Not proved: the receiver is the tooth body from S9, which the solid harness cannot build. The
harness would also refuse the selection itself — it accepts a cap-loop chamfer only when the
selection covers a *complete* cap loop, and this one deliberately omits the root arc.

**From:** `spec/spurgear/instructions.md` L280–287, L461–467,
`spec/helicalgear/fusion.md` L76–84,
`.claude/skills/generate-gear/PLAYBOOK.md` L480–486, L571–580

## S11 `[GO]` Extrude the Gear Body

Proof function: `stepExtrudeGearBody`.

Timeline entry. `buildBody(ctx)` finds the gear body profile — the solid disc inside the root
circle, bounded by **exactly 2 arcs**, the two pieces the tooth's flank-to-root lines cut the root
circle into — with `find_profile_by_curve_counts(sketch, arcs=2)`. It is not an annulus, and the tip
circle is not part of it: the tip circle is construction geometry and bounds no profile.

Extrude it from the target plane to the Extrusion End Plane as a **new body**, exactly as S9 does:
`component.features.extrudeFeatures.createInput(profile, operation)` →
`extrudeInput.setOneSideExtent(extent, direction)` with
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)` and
`adsk.fusion.ExtentDirections.PositiveExtentDirection` →
`component.features.extrudeFeatures.add(input)`. Name the feature `Extrude body`, name the resulting
body `Gear Body`, and store it on `ctx.gearBody`.

While iterating `extrude.bodies.item(0).faces`, capture the far end cap by
`face.geometry.surfaceType`: among faces whose type is
`adsk.core.SurfaceTypes.PlaneSurfaceType`, the one where
`sketchPlane.isParallelToPlane(plane)` is true and `sketchPlane.isCoPlanarTo(plane)` is false — the
near cap is coplanar, so that rules it out. Store it as `ctx.extrusionExtent`; raise if it is not
found. Use the plane-geometry API rather than a hand-rolled dot product.

Proof note: the proof asserts the volume is the full root disc times the thickness — the claim that
the profile is a disc and not an annulus — that the body spans exactly 0 to `Thickness` axially,
that it reaches the root radius laterally, and that its faces divide into two planar end caps, each
a full root disc, plus at least one cylindrical face for S12 to build the axis from. There is more
than one cylindrical face, because the two-arc boundary makes two walls.

The proof draws the two root arcs outright rather than letting profile detection split one root
circle. The region is identical; only who splits the circle differs. The solid harness will not
record a boundary edge that is a fragment of a curve whose trim it cannot certify, and the sketch
engine withholds that certificate from every edge of a sketch that holds a free-form curve — which
the Gear Profile sketch does, twice.

**From:** `spec/spurgear/instructions.md` L226, L228, L255, L399–404, L469–478,
`.claude/skills/generate-gear/PLAYBOOK.md` L151–158, L571–580, L639–660

## S12 `[PROSE]` Gear Center Construction Axis

Timeline entry, created inside `buildBody(ctx)` from the same face walk as S11. Take any face of the
new body whose `face.geometry.surfaceType` is `adsk.core.SurfaceTypes.CylinderSurfaceType` and build
`component.constructionAxes.createInput()` → `axisInput.setByCircularFace(circularFace)` →
`component.constructionAxes.add(axisInput)`. Name it `Gear Center`, set
`axis.isLightBulbOn = False`, and store it on `ctx.centerAxis`. Raise if no cylindrical face is
found.

`SurfaceTypes` lives in `adsk.core`, not `adsk.fusion`; the wrong module is a runtime
`AttributeError` that no parse check sees.

**From:** `spec/spurgear/instructions.md` L227, L473–478,
`.claude/skills/generate-gear/PLAYBOOK.md` L410, L686–689

## S13 `[PROSE]` Pattern the Teeth

Timeline entry. `patternTeeth(ctx)` circular-patterns `ctx.toothBody` around the `Gear Center` axis.
Put the tooth body into an `adsk.core.ObjectCollection.create()`, then
`component.features.circularPatternFeatures.createInput(inputEntities, axis)`, then pin all three
inputs explicitly rather than trusting Fusion's defaults: `patternInput.quantity` from the
`ToothNumber` value, `patternInput.totalAngle` as
`adsk.core.ValueInput.createByString('360 deg')`, and `patternInput.isSymmetric = False`. Then
`component.features.circularPatternFeatures.add(input)`.

Not proved: the pattern's seed is the tooth body from S9.

**From:** `spec/spurgear/instructions.md` L256, L480–482,
`.claude/skills/generate-gear/PLAYBOOK.md` L597–602

## S14 `[PROSE]` Combine the Patterned Teeth into the Gear Body

Timeline entry, still inside `patternTeeth(ctx)`. One Combine-Join of every patterned tooth body
into `Gear Body`. `CircularPatternFeature.bodies` already contains the seed body plus the copies, so
feed that collection as-is and do not re-add the seed — but copy it into a fresh
`adsk.core.ObjectCollection.create()` first with `bodies.add(item)`, because
`component.features.combineFeatures.createInput(targetBody, toolBodies)` wants an `ObjectCollection`
and rejects a `BRepBodies`. Set `combineInput.operation` to
`adsk.fusion.FeatureOperations.JoinFeatureOperation`, then
`component.features.combineFeatures.add(input)`. `patternTeeth` calls `self.createFillets(ctx)` as
its last action.

Not proved: the tool bodies are the patterned teeth from S13.

**From:** `spec/spurgear/instructions.md` L256, L482–484,
`.claude/skills/generate-gear/PLAYBOOK.md` L592–596

## S15 `[PROSE]` Root Fillets

Timeline entry, and only when the `FilletRadius` parameter's `.value` is greater than 0.
`createFillets(ctx)` rounds the inside corner where each root valley floor meets a tooth flank —
the corner that runs the full thickness parallel to the gear axis, where root bending stress
concentrates. Not the front or back rim, which is cosmetic and unwanted here.

Collect **every** cylindrical face whose radius equals `RootCircleRadius`, not just the first: after
the pattern and combine, the root cylinder is usually one patch per valley rather than one
continuous surface. On each such face, filter edges to
`adsk.core.Curve3DTypes.Line3DCurveType`, take each line's direction from its geometry endpoints —
`edge.geometry.startPoint.vectorTo(point)` on `edge.geometry.endPoint`, then
`direction.normalize()` — and keep it when it is parallel to the target plane's normal, from
`get_normal(entity)`: `abs(abs(direction.dotProduct(vector)) - 1.0) < 0.01`. Use that tolerance
exactly; a tighter `> 0.999` test drops valid axial edges that tessellation left slightly off, and
the root fillets go missing. The circular edges that wrap the circumference at the end caps are
rims, not root corners, and the line-type filter is what removes them.
Never read the direction through `edge.evaluator.getTangent(0)`: parameter 0 is not guaranteed to
lie inside the edge's parameter range and Fusion raises `RuntimeError: invalid argument parameter`.

Apply with `component.features.filletFeatures.createInput()` →
`filletInput.addConstantRadiusEdgeSet(entities, radius, isTangentChain)` — on the input **itself**,
with the `FilletRadius` numeric value and `isTangentChain` **`False`**, since the collected edges
are exactly the axial root corners and tangent-chaining would pull in neighbours — then
`component.features.filletFeatures.add(input)`.

If the edge collection ends up **empty**, return without creating the feature: no error, no fillet.
An empty edge set must never reach `filletFeatures.add`.

Not proved: the receiver is the combined gear body from S14.

**From:** `spec/spurgear/instructions.md` L285–287, L486–495,
`.claude/skills/generate-gear/PLAYBOOK.md` L412, L480–486

## S16 `[GO]` Bore Profile Sketch

Proof function: `stepBoreProfileSketch`.

Timeline entry, and only on the full-build path with `BoreDiameter > 0`. `buildBore(ctx)` runs
unconditionally from `generate()`, after `buildMainGearBody`, so it must itself return early in two
cases: when `SketchOnly` is set, and when the bore diameter is not positive. The `SketchOnly` guard
is the essential one — in that mode `ctx.gearBody` and `ctx.extrusionExtent` were never set, and
proceeding would dereference `None`. Do not lean on the bore diameter being 0 in sketch-only mode;
the user may have set both.

Otherwise create a sketch named `Bore Profile` on the target plane and draw the bore circle by
instantiating the tooth generator on it — `SpurGearInvoluteToothDesignGenerator(boreSketch, self)` —
and calling `drawBore(ctx.anchorPoint, boreDiameter)` with the diameter in cm. That method projects
`ctx.anchorPoint` into this sketch — `boreSketch.project(entity)` — draws a solid (non-construction)
circle of that diameter centred on the projection with
`sketch.sketchCurves.sketchCircles.addByCenterRadius(centerPoint, radius)`, gives it a driving
`sketch.sketchDimensions.addDiameterDimension(entity, textPoint)`, and returns the circle.

The tooth generator's constructor always adds its local-origin `SketchPoint` at (0,0,0), so this
sketch carries one stray unused point. That is faithful — do not suppress it — but it must be
grounded, exactly as S7 grounds its own: `sketch.geometricConstraints.addCoincident(point, entity)`
between `toothGen.anchorPoint` and the same projection `drawBore` already made. Not on
`boreSketch.originPoint`: constraining to the sketch origin has been observed to throw
`VCS_SKETCH_SOLVING_FAILED`, and it would pin the point to the plane rather than to the gear. With
no grounding at all the point is free in two directions and the sketch never fully constrains.

Proof note: the proof gates this sketch on the same full verdict as S7, so the stray point being
grounded is what carries it to DOF 0.

**From:** `spec/spurgear/instructions.md` L210–212, L257, L321–326, L497–501,
`spec/spurgear/fusion.md` L26–31,
`.claude/skills/generate-gear/PLAYBOOK.md` L423–429

## S17 `[GO]` Bore Extrude-Cut

Proof function: `stepBoreCut`.

Timeline entry, under the same two guards as S16. Extrude-cut the bore profile from the target plane
to `ctx.extrusionExtent`, the far end-cap face captured in S11:
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`
→ `extrudeInput.setOneSideExtent(extent, direction)` with
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)` →
`extrudeInput.participantBodies` set to `[ctx.gearBody]` so only the gear body is affected →
`component.features.extrudeFeatures.add(input)`. Ending on the far face is what guarantees the bore
goes all the way through whatever the thickness is.

Proof note: the proof asserts the bored body's volume is the root disc less the bore cylinder over
the full thickness, and that the body still spans 0 to `Thickness` axially — a bore that stopped
short would leave the far cap whole and show up here. It runs on the plain root disc rather than the
toothed gear body, because the tooth ring is what the harness cannot build; the bore is concentric
and well inside the root circle, so the teeth are not part of what the cut touches. The cut is
evaluated on a faceted mesh, so the readings converge on the analytic answer from below and the
assertions allow a tenth of a percent — far tighter than a wrong diameter or a short bore would be.

**From:** `spec/spurgear/instructions.md` L228, L257, L497–501,
`.claude/skills/generate-gear/PLAYBOOK.md` L628–631

## S18 `[PROSE]` Cleanup

Not a timeline entry. `cleanup(ctx)` is the very last action of `generate()`, after `buildBore`, and
is called **unconditionally** in both modes; the mode distinction lives inside it. It must not move
up into `buildMainGearBody`, because `buildBore` re-projects `ctx.anchorPoint` from the Tools sketch
and projection fails once that sketch is hidden.

- **Always, in both modes:** `plane.isLightBulbOn = False` on every construction plane and axis this
  build created — the Extrusion End Plane, the `Gear Center` axis, and the normalized target plane
  if S4 made one.
- **Full build only:** `sketch.isVisible = False` on the Tools, Gear Profile and Bore Profile
  sketches. Sketch-only mode leaves Tools and Gear Profile visible for inspection, which is the
  whole point of that mode.

Guard each entity individually — the `Gear Center` axis and the Bore Profile sketch do not exist in
sketch-only mode. Never cross the two properties: `isVisible` hides sketches, `isLightBulbOn` hides
construction planes and axes.

**From:** `spec/spurgear/instructions.md` L203–205, L257–268, L451–453,
`spec/spurgear/fusion.md` L190–200,
`.claude/skills/generate-gear/PLAYBOOK.md` L558–570
