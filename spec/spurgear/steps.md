# Spur Gear — compiled step list

The proof for this step list is `proof/spurgear/geometry_test.go`, `proof/spurgear/sketches_test.go`, `proof/spurgear/solids_test.go` and the generated `proof/spurgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/spurgear/instructions.md` | `560549a350c789b6598ee49b72d3cd7b4460e214` |
| `spec/spurgear/fusion.md` | `ea678245854cfec80055d67c46a8788772b0f9d4` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `dadae022d2220a73b25e07b24ef99075a8be23a5` |

## S01 `[PROSE]` Module surface — imports, exported constants, the four classes

`lib/geargen/spurgear.py` imports explicitly, never with a star import:

```python
import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import Generator, GenerationContext, get_value, get_boolean, get_selection
from .utilities import get_normal, find_profile_by_curve_counts
```

Every dialog input id and user-parameter name is a module-level constant, because dependent modules
import them by name and renaming one is a breaking change. The full roster, with the string each
holds:

| constant | value | constant | value |
|---|---|---|---|
| `INPUT_ID_PARENT` | `'parentComponent'` | `PARAM_MODULE` | `'Module'` |
| `INPUT_ID_PLANE` | `'plane'` | `PARAM_TOOTH_NUMBER` | `'ToothNumber'` |
| `INPUT_ID_ANCHOR_POINT` | `'anchorPoint'` | `PARAM_PRESSURE_ANGLE` | `'PressureAngle'` |
| `INPUT_ID_MODULE` | `'module'` | `PARAM_BORE_DIAMETER` | `'BoreDiameter'` |
| `INPUT_ID_TOOTH_NUMBER` | `'toothNumber'` | `PARAM_THICKNESS` | `'Thickness'` |
| `INPUT_ID_PRESSURE_ANGLE` | `'pressureAngle'` | `PARAM_CHAMFER_TOOTH` | `'ChamferTooth'` |
| `INPUT_ID_BORE_DIAMETER` | `'boreDiameter'` | `PARAM_SKETCH_ONLY` | `'SketchOnly'` |
| `INPUT_ID_THICKNESS` | `'thickness'` | `PARAM_PITCH_DIAMETER` | `'PitchCircleDiameter'` |
| `INPUT_ID_CHAMFER_TOOTH` | `'chamferTooth'` | `PARAM_PITCH_RADIUS` | `'PitchCircleRadius'` |
| `INPUT_ID_SKETCH_ONLY` | `'sketchOnly'` | `PARAM_BASE_DIAMETER` | `'BaseCircleDiameter'` |
| | | `PARAM_BASE_RADIUS` | `'BaseCircleRadius'` |
| | | `PARAM_ROOT_DIAMETER` | `'RootCircleDiameter'` |
| | | `PARAM_ROOT_RADIUS` | `'RootCircleRadius'` |
| | | `PARAM_TIP_DIAMETER` | `'TipCircleDiameter'` |
| | | `PARAM_TIP_RADIUS` | `'TipCircleRadius'` |
| | | `PARAM_INVOLUTE_STEPS` | `'InvoluteSteps'` |
| | | `PARAM_TOOTH_SPACE_ANGLE` | `'ToothSpaceAngleAtRoot'` |
| | | `PARAM_TOOTH_SPACE_ARC` | `'ToothSpaceArcAtRoot'` |
| | | `PARAM_FILLET_CLEARANCE` | `'FilletClearance'` |
| | | `PARAM_FILLET_RADIUS` | `'FilletRadius'` |

The module defines exactly four classes, and each name is public API that helical, herringbone and
bevel bind to:

1. `SpurGearCommandInputsConfigurator` — a plain class with `@classmethod def configure(cls, cmd)`.
2. `SpurGearGenerationContext(GenerationContext)` — the data carrier. Its `__init__` declares
   `plane`, `anchorPoint`, `extrusionEndPlane`, `gearProfileSketch`, `toothBody`, `gearBody`,
   `centerAxis`, `extrusionExtent`, each `cast(None)`-initialised, and `toothProfileIsEmbedded`
   starting `False`.
3. `SpurGearInvoluteToothDesignGenerator` — a plain class constructed as `(sketch, parent, angle=0)`.
   It stores `self.toothAngle = angle`, adds its movable local origin as the field
   `self.anchorPoint`, and defines `drawCircles`, `drawTooth`, `drawBore`,
   `calculateInvolutePoint`, `getParameter`, `getParameterValue` and `draw`.
4. `SpurGearGenerator(Generator)` — the orchestrator. `prefixBase()` returns `'SpurGear'`, and
   `__init__` pre-initialises `self._lastToothEmbedded = False`, `self.toolsSketch = None` and
   `self.boreSketch = None`.

`calculateInvolutePoint(baseRadius, intersectionRadius)` is fully pinned and must not be inferred.
It returns `None` when `intersectionRadius < baseRadius`; otherwise `alpha = acos(baseRadius /
intersectionRadius)`, `t = tan(alpha)`, `x = baseRadius * (cos(t) + t * sin(t))`,
`y = baseRadius * (sin(t) - t * cos(t))`. The curve parameter is `tan(alpha)`, **not**
`inv(alpha) = tan(alpha) - alpha`; the involute function produces a mis-parameterised flank.

`drawCircles`, `drawTooth`, `draw` and the helpers they call may read parameters only from the key
set `spurproxy.VirtualSpurProxy` serves, because `bevelgear.py` borrows the tooth generator with
that proxy standing in for the parent: `Module`, `ToothNumber`, `PressureAngle`,
`PitchCircleDiameter`, `PitchCircleRadius`, `BaseCircleDiameter`, `BaseCircleRadius`,
`RootCircleDiameter`, `RootCircleRadius`, `TipCircleDiameter`, `TipCircleRadius`, `InvoluteSteps`.
Any other key raises `KeyError` in the bevel build.

The names in this step are declarations, not calls the module makes: `configure` is called by the
Fusion command framework, and `calculateInvolutePoint`, `drawCircles`, `drawTooth`, `drawBore`,
`getParameterValue` and `prefixBase` are defined here and consumed in the steps below.

The import block above also names `math`, `adsk`, `futil`, `Generator` and `GenerationContext`,
which are a module, a package, a logging facade and two base classes rather than calls.

<!-- check-step-calls: ignore configure calculateInvolutePoint drawCircles drawTooth drawBore getParameterValue prefixBase math adsk futil Generator GenerationContext -->

This step is `[PROSE]` because it declares names and constants and builds no geometry; there is
nothing for either harness to construct or measure.

**Playbook:** [PB-PRECOMPUTED-MODE] — the bevel proxy serves the tooth generator precomputed
values instead of real user parameters, which is why the borrowing key set above is closed.

**From:** `spec/spurgear/instructions.md` L13-33, L152-169, L253-268, L338-379, L381-406;
`.claude/skills/generate-gear/PLAYBOOK.md` L17-40, L42-98.

## S02 `[PROSE]` Dialog inputs — `SpurGearCommandInputsConfigurator.configure`

`configure(cls, cmd)` adds the inputs to `cmd.commandInputs` in exactly this order. The order is the
display order and it is fixed; do **not** regroup by input type, and do not move the selections to
the bottom because `processInputs` reads them first — that has the rule backwards.

1. Target Plane — `addSelectionInput('plane', …)`, filters `ConstructionPlanes` and `PlanarFaces`,
   `setSelectionLimits(1, 1)`.
2. Anchor Point — `addSelectionInput('anchorPoint', …)`, filters `ConstructionPoints` and
   `SketchPoints`, `setSelectionLimits(1, 1)`.
3. Module — `addValueInput('module', …, '', ValueInput.createByReal(1))`. The unit string is the
   empty string, not `'mm'`.
4. Tooth Number — `addValueInput('toothNumber', …, '', ValueInput.createByReal(17))`.
5. Pressure Angle — `addValueInput('pressureAngle', …, 'deg', ValueInput.createByReal(math.radians(20)))`.
   The display unit is degrees and the default is in radians.
6. Bore Diameter — `addStringValueInput('boreDiameter', …, '0 mm')`, so it accepts expressions.
7. Thickness — `addValueInput('thickness', …, 'mm', ValueInput.createByReal(to_cm(10)))`. The
   display unit is millimetres and the default is in centimetres.
8. Apply chamfer to teeth — `addValueInput('chamferTooth', …, 'mm', ValueInput.createByReal(0))`.
9. Generate sketches, but do not build body — `addBoolValueInput('sketchOnly', …, True)`.
10. Parent Component — `addSelectionInput('parentComponent', …)`, filters `Occurrences` and
    `RootComponents`, `setSelectionLimits(1, 1)`, pre-selecting `get_design().rootComponent` with
    `addSelection`.

Every default passed to `createByReal` is in Fusion's internal units — centimetres for length,
radians for angle — whatever the display unit string says. Every selection filter is an enum member
of `adsk.core.SelectionCommandInput`, never a quoted string.

A subclass adds its own input by subclassing this configurator and appending after
`super().configure(cmd)`, so its extra input necessarily lands after Parent Component.

`configure` is the method this step defines for the framework to call, and `addSelection` is named
above as part of the Parent Component declaration, so neither is a call the step requires on its own
account.

<!-- check-step-calls: ignore configure -->

This step is `[PROSE]` because a command dialog is not geometry: neither harness models Fusion
command inputs, so there is nothing to build or measure.

**Playbook:** [PB-DIALOG-DEFAULT-UNITS] — every createByReal default above is in internal units,
cm for length and radians for angle, whatever the display unit string says; no mechanical gate
catches a wrong one. [PB-SELECTION-FILTER-ENUM] — the filters are enum constants on the selection
input class, never quoted strings. [PB-AUTOFOCUS-FIRST] — Fusion focuses the first selection input
and ignores a later focus flag, which is part of why Target Plane is added first.

**From:** `spec/spurgear/instructions.md` L90-150, L171-178;
`.claude/skills/generate-gear/PLAYBOOK.md` L53-61, L128-144, L499-504.

## S03 `[PROSE]` Read the inputs and register the parameters — `processInputs`

Order is load-bearing. As soon as anything creates the gear occurrence — `getOccurrence()` directly,
or `addParameter()` / `parameterName()` transitively — Fusion's active component context shifts and
a `SelectionCommandInput` holding an entity in another component can drop its selection. So:

1. `get_selection(inputs, INPUT_ID_PARENT)` and resolve an `Occurrence` to its `.component` into
   `self.parentComponent`; raise on a wrong count or type.
2. `get_selection` for the plane and the anchor point into `self.plane` and `self.anchorPoint`.
   Nothing has touched the design yet at this point.
3. Read every numeric input with `get_value(inputs, id, units)` and the one boolean with
   `get_boolean(inputs, INPUT_ID_SKETCH_ONLY)`, then `addParameter(...)` each. `get_value` returns a
   `ValueInput` ready to pass straight through and raises on an invalid expression, so there is no
   ok-flag to handle. Calling `get_value` on the boolean input raises `AttributeError:
   'BoolValueCommandInput' object has no attribute 'expression'`; `SketchOnly` is persisted as a
   real-valued parameter holding 1 or 0 and read back with `getParameterAsBoolean`.
4. Call the `addExtraPrimaryParameters(inputs)` hook. It is a no-op on the spur base and must exist,
   because subclasses register their own primary parameters there — before the derived parameters
   that reference them.
5. Register the derived parameters as live expressions with
   `adsk.core.ValueInput.createByString(...)`, in dependency order:

   | parameter | units | expression |
   |---|---|---|
   | `Module` | `''` | from the input — unitless, **not** `'mm'` |
   | `ToothNumber` | `''` | from the input |
   | `PressureAngle` | `'rad'` | from the input |
   | `BoreDiameter` | `'mm'` | from the input |
   | `Thickness` | `'mm'` | from the input |
   | `ChamferTooth` | `'mm'` | from the input |
   | `SketchOnly` | `''` | 1 or 0 |
   | `PitchCircleDiameter` | `'mm'` | `Module * ToothNumber` |
   | `PitchCircleRadius` | `'mm'` | `PitchCircleDiameter / 2` |
   | `BaseCircleDiameter` | `'mm'` | `PitchCircleDiameter * cos(PressureAngle)` |
   | `BaseCircleRadius` | `'mm'` | `BaseCircleDiameter / 2` |
   | `RootCircleDiameter` | `'mm'` | `PitchCircleDiameter - 2.5 * Module` |
   | `RootCircleRadius` | `'mm'` | `RootCircleDiameter / 2` |
   | `TipCircleDiameter` | `'mm'` | `PitchCircleDiameter + 2 * Module` |
   | `TipCircleRadius` | `'mm'` | `TipCircleDiameter / 2` |
   | `InvoluteSteps` | `''` | 15 |
   | `ToothSpaceAngleAtRoot` | `''` | pre-computed in Python |
   | `ToothSpaceArcAtRoot` | `'mm'` | `RootCircleRadius * ToothSpaceAngleAtRoot` |
   | `FilletClearance` | `''` | 0.9 |
   | `FilletRadius` | `'mm'` | `(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>` |

   `Module` is registered unitless so `generateName` renders `M=1` with no unit suffix and the
   `mm`-registered expressions above read it as a bare factor.

   `ToothSpaceAngleAtRoot` is the one parameter Fusion's expression engine cannot evaluate: it
   refuses to mix the unitless output of `tan()` with the radian-valued `PressureAngle` in a
   subtraction. Compute `pi / ToothNumber - 2 * (tan(PressureAngle) - PressureAngle)` in Python and
   register it with `adsk.core.ValueInput.createByReal(...)`. Register it **unitless**, not `'rad'`,
   even though it holds a radian value: the next parameter multiplies it by a length, and Fusion
   accepts that product as a length only if this factor is unitless. Registering it as `'rad'` makes
   the product `mm·rad` and Fusion rejects the dependent parameter with `RuntimeError: Invalid
   expression`. A radian magnitude is dimensionless, so unitless is also the correct reading.

   `<factor>` in `FilletRadius` is whatever `filletHelixFactorExpression()` returns — `'1'` on the
   spur base, `'cos(<prefix>_HelixAngle)'` on helical. It is spliced in here and **nowhere else**;
   `createFillets` never calls it and reads only the resulting parameter's numeric `.value`.

`generateName()` returns
`'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression, thickness.expression)`
— the parameters' `.expression` strings, so units show through, not their `.value`.

`addExtraPrimaryParameters`, `filletHelixFactorExpression` and `generateName` are hooks this step
defines; `getParameterAsBoolean` names the read helper the later steps use rather than one this step
calls.

`getOccurrence()` is named in the ordering rule above as one of several things that can trip the
context shift, not as a call this step makes. It is defined and called inside `base.Generator`, and
this module reaches it only indirectly, through `getComponent()` and through the `addParameter()` /
`parameterName()` calls that create the occurrence transitively. The rule is a statement about when
the shift happens, so naming the direct route alongside the transitive ones is what makes it
readable; requiring the module to call it would contradict the very next sentence, which says the
transitive route is the one `processInputs` actually takes.

<!-- check-step-calls: ignore addExtraPrimaryParameters filletHelixFactorExpression generateName getParameterAsBoolean getOccurrence -->

This step is `[PROSE]` because a user-parameter table is not geometry; the numbers it registers are
proved where they become geometry, in S08 and the solid steps.

**Playbook:** [PB-INPUT-READ] — the read helper is fixed by the add-input method each field was
declared with, which is why the boolean is read with the boolean helper.
[PB-GET-VALUE-CONTRACT] — the value helper always returns a ValueInput ready to register and raises
on a bad expression, so there is no ok-flag to handle and no wrapping to do.

**From:** `spec/spurgear/instructions.md` L36-88, L324-336, L408-417;
`.claude/skills/generate-gear/PLAYBOOK.md` L99-127, L196-219.

## S04 `[PROSE]` `generate()` orchestration and the call graph

The call graph is public API. Helical and herringbone override specific methods and call `super()`
at specific points, so a reconstruction that merges or reorders these — even one that draws an
identical spur gear — breaks both dependents.

```
generate(inputs)
  → processInputs(inputs)
  → component = self.getComponent(); component.name = self.generateName()
  → normalize self.plane to a ConstructionPlane                       (S05)
  → ctx = self.newContext()
  → prepareTools(ctx)          # Tools sketch, ctx.anchorPoint, ctx.extrusionEndPlane (S06, S07)
  → buildMainGearBody(ctx)
        → buildSketches(ctx)   # Gear Profile sketch; runs the tooth generator      (S08)
        → if SketchOnly: show the Gear Profile sketch and stop                      (S09)
          else:
            → buildTooth(ctx)  # extrude the tooth → ctx.toothBody (S10)
            → buildBody(ctx)   # extrude the annular body → ctx.gearBody, centerAxis, extrusionExtent (S12)
            → patternTeeth(ctx) # circular pattern + combine (S13, S14)
            → createFillets(ctx) # root fillets (S15)
  → buildBore(ctx)             # optional bore                                       (S16, S17)
  → chamferTeeth(ctx)          # completed-gear chamfer                            (S18)
  → cleanup(ctx)               # always                                              (S19)
```

`cleanup(ctx)` is the last action of `generate()`, after `buildBore`, and is called
**unconditionally** in both modes. Do not move it into `buildMainGearBody` and do not guard the
call — the SketchOnly distinction lives inside it. Placement after `buildBore` matters because
`buildBore` re-projects `ctx.anchorPoint` from the Tools sketch and projection fails once that
sketch is hidden.

`buildSketches(ctx)` owns creating the Gear Profile sketch and invoking the tooth generator; helical
overrides it, calls `super().buildSketches(ctx)`, then draws a second twisted profile sketch.
`buildTooth(ctx)` owns turning the profile into `ctx.toothBody`. `chamferTeeth(ctx)` runs after the
optional bore, so it sees the completed patterned and filleted gear.

Every method named in the graph is defined by this module and called by it, except the ones a
subclass reaches: `newContext` is inherited.

`generate` is the one name in the graph that runs the other way round. It is the abstract method
`base.Generator` declares and this module implements, and its caller is the Fusion command
framework: the shared command wiring in commands/_gear_command.py constructs the generator class
and calls generate on it from the execute handler. So the module defines `generate` and never calls
it, and the same holds for the two later steps that name `generate()` to say where they sit in it,
S16 and S18.

<!-- check-step-calls: ignore newContext generate -->

This step is `[PROSE]` because a call graph is control flow: it produces no timeline entry and no
geometry either harness can hold.

**Playbook:** [PB-LOGGING] — log step progress through the framework logger and let the entry point's try/except and component deletion handle rollback; do not invent silent failure paths inside these methods.

**From:** `spec/spurgear/instructions.md` L270-336, L408-417;
`.claude/skills/generate-gear/PLAYBOOK.md` L244-254, L256-280.

## S05 `[GO]` Normalize the Target Plane

If the user's Target Plane selection is already a `ConstructionPlane`, use it as it stands. If it is
anything else — a planar face, say — build a coplanar construction plane from it and use that for
every later operation, so profile detection is never confused by the selected face's own profile.

```python
planeInput = component.constructionPlanes.createInput()
planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))
self.plane = component.constructionPlanes.add(planeInput)
```

The offset argument is a `ValueInput`, not a bare number: `setByOffset(plane, 0)` is a runtime
`TypeError`. Keep the normalized plane on both `self.plane` and `ctx.plane`; subclasses read
`self.plane` directly.

If this step created a plane, the end-of-build cleanup switches its light bulb off (S18).

The proof creates the plane in the sketch world and asserts its frame against the selected plane's:
the normals are parallel and the offset between the two origins along that normal is zero. Both
sides of the branch have a case.

**Proof:** `stepNormalizeTargetPlane` in `proof/spurgear/sketches_test.go`.

<!-- proof-run: proofkit.Run(normalizePlaneCases, stepNormalizeTargetPlane) -->

**Playbook:** [PB-CONSTRUCTION-PLANES] — the offset constructor's signature, and the reason the offset argument is a value rather than a bare number.

**From:** `spec/spurgear/instructions.md` L39, L421-423;
`.claude/skills/generate-gear/PLAYBOOK.md` L699-710.

## S06 `[GO]` Tools sketch and the anchor projection

Create a sketch named `Tools` on the target plane with `createSketchObject`, and project the user's
Anchor Point into it. Keep the projected `SketchPoint` as `ctx.anchorPoint`: it is the canonical
handle every later sketch re-projects from, so the whole gear tracks the user's original anchor
entity if that anchor moves.

```python
self.toolsSketch = self.createSketchObject('Tools', self.plane)
self.toolsSketch.isVisible = True
ctx.anchorPoint = self.toolsSketch.project(self.anchorPoint).item(0)
```

The sketch draws no geometry of its own; it exists to own that one reference. Leave it visible while
later sketches are still projecting from it, and hide it only in the final cleanup — projection has
failed on an invisible sketch in this repo's history, and `buildBore` re-projects from it.

A projection does not fix the projected geometry; it is a reference that tracks its source. That is
why the Gear Profile sketch does not hang off it directly but constrains its own local origin to it
in S08.

!! API CONFLICT, unresolved. The spec writes this call as `sketch.project(...)`, and the compiled
Fusion API reference (`FusionAPIReference` at `07814d19`) has no `Sketch.project`: the members it
records are `project2(entities: list[core.Base], isLinked: bool) -> list[SketchEntity]`,
`projectCutEdges` and `projectToSurface`. The step keeps the spec's call, since the spec is what
binds here; whoever settles this against a running Fusion should fix the losing side rather than let
the two disagree. `project2` is named only to record the conflict.

<!-- check-step-calls: ignore project2 createSketchObject -->

The proof builds the sketch as one projected reference point and asserts it is a reference, that it
carries no free motion of its own, and that it lands on its source position. The cases move the
anchor well off the plane origin, which is the case the whole projection chain exists for.

**Proof:** `stepToolsSketch` in `proof/spurgear/sketches_test.go`.

<!-- proof-run: proofkit.Run(toolsSketchCases, stepToolsSketch) -->

**Playbook:** [PB-HIDE-AFTER-USE] — draw, project, constrain, run the features, and only then
hide; this sketch stays visible until the bore is cut.

**From:** `spec/spurgear/instructions.md` L41, L230-235, L425-427;
`spec/spurgear/fusion.md` L19-31; `.claude/skills/generate-gear/PLAYBOOK.md` L455-469, L583-595.

## S07 `[GO]` Extrusion End Plane

Create an offset construction plane named `Extrusion End Plane` at distance `Thickness` from the
target plane, and keep it as `ctx.extrusionEndPlane`.

```python
endPlaneInput = component.constructionPlanes.createInput()
endPlaneInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))
ctx.extrusionEndPlane = component.constructionPlanes.add(endPlaneInput)
ctx.extrusionEndPlane.name = 'Extrusion End Plane'
```

The offset is a `ValueInput` here for the same reason as in S05. `thickness` is the current numeric
value of the `Thickness` parameter in centimetres, taken at generation time — a numeric snapshot,
not a live expression.

Its only purpose is to be the to-entity target for the tooth and body extrudes, so both end on the
same well-defined face. Leave it visible while those extrudes run; it is hidden at the very end with
`isLightBulbOn = False`, since `isVisible = False` does not hide a construction plane.

The proof builds the plane in the sketch world and asserts it is parallel to the target plane and
exactly `Thickness` away along that plane's normal. Thickness is swept across the range the dialog
allows, which is positive throughout.

**Proof:** `stepExtrusionEndPlane` in `proof/spurgear/sketches_test.go`.

<!-- proof-run: proofkit.Run(extrusionEndPlaneCases, stepExtrusionEndPlane) -->

**Playbook:** [PB-CONSTRUCTION-PLANES] — the offset constructor and its value argument, as in S05. [PB-NUMERIC-SNAPSHOT] — the offset is the parameter's current numeric value at generation time, not a live link, so editing the parameter afterwards does not move this plane. [PB-HIDE-AFTER-USE] — a construction plane is hidden by its light bulb and not by sketch visibility; the two are never crossed.

**From:** `spec/spurgear/instructions.md` L56, L249-251, L423, L429;
`spec/spurgear/fusion.md` L216-221; `.claude/skills/generate-gear/PLAYBOOK.md` L220-228, L699-710.

## S08 `[GO]` Gear Profile sketch

One sketch, one timeline entry, and everything the tooth generator's `draw(anchorPoint, angle=0)`
does inside it: the circles, the involute tooth, the ribs, the spine and its angular pin, the
flank-to-root closure, and the anchoring. `buildSketches` creates the sketch, runs the generator,
and afterwards copies the embedded flag across with
`ctx.toothProfileIsEmbedded = self._lastToothEmbedded`.

```python
ctx.gearProfileSketch = self.createSketchObject('Gear Profile', self.plane)
toothGen = SpurGearInvoluteToothDesignGenerator(ctx.gearProfileSketch, self)
toothGen.draw(ctx.anchorPoint)
ctx.toothProfileIsEmbedded = self._lastToothEmbedded
```

`draw` performs, in order: `drawCircles()`, `drawTooth(angle)`, the anchoring below, and then — only
when `angle != 0` — the confirming angular dimension's value as its very last action.
**`drawTooth` rotates by the `angle` argument that arrives at call time, never by the
constructor-stored `self.toothAngle`.** Helical and herringbone construct the generator with the
default `angle=0` and then call `draw(ctx.anchorPoint, angle=helixAngle)`; a `drawTooth` reading
`self.toothAngle` would draw a flat tooth and the helical loft would have no twist.

**The local origin.** The constructor adds a fresh `SketchPoint` at (0, 0, 0) as the field
`self.anchorPoint`. It is not `sketch.originPoint`, which is immutable and cannot be
coincident-constrained to anything brought in from elsewhere. All geometry below is drawn relative
to it.

```python
self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))
```

**`drawCircles` — the four circles.** In order: the **Root Circle** at `RootCircleRadius`, solid; the
**Tip Circle** at `TipCircleRadius`, the **Base Circle** at `BaseCircleRadius` and the **Pitch
Circle** at `PitchCircleRadius`, all three construction. Centre every one on the local origin by
passing that `SketchPoint` **directly**, so all four share it; do not pass `localOrigin.geometry` and
then add a centre coincident. Give each a driving diameter dimension, whose text point must be off
the centre.

```python
circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(self.anchorPoint, radius)
circle.isConstruction = True
sketch.sketchDimensions.addDiameterDimension(circle, adsk.core.Point3D.create(radius, 0, 0))
```

Never pass the trailing `isDriven=True`: a driven dimension measures instead of driving and lets the
geometry float.

Each circle also carries an along-path label. The string is
`'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` — the circle's name, its radius, and
`size = TipCircleRadius - RootCircleRadius`, all from the radii's internal `.value` in centimetres —
and that same `size` is the text height.

```python
textInput = sketch.sketchTexts.createInput2(label, size)
textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
sketch.sketchTexts.add(textInput)
```

!! API CONFLICT, unresolved. The spec and `[PB-SKETCH-TEXT]` both name createInput2(text, height)
with a plain centimetre height. The compiled API reference records no `createInput2` on
`SketchTexts`; what it records is `createInput3(expression: str, height: core.ValueInput)`, whose
height is a `ValueInput` rather than a float. The step keeps the spec's call. `createInput3` is
named only to record the conflict.

<!-- check-step-calls: ignore createInput3 -->

**`drawTooth` — the involute flanks.** Sample the flank from the base circle outward toward the tip
in equal radial steps, endpoint-inclusive: with `steps = InvoluteSteps`, sample `i` for
`i = 0 … steps-1` sits at `r = BaseCircleRadius + (TipCircleRadius - BaseCircleRadius) * i / (steps - 1)`,
so the first sample is exactly on the base circle and the last exactly on the tip circle. Do not
clamp the start to `max(BaseCircleRadius, RootCircleRadius)` — the flank is sampled from the base
circle even when the base circle sits inside the root circle. Each sample is
`calculateInvolutePoint(BaseCircleRadius, r)`; drop any that returns `None`.

Then, in this order:

1. **Mirror across +X** — negate `y` on every sample. The standard parametric involute spirals so
   its angular position grows with radius, which as a left flank gives a tooth wider at the tip than
   at the root.
2. **Rotate so the tooth centres on +X.** With `(px, py) = calculateInvolutePoint(BaseCircleRadius,
   PitchCircleRadius)`, the mirrored pitch crossing sits at polar angle `atan2(-py, px)`, so
   `rotate_angle = pi / (2 * ToothNumber) - atan2(-py, px)`. The `-py` is the mirror applied to the
   analytic point; `atan2(py, px)` is wrong. Compute it analytically rather than by interpolating
   between samples, so the tooth lands correctly however few samples are taken.
3. **Mirror the rotated result across the X axis** to get the right flank.
4. **Rotate the whole tooth by `angle`** — both flank collections, the tooth-top point, and the rib
   midpoint seeds — right here, in the same Python point math, so the tooth is drawn at its final
   angular position. Do not leave it at +X and rely on the spine's angular dimension to swing it
   there: Fusion then picks a solver branch roughly half a turn off, and the helical loft passes
   through the gear centre. For `angle = 0` this is a rotation by zero.

Draw each flank as a fitted spline through its points.

```python
points = adsk.core.ObjectCollection.create()
spline = sketch.sketchCurves.sketchFittedSplines.add(points)
```

**The tooth-top arc.** Materialize a tooth-top `SketchPoint` at
`(TipCircleRadius * cos(angle), TipCircleRadius * sin(angle))`, constrained coincident to the tip
circle. Create the arc by passing the local origin and the two flanks' end `SketchPoint`s directly,
so it shares all three and needs no separate coincidences, and add **no** diameter dimension: the
shared centre and the two shared ends already determine it.

```python
sketch.geometricConstraints.addCoincident(toothTopPoint, tipCircle)
topArc = sketch.sketchCurves.sketchArcs.addByCenterStartEnd(self.anchorPoint, rightSpline.endSketchPoint, leftSpline.endSketchPoint)
```

A free centre plus a diameter dimension would determine the arc's size but not which way it curves —
an arc of the same radius through the same two ends can bulge inward, back through the tooth — and
the sketch would reach DOF 0 with two valid answers.

**The spine, the +X reference, and the angular pin.** Draw the spine as a construction line from the
local origin to the tooth-top point, passing both existing `SketchPoint`s directly. Do not create it
from `.geometry`, do not add a separate start coincident to the origin, and do not constrain its end
onto the arc.

Build the +X reference for **every** angle, including zero:

1. Add a far endpoint at `(TipCircleRadius, 0)` and pin it with two axis dimensions from the local
   origin — a horizontal one at `TipCircleRadius` and a vertical one at `0`. Pin it this way rather
   than with a coincident onto the tip circle: a point on a circle has two answers, and the extreme
   of the circle is where the numbers go unstable.
2. Draw the reference line from the origin to that endpoint and mark it construction.
3. Add an angular dimension **from the reference to the spine, in that argument order**, with its
   text point on the bisector of the intended angle, `(R*cos(angle/2), R*sin(angle/2))` for small
   `R`, so Fusion selects the angle and not its supplement.

```python
spine = sketch.sketchCurves.sketchLines.addByTwoPoints(self.anchorPoint, toothTopPoint)
spine.isConstruction = True
sketch.sketchDimensions.addDistanceDimension(self.anchorPoint, refEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
sketch.sketchDimensions.addDistanceDimension(self.anchorPoint, refEnd, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, textPoint)
spineAngularDimension = sketch.sketchDimensions.addAngularDimension(referenceLine, spine, bisectorPoint)
```

Do not use a plain horizontal on the spine for the `angle = 0` case. Horizontal fixes the line's
direction and says nothing about which way it points, so the tooth can settle at either end of the
tip circle and come out half a turn around. The proof catches exactly this: swapping the angular
dimension for a horizontal leaves the sketch at DOF 0 and reports two discrete configurations.
`addHorizontal` is named here only to forbid it.

<!-- check-step-calls: ignore addHorizontal -->

**The ribs.** One construction line per fit-point index, for **all N indices** including the first
(base-circle) pair and the last (tip) pair. The fit points carry no other constraint, so an omitted
endpoint rib leaves that point free. Build each rib in exactly this order:

1. `addByTwoPoints(leftSpline.fitPoints[i], rightSpline.fitPoints[i])`, passing the two fit-point
   `SketchPoint`s directly so the rib shares them; mark it construction.
2. Dimension the rib with an **axis** dimension, not an aligned one. Use vertical for the rib and
   horizontal for the chain when `|cos(angle)| >= |sin(angle)|`, and swap both otherwise. An aligned
   dimension gives only a length, which the left and right flanks satisfy equally well swapped over,
   so the tooth can come out mirrored; the axis dimension's direction, captured from the seed at
   creation, is what forbids the swap.
3. Add a fresh `SketchPoint` for the midpoint, created **already on the spine**: with
   `t = fitX*cos(angle) + fitY*sin(angle)`, seed it at `(t*cos(angle), t*sin(angle))`. Not at the
   rib's true 2-D midpoint, and not at `(fitX, 0)` for a rotated tooth.
4. `addCoincident(midpoint, spine)` — on the spine first.
5. `addMidPoint(midpoint, rib)` — then the rib's midpoint.
6. `addPerpendicular(spine, rib)` — then perpendicular. **Skip this on the last rib**: the tooth-top
   arc already holds the two flank tips at equal radius either side of the spine, so it says nothing
   new and Fusion rejects it with `VCS_SKETCH_OVER_CONSTRAINTS`. The proof reproduces that: keeping
   it reports one redundant perpendicular.

Then dimension each rib's midpoint from the previous one with an axis dimension along the spine
direction, and **start the chain at the local origin** — `previous = localOrigin` for the first rib.
Without that origin-to-first dimension the whole chain has one residual degree of freedom and slides
along the spine as a unit; the proof reports DOF 1 with 45 free points when it is removed.

A dimension's value is a magnitude plus a direction captured at creation. Seed the geometry on the
intended side before creating the dimension and assign only `abs(delta)`; a negative value flips the
point to the other side.

**Closing the tooth at the root.** Let `firstRadius` be the distance from the local origin to the
left flank's first fit point. The test is strict: `embedded = firstRadius < RootCircleRadius`,
compared on raw values with no tolerance. Exact equality therefore counts as **not** embedded and
draws a zero-length stub, which is the ill-conditioned region; keep the strict comparison and do not
soften it to `<=` or add a tolerance.

When the profile is not embedded, draw a short radial flank-to-root line on each side. Build each by
passing the flank spline's **start `SketchPoint`** directly as the far endpoint, and place the root
end with **exactly these two axis dimensions from the local origin and no others**: one horizontal,
one vertical. Seed the root end at its exact computed position first, so each dimension captures its
direction from that seed.

Do **not** place it instead with "root end on the root circle" plus "local origin on the line".
Those two are satisfied by two points, because the line through the flank start and the centre
carries on and meets the root circle again on the far side; the stub then becomes a long line across
the gear and the sketch reaches DOF 0 with both answers available.

Record the outcome on the parent — `self.parent._lastToothEmbedded = True/False` — because the tooth
generator cannot reach `ctx`.

**The anchoring, S08's last constraint.** Project the Tools-sketch anchor into this sketch and
constrain that fresh projection coincident with the local origin — not with `sketch.originPoint`.
Because everything above is drawn relative to the local origin, this drags the whole tooth profile
onto the user's anchor as a unit. It happens inside `draw()`, not in `buildSketches` afterwards:
helical and herringbone build their twisted loft profile by calling the generator's `draw` directly
and rely on that single call to anchor the sketch.

```python
projectedAnchor = sketch.project(anchorPoint).item(0)
sketch.geometricConstraints.addCoincident(self.anchorPoint, projectedAnchor)
```

Then, and only when `angle != 0`, as the very last action after the entire constraint network
exists: `spineAngularDimension.parameter.value = angle`. Drawing the tooth already rotated puts the
geometry on the correct solver branch; this confirms and locks that rotation rather than swinging
the tooth into place from +X. Both are required.

**The contract this sketch owes the extrude steps.** It closes exactly two regions. The tooth loop
carries **6 curves** — 2 splines, 2 flank-to-root lines, the tooth-top arc and the root arc — or
**4 curves** when the profile is embedded, and the disc inside the root circle is bounded by
**exactly 2 arcs**, the two pieces the tooth cuts the root circle into. Both loops exist only
because the tooth meets the root circle and splits it in two. A sketch that closes neither region,
or closes them with different counts, is a broken sketch and not a later step's problem.

The proof draws this sketch entity for entity and gates it on the sketch engine's own verdict, which
asks for more than DOF 0: no conflicting or redundant constraint, valid profiles, a system that is
not near-singular, and no discrete ambiguity. It then reads the regions the sketch actually closed,
finds each by its curve counts the way `find_profile_by_curve_counts` does, checks the disc's area
against `pi * RootCircleRadius^2`, and checks that the tooth cuts the root circle in exactly two
places — which is where Fusion's two root arcs come from. The cases are the regime the spec names:
coarse and fine sizes, the angle at 0, ±30°, ±90° and 180°, the rib count at 15 and at 3, both
routes into the embedded profile, and the anchor off the plane origin.

**Proof:** `stepGearProfileSketch` in `proof/spurgear/sketches_test.go`.

<!-- proof-run: proofkit.Run(gearProfileCases, stepGearProfileSketch) -->

**Playbook:** [PB-SHARE-XOR-COINCIDENT] — share a point or coincident a fresh one, never both;
this is the rule the four circle centres, the arc, the spine and the ribs all rest on.
[PB-DIM-VALUE-SEMANTICS] — a linear dimension is a magnitude plus a direction captured at
creation, so the side is chosen by the seed and only the absolute magnitude is ever assigned.
[PB-ANGULAR-DIM] — the angular dimension measures the wedge its text point sits in, which is why
that point goes on the bisector. [PB-DRIVING-DIM] — every dimension here is driving; the trailing
driven flag is never passed. [PB-RADIAL-DIM] — a diameter dimension's text point must be off the
centre or Fusion rejects the arguments. [PB-SKETCH-TEXT] — the fixed three-call shape of the
along-path label. [PB-PROFILE-MATCH] — the two regions this sketch owes the extrude steps are
found by curve count and type, not by index.

**From:** `spec/spurgear/instructions.md` L180-251, L338-379, L431-440, L442-479, L481-485;
`spec/spurgear/fusion.md` L19-43, L47-60, L69-87, L90-114, L116-156, L158-198;
`.claude/skills/generate-gear/PLAYBOOK.md` L230-242, L438-475, L483-498, L559-560, L572-580, L596-616.

## S09 `[PROSE]` Sketch-only short-circuit

If the `SketchOnly` parameter reads true, set the Gear Profile sketch's `isVisible = True` and stop:
no tooth extrude, no body extrude, no pattern, no fillet, no chamfer, no bore. `cleanup` still runs
afterwards, unconditionally; its per-mode split leaves the sketches visible for inspection, which is
the whole point of the mode.

`getParameterAsBoolean` is the read this branch turns on, and it is required by S03 rather than by
this step.

<!-- check-step-calls: ignore getParameterAsBoolean -->

This step is `[PROSE]` because it is a branch, not a timeline entry: taking it produces no feature
and leaves no geometry, and the sketch it makes visible is the one S08 already proves.

**From:** `spec/spurgear/instructions.md` L60, L290, L487-489; `spec/spurgear/fusion.md` L202-212.

## S10 `[GO]` Extrude the tooth

Find the single tooth cross-section in the Gear Profile sketch and extrude it from the target plane
to the Extrusion End Plane as a **New Body**, named `Extrude tooth`, stored on `ctx.toothBody`.

The profile has 2 NURBS — the two flanks — 2 arcs, the tooth top and the root arc between them, and,
unless `toothProfileIsEmbedded`, 2 short line segments, the flank-to-root lines. Find it with the
framework helper and nothing else; it rejects loops whose curve counts do not match and raises when
nothing does.

```python
profile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)
extrudeInput = component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
extrude = component.features.extrudeFeatures.add(extrudeInput)
ctx.toothBody = extrude.bodies.item(0)
```

The proof builds the tooth section with its flanks as fitted splines, extrudes it the distance the
Extrusion End Plane sits at, and asserts what the next two steps select on: the body starts on the
target plane and ends on the end plane, its volume is the section area carried that far, and its
two planar cap faces each carry exactly **6** edges, or **4** when the profile is embedded, with
exactly one edge on the root circle and one on the tip circle. Those counts describe the single
tooth only; the completed-gear chamfer later scans its final end caps instead.

Two substitutions, both recorded in the proof beside the code that makes them. The tooth section is
drawn as its own closed loop with an explicit root arc, because decad refuses to record a fragment
of a circle as an exact trim, which is what Fusion's cut root circle is. And the flank is sampled at
eight points rather than fifteen, because decad's exact free-form integration exhausts its fixed
work budget above eight samples on a section carrying two splines.

**Proof:** `stepExtrudeTooth` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(toothCases, stepExtrudeTooth, assertExtrudeTooth) -->

**Playbook:** [PB-PROFILE-MATCH] — the tooth profile is found by the count and type of the curves on its loop, and the framework helper is what does the matching; a curve's type is read from its geometry and compared against the Curve3DTypes enum, whose member names all end in CurveType.

**From:** `spec/spurgear/instructions.md` L218-226, L262, L265, L491-495;
`.claude/skills/generate-gear/PLAYBOOK.md` L151-157, L596-605.

## S18 `[GO]` Chamfer the completed gear

After the optional bore, `generate` calls `self.chamferTeeth(ctx)`. It returns in SketchOnly mode
or when `ChamferTooth` is zero.

Walk every planar face of `ctx.gearBody` that is parallel to the Gear Profile sketch plane. There
must be at least one such end-cap face; raise otherwise. The selection is deliberately made after
the pattern, root fillets, and optional bore, not on the single tooth.

Add every selected face edge once, deduplicated by `edge.tempId`. Root-radius arcs remain in the
set. Exclude only a `Circle3DCurveType` edge whose radius matches the positive Bore Diameter divided
by two within `0.001` cm. If no edge remains, raise rather than create a partial chamfer.

```python
chamferInput = component.features.chamferFeatures.createInput2()
chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(chamferDistance), False)
component.features.chamferFeatures.add(chamferInput)
```

The edge set goes on the input's `chamferEdgeSets` collection — the chamfer side of the asymmetry
with fillet, which takes its edge set the other way (S15).

Helical and herringbone inherit this step unchanged. The selection does not depend on a single
tooth's profile edge count, so embedded spur geometry does not create a separate chamfer branch.

The proof asserts completed-gear cap selection, including root-radius arcs, and verifies that bore
cap circles are excluded. Both sides of the `ChamferTooth > 0` guard have a case.

Two substitutions, recorded in the proof next to the code: the body is the chorded tooth, because
decad refuses every modify operation on a spline-walled prism; and the edges actually chamfered are
the tooth's axial flank edges rather than the front-face loop, because a cap-loop chamfer of this
contour builds but its volume reading's proven bound lands outside decad's default relative
tolerance at every size and setback tried, so the solid gate cannot bless it. What is still proved
is both-cap selection — which is the part the spec says goes wrong.

**Proof:** `stepChamferTeeth` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(chamferCases, stepChamferTeeth, assertChamferTeeth) -->

**Playbook:** [PB-FILLET-CHAMFER] — the chamfer half of the asymmetry: the edge set goes on the
input's chamfer-edge-set collection, which is the opposite of where the fillet's goes.
[PB-PROFILE-MATCH] — the arc type constant used to test the root edge comes from the Curve3DTypes
enum named in that anchor. [PB-EMPTY-RESULT] — a face search can legitimately find nothing, and
this one must raise rather than fall back to a partial match.

**From:** `spec/spurgear/instructions.md` L58, L287-323, L530-545;
`.claude/skills/generate-gear/PLAYBOOK.md` L505-511, L596-605.

## S12 `[GO]` Extrude the body

Find the gear body profile — the solid disc inside the root circle, whose boundary is **exactly 2
arcs**, the two pieces the tooth's flank-to-root lines cut the root circle into. It is not an
annulus and the tip circle is not part of it: the tip circle is construction geometry and
construction geometry bounds no profile.

```python
profile = find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)
```

Extrude it from the target plane to the Extrusion End Plane as a **New Body**, name the feature
`Extrude body` and the body `Gear Body`, and store it on `ctx.gearBody`.

While iterating `extrude.bodies.item(0).faces`, capture two references, classifying each face by
`face.geometry.surfaceType`:

- The **`Gear Center` construction axis**, from any face whose surface type is
  `adsk.core.SurfaceTypes.CylinderSurfaceType`. Name it `Gear Center`, set `isLightBulbOn = False`,
  and store it on `ctx.centerAxis`.

  ```python
  axisInput = component.constructionAxes.createInput()
  axisInput.setByCircularFace(cylindricalFace)
  ctx.centerAxis = component.constructionAxes.add(axisInput)
  ```

- **`ctx.extrusionExtent`**, the far end-cap face the bore cut later ends on: among faces whose
  surface type is `adsk.core.SurfaceTypes.PlaneSurfaceType`, the one parallel to but **not** coplanar
  with the gear's sketch plane. Test it with the plane geometry rather than a hand-rolled dot
  product. The near cap is coplanar, which is what rules it out.

  ```python
  sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry)
  ```

Raise if either reference is not found.

The proof extrudes the disc and asserts both references on the body it built: exactly one cylindrical
face at the root radius whose axis is the target plane's normal, and exactly one planar face parallel
to but not coplanar with the sketch plane, at the end plane. It also checks the volume is
`pi * RootCircleRadius^2 * Thickness` and the body spans the target plane to the end plane.

**Proof:** `stepExtrudeBody` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(bodyCases, stepExtrudeBody, assertExtrudeBody) -->

**Playbook:** [PB-PROFILE-MATCH] — the disc profile is found by curve count, and the tip circle
bounds nothing because it is construction geometry. [PB-EMPTY-RESULT] — neither reference may be
assumed present; both are guarded where they are produced.

**From:** `spec/spurgear/instructions.md` L262-264, L505-514;
`.claude/skills/generate-gear/PLAYBOOK.md` L151-157, L664-685, L711-714.

## S13 `[GO]` Pattern the teeth

Circular-pattern `ctx.toothBody` around the `Gear Center` axis. Pin all three inputs explicitly
rather than relying on Fusion's defaults: quantity is `ToothNumber`, `totalAngle` is the string
expression `'360 deg'`, and `isSymmetric` is `False`.

```python
bodies = adsk.core.ObjectCollection.create()
bodies.add(ctx.toothBody)
patternInput = component.features.circularPatternFeatures.createInput(bodies, ctx.centerAxis)
patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
patternInput.isSymmetric = False
pattern = component.features.circularPatternFeatures.add(patternInput)
```

The proof places the tooth body's copies by explicit rotation about the gear axis, which is what a
circular pattern is, and asserts each copy has the seed's volume, sits at the seed's radius, and is
turned from it by exactly one full turn divided by `ToothNumber`, that many times.

SUBSTITUTION, recorded in the proof: decad has no pattern feature, and only three of the `N-1`
copies are placed — the first, the second, and the last. All of them build, but decad cannot decide
the interference pair between two of the full set and reports the document Suspect, which no gate
may pass. Those three pin the increment, its repetition, and the wrap to a full turn, which is what
quantity, `totalAngle` and `isSymmetric` between them say; what is not proved is that all N copies
coexist without interfering.

**Proof:** `stepPatternTeeth` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(patternCases, stepPatternTeeth, assertPatternTeeth) -->

**Playbook:** [PB-CIRCULAR-PATTERN] — the input shape and the three properties that must be pinned explicitly rather than left to Fusion's defaults.

**From:** `spec/spurgear/instructions.md` L516-518;
`.claude/skills/generate-gear/PLAYBOOK.md` L617-627.

## S14 `[GO]` Combine the patterned teeth into the gear body

Join the patterned tooth bodies into `Gear Body` with a single Combine-Join.

Feed the pattern's `bodies` collection to the combine as it stands: a `CircularPatternFeature`'s
`bodies` already holds the seed body plus the copies, so the seed is never re-added. Copy them into
a fresh `ObjectCollection` first, because `pattern.bodies` is a `BRepBodies` and
`combineFeatures.createInput` takes an `ObjectCollection`.

```python
tools = adsk.core.ObjectCollection.create()
for i in range(pattern.bodies.count):
    tools.add(pattern.bodies.item(i))
combineInput = component.features.combineFeatures.createInput(ctx.gearBody, tools)
combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
component.features.combineFeatures.add(combineInput)
```

The proof builds the solid this join has to leave behind and asserts it is one lump, watertight,
manifold and free of voids; that it spans the target plane to the end plane; that its volume is its
section carried that far; and that it carries exactly `ToothNumber` tooth tops and `ToothNumber` root
valleys, every one of them coaxial with the gear centre.

SUBSTITUTION, recorded in the proof: the joined solid is built as one extrude of the whole gear
cross-section rather than by a boolean. decad's booleans refuse both halves of the real route — a
free-form-walled operand cannot be tessellated, and even chorded, the tooth and the disc share their
root cylinder and both cap planes exactly, which the predicates will not classify. What is not
proved is that Fusion's Combine-Join runs; what is proved is the solid it has to produce.

**Proof:** `stepCombineTeeth` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(combineCases, stepCombineTeeth, assertCombineTeeth) -->

**Playbook:** [PB-PATTERN-BODIES] — the pattern's body collection already holds the seed plus the copies, and it must be copied into a fresh object collection before the combine will accept it.

**From:** `spec/spurgear/instructions.md` L518-520;
`.claude/skills/generate-gear/PLAYBOOK.md` L617-621.

## S15 `[GO]` Root fillets

`patternTeeth` calls `self.createFillets(ctx)` as its last action. If the `FilletRadius` parameter's
numeric `.value` is not greater than zero, return without creating anything.

Otherwise round the corner where the root valley floor meets each tooth flank — the sharp inside
corner that runs the full thickness of the gear, parallel to its main axis. That is where bending
stress concentrates at the tooth root; it is not the front or back rim, which is a cosmetic rounding
the user does not want here.

Two things make the edges fiddly:

- After the pattern and combine, the root cylinder is usually split into one patch per valley rather
  than one continuous surface. Collect **every** cylindrical face whose radius equals
  `RootCircleRadius`, not just the first one found.
- On each such face keep the **axial straight edges**, the two valley-floor-to-tooth-flank corners.
  Filter first to `adsk.core.Curve3DTypes.Line3DCurveType` edges, take each line's direction from its
  **geometry endpoints** — `geometry.startPoint.vectorTo(geometry.endPoint)` — normalize it, and keep
  it when `abs(abs(dot(direction, axisNormal)) - 1.0) < 0.01`. Use exactly that tolerance: a tighter
  test like `> 0.999` drops valid axial edges that are slightly off from tessellation and leaves root
  fillets missing. The circular edges wrapping the circumference at the end caps are rims, not
  structural corners, and are dropped. `get_normal` supplies the axis normal from the target plane.

Do **not** read the direction with `edge.evaluator.getTangent(0)`: parameter 0 is not guaranteed to
lie inside the edge's parameter range and Fusion raises `RuntimeError: invalid argument parameter`.
That call is named here only to forbid it.

```python
filletInput = component.features.filletFeatures.createInput()
filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), False)
component.features.filletFeatures.add(filletInput)
```

`isTangentChain` must be `False`: the collected edges are exactly the axial root corners, and
tangent-chaining would let Fusion pull in tangent-adjacent edges and round more than the intended
corner.

If the edge collection ends up **empty**, silently skip the fillet — return without creating the
feature, no error. An empty edge set must not reach `filletFeatures.add`.

!! API CONFLICT, unresolved, and the sharpest of the three. The spec's step 11 and
`[PB-FILLET-CHAMFER]` both say to add the edge set on the fillet input **itself** and state that
`filletInput.edgeSetInputs` does not exist and raises `AttributeError`. The compiled API reference
records the opposite: `FilletFeatureInput` has five members and `addConstantRadiusEdgeSet` is not
among them, while `edgeSetInputs: FilletEdgeSetInputs` is, and the method lives on
`FilletEdgeSetInputs` as
addConstantRadiusEdgeSet(entities: ObjectCollection, radius: ValueInput, isTangentChain: bool).
The step keeps the spec's call, since the spec binds here and the playbook records it as a
debugged-in-Fusion finding; one of the two is wrong and it has to be settled against a running
Fusion, not silently. `edgeSetInputs` is named only to record the conflict.

<!-- check-step-calls: ignore getTangent edgeSetInputs -->

The proof asserts the edge selection on the gear it built — one root-radius cylindrical face per
tooth, and exactly two axial straight edges on each, which is the `2 * ToothNumber` corners the
fillet consumes — then applies a constant-radius fillet to them and checks the result: the gear keeps
its extent, it gains material rather than losing it, and it carries two fillet walls at the fillet
radius per valley. Both sides of the radius guard have a case.

**Proof:** `stepRootFillets` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(filletCases, stepRootFillets, assertRootFillets) -->

**Playbook:** [PB-FILLET-CHAMFER] — the fillet half of the asymmetry, and the anchor the conflict
below is recorded against. [PB-PROFILE-MATCH] — the straight-edge filter uses the line type
constant from the Curve3DTypes enum. [PB-EMPTY-RESULT] — an empty edge collection is a legitimate
outcome that must skip gracefully rather than reach the fillet feature. [PB-NUMERIC-SNAPSHOT] — the
radius is the parameter's current numeric value at generation time, not a live link.

**From:** `spec/spurgear/instructions.md` L86-88, L292, L316-323, L522-531;
`.claude/skills/generate-gear/PLAYBOOK.md` L151-157, L505-511, L596-605.

## S16 `[GO]` Bore Profile sketch

`buildBore` runs unconditionally from `generate()`, so it must itself return early in **two** cases:
when `SketchOnly` is set, and when `BoreDiameter` is not greater than zero. The SketchOnly guard is
essential — in that mode `buildMainGearBody` short-circuits before `buildBody`, so `ctx.gearBody` and
`ctx.extrusionExtent` were never set and proceeding into the cut dereferences `None`. Do not rely on
the bore diameter being zero in sketch-only mode; the user may have set both.

Otherwise create a separate sketch named `Bore Profile` on the target plane and draw the bore circle
by instantiating the tooth generator on that sketch and calling its `drawBore`:

```python
self.boreSketch = self.createSketchObject('Bore Profile', self.plane)
toothGen = SpurGearInvoluteToothDesignGenerator(self.boreSketch, self)
boreCircle = toothGen.drawBore(ctx.anchorPoint, boreDiameter)
```

`drawBore(anchorPoint, diameter)` projects the anchor into this sketch, draws the construction-less
circle of that diameter centred on the projection with a driving diameter dimension, and returns the
circle.

The tooth generator's **constructor** always adds its local-origin `(0, 0, 0)` `SketchPoint`, so the
Bore Profile sketch carries one stray, unused sketch point. That is faithful behaviour; do not
suppress it. **Ground it on the projected anchor**, exactly as S08 grounds the Gear Profile's local
origin — `drawBore` already projected `ctx.anchorPoint` into this sketch to place the circle's
centre, so reuse that same projection. Do **not** ground it on `boreSketch.originPoint` instead: that
pins the point to the plane rather than to the gear, and constraining to the origin point has been
observed to throw `VCS_SKETCH_SOLVING_FAILED`. With no grounding at all the point is free in two
directions and the sketch never reaches `isFullyConstrained`.

```python
sketch.geometricConstraints.addCoincident(toothGen.anchorPoint, projectedAnchor)
```

`originPoint` is named above only to forbid it, and `generate()` is named in the opening sentence
only to say where `buildBore` sits in the call graph — the framework calls `generate`, this module
defines it, and S04 carries the full reason.

<!-- check-step-calls: ignore originPoint generate -->

The proof builds this sketch — the projected anchor, the stray local origin, the bore circle sharing
the projection as its centre with a driving diameter dimension, and the coincidence that grounds the
stray point — and gates it on the sketch engine's verdict, so a missing grounding shows up as a free
degree of freedom. The cases move the anchor off the plane origin and sweep the bore across the
range above zero.

**Proof:** `stepBoreProfileSketch` in `proof/spurgear/sketches_test.go`.

<!-- proof-run: proofkit.Run(boreProfileCases, stepBoreProfileSketch) -->

**Playbook:** [PB-CIRCLE-CENTER] — the stray local origin is grounded on the projected anchor,
rather than constrained to the sketch's own origin point. [PB-SHARE-XOR-COINCIDENT] — the bore
circle shares the projection as its centre and the stray point gets exactly one coincident to it;
never both on the same point.
[PB-DRIVING-DIM] — the bore's diameter dimension is driving.

**From:** `spec/spurgear/instructions.md` L54, L246-248, L357-362, L533-537;
`spec/spurgear/fusion.md` L26-31; `.claude/skills/generate-gear/PLAYBOOK.md` L448-454, L438-447.

## S17 `[GO]` Bore cut

Extrude-cut the bore profile from the target plane to `ctx.extrusionExtent`, the far end-cap face
captured in S12, affecting only `ctx.gearBody`. Ending on the far face guarantees the bore goes all
the way through whatever `Thickness` is.

```python
cutInput = component.features.extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
cutExtent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
cutInput.setOneSideExtent(cutExtent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
cutInput.participantBodies = [ctx.gearBody]
component.features.extrudeFeatures.add(cutInput)
```

The proof cuts a bore tool through the gear body and asserts the result: the body still spans the
target plane to the end plane, its volume is the disc less the bore, and it carries exactly one wall
at the bore radius — or none at all in the case where the bore is off. The cut body stays a single
watertight lump with no voids, which is what a through hole in a disc has to be.

Two substitutions, recorded in the proof. The body cut is the plain disc rather than the toothed
gear, because decad refuses the boolean against the full toothed section — its analytic cut scene
exceeds the evaluator's arrangement work cap — and the teeth stand outside the root circle while the
bore is inside it, so nothing about the cut depends on them. And the tool runs past both caps rather
than ending on the far end-cap face, because decad will not classify two operands whose caps are
coplanar, which a tool ending exactly on that face is; running it past both ends leaves the same
through hole.

**Proof:** `stepBoreCut` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(boreCases, stepBoreCut, assertBoreCut) -->

**From:** `spec/spurgear/instructions.md` L264, L533-537;
`.claude/skills/generate-gear/PLAYBOOK.md` L653-656.

## S18 `[PROSE]` Cleanup

`cleanup(ctx)` is the last action of `generate()` and runs in both modes. Hide each entity with the
property that hides it, never crossed: `isLightBulbOn = False` for construction planes and axes,
`isVisible = False` for sketches. A `ConstructionPlane` is not hidden by `isVisible = False`.

- The construction-plane and axis hiding **always runs, in both modes**, so no stray plane floats:
  the `Extrusion End Plane`, the `Gear Center` axis, and the normalized target plane if S05 created
  one.
- The **sketch** hiding runs **only on the full-build path**: the `Tools`, `Gear Profile` and
  `Bore Profile` sketches. Sketch-only mode leaves Tools and Gear Profile visible for inspection,
  which is the point of the mode.

Guard each entity individually and hide it only if it was actually created — the `Gear Center` axis
and the `Bore Profile` sketch do not exist in sketch-only mode.

`generate()` is named in the opening sentence only to place `cleanup` last in the call graph. The
framework calls `generate`; this module defines it and never calls it. S04 carries the full reason.

<!-- check-step-calls: ignore generate -->

This step is `[PROSE]` because visibility flags are display state: they add no timeline entry and
change no geometry, so neither harness has anything to build or measure.

**Playbook:** [PB-HIDE-AFTER-USE] — the property that hides each kind of entity, and the rule that a sketch and a construction plane are never hidden by the other's property.

**From:** `spec/spurgear/instructions.md` L239-241, L294-304, L489;
`spec/spurgear/fusion.md` L202-212; `.claude/skills/generate-gear/PLAYBOOK.md` L583-595.
