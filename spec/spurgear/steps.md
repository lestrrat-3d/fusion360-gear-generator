# Spur gear — compiled step list

The proof for these steps is `proof/spurgear/geometry_test.go`, `proof/spurgear/sketches_test.go`,
`proof/spurgear/solids_test.go` and the generated `proof/spurgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `spec/spurgear/fusion.md` | `5dccd871606c3709ecfa07c05f58c126369f2927` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9ee2dcbaed7b5480aa69e9295e8b61acaea081f3` |

## 0a `[PROSE]` Command dialog inputs

`SpurGearCommandInputsConfigurator` is a plain class with no base, carrying one
`@classmethod def configure(cls, cmd)` that adds the dialog inputs to `cmd.commandInputs`. The
class name is public API: `commands/spurgear/entry.py` binds it, and the helical and herringbone
configurators subclass it and append their own inputs after `super().configure(cmd)`, which is why
Parent Component being added last leaves a subclass's extra input below it
([SPUR-SUBCLASS-INPUT], and the four-class pattern the class belongs to).

**The add order is fixed and is the order below.** Do not regroup by input type. Target Plane and
Anchor Point are the first two inputs and Parent Component is the last one; the `processInputs`
*read* order in step 0b is a different thing and has no bearing on this. Target Plane being first
also decides which selection the dialog opens on, because Fusion auto-focuses the first
`SelectionCommandInput` and ignores a later focus flag ([PB-AUTOFOCUS-FIRST]).

| # | dialog label | input id | how it is added |
|---|---|---|---|
| 1 | Target Plane | `plane` | `addSelectionInput` |
| 2 | Anchor Point | `anchorPoint` | `addSelectionInput` |
| 3 | Module | `module` | `addValueInput` |
| 4 | Tooth Number | `toothNumber` | `addValueInput` |
| 5 | Pressure Angle | `pressureAngle` | `addValueInput` |
| 6 | Bore Diameter | `boreDiameter` | `addStringValueInput` |
| 7 | Thickness | `thickness` | `addValueInput` |
| 8 | Apply chamfer to teeth | `chamferTooth` | `addValueInput` |
| 9 | Generate sketches, but do not build body | `sketchOnly` | `addBoolValueInput` |
| 10 | Parent Component | `parentComponent` | `addSelectionInput` |

**The three selection inputs take a third argument, the command prompt Fusion shows beside the
cursor while the user picks.** It is not the label. Write these three verbatim:

| input id | `name` argument | `commandPrompt` argument |
|---|---|---|
| `plane` | `Target Plane` | `Select the plane to build the gear on` |
| `anchorPoint` | `Anchor Point` | `Select the point the gear is centered on` |
| `parentComponent` | `Parent Component` | `Select the component to build the gear in` |

So each is `cmd.commandInputs.addSelectionInput(id, name, commandPrompt)`, then its filters, then
`setSelectionLimits(1, 1)` — exactly one selection each; the filter set and the limits are contract
surface the spec declares per input, not something to improvise ([PB-SELECTION-DECL]). The filters
are named constants, never quoted literals ([PB-SELECTION-FILTER-ENUM]), and each is added with its
own call:

- `plane`: `planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)` and
  `planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)`.
- `anchorPoint`: `adsk.core.SelectionCommandInput.ConstructionPoints` and
  `adsk.core.SelectionCommandInput.SketchPoints`.
- `parentComponent`: `adsk.core.SelectionCommandInput.Occurrences` and
  `adsk.core.SelectionCommandInput.RootComponents`, and it pre-selects the root component with
  `parentInput.addSelection(get_design().rootComponent)`.

**A `createByReal` default is in Fusion's INTERNAL units — centimetres for a length, radians for
an angle — whatever the input's display unit string says ([PB-DIALOG-DEFAULT-UNITS]).** No gate
catches a wrong one, since it is valid code and a valid dialog. The five value inputs are therefore:

| input id | label | unit string | initial value |
|---|---|---|---|
| `module` | `Module` | `''` | `adsk.core.ValueInput.createByReal(1)` |
| `toothNumber` | `Tooth Number` | `''` | `adsk.core.ValueInput.createByReal(17)` |
| `pressureAngle` | `Pressure Angle` | `'deg'` | `adsk.core.ValueInput.createByReal(math.radians(20))` |
| `thickness` | `Thickness` | `'mm'` | `adsk.core.ValueInput.createByReal(to_cm(10))` |
| `chamferTooth` | `Apply chamfer to teeth` | `'mm'` | `adsk.core.ValueInput.createByReal(0)` |

Each of those five is added as
`cmd.commandInputs.addValueInput(id, label, unitString, initialValue)`. Bore Diameter is a string
input instead, so it accepts expressions —
`cmd.commandInputs.addStringValueInput(INPUT_ID_BORE_DIAMETER, 'Bore Diameter', '0 mm')` — and the
last input is a checkbox,
`cmd.commandInputs.addBoolValueInput(INPUT_ID_SKETCH_ONLY, label, True)`, whose label is the string
`Generate sketches, but do not build body` and whose third argument `True` asks for a check box
rather than a button.

`configure` is a definition the command framework calls, not a call this module makes, and so are
`prefixBase` and the four class constructors named above.

<!-- check-step-calls: ignore configure -->

**From:** `spec/spurgear/instructions.md` L13-33 L39-62 L90-106 L167-180 L182-203 L205-219 L226-243 L245-252,
`.claude/skills/generate-gear/PLAYBOOK.md` L42-74 L128-136 L138-143 L557-568

## 0b `[PROSE]` Read the inputs, register the parameters, name the component

`SpurGearGenerator(Generator)` subclasses `base.Generator`; `SpurGearGenerationContext(GenerationContext)`
is the data carrier whose `__init__` declares `plane`, `anchorPoint`, `extrusionEndPlane`,
`gearProfileSketch`, `toothBody`, `gearBody`, `centerAxis`, `extrusionExtent` and
`toothProfileIsEmbedded`, each `cast(None)`-initialised except `toothProfileIsEmbedded`, which
starts `False`. `SpurGearGenerator.__init__` also pre-initialises `self._lastToothEmbedded = False`
alongside `self.toolsSketch = None` and `self.boreSketch = None`.

**Read the three selection inputs before anything touches the design.** Creating the occurrence —
`self.getOccurrence()`, or any `self.parameterName(...)` / `self.addParameter(...)` that calls it
transitively — shifts Fusion's active component context, and a `SelectionCommandInput` holding an
entity that lives in another component can drop its selection when that happens
([PB-SELECTION-STASH]). Numeric and boolean inputs are immune. So `processInputs(inputs)` runs in
this order:

1. `get_selection(inputs, INPUT_ID_PARENT)`, resolving an `Occurrence` to its `.component`, into
   `self.parentComponent`; raise on the wrong count or type.
2. `get_selection(inputs, INPUT_ID_PLANE)` into `self.plane` and
   `get_selection(inputs, INPUT_ID_ANCHOR_POINT)` into `self.anchorPoint`.
3. Register the input-sourced parameters with `get_value(inputs, id, units)` for every value and
   string input and `get_boolean(inputs, INPUT_ID_SKETCH_ONLY)` for the checkbox. `get_value`
   returns a `ValueInput` ready to hand straight to `self.addParameter`; a checkbox has no
   `.expression`, so reading it with `get_value` raises `AttributeError` at generation time — the
   read helper is fixed by the `add*Input` that declared the input ([PB-INPUT-READ]), and
   `get_value` always hands back a `ValueInput` ready to register, raising rather than returning
   `None` on a bad expression ([PB-GET-VALUE-CONTRACT]).
   `SketchOnly` is registered as a real-valued 1/0 parameter, since the framework reads booleans
   back numerically with `self.getParameterAsBoolean(PARAM_SKETCH_ONLY)`.
4. `self.addExtraPrimaryParameters(inputs)` — the [SPUR-EXTRA-PARAMS] hook, a no-op on the spur base, that
   exists so a subclass can register its own primary parameters between the input-sourced ones and
   the derived ones.
5. `self.registerDerivedParameters()`.

**`addParameter(name, ValueInput, units, comment)` takes a fourth string that Fusion shows in the
parameter table's Comment column.** It is what the user reads, so it is not free text. These are
the twenty parameters, their units and their comments, verbatim:

| constant | parameter name | units | `comment` |
|---|---|---|---|
| `PARAM_MODULE` | `Module` | `''` | `Module of the gear` |
| `PARAM_TOOTH_NUMBER` | `ToothNumber` | `''` | `Number of teeth` |
| `PARAM_PRESSURE_ANGLE` | `PressureAngle` | `'rad'` | `Pressure angle` |
| `PARAM_BORE_DIAMETER` | `BoreDiameter` | `'mm'` | `Bore diameter` |
| `PARAM_THICKNESS` | `Thickness` | `'mm'` | `Thickness of the gear` |
| `PARAM_CHAMFER_TOOTH` | `ChamferTooth` | `'mm'` | `Chamfer distance applied to the teeth` |
| `PARAM_SKETCH_ONLY` | `SketchOnly` | `''` | `Generate sketches only` |
| `PARAM_PITCH_DIAMETER` | `PitchCircleDiameter` | `'mm'` | `Pitch circle diameter` |
| `PARAM_PITCH_RADIUS` | `PitchCircleRadius` | `'mm'` | `Pitch circle radius` |
| `PARAM_BASE_DIAMETER` | `BaseCircleDiameter` | `'mm'` | `Base circle diameter` |
| `PARAM_BASE_RADIUS` | `BaseCircleRadius` | `'mm'` | `Base circle radius` |
| `PARAM_ROOT_DIAMETER` | `RootCircleDiameter` | `'mm'` | `Root circle diameter` |
| `PARAM_ROOT_RADIUS` | `RootCircleRadius` | `'mm'` | `Root circle radius` |
| `PARAM_TIP_DIAMETER` | `TipCircleDiameter` | `'mm'` | `Tip circle diameter` |
| `PARAM_TIP_RADIUS` | `TipCircleRadius` | `'mm'` | `Tip circle radius` |
| `PARAM_INVOLUTE_STEPS` | `InvoluteSteps` | `''` | `Number of points sampled along each involute flank` |
| `PARAM_TOOTH_SPACE_ANGLE` | `ToothSpaceAngleAtRoot` | `''` | `Angular width of the tooth space at the root circle` |
| `PARAM_TOOTH_SPACE_ARC` | `ToothSpaceArcAtRoot` | `'mm'` | `Arc length of the tooth space at the root circle` |
| `PARAM_FILLET_CLEARANCE` | `FilletClearance` | `''` | `Clearance factor applied to the root fillet radius` |
| `PARAM_FILLET_RADIUS` | `FilletRadius` | `'mm'` | `Radius of the root fillets` |

Every id and every parameter name above is exported as a module-level constant of exactly the
name in the left column — `INPUT_ID_PARENT`, `INPUT_ID_PLANE`, `INPUT_ID_ANCHOR_POINT`,
`INPUT_ID_MODULE`, `INPUT_ID_TOOTH_NUMBER`, `INPUT_ID_PRESSURE_ANGLE`, `INPUT_ID_BORE_DIAMETER`,
`INPUT_ID_THICKNESS`, `INPUT_ID_CHAMFER_TOOTH`, `INPUT_ID_SKETCH_ONLY` and the twenty `PARAM_…`
names — because `helicalgear.py` and `herringbonegear.py` import `PARAM_MODULE`,
`PARAM_TOOTH_NUMBER` and `PARAM_THICKNESS` from `.spurgear` by name ([SPUR-EXPORTED-CONSTANTS]).

**`Module` is registered unitless (`''`), not `'mm'`.** That is what makes `generateName` render
`M=1` with no unit suffix and what lets the `mm`-registered derived expressions read the unitless
factor.

The derived parameters are registered as live expression strings with
`adsk.core.ValueInput.createByString(...)`, using `self.parameterName(...)` to build each
reference:

- `PitchCircleDiameter` = `Module * ToothNumber`
- `PitchCircleRadius` = `PitchCircleDiameter / 2`
- `BaseCircleDiameter` = `PitchCircleDiameter * cos(PressureAngle)`
- `BaseCircleRadius` = `BaseCircleDiameter / 2`
- `RootCircleDiameter` = `PitchCircleDiameter - 2.5 * Module`
- `RootCircleRadius` = `RootCircleDiameter / 2`
- `TipCircleDiameter` = `PitchCircleDiameter + 2 * Module`
- `TipCircleRadius` = `TipCircleDiameter / 2`
- `InvoluteSteps` = `15`, unitless
- `FilletClearance` = `0.9`, unitless
- `ToothSpaceArcAtRoot` = `RootCircleRadius * ToothSpaceAngleAtRoot`
- `FilletRadius` = `(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>`, where `<factor>` is
  the string `self.filletHelixFactorExpression()` returns — `'1'` on the spur base, spliced in
  here and nowhere else. `createFillets` never reads that hook; it reads the resulting
  `FilletRadius` parameter's numeric `.value`.

**`ToothSpaceAngleAtRoot` is pre-computed in Python and registered with
`adsk.core.ValueInput.createByReal(...)`, unitless (`''`), not `'rad'`.** Its value is
`math.pi / ToothNumber - 2 * (math.tan(pressureAngle) - pressureAngle)`. Fusion's expression engine
refuses to subtract a radian-valued term from the unitless output of `tan()`, which is why it is
not a live expression; and registering it as `'rad'` makes the `ToothSpaceArcAtRoot` product read
as `mm·rad`, which Fusion rejects with `RuntimeError: Invalid expression`. A radian magnitude is
dimensionless, so unitless is also the correct reading.

`generate(inputs)` then calls `self.processInputs(inputs)`, takes
`component = self.getComponent()` and sets `component.name = self.generateName()`.
`generateName` returns
`'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression, thickness.expression)`
— the `.expression` strings of the `Module`, `ToothNumber` and `Thickness` parameters, not their
`.value`, so units show through as in `Spur Gear (M=1, Tooth=17, Thickness=10 mm)`.

**Five methods must carry a return annotation**, because helical and herringbone narrow on them
and an unannotated parent is read as returning the literal it happens to return:
`SpurGearGenerator.prefixBase -> str`, `SpurGearGenerator.generateName -> str`,
`SpurGearGenerator.filletHelixFactorExpression -> str`,
`SpurGearGenerator.newContext -> SpurGearGenerationContext`, and
`SpurGearInvoluteToothDesignGenerator.getParameterValue -> float`. `prefixBase` returns
`'SpurGear'`.

The rest of `generate` runs, in this order and with these method boundaries, which subclasses
override at: `self.prepareTools(ctx)` (steps 1 and 2), `self.buildMainGearBody(ctx)` — which calls
`self.buildSketches(ctx)`, then either short-circuits on SketchOnly or calls `self.buildTooth(ctx)`,
`self.buildBody(ctx)`, `self.patternTeeth(ctx)` and `self.createFillets(ctx)` — then
`self.buildBore(ctx)`, `self.chamferTeeth(ctx)` and finally `self.cleanup(ctx)`.

Every dimension and feature input written from here on is a numeric snapshot of its parameter's
`.value` at generation time, never a live expression ([PB-NUMERIC-SNAPSHOT], applied to this gear
by [SPUR-F-SNAPSHOT]): editing a `<prefix>_…` parameter afterwards does not change an existing
gear, and the user re-runs the dialog instead.

Three names above are mentions rather than requirements. `generate` is the method this step
describes, defined here for the command framework to call and not called by this module.
`getOccurrence` is named only to say what triggers the context shift the read order exists to
dodge — the occurrence is normally reached transitively, through the first parameter registration,
so a module that never writes the call still obeys the rule. `prefixBase` is likewise a definition
the inherited framework calls.

<!-- check-step-calls: ignore generate getOccurrence prefixBase -->

**From:** `spec/spurgear/instructions.md` L13-33 L37-88 L108-124 L133-165 L221-243 L327-342 L344-410
L454-479 L481-490,
`spec/spurgear/fusion.md` L212-217 L233-240,
`.claude/skills/generate-gear/PLAYBOOK.md` L75-102 L103-118 L120-126 L205-227 L229-237 L253-263

## 1 `[PROSE]` Normalize the Target Plane

If `self.plane` is not already a `ConstructionPlane` — the user may have picked a planar face —
build a coplanar construction plane and replace `self.plane` with it, so later profile detection
never has to filter out the selected face's own profile. Store the same plane on `ctx.plane`.

The offset argument is a `ValueInput`, not a bare number:
`planeInput = component.constructionPlanes.createInput()`, then
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))`, then
`component.constructionPlanes.add(planeInput)`. Passing a bare `0` is a runtime `TypeError`;
[PB-CONSTRUCTION-PLANES] gives the signature.

Remember whether a plane was created here; step 14's cleanup turns its light bulb off only if it
exists.

**From:** `spec/spurgear/instructions.md` L39 L332 L494-496,
`.claude/skills/generate-gear/PLAYBOOK.md` L253-263 L775-786

## 2 `[PROSE]` Tools sketch and Extrusion End Plane

`prepareTools(ctx)` creates a sketch named `Tools` on the target plane with
`self.createSketchObject('Tools', self.plane)`, makes it visible, and projects the user's anchor
point into it: `toolsSketch.project(self.anchorPoint)`, keeping the resulting `SketchPoint` as
`ctx.anchorPoint`. That projection is the canonical handle — every later sketch projects *this* in
again ([SPUR-F-ANCHOR-CHAIN]), so the whole gear follows the user's original anchor entity if it
moves. The sketch draws no
geometry of its own. Keep it on `self.toolsSketch` and leave it visible: step 12 re-projects from
it, and projection fails once it is hidden.

Then create an offset construction plane named `Extrusion End Plane` at distance `Thickness` from
the target plane, again with a `ValueInput`:
`endPlaneInput = component.constructionPlanes.createInput()`,
`endPlaneInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))`,
`component.constructionPlanes.add(endPlaneInput)`. `thickness` is the `Thickness` parameter's
numeric `.value`, in internal centimetres. Store it as `ctx.extrusionEndPlane` and leave it visible
while the two extrudes run; step 14 switches its light bulb off, because `isVisible = False` does
not hide a construction plane ([PB-HIDE-AFTER-USE]).

This step is `[PROSE]` because neither harness has a counterpart for it. The Tools sketch holds one
projected reference point and no constraint scheme to prove, and the offset construction plane is
not a body — the solid harness builds bodies and has nothing that stands for a datum. What the
projection chain buys is proven where it bites instead: the Gear Profile and Bore Profile sketches
of steps 3 and 12a are each drawn against a projected anchor away from the sketch origin, and each
is held to full constraint after the drag.

**From:** `spec/spurgear/instructions.md` L41 L332-338 L498-502, `spec/spurgear/fusion.md` L19-24,
`.claude/skills/generate-gear/PLAYBOOK.md` L659-671 L775-786

## 3 `[GO]` Gear Profile sketch — circles, involute tooth, anchoring

Proof function `stepGearProfile` in `proof/spurgear/sketches_test.go`.

<!-- proof-run: proofkit.RunWithExpectedFailures(profileCases, stepGearProfile, profileFailures) -->

One sketch is one entry in the Fusion timeline, so the four circles, the whole involute tooth and
the anchoring are this single step. This is the scheme [PB-SKETCH-FIRST] requires to be proven on
the bench before any Fusion code is written, and it has to end fully constrained
([PB-FULL-CONSTRAINT]) without a single redundant constraint added to get there
([PB-NO-OVERCONSTRAIN]). `buildSketches(ctx)` owns creating the sketch and running the
tooth generator, and nothing else:
`sketch = self.createSketchObject('Gear Profile', self.plane)`, store it on `ctx.gearProfileSketch`,
make it visible, then
`toothGen = SpurGearInvoluteToothDesignGenerator(sketch, self)` and
`toothGen.draw(ctx.anchorPoint)`. Afterwards copy the embedded flag across:
`ctx.toothProfileIsEmbedded = self._lastToothEmbedded`. Helical overrides this method, calls
`super().buildSketches(ctx)` and then draws a second, twisted profile sketch, so the work must stay
inside this boundary.

The generator's constructor is `(sketch, parent, angle=0)`. It stores `self.toothAngle = angle` as
an incidental field and adds its movable **local origin** — a fresh `SketchPoint` at (0, 0, 0),
kept in the field `self.anchorPoint`, never `sketch.originPoint`, which is immutable and cannot be
made coincident with anything brought in from elsewhere ([SPUR-F-LOCAL-ORIGIN]). All geometry below is drawn relative to
that point and dragged onto the anchor at the end.

`draw(anchorPoint, angle=0)` performs, in order, `self.drawCircles()`, `self.drawTooth(angle)`, the
anchoring, and then — only when `angle != 0`, and as the very last action after the whole
constraint network exists — the confirming angular dimension's value assignment. Drawing the
rotation and confirming it are two distinct, both-required actions ([SPUR-F-ROTATE-CONFIRM]).
**`drawTooth` must rotate by the `angle` argument that flows in from `draw` at call time, never by
the stored `self.toothAngle`**: helical and herringbone construct the generator with the default
`angle=0` and then call `draw(ctx.anchorPoint, angle=helixAngle)`, so reading the stored value
would draw a flat tooth and the loft would have no twist.

**drawCircles.** Four circles, each centred by passing the local-origin `SketchPoint` *directly* as
the centre so all four share that one point — never `localOrigin.geometry` followed by a coincident,
which stacks a redundant self-coincident and kills the solver ([PB-SHARE-XOR-COINCIDENT], applied
to this loop by [SPUR-F-SHARED-ADJACENCY]). Each gets a driving diameter dimension; none is ever
created with `isDriven=True` ([PB-DRIVING-DIM]). The curve collections live under
`sketch.sketchCurves`, never on the sketch directly ([PB-SKETCHCURVES]), and every constraint
method name is copied exactly rather than inferred ([PB-API-SPELLING]).

| order | circle | radius parameter | construction? |
|---|---|---|---|
| 1 | Root Circle | `RootCircleRadius` | no — solid |
| 2 | Tip Circle | `TipCircleRadius` | yes |
| 3 | Base Circle | `BaseCircleRadius` | yes |
| 4 | Pitch Circle | `PitchCircleRadius` | yes |

Each is `sketch.sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)` followed by
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`, whose text point must be off the
centre — a text point at the centre is rejected, because there is no radial direction there
([PB-RADIAL-DIM]).

Each circle is also labelled with along-path text. The label string is
`'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` — the circle's name, its radius, and
`size`, all from the radii's internal `.value` in centimetres — where
`size = TipCircleRadius - RootCircleRadius`, and that same `size` is the text height. The three
calls are the fixed shape [PB-SKETCH-TEXT] gives: `textInput = sketch.sketchTexts.createInput2(text, size)`, then
`textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`,
then `sketch.sketchTexts.add(textInput)`. Those four labels are also why this sketch cannot be
gated on `isFullyConstrained` in Fusion: text carries its own position along the curve and nothing
pins it, so a labelled sketch reads under-constrained for a reason the geometry has nothing to do
with ([PB-TEXT-HOLDS-DOF]). Log the reading, never raise on it.

**drawTooth — the point math.** With `steps = InvoluteSteps`, sample `i` for `i = 0 … steps-1` sits
at radius `r = BaseCircleRadius + (TipCircleRadius - BaseCircleRadius) * i / (steps - 1)`, so the
first sample is exactly on the base circle and the last exactly on the tip circle. Do **not** clamp
the start to `max(BaseCircleRadius, RootCircleRadius)`; the embedded case is detected later from
where the flank *start* lands, not by trimming the sampling. Each sample is
`self.calculateInvolutePoint(baseRadius, r)`, whose exact math is

```
alpha = acos(baseRadius / intersectionRadius)
t     = tan(alpha)
x = baseRadius * (cos(t) + t * sin(t))
y = baseRadius * (sin(t) - t * cos(t))
```

returning `None` when `intersectionRadius < baseRadius`. The curve parameter is `tan(alpha)`, not
`inv(alpha) = tan(alpha) - alpha`; the involute function is the common substitution and gives a
mis-parameterised flank. Drop every `None` sample.

Then, in this order:

1. **Mirror** every sample across +X (negate y). The standard parametric involute spirals so its
   angular position grows with radius, which as a left flank gives a tooth wider at the tip than at
   the root.
2. **Rotate** the mirrored samples by
   `rotate_angle = math.pi / (2 * ToothNumber) - math.atan2(-py, px)`, where
   `(px, py) = self.calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`. The `-py` is the
   step-1 mirror applied to the analytic point; `atan2(py, px)` is wrong. Computing the pitch
   crossing analytically rather than interpolating between samples is what places the tooth at
   exactly the right angle however few samples are taken. This is the **left** flank.
3. **Mirror the rotated left flank across the X axis** to get the **right** flank.
4. **Rotate both flank collections by `angle`**, and place the tooth-top point and seed every rib
   midpoint at their `angle`-rotated positions too. Draw the tooth directly at its final angular
   position; do not leave it at +X and rely on the spine's angular dimension to swing it there,
   which lets Fusion pick a branch about 180 degrees away and ruins the helical loft. At
   `angle = 0` this is a no-op.

**drawTooth — the geometry.** Every seed coordinate below is placed where the constraints will
leave it, because the solver is seed-sensitive and a seed on the wrong side of a target can fail to
converge on a perfectly solvable system ([PB-SEED-NEAR]). Draw each flank as a fitted spline
through its point collection:
build an `adsk.core.ObjectCollection.create()` of `adsk.core.Point3D.create(x, y, 0)` points and
pass it to `sketch.sketchCurves.sketchFittedSplines.add(points)`.

*Tooth-top arc.* Materialize a tooth-top point at
`(TipCircleRadius * cos(angle), TipCircleRadius * sin(angle))` with
`sketch.sketchPoints.add(...)` and constrain it coincident to the tip circle:
`sketch.geometricConstraints.addCoincident(toothTopPoint, tipCircle)`. Create the arc with
`sketch.sketchCurves.sketchArcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)`,
passing the two flank splines' end `SketchPoint`s directly — that call shares the start and end but
**copies the centre** ([PB-SHARE-XOR-COINCIDENT]'s own exception, and the one place in this sketch
where passing a point and then coincidenting to it is right) — and then tie the centre back with
`sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, localOrigin)`. Add no diameter
dimension: the coincident centre and the two shared ends already determine the arc, and a free
centre with a diameter would fix its size but not which way it bulges. The whole recipe, and the
reason it is four steps and not three, is [SPUR-F-TOOTHTOP-ARC].

*Spine, +X reference and the confirming angular dimension* ([SPUR-F-SPINE]). The spine is a construction line
`sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, toothTopPoint)`, sharing both existing
points — no extra start-coincident, and no constraint of its end onto the arc. Build the +X
reference line for **every** angle including 0: add a far endpoint at `(TipCircleRadius, 0)`, pin
it with two axis dimensions from the local origin —
`sketch.sketchDimensions.addDistanceDimension(localOrigin, referenceEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
at `TipCircleRadius` and the same with
`adsk.fusion.DimensionOrientations.VerticalDimensionOrientation` at `0` — draw the line from the
origin to it and mark it construction. Pin it this way rather than by putting its end on the tip
circle: a point on a circle has two answers, and the tangency at the extreme is numerically
unstable. Then add
`sketch.sketchDimensions.addAngularDimension(referenceLine, spine, textPoint)`, in that argument
order, with the text point on the bisector of the intended angle,
`(R * cos(angle / 2), R * sin(angle / 2))` for a small `R`, so Fusion measures `angle` and not its
supplement — an angular dimension measures the wedge its text point sits in ([PB-ANGULAR-DIM]).
A plain horizontal constraint on the spine will not do for `angle = 0`: horizontal
fixes the line's direction but not which way it points, so the tooth can settle 180 degrees around.

*Ribs* ([SPUR-F-RIBS]). One rib per fit-point index `i`, for all N indices, endpoints included —
with N samples per flank there are N ribs, and a missing endpoint rib leaves that fit point free.
Build each in this exact order; a different order over-constrains the sketch:

1. `sketch.sketchCurves.sketchLines.addByTwoPoints(leftSpline.fitPoints.item(i), rightSpline.fitPoints.item(i))`,
   sharing the two fit points, marked construction.
2. An **axis** dimension across the spine, created with the fit points already at their seeded
   positions and left at the measured magnitude, the direction being captured from the seed at
   creation rather than carried by the value ([PB-DIM-VALUE-SEMANTICS]): vertical when
   `abs(cos(angle)) >= abs(sin(angle))`, horizontal otherwise. An aligned dimension gives only the
   length, which the two flanks satisfy equally well swapped over, and the tooth can come out
   mirrored; the axis dimension's captured direction forbids the swap.
3. A fresh midpoint `SketchPoint`, created **already on the spine**, at the foot of the left fit
   point on it: with `t = fitX * cos(angle) + fitY * sin(angle)`, the seed is
   `(t * cos(angle), t * sin(angle))`. Never the rib's true 2-D midpoint, and never `(fitX, 0)` for
   a rotated tooth.
4. `sketch.geometricConstraints.addCoincident(midpoint, spine)`.
5. `sketch.geometricConstraints.addMidPoint(midpoint, rib)`.
6. `sketch.geometricConstraints.addPerpendicular(spine, rib)` — **skipped on the last rib only**,
   because the tooth-top arc already holds the two flank tips at equal radius either side of the
   spine and Fusion rejects the redundant perpendicular.

Then dimension each rib's midpoint from the previous one with an **axis** dimension along the spine
— horizontal when `abs(cos(angle)) >= abs(sin(angle))`, vertical otherwise — and **for the first
rib dimension it from the local origin**, starting the chain with `previous = localOrigin`. Without
that origin-to-first link the whole chain slides along the spine as a unit and the sketch never
fully constrains.

*Flank-to-root lines and the embedded test* ([SPUR-F-FLANK-ROOT]). Let `firstRadius` be the distance from the local
origin to the left flank's first fit point. The test is strict:
`embedded = firstRadius < RootCircleRadius`, comparing raw values with no tolerance, so exact
equality counts as **not** embedded and draws a zero-length stub. Do not relax it to `<=`.

- **Not embedded** (the common case): on each side, seed the root end at its exact computed
  position, draw
  `sketch.sketchCurves.sketchLines.addByTwoPoints(rootEndPoint, flankStartFitPoint)` — sharing the
  spline's start point, with no separate coincident — and place the root end with **exactly two**
  axis dimensions from the local origin and no others: `addDistanceDimension` with
  `HorizontalDimensionOrientation` and the same with `VerticalDimensionOrientation`. Set their
  values to `abs(dx)` and `abs(dy)` only; a negative `parameter.value` flips the point to the other
  side of the origin ([PB-DIM-VALUE-SEMANTICS]), which mirrored the right-hand root end and left
  the tooth loop open when it was found in Fusion. Do **not** place it instead with the root end on the root circle plus the
  local origin on the line: those two are satisfied by the far intersection as well, and the stub
  becomes a line straight across the gear. The tooth loop then has **6 curves** — 2 splines,
  2 flank-to-root lines, 2 arcs.
- **Embedded**: no flank-to-root line is drawn and the loop has **4 curves** — 2 splines, 2 arcs.
  This happens above `2.5 / (1 - cos(PressureAngle))` teeth: 41.5 at 20 degrees, 78.5 at 14.5 and
  26.7 at 25, so a larger pressure angle brings it on sooner.

Record which shape was drawn by writing `self.parent._lastToothEmbedded = True` or `False` from
inside `drawTooth`; the tooth generator cannot reach `ctx`, which is why `buildSketches` copies the
flag across. The bevel gear reads the same slot off its proxy after `draw` returns, so the write
has to stay.

**The anchoring, inside `draw`.** After `drawTooth` returns, project the Tools-sketch anchor into
this sketch — `sketch.project(anchorPoint)` — and add
`sketch.geometricConstraints.addCoincident(self.anchorPoint, projectedAnchor)` between the freshly
projected point and the generator's local origin, not `sketch.originPoint` — a projection is
brought in associatively and still carries free degrees of freedom, so coincidenting it to the one
natural anchor is what turns it into fully-constrained local geometry ([PB-PROJECT-NOT-FIXED]). Because every piece of
geometry is constrained relative to the local origin, this one constraint drags the whole tooth
profile onto the anchor as a unit. It happens here rather than in `buildSketches` because helical
and herringbone call `draw` directly on their loft sketch and rely on this call to anchor it.
Finally, when `angle != 0`, set `spineAngularDimension.parameter.value = angle` — the very last
action, after the entire constraint network exists.

**Borrowing constraint.** Inside `drawCircles`, `drawTooth` and `draw`, and in every helper they
call, read parameters only from the keys the bevel gear's proxy serves: `Module`, `ToothNumber`,
`PressureAngle`, `PitchCircleDiameter`, `PitchCircleRadius`, `BaseCircleDiameter`,
`BaseCircleRadius`, `RootCircleDiameter`, `RootCircleRadius`, `TipCircleDiameter`,
`TipCircleRadius`, `InvoluteSteps`. Any other key raises `KeyError` and breaks the bevel build,
which reaches this drawer through a precomputed-value proxy rather than a Fusion parameter table
([PB-PRECOMPUTED-MODE]).
Read them through `self.getParameter(name)` and `self.getParameterValue(name)`, and when a drawing
step needs one of the four circles back, either keep the reference from `drawCircles` or locate it
with `find_circle_by_radius(sketch, radius)` — the two are alternatives the spec allows, and
neither may fall back to an arbitrary circle on a failed match.

`find_circle_by_radius` is named here as one of two permitted ways to recover a circle, not as a
call the module must make.

<!-- check-step-calls: ignore find_circle_by_radius -->

**What the proof checks.** `stepGearProfile` rebuilds this scheme in the sketch engine and holds it
to the engine's own verdict — DOF 0, no conflicting or redundant constraint, valid profiles, a
system that is not near-singular, and no discrete ambiguity — across the whole regime the spec
names: coarse and fine sizes, the signed angle range from a negative helix through zero and a
quarter turn to 180 degrees, three involute samples as well as fifteen, and both routes into the
embedded shape. Several cases drag the sketch onto an anchor well away from the sketch origin,
which is the case a spur gear normally hides. It then asserts the contract steps 7 and 9 select on:
the sketch closes exactly two regions, the tooth loop carries 2 splines + 2 arcs + 2 lines (or
+ 0 lines when embedded), the disc is bounded by the root circle alone with area `pi * r^2`, and
the tooth-top arc's solved radius is the tip radius with its centre on the anchor.
`profileFailures` carries the negative control the spec requires: the same sketch with the arc's
centre left free reports DOF 2 and underconstrained, and must keep failing.

**From:** `spec/spurgear/instructions.md` L254-325 L411-452 L504-513 L515-552 L554-558,
`spec/spurgear/fusion.md` L19-43 L47-60 L69-106 L108-133 L135-175 L177-217,
`.claude/skills/generate-gear/PLAYBOOK.md` L239-251 L359-431 L441-478 L501-516 L517-532 L606-634 L635-636
L652-656 L682-692

## 6 `[PROSE]` Sketch-only short circuit

Inside `buildMainGearBody`, after `buildSketches` returns, read the boolean back with
`self.getParameterAsBoolean(PARAM_SKETCH_ONLY)`. When it is true, set the Gear Profile sketch's
`isVisible = True` and return, skipping the tooth extrude, the body extrude, the pattern, the
combine and the fillets. `buildBore` and `chamferTeeth` still run from `generate` and guard
themselves, and `cleanup` still runs unconditionally with the per-mode split [SPUR-F-CLEANUP]
owns and step 14 describes.

**From:** `spec/spurgear/instructions.md` L60 L344-380 L560-562, `spec/spurgear/fusion.md` L221-231

## 7 `[GO]` Extrude the tooth

Proof function `stepExtrudeTooth` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeTooth, assertExtrudeTooth) -->

`buildTooth(ctx)` owns this step and nothing else; helical overrides it to loft instead, and it
never applies a chamfer.

Find the single tooth cross-section in the Gear Profile sketch with the framework helper, which
rejects loops whose curve counts do not match and raises when nothing does ([PB-PROFILE-MATCH]) —
do not re-implement the loop search:

`find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`

Extrude that profile from the target plane to the Extrusion End Plane as a **New Body**:

- `extrudeInput = component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`
- `extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`
- `extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`
- `extrude = component.features.extrudeFeatures.add(extrudeInput)`

Name the feature `Extrude tooth` and store the resulting body as `ctx.toothBody`.

**What the proof checks.** `stepExtrudeTooth` draws the tooth boundary and extrudes it by
Thickness, then asserts that the prism's volume is its cross-section times Thickness, that it runs
from the target plane to exactly Thickness and no further, and that it reaches the tip radius. Two
substitutions are recorded in the proof file: the flank is sampled at seven points rather than
fifteen, because the solid engine's free-form work budget refuses a fifteen-point spline and the
chorded flank costs three parts in ten thousand of tooth area; and the root arc is drawn explicitly
rather than derived by splitting the root circle, because the engine will not record a circle
fragment whose trim it could not certify. That the split produces that boundary is what step 3
asserts instead.

**From:** `spec/spurgear/instructions.md` L297-300 L334-339 L344-371 L564-568,
`.claude/skills/generate-gear/PLAYBOOK.md` L151-158 L672-681

## 9 `[GO]` Extrude the gear body

Proof function `stepExtrudeBody` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeBody, assertExtrudeBody) -->

`buildBody(ctx)` finds the gear body profile — the solid disc inside the root circle, whose
boundary is **exactly 2 arcs**, the two pieces the tooth cuts the root circle into:
`find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)` ([PB-PROFILE-MATCH]). It is not an annulus and the tip
circle is no part of it, because the tip circle is construction geometry and construction geometry
bounds no profile.

Extrude it from the target plane to the Extrusion End Plane as a **New Body**, exactly as step 7
does: `component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`component.features.extrudeFeatures.add(extrudeInput)`. Name the feature `Extrude body` and the
resulting body `Gear Body`, and store it on `ctx.gearBody`.

While iterating `extrude.bodies.item(0).faces`, classify each face by `face.geometry.surfaceType`,
which is the surface-kind search [PB-FACE-BY-MIDPOINT] describes, and capture two references.
Raise if either is not found rather than carrying an empty collection forward
([PB-EMPTY-RESULT], [PB-SELF-DIAGNOSING]).

- **`Gear Center` construction axis** — from any face whose `surfaceType` is
  `adsk.core.SurfaceTypes.CylinderSurfaceType`. Build it with
  `axisInput = component.constructionAxes.createInput()`, then
  `axisInput.setByCircularFace(cylindricalFace)`, then
  `component.constructionAxes.add(axisInput)` ([PB-CONSTRUCTION-AXES]). Name it `Gear Center`, set its
  `isLightBulbOn = False`, and store it on `ctx.centerAxis`.
- **`ctx.extrusionExtent`** — the far end-cap face the bore cut ends on. Among the faces whose
  `surfaceType` is `adsk.core.SurfaceTypes.PlaneSurfaceType`, take the one that is parallel to but
  not coplanar with the sketch plane. Test it with the plane-geometry API rather than a hand-rolled
  dot product: with `sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`, pick the face
  where `sketchPlane.isParallelToPlane(face.geometry)` and not
  `sketchPlane.isCoPlanarTo(face.geometry)`. The near cap is coplanar, so that rules it out, and
  the cylindrical face is not planar at all.

**What the proof checks.** `stepExtrudeBody` extrudes the disc and asserts its volume is
`pi * rootRadius^2 * Thickness` — an annulus, or a disc taken at the tip radius, fails there — that
its bounding box is the root radius either way and the target plane to Thickness in z, and that it
carries exactly one cylindrical face and two planar ones, which is what leaves both of the searches
above something to find.

**From:** `spec/spurgear/instructions.md` L297-300 L334-338 L570-579,
`.claude/skills/generate-gear/PLAYBOOK.md` L151-158 L672-681 L740-761 L787-790

## 10 `[GO]` Pattern the teeth and join them

Proof function `stepPatternTeeth` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(patternCases, stepPatternTeeth, assertPatternTeeth) -->

`patternTeeth(ctx)` circular-patterns `ctx.toothBody` around the `Gear Center` axis and joins the
result into `Gear Body`.

The input shape is [PB-CIRCULAR-PATTERN]'s. Put the seed body in an
`adsk.core.ObjectCollection.create()` and pass it with the axis:
`patternInput = component.features.circularPatternFeatures.createInput(bodies, ctx.centerAxis)`.
Pin all three settings explicitly rather than relying on Fusion's defaults:

- `patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)` — the `ToothNumber`
  parameter's numeric value.
- `patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')` — a full turn, set as
  a string expression.
- `patternInput.isSymmetric = False`.

Then `pattern = component.features.circularPatternFeatures.add(patternInput)`.

Feed the pattern's `bodies` collection to the combine as it stands: it already includes the
original tooth body, so do not re-add the seed ([PB-PATTERN-BODIES]). It is a `BRepBodies` and the
combine input rejects that, so copy each `pattern.bodies.item(i)` into a fresh
`adsk.core.ObjectCollection.create()` first. Then one Combine-Join:
`combineInput = component.features.combineFeatures.createInput(ctx.gearBody, toolBodies)`,
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`,
`component.features.combineFeatures.add(combineInput)`.

**What the proof checks, and what it does not.** `stepPatternTeeth` builds the seed tooth and then
walks it round the full turn one step of `360/quantity` at a time, `quantity` steps in all,
asserting at each step that the moved body's volume agrees with the seed's and that its centroid is
the seed's centroid turned by exactly that much — so a pattern spread over half a turn, or one that
counted the seed twice, fails — and that the last step lands back on the seed, which is the closure
a full turn with `isSymmetric = False` has to have. It also checks that one tooth's angular
half-width stays inside half the angular pitch, without which the teeth would run into each other
whatever the pattern did.

The **Join is not built**. Its two operands share both cap planes, because both extrudes run from
the target plane by the same Thickness, and they touch along the root arc without interpenetrating;
the solid engine refuses to classify that contact, and sinking the tooth inside the root circle
trades the second refusal for the first. The proof file records this next to the step, along with
the two substitutes that were measured and rejected for the fillet of step 11.

**From:** `spec/spurgear/instructions.md` L335-336 L344-371 L581-585,
`.claude/skills/generate-gear/PLAYBOOK.md` L693-703

## 11 `[PROSE]` Root fillets

`createFillets(ctx)` runs only when the `FilletRadius` parameter's numeric `.value` is above zero.
It rounds the corner where the root valley floor meets each tooth flank — the sharp inside corner
running the full thickness of the gear, parallel to its main axis, where bending stress
concentrates. It is not the front or back rim, which is a cosmetic rounding the user does not want
here.

Two things make the edge selection fiddly.

- After the pattern and combine, the root cylinder is usually split into one patch per valley
  rather than one continuous surface. Collect **every** cylindrical face whose radius equals
  `RootCircleRadius`, not just the first found. Floating-point radii never compare exactly equal,
  so the test is `abs(face.geometry.radius - rootRadius) <= 0.0001`, in centimetres — the same
  default `find_circle_by_radius` uses, so the two ways of finding a circle in this codebase agree.
- On each such face keep the **axial straight edges**, the two valley-floor-to-tooth-flank corners
  per valley patch, and drop the circular edges that wrap the circumference at the front and back
  end caps. Filter first to edges whose `edge.geometry.curveType` is
  `adsk.core.Curve3DTypes.Line3DCurveType`, take each line's direction from its geometry endpoints
  with `edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)`, `direction.normalize()`, and
  keep it when `abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01`, where `axisNormal` is the
  target plane's normal from `get_normal(self.plane)`. Use exactly that tolerance: a tighter test
  such as `> 0.999` drops valid axial edges that tessellation left slightly off, and the root
  fillets come out missing. The radius match uses the same `0.0001` cm default
  `find_circle_by_radius` carries, so the two ways of finding a circle in this codebase agree.

Do **not** read the direction through `edge.evaluator.getTangent(0)`; parameter `0` is not
guaranteed to lie inside the edge's parameter range and Fusion raises
`RuntimeError: invalid argument parameter`.

Apply the fillet with `filletInput = component.features.filletFeatures.createInput()` and
`filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), False)`
— the edge set goes on the input **itself**; `filletInput.edgeSetInputs` is the chamfer-side shape
and does not exist here ([PB-FILLET-CHAMFER]). `isTangentChain` must be `False`: the collected edges are exactly the
axial root corners, and tangent-chaining would let Fusion pull in tangent-adjacent edges and round
more than the intended corner. Then `component.features.filletFeatures.add(filletInput)`.

If the edge collection ends up **empty**, return without creating the feature — silently, no error.
An empty edge set must not reach the add call; zero is a legitimate outcome here, which is the
graceful-skip branch [PB-EMPTY-RESULT] asks a zero-able collection to declare.

**Why this step is `[PROSE]`.** The corner it rounds exists only on the joined and patterned body,
which step 10 cannot build here. Two substitutes were measured against the pinned engine and
neither survives: cutting one valley out of a tip-radius blank does produce the corner, but leaves
a faceted body and the engine fillets a straight prism only; extruding one tooth pitch of the
gear's cross-section gives a straight prism whose concave axial edges the selector finds correctly
— both root corners and nothing else — but the fillet then reports the two walls as meeting
smoothly and refuses the corner. Both findings are written down in `proof/spurgear/solids_test.go`
beside the pattern step, which is the nearest thing the proof does build, and the arithmetic the
step's own guard turns on is checked there: `FilletRadius` is 0.45 of the valley arc, under the
half-arc at which fillets from adjacent flanks would meet, and the valley arc itself goes negative
at a high tooth count with a large pressure angle, which is what the above-zero guard is for.

`getTangent` and `edgeSetInputs` are named here only to forbid them.

<!-- check-step-calls: ignore getTangent -->

**From:** `spec/spurgear/instructions.md` L86-88 L126-131 L392-396 L587-596,
`.claude/skills/generate-gear/PLAYBOOK.md` L151-158 L569-575 L672-681 L740-761

## 12a `[GO]` Bore Profile sketch

Proof function `stepBoreProfile` in `proof/spurgear/sketches_test.go`.

<!-- proof-run: proofkit.Run(boreCases, stepBoreProfile) -->

`buildBore(ctx)` runs unconditionally from `generate`, after `buildMainGearBody`, so it must
itself return early in **two** cases: when SketchOnly is set, and when the `BoreDiameter`
parameter's value is at or below zero. The SketchOnly guard is essential — in that mode
`buildMainGearBody` short-circuits before `buildBody`, so `ctx.gearBody` and `ctx.extrusionExtent`
are never set and the cut would dereference `None`. Do not rely on the bore diameter being zero in
sketch-only mode; the user may have set both.

Otherwise create a separate sketch named `Bore Profile` on the target plane with
`self.createSketchObject('Bore Profile', self.plane)`, keep it on `self.boreSketch`, and draw the
bore circle by instantiating the tooth generator on that sketch —
`toothGen = SpurGearInvoluteToothDesignGenerator(boreSketch, self)` — and calling
`toothGen.drawBore(ctx.anchorPoint, boreDiameter)` with the diameter in internal centimetres.

`drawBore(anchorPoint, diameter)` projects the anchor into this sketch with
`sketch.project(anchorPoint)`, draws a construction-less circle of that diameter centred on the
projection with `sketch.sketchCurves.sketchCircles.addByCenterRadius(projectedAnchor, diameter / 2)`
and a driving `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`, and returns the
circle.

The tooth generator's constructor always adds its local-origin (0, 0, 0) `SketchPoint`, so this
sketch carries one stray unused point ([SPUR-F-LOCAL-ORIGIN]). That is faithful behaviour — do not
suppress it — but **ground it on the projected anchor**, exactly as step 3 grounds the Gear Profile
sketch, so this sketch tracks the user's anchor through the same projection chain
([SPUR-F-ANCHOR-CHAIN]):
`sketch.geometricConstraints.addCoincident(toothGen.anchorPoint, projectedAnchor)`, using the same
projection `drawBore` made. Do **not** ground it on `boreSketch.originPoint`, which pins the point
to the plane rather than to the gear and has been observed to fail the solver ([PB-CIRCLE-CENTER]);
without any grounding the point is free in two directions and the sketch never reaches
`isFullyConstrained`, which [PB-FULL-CONSTRAINT] does not allow. The circle itself is added through
`sketch.sketchCurves`, never off the sketch directly ([PB-SKETCHCURVES]).

**What the proof checks.** `stepBoreProfile` rebuilds this sketch in the sketch engine and holds it
to the same full verdict as step 3, across bore diameters from 0.4 mm to 40 mm at the gear sizes
that bound them, and both on and off the sketch origin — a bore at the origin cannot tell a point
grounded on the projected anchor from one grounded on the sketch's own origin point, which is the
substitution that fails. It then takes the sketch's single profile and asserts its area is
`pi * (D/2)^2` and its centre is the anchor.

**From:** `spec/spurgear/instructions.md` L320-322 L430-435 L598-602, `spec/spurgear/fusion.md` L26-31,
`.claude/skills/generate-gear/PLAYBOOK.md` L441-457 L509-516

## 12b `[GO]` Cut the bore

Proof function `stepBoreCut` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(boreSolidCases, stepBoreCut, assertBoreCut) -->

Extrude-cut the Bore Profile circle from the target plane to `ctx.extrusionExtent`, the gear body's
far end-cap face, affecting only `ctx.gearBody`. Ending the cut on that face is what guarantees the
hole goes all the way through whatever Thickness is:

- `cutInput = component.features.extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)`
- `cutExtent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)`
- `cutInput.setOneSideExtent(cutExtent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`
- `cutInput.participantBodies = [ctx.gearBody]`
- `component.features.extrudeFeatures.add(cutInput)`

The to-entity extent is what makes the hole pierce the body whatever Thickness is; a symmetric
over-length extent is the other shape with that property ([PB-THROUGH-CUT]), and it is what the
proof substitutes below.

**What the proof checks.** `stepBoreCut` cuts the bore through the gear body disc and asserts that
the volume removed is the full `pi * (D/2)^2 * Thickness` — a blind hole would fail — that the
body's bounding box is unchanged, and that it now carries two cylindrical faces, the rim and the
bore wall, and two planar caps. Two substitutions are recorded in the proof file: the cut is made
against the disc of step 9 rather than the finished gear, since step 10's join cannot be built
there and the bore is well inside the root circle where the teeth are not, and the tool is swept
symmetrically past both caps instead of stopping on the far face, because a tool that stopped
exactly on it would share that cap plane with its target and the engine refuses to classify that.
The material removed is the same either way, which is the point of ending on the far face.

**From:** `spec/spurgear/instructions.md` L338 L598-602, `.claude/skills/generate-gear/PLAYBOOK.md` L729-732

## 13 `[GO]` Chamfer the completed gear

Proof function `stepChamferTeeth` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(chamferCases, stepChamferTeeth, assertChamferTeeth) -->

`generate` calls `chamferTeeth(ctx)` after the optional bore, so the chamfer sees the teeth already
patterned and joined, the root fillets already applied, and the bore already cut. It returns in
SketchOnly mode and when the `ChamferTooth` parameter's value is zero. Helical and herringbone
inherit this selection unchanged.

Walk every planar face of `ctx.gearBody` parallel to the Gear Profile sketch plane, and add each
edge of those faces once, deduplicated by `edge.tempId`. That set includes the tooth flanks, the
tooth tops and the root-radius arcs. Exclude only an edge whose `edge.geometry.curveType` is
`adsk.core.Curve3DTypes.Circle3DCurveType` and whose radius is the positive Bore Diameter divided
by two, within `0.001` cm, so a bore never receives a chamfer. Raise when no end-cap face is found
or no chamfer edge remains ([PB-EMPTY-RESULT]); do not create a partial chamfer.

Apply the set with `chamferInput = component.features.chamferFeatures.createInput2()` and
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(chamferDistance), False)`,
then `component.features.chamferFeatures.add(chamferInput)`. Note the asymmetry with the fillet of
step 11: a chamfer's edge set goes on the input's `chamferEdgeSets` collection, a fillet's on the
input itself, and the two must not be mirrored onto each other ([PB-FILLET-CHAMFER]). The curve
type constants end in `...CurveType` and are compared against `adsk.core.Curve3DTypes`
([PB-PROFILE-MATCH]).

**What the proof checks.** `stepChamferTeeth` chamfers the bored gear body's end-cap edges. It
first resolves the unfiltered selection and asserts it finds four circular cap edges — two rim and
two bore — then resolves the step's own selection and asserts it keeps exactly two, so the bore
edges are demonstrably excluded rather than merely absent; it then applies the equal-distance
chamfer and checks the volume against the cone frustum an equal-distance chamfer of that distance
takes off each end, and that the body still reaches both cap planes at the bore, which a chamfered
bore edge would have pulled back from. The substitution recorded in the proof file is the tooth
part of the edge set: the tooth flanks, tops and root arcs are free-form or derived from a
free-form neighbour, and the engine's corner rewrite does not support a free-form boundary segment,
so what is proven is the rest of the rule — equal distance, end-cap edges, bore excluded — with the
exclusion tested by circumference, the same separation the spec makes by radius.

**From:** `spec/spurgear/instructions.md` L58 L344-371 L604-619,
`.claude/skills/generate-gear/PLAYBOOK.md` L569-575 L672-681

## 14 `[PROSE]` End-of-build cleanup

`cleanup(ctx)` is the very last action of `generate`, after `chamferTeeth`, and it is called
**unconditionally** in both modes. Its placement after `buildBore` matters, because `buildBore`
re-projects `ctx.anchorPoint` out of the Tools sketch and projection fails once that sketch is
hidden — so the Tools sketch has to stay visible through the bore and the chamfer. Do not move the
call up into `buildMainGearBody`, and do not guard the call itself; the mode split lives inside.

Hide construction geometry and sketches with the right property, never crossed
([PB-HIDE-AFTER-USE]): `isLightBulbOn = False` for a construction plane or axis,
`isVisible = False` for a sketch. The spur recipe — which entities, and the per-mode split — is
[SPUR-F-CLEANUP].

- **Always, in both modes** — including sketch-only, so no stray plane floats — turn off the light
  bulb on every construction plane and axis this generator created: `ctx.extrusionEndPlane`, the
  `Gear Center` axis `ctx.centerAxis`, and the normalized target plane if step 1 created one.
- **Only on the full-build path**, set `isVisible = False` on the Tools, Gear Profile and Bore
  Profile sketches. Sketch-only mode leaves Tools and Gear Profile visible for inspection, which is
  the whole point of that mode.

Guard each entity individually and hide it only if it was actually created: the `Gear Center` axis
and the Bore Profile sketch do not exist in sketch-only mode.

**From:** `spec/spurgear/instructions.md` L313-315 L344-380 L560-562, `spec/spurgear/fusion.md` L221-231,
`.claude/skills/generate-gear/PLAYBOOK.md` L659-671
