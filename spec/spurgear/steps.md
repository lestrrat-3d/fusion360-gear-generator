# Spur Gear — compiled step list

The proof for these steps is `proof/spurgear/sketches_test.go`, `proof/spurgear/solids_test.go` and
the generated registration file `proof/spurgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/spurgear/instructions.md` | `8cb886a7827d6745fde7c876475066918c328283` |
| `spec/spurgear/fusion.md` | `7cd4e5b0fa38dcd39cbd1b5bad1cf8489e2bc2ae` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `1b3078d6767d6a3f56c228e1e934c82ccfbf53fe` |

## S1 `[PROSE]` Dialog inputs — `SpurGearCommandInputsConfigurator.configure`

`SpurGearCommandInputsConfigurator` is a plain class (no base) with one `@classmethod` named
`configure(cls, cmd)` that adds the dialog inputs to `cmd.commandInputs`. It is one of the four
public class names helical and herringbone bind to, so the name is fixed.

<!-- check-step-calls: ignore configure -->
`configure` is a method the module DEFINES for the command framework to call — `entry.py` hands the
configurator to `GearCommand` — so the module itself never calls it.

**Add the inputs in exactly this order.** The order is the dialog's display order and is not the
order `processInputs` reads them in (S2 reads the three selections first); a configurator that puts
the selections last because they are read first has the rule backwards. Do not group by input type.

| # | Dialog label | input id | registered user-parameter |
|---|---|---|---|
| 1 | Target Plane | `plane` | — |
| 2 | Anchor Point | `anchorPoint` | — |
| 3 | Module | `module` | `Module` |
| 4 | Tooth Number | `toothNumber` | `ToothNumber` |
| 5 | Pressure Angle | `pressureAngle` | `PressureAngle` |
| 6 | Bore Diameter | `boreDiameter` | `BoreDiameter` |
| 7 | Thickness | `thickness` | `Thickness` |
| 8 | Apply chamfer to teeth | `chamferTooth` | `ChamferTooth` |
| 9 | Generate sketches, but do not build body | `sketchOnly` | `SketchOnly` |
| 10 | Parent Component | `parentComponent` | — |

Target Plane and Anchor Point are the **first two** inputs, and Parent Component is **last** — its
default, the root component, is right for most uses. Fusion auto-focuses the first
`SelectionCommandInput` and ignores a later `hasFocus`, so this add-order is what decides the
dialog opens on Target Plane (`[PB-AUTOFOCUS-FIRST]`).

**The three selection inputs.** Each is added with
`cmd.commandInputs.addSelectionInput(id, name, commandPrompt)`, then given its filters with
`selectionInput.addSelectionFilter(...)`, then limited with
`selectionInput.setSelectionLimits(1, 1)` — exactly one selection each.

The third argument is the **command prompt**, which Fusion shows beside the cursor while the user
picks, and it is a different string from the label. All three prompts are reproduced surface, so
write them exactly as they stand here; filling them with the label text is the failure this table
exists to stop:

| input id | `name` (the dialog label) | `commandPrompt` |
|---|---|---|
| `plane` | `Target Plane` | `Select the plane to build the gear on` |
| `anchorPoint` | `Anchor Point` | `Select the point the gear is centered on` |
| `parentComponent` | `Parent Component` | `Select the component to build the gear in` |

Filters are written as the named constants on `adsk.core.SelectionCommandInput`, never as quoted
literals (`[PB-SELECTION-FILTER-ENUM]`; `[PB-SELECTION-DECL]` makes the filter set and the limits
contract surface the spec declares per input):

- Target Plane: `adsk.core.SelectionCommandInput.ConstructionPlanes` and
  `adsk.core.SelectionCommandInput.PlanarFaces`.
- Anchor Point: `adsk.core.SelectionCommandInput.ConstructionPoints` and
  `adsk.core.SelectionCommandInput.SketchPoints`.
- Parent Component: `adsk.core.SelectionCommandInput.Occurrences` and
  `adsk.core.SelectionCommandInput.RootComponents`. Pre-select the root component with
  `parentInput.addSelection(get_design().rootComponent)`.

**The value inputs.** `addValueInput` defaults are in Fusion's INTERNAL units — cm for length,
radians for angle — whatever the display unit string says (`[PB-DIALOG-DEFAULT-UNITS]`):

- Module: `cmd.commandInputs.addValueInput(INPUT_ID_MODULE, 'Module', '', adsk.core.ValueInput.createByReal(1))`
  — unit string `''`, default `1`.
- Tooth Number: `addValueInput(INPUT_ID_TOOTH_NUMBER, 'Tooth Number', '', adsk.core.ValueInput.createByReal(17))`
  — unit string `''`, default `17`.
- Pressure Angle: `addValueInput(INPUT_ID_PRESSURE_ANGLE, 'Pressure Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(20)))`
  — display unit `'deg'`, default in radians.
- Bore Diameter: `cmd.commandInputs.addStringValueInput(INPUT_ID_BORE_DIAMETER, 'Bore Diameter', '0 mm')`
  — a **string** value input so it accepts expressions; default string `'0 mm'`.
- Thickness: `addValueInput(INPUT_ID_THICKNESS, 'Thickness', 'mm', adsk.core.ValueInput.createByReal(to_cm(10)))`
  — display unit `'mm'`, default 10 mm expressed in cm.
- Apply chamfer to teeth: `addValueInput(INPUT_ID_CHAMFER_TOOTH, 'Apply chamfer to teeth', 'mm', adsk.core.ValueInput.createByReal(0))`
  — display unit `'mm'`, default `0` (no chamfer).
- The sketch-only checkbox is added with
  `cmd.commandInputs.addBoolValueInput(INPUT_ID_SKETCH_ONLY, label, True)`, where the trailing
  `True` makes it a check box and the unpassed `initialValue` leaves its default `false`.

The `label` in that call is the row-9 string, written out here so it need not be fetched from
anywhere: `Generate sketches, but do not build body`.

`configure` is a `@classmethod` (`[SPUR-SUBCLASS-INPUT]`), and a subclass gear adds its own input
by subclassing the configurator and appending after a `super()` call to it. Because Parent Component
is already last, a subclass's extra input necessarily lands below it. Keep the seam.

Every id and parameter name above is a module-level constant of `spurgear.py`
(`[SPUR-EXPORTED-CONSTANTS]`), imported by name elsewhere; renaming one is a breaking change. The full roster, with
its value:

- `INPUT_ID_PARENT = 'parentComponent'`, `INPUT_ID_PLANE = 'plane'`,
  `INPUT_ID_ANCHOR_POINT = 'anchorPoint'`, `INPUT_ID_MODULE = 'module'`,
  `INPUT_ID_TOOTH_NUMBER = 'toothNumber'`, `INPUT_ID_PRESSURE_ANGLE = 'pressureAngle'`,
  `INPUT_ID_BORE_DIAMETER = 'boreDiameter'`, `INPUT_ID_THICKNESS = 'thickness'`,
  `INPUT_ID_CHAMFER_TOOTH = 'chamferTooth'`, `INPUT_ID_SKETCH_ONLY = 'sketchOnly'`.
- `PARAM_MODULE = 'Module'`, `PARAM_TOOTH_NUMBER = 'ToothNumber'`,
  `PARAM_PRESSURE_ANGLE = 'PressureAngle'`, `PARAM_BORE_DIAMETER = 'BoreDiameter'`,
  `PARAM_THICKNESS = 'Thickness'`, `PARAM_CHAMFER_TOOTH = 'ChamferTooth'`,
  `PARAM_SKETCH_ONLY = 'SketchOnly'`, `PARAM_PITCH_DIAMETER = 'PitchCircleDiameter'`,
  `PARAM_PITCH_RADIUS = 'PitchCircleRadius'`, `PARAM_BASE_DIAMETER = 'BaseCircleDiameter'`,
  `PARAM_BASE_RADIUS = 'BaseCircleRadius'`, `PARAM_ROOT_DIAMETER = 'RootCircleDiameter'`,
  `PARAM_ROOT_RADIUS = 'RootCircleRadius'`, `PARAM_TIP_DIAMETER = 'TipCircleDiameter'`,
  `PARAM_TIP_RADIUS = 'TipCircleRadius'`, `PARAM_INVOLUTE_STEPS = 'InvoluteSteps'`,
  `PARAM_TOOTH_SPACE_ANGLE = 'ToothSpaceAngleAtRoot'`,
  `PARAM_TOOTH_SPACE_ARC = 'ToothSpaceArcAtRoot'`,
  `PARAM_FILLET_CLEARANCE = 'FilletClearance'`, `PARAM_FILLET_RADIUS = 'FilletRadius'`.

`helicalgear.py` and `herringbonegear.py` each import `PARAM_MODULE`, `PARAM_TOOTH_NUMBER` and
`PARAM_THICKNESS` from `.spurgear`.

Two of those values are not written out anywhere in the prose spec. It spells the parameter
`ToothSpaceArcAtRoot` inside the `FilletRadius` expression, and it names the other one only as the
prose phrase "Tooth Space Angle At Root" while saying the derived parameters keep exactly those
names. `ToothSpaceAngleAtRoot` above is the reading that makes the pair consistent; it is an
inference, and a Fusion session or a spec edit is what settles it.

This step draws no geometry, so no proof function realises it.

**From:** `spec/spurgear/instructions.md` L13-34, L35-89, L90-253; `.claude/skills/generate-gear/PLAYBOOK.md` L42-74, L128-143, L346-349.

## S2 `[PROSE]` Read the inputs and register the user parameters

`SpurGearGenerator(Generator)` is the orchestrator and subclasses `base.Generator`;
`SpurGearGenerationContext(GenerationContext)` is the data carrier. `GenerationContext` and
`Generator` are imported from `.base`. `prefixBase` returns `'SpurGear'`.

**Two of this step's methods carry a return annotation.** `helicalgear` and `herringbonegear`
override both and annotate their own returns. Python does not care, but a type checker reads an
unannotated parent as returning the literal it happens to return — `prefixBase` infers as
`Literal['SpurGear']` — and the subclass's wider `str` is then reported as an incompatible
override. A regeneration that dropped them made `helicalgear` draw two
`reportIncompatibleMethodOverride` complaints no shipped gear had produced before. They are
contract surface for the subclasses, not implementation taste:

| class | method | return |
|---|---|---|
| `SpurGearGenerator` | `prefixBase` | `-> str` |
| `SpurGearGenerator` | `filletHelixFactorExpression` | `-> str` |

**Order is load-bearing.** As soon as anything creates the gear occurrence — `getOccurrence`
directly, or `parameterName()` / `addParameter()` transitively — Fusion's active component context
shifts, and a `SelectionCommandInput` holding an entity in another component can drop its selection
(`[PB-SELECTION-STASH]`). Numeric and boolean inputs are unaffected. So `processInputs(inputs)`:

1. `get_selection(inputs, INPUT_ID_PARENT)` — resolve `Occurrence.component` versus `Component`
   into `self.parentComponent`; raise on the wrong count or type.
2. `get_selection(inputs, INPUT_ID_PLANE)` into `self.plane`, and
   `get_selection(inputs, INPUT_ID_ANCHOR_POINT)` into `self.anchorPoint`. Nothing touching the
   design has run yet.
3. Register the input-sourced parameters. Read each input with the helper matching the type it was
   declared with (`[PB-INPUT-READ]`): `get_value(inputs, id, units)` for every `addValueInput` and
   `addStringValueInput`, and `get_boolean(inputs, INPUT_ID_SKETCH_ONLY)` for the checkbox.
   `get_value` already returns a `ValueInput` ready to hand to `addParameter`
   (`[PB-GET-VALUE-CONTRACT]`), so pass it straight through. `SketchOnly` is persisted as a
   real-valued parameter, 1 for true and 0 for false, since the framework reads booleans back with
   `getParameterAsBoolean(PARAM_SKETCH_ONLY)`.
   Units per parameter: `Module` **`''`** (unitless, NOT `'mm'` — `generateName` must render `M=1`
   with no unit suffix, and the `mm` expressions below read the unitless factor),
   `ToothNumber` `''`, `PressureAngle` `'rad'`, `BoreDiameter` `'mm'`, `Thickness` `'mm'`,
   `ChamferTooth` `'mm'`, `SketchOnly` `''`.
4. `self.addExtraPrimaryParameters(inputs)` — an overridable hook (`[SPUR-EXTRA-PARAMS]`) that is
   a no-op on the spur base. It must exist and be called here, **between** the input-sourced
   parameters and the derived ones, so a subclass can register its own primary parameter before
   anything derived references it.
5. Register the derived parameters as live Fusion expression strings via
   `adsk.core.ValueInput.createByString(...)`, using `self.parameterName(name)` to build each
   reference. In this order and with these exact formulas, where `<p>` is the parameter prefix:
   - `PitchCircleDiameter` (`'mm'`) = `<p>_Module * <p>_ToothNumber`
   - `PitchCircleRadius` (`'mm'`) = `<p>_PitchCircleDiameter / 2`
   - `BaseCircleDiameter` (`'mm'`) = `<p>_PitchCircleDiameter * cos(<p>_PressureAngle)`
   - `BaseCircleRadius` (`'mm'`) = `<p>_BaseCircleDiameter / 2`
   - `RootCircleDiameter` (`'mm'`) = `<p>_PitchCircleDiameter - 2.5 * <p>_Module` (dedendum
     1.25 · Module)
   - `RootCircleRadius` (`'mm'`) = `<p>_RootCircleDiameter / 2`
   - `TipCircleDiameter` (`'mm'`) = `<p>_PitchCircleDiameter + 2 * <p>_Module` (addendum
     1.0 · Module)
   - `TipCircleRadius` (`'mm'`) = `<p>_TipCircleDiameter / 2`
   - `InvoluteSteps` (`''`) = `15`
   - `ToothSpaceAngleAtRoot` — **pre-computed in Python** and registered with
     `adsk.core.ValueInput.createByReal(...)`, because Fusion's expression engine refuses to
     subtract a radian-valued Pressure Angle from the unitless output of `tan()`. The value is
     `math.pi / toothNumber - 2 * (math.tan(pressureAngle) - pressureAngle)`. Register it
     **unitless (`''`), not `'rad'`** — the next parameter multiplies it by a length, and Fusion
     accepts that product as `mm` only when this factor is unitless; `'rad'` makes the product
     `mm·rad` and Fusion rejects it with `RuntimeError: Invalid expression`.
   - `ToothSpaceArcAtRoot` (`'mm'`) = `<p>_RootCircleRadius * <p>_ToothSpaceAngleAtRoot`
   - `FilletClearance` (`''`) = `0.9`
   - `FilletRadius` (`'mm'`) =
     `(<p>_ToothSpaceArcAtRoot / 2) * <p>_FilletClearance * <factor>`, where `<factor>` is the
     string returned by `self.filletHelixFactorExpression()` — an overridable hook returning `'1'`
     on the spur base. That hook is read **only here**; `createFillets` reads the resulting
     `FilletRadius` parameter's numeric `.value` and never calls the hook.

**Every parameter's comment.** The registration call is
`addParameter(name, ValueInput, units, comment)`, and the fourth argument is the string Fusion shows
in the parameter table's Comment column, beside all twenty `<prefix>_…` parameters. It is what the
user reads there, so it is reproduced surface and each one is written out here with its unit string
beside it, the two being read together:

| constant | units | `comment` |
|---|---|---|
| `PARAM_MODULE` | `''` | `Module of the gear` |
| `PARAM_TOOTH_NUMBER` | `''` | `Number of teeth` |
| `PARAM_PRESSURE_ANGLE` | `'rad'` | `Pressure angle` |
| `PARAM_BORE_DIAMETER` | `'mm'` | `Bore diameter` |
| `PARAM_THICKNESS` | `'mm'` | `Thickness of the gear` |
| `PARAM_CHAMFER_TOOTH` | `'mm'` | `Chamfer distance applied to the teeth` |
| `PARAM_SKETCH_ONLY` | `''` | `Generate sketches only` |
| `PARAM_PITCH_DIAMETER` | `'mm'` | `Pitch circle diameter` |
| `PARAM_PITCH_RADIUS` | `'mm'` | `Pitch circle radius` |
| `PARAM_BASE_DIAMETER` | `'mm'` | `Base circle diameter` |
| `PARAM_BASE_RADIUS` | `'mm'` | `Base circle radius` |
| `PARAM_ROOT_DIAMETER` | `'mm'` | `Root circle diameter` |
| `PARAM_ROOT_RADIUS` | `'mm'` | `Root circle radius` |
| `PARAM_TIP_DIAMETER` | `'mm'` | `Tip circle diameter` |
| `PARAM_TIP_RADIUS` | `'mm'` | `Tip circle radius` |
| `PARAM_INVOLUTE_STEPS` | `''` | `Number of points sampled along each involute flank` |
| `PARAM_TOOTH_SPACE_ANGLE` | `''` | `Angular width of the tooth space at the root circle` |
| `PARAM_TOOTH_SPACE_ARC` | `'mm'` | `Arc length of the tooth space at the root circle` |
| `PARAM_FILLET_CLEARANCE` | `''` | `Clearance factor applied to the root fillet radius` |
| `PARAM_FILLET_RADIUS` | `'mm'` | `Radius of the root fillets` |

The units column above and the ones given per parameter in items 3 and 5 are the same values, said
twice; a comment filled with the parameter's own name is the failure this table exists to stop.

Every sketch dimension and feature input below is set from the *current numeric value* of its
source parameter at generation time, never as a live expression (`[SPUR-F-SNAPSHOT]`,
`[PB-NUMERIC-SNAPSHOT]`). Editing a `<p>_…` parameter afterwards does not change an existing gear; the user
re-runs the dialog.

This step registers parameters and draws no geometry, so no proof function realises it.

**From:** `spec/spurgear/instructions.md` L35-89, L108-124, L205-225, L327-343, L392-410, L481-491; `spec/spurgear/fusion.md` L231-238; `.claude/skills/generate-gear/PLAYBOOK.md` L75-102, L103-143, L196-228.

## S3 `[PROSE]` Create the gear occurrence and name the component

`generate` runs, in this exact order:

```
generate(inputs)
  -> processInputs(inputs)                      # S2
  -> component = self.getComponent(); component.name = self.generateName()
  -> normalize self.plane to a ConstructionPlane        # S4
  -> ctx = self.newContext()
  -> self.prepareTools(ctx)                     # S5, S6
  -> self.buildMainGearBody(ctx)
        -> self.buildSketches(ctx)              # S7
        -> if SketchOnly: show the Gear Profile sketch and stop   # S8
           else:
             -> self.buildTooth(ctx)            # S9
             -> self.buildBody(ctx)             # S10
             -> self.patternTeeth(ctx)          # S11, S12
             -> self.createFillets(ctx)         # S13
  -> self.buildBore(ctx)                        # S14, S15
  -> self.chamferTeeth(ctx)                     # S16
  -> self.cleanup(ctx)                          # S17
```

These method names and the boundaries between them are public API: helical and herringbone override
specific ones and call `super()` at specific points, so a reconstruction that merges or reorders
them breaks those gears even if it draws an identical spur gear. Only the contents of each method
may vary.

A spur gear is one cylindrical body with straight teeth cut along the axis, and one invocation
produces exactly one gear — there is no pairing. The new gear is a child occurrence of the
user-selected Parent Component.

`generateName()` returns
`'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression, thickness.expression)`
— the `Module`, `ToothNumber` and `Thickness` parameters' **`.expression`** strings, not their
`.value`, so units show through, e.g. `Spur Gear (M=1, Tooth=17, Thickness=10 mm)`.

`SpurGearGenerationContext.__init__` declares these fields, each `cast(None)`-initialised except
`toothProfileIsEmbedded`, which starts `False`. Subclasses read them by name:

- `ctx.plane` — the `ConstructionPlane` every sketch is built on (normalised in S4). The generator
  also keeps `self.plane`; subclasses read `self.plane` directly, so keep both available.
- `ctx.anchorPoint` — the `SketchPoint` that is the Tools-sketch projection of the user's anchor.
- `ctx.extrusionEndPlane` — the offset construction plane both extrudes end on.
- `ctx.gearProfileSketch` — the sketch holding the tooth profile and the four gear circles.
- `ctx.toothBody` — the single extruded tooth, before the pattern.
- `ctx.gearBody` — the cylindrical body the teeth are joined into.
- `ctx.centerAxis` — the `Gear Center` construction axis.
- `ctx.extrusionExtent` — the far end-cap face, the bore cut's to-entity.
- `ctx.toothProfileIsEmbedded` — `True` when the base circle sits inside the root circle.

**Two of this step's methods carry a return annotation.** `helicalgear` and `herringbonegear`
override both and annotate their own returns, and a type checker reads an unannotated parent as
returning the literal it happens to return, so the subclass's wider annotation is then reported as
an incompatible override. They are contract surface for the subclasses, not implementation taste:

| class | method | return |
|---|---|---|
| `SpurGearGenerator` | `generateName` | `-> str` |
| `SpurGearGenerator` | `newContext` | `-> SpurGearGenerationContext` |

`SpurGearGenerator.__init__` must additionally pre-initialise `self._lastToothEmbedded = False`,
`self.toolsSketch = None` and `self.boreSketch = None` (`[SPUR-F-FLANK-ROOT]`).

Spur imports only the framework: `Generator, GenerationContext, get_value, get_boolean,
get_selection` from `.base`; `get_normal, find_profile_by_curve_counts` from `.utilities`; `to_cm,
get_design` from `.misc`. It depends on no other gear.

This step creates a component and draws no geometry, so no proof function realises it.

<!-- check-step-calls: ignore generate -->
`generate` is a method the module DEFINES for the command framework to call —
`commands/_gear_command.py` constructs the generator class and calls it — so the module
itself never calls it. Every other method in the graph above is called by the module, from the
method the graph shows above it.

**From:** `spec/spurgear/instructions.md` L9-12, L108-124, L327-343, L344-410, L454-480; `.claude/skills/generate-gear/PLAYBOOK.md` L75-102, L244-255, L256-281.

## S4 `[PROSE]` Normalize the Target Plane

If the user's selection is already a `ConstructionPlane`, use it. Otherwise — a planar face, say —
build a coplanar construction plane and use that instead, so downstream profile detection is not
confused by the selected face's own profile:

`planeInput = self.getComponent().constructionPlanes.createInput()`, then
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))`, then
`self.plane = self.getComponent().constructionPlanes.add(planeInput)`.

The offset argument is a **`ValueInput`, not a bare number**: `setByOffset(plane, 0)` is a runtime
`TypeError` (`[PB-CONSTRUCTION-PLANES]` gives the signature). Store the result on both `self.plane`
and `ctx.plane`, and keep a handle so S17 can switch its light bulb off if one was created here.

This step creates a construction plane and no measurable geometry, so no proof function realises it.

**From:** `spec/spurgear/instructions.md` L494-496; `.claude/skills/generate-gear/PLAYBOOK.md` L742-753.

## S5 `[GO]` Tools sketch and the anchor projection

<!-- proof-run: proofkit.Run(toolsCases, stepToolsSketch) -->

Create a sketch named `Tools` on the target plane with
`self.createSketchObject('Tools', plane=self.plane)`. Project the user's Anchor Point into it with
`toolsSketch.project(self.anchorPoint)` and keep the resulting `SketchPoint` as `ctx.anchorPoint`.

The sketch draws no geometry of its own; it exists to own this one reference.
That projection is the canonical handle (`[SPUR-F-ANCHOR-CHAIN]`): a sketch cannot reference a
`SketchPoint` owned by another sketch, so every later sketch projects *this* point in again,
forming a chain back to the user's original anchor entity, and the whole gear moves if the anchor
moves. Note `[PB-PROJECT-NOT-FIXED]`: the projection is associative, not fixed, so it carries free
degrees of freedom until something constrains it — which is what S7's anchoring does.

Leave the Tools sketch **visible** for the whole build. S14 re-projects `ctx.anchorPoint` out of it,
and projection fails once the sketch is hidden, so it is hidden only in S17
(`[PB-HIDE-AFTER-USE]`). Keep the sketch on `self.toolsSketch`.

### What the proof establishes

`stepToolsSketch` builds this sketch and gates it: it holds the one projected anchor and **no drawn
entity at all**, which is the whole shape of the step. Beside it the proof checks what that
projection is for, which is where the content of the anchoring rule actually sits — a later sketch's
local origin made coincident to the projection reaches DOF 0, and the same local origin left
unanchored keeps the two degrees of freedom it was born with. The anchor is swept both on the sketch
origin and off it, because nothing in the dialog requires the user to put it on the origin and every
later sketch is dragged onto wherever it is.

Two things the proof cannot reach are recorded in the proof file next to that check.
`[PB-PROJECT-NOT-FIXED]`'s free degrees of freedom are not reproduced: the sketch engine's reference
point is coordinate-locked, so the projection is modelled as already pinned and only its consequence
— that the local origin hanging off it still needs a constraint — is proved. And the chain of
`[SPUR-F-ANCHOR-CHAIN]` is not reproduced: the engine refuses a reference to another sketch's point
as a foreign handle, exactly as Fusion does, so what each sketch carries is its own local endpoint
of the chain.

**From:** `spec/spurgear/instructions.md` L498-500, L302-307, L373-381; `spec/spurgear/fusion.md` L17-32; `.claude/skills/generate-gear/PLAYBOOK.md` L455-469, L626-638.

## S6 `[PROSE]` Extrusion End Plane

Create an offset construction plane named `Extrusion End Plane` at distance `Thickness` from the
target plane:
`planeInput = self.getComponent().constructionPlanes.createInput()`,
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))` where
`thickness` is the `Thickness` parameter's numeric `.value` in cm, then
`ctx.extrusionEndPlane = self.getComponent().constructionPlanes.add(planeInput)` and name it
`Extrusion End Plane`.

Its only purpose is to be the `to-entity` target of the tooth extrude (S9) and the body extrude
(S10), so both end on the same well-defined face. Leave it visible while those run; S17 hides it
with `isLightBulbOn = False`, since `isVisible = False` does not hide a construction plane
(`[PB-HIDE-AFTER-USE]`).

This step creates a construction plane and no measurable geometry, so no proof function realises it.

**From:** `spec/spurgear/instructions.md` L502, L313-315; `.claude/skills/generate-gear/PLAYBOOK.md` L626-638, L742-753.

## S7 `[GO]` Gear Profile sketch — circles, involute tooth, anchoring

<!-- proof-run: proofkit.Run(profileCases, stepGearProfileSketch) -->

One sketch, one timeline entry. `buildSketches(ctx)` creates it —
`ctx.gearProfileSketch = self.createSketchObject('Gear Profile', plane=self.plane)` — then
constructs `SpurGearInvoluteToothDesignGenerator(ctx.gearProfileSketch, self)` and calls
`toothGen.draw(ctx.anchorPoint, angle=0)`. Everything below happens inside that one `draw` call, in
this order: `drawCircles()`, `drawTooth(angle)`, the anchoring, then the confirming angular
dimension's value.

Helical overrides `buildSketches`, calls `super().buildSketches(ctx)` and then draws a second,
twisted profile sketch with the same generator at `angle=helixAngle`, so this boundary and this
call shape are contract.

### The tooth generator's reproduced surface

Constructor `(sketch, parent, angle=0)`. It stores `self.toothAngle = angle`, and **that stored
value is not what `drawTooth` rotates by**: `drawTooth` must rotate by the `angle` argument that
flows in from `draw()` at call time. Helical constructs the generator with the default `angle=0`
and then calls `draw(ctx.anchorPoint, angle=helixAngle)`; a `drawTooth` reading `self.toothAngle`
would draw a flat tooth and the helical loft would have no twist.

The movable local origin is a field named `self.anchorPoint` (`[SPUR-F-LOCAL-ORIGIN]`) — a fresh
`SketchPoint` added at (0, 0, 0) in the constructor with
`sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`. It is **not** `sketch.originPoint`,
which is immutable and cannot be coincident-constrained to anything brought in from elsewhere. All
the geometry below is drawn relative to it, and S7's anchoring then slides the whole sketch onto the
user's anchor as a unit.

The methods `drawCircles`, `drawTooth`, `drawBore` and
`calculateInvolutePoint(baseRadius, intersectionRadius)` must all exist, as must the parameter
accessors `getParameter(name)` and `getParameterValue(name)`.

<!-- check-step-calls: ignore getParameterValue -->
`getParameterValue` is named as a member the module must DEFINE — it is part of the reproduced
surface a borrowing gear may read — not as a call this step requires; the drawing code reads its
parameters through `getParameter`.

**One of this step's methods carries a return annotation.** A subclass that overrides it and
annotates its own return is reported as an incompatible override whenever the parent is
unannotated, because a type checker reads the parent as returning the literal it happens to return.
It is contract surface for the subclasses, not implementation taste:

| class | method | return |
|---|---|---|
| `SpurGearInvoluteToothDesignGenerator` | `getParameterValue` | `-> float` |

**Borrowing constraint.** `bevelgear.py` constructs this generator with a
`spurproxy.VirtualSpurProxy` as `parent` (`[PB-PRECOMPUTED-MODE]`). Inside `drawCircles`,
`drawTooth` and `draw`, and inside every helper they call, parameters may be read ONLY from the key
set that proxy serves: `Module`, `ToothNumber`, `PressureAngle`, `PitchCircleDiameter`,
`PitchCircleRadius`, `BaseCircleDiameter`, `BaseCircleRadius`, `RootCircleDiameter`,
`RootCircleRadius`, `TipCircleDiameter`, `TipCircleRadius`, `InvoluteSteps`. Reading any other key
on those paths raises `KeyError` and breaks the bevel build.

**`calculateInvolutePoint(baseRadius, intersectionRadius)` — exact math.** Returns `None` when
`intersectionRadius < baseRadius`; otherwise:

```
alpha = math.acos(baseRadius / intersectionRadius)
t     = math.tan(alpha)          # the curve parameter is tan(alpha), NOT inv(alpha) = tan(alpha) - alpha
x = baseRadius * (math.cos(t) + t * math.sin(t))
y = baseRadius * (math.sin(t) - t * math.cos(t))
```

Using `inv(alpha)` as the parameter instead of `tan(alpha)` produces a mis-parameterised flank.

### 1. drawCircles — the four circles

Draw, in this order, all centred on the local origin by passing the `SketchPoint` **directly** as
the centre so the four share it (`[SPUR-F-SHARED-ADJACENCY]`, `[PB-SHARE-XOR-COINCIDENT]`); do not
pass `localOrigin.geometry` and then add a centre coincident, and do not rely on
`[PB-CIRCLE-CENTER]`'s `isFixed` route, which is for a circle with no shared anchor:

1. **Root Circle** at `RootCircleRadius` — **solid**, not construction.
2. **Tip Circle** at `TipCircleRadius` — construction.
3. **Base Circle** at `BaseCircleRadius` — construction.
4. **Pitch Circle** at `PitchCircleRadius` — construction.

Each is `sketch.sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)`
(`[PB-SKETCHCURVES]`: the curve collections live under `sketch.sketchCurves`, never on the sketch).
Give each a **driving** diameter dimension with
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`, placing `textPoint` **off-centre**,
on or near the circle.

Driving is the default, so `isDriven=True` is never passed (`[PB-DRIVING-DIM]`). The off-centre text
point matters because a text point at the curve's centre is rejected with
`RuntimeError: 3 : ... some input arguments are invalid` — at the centre there is no radial
direction to place the dimension (`[PB-RADIAL-DIM]`).

Label each circle with along-path sketch text (`[PB-SKETCH-TEXT]`), three calls exactly:

```python
textInput = sketch.sketchTexts.createInput2(label, size)
textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
sketch.sketchTexts.add(textInput)
```

Written out as calls this step must make:
`sketch.sketchTexts.createInput2(label, size)`,
`textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`,
`sketch.sketchTexts.add(textInput)`.

The label string is `'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` — the circle's name,
its radius and `size`, all from the radii's internal `.value` in cm — where
`size = TipCircleRadius - RootCircleRadius`. That same `size` is the text **height** argument to
`createInput2`.

When a later part of this step needs one of these circles — the tip circle for the tooth-top point,
the root circle for the flank-to-root stubs — either keep the direct reference returned here or
locate it with the framework helper `find_circle_by_radius(sketch, radius)` from `.utilities`. Never
fall back to an arbitrary circle on a failed radius match.

<!-- check-step-calls: ignore find_circle_by_radius -->
`find_circle_by_radius` is one of the two routes the spec allows and not a required call: an
implementation that keeps the references `drawCircles` already has needs neither the helper nor the
lookup.

This text is why the Gear Profile sketch never reports `isFullyConstrained`
(`[PB-TEXT-HOLDS-DOF]`): text placed with `setAsAlongPath` carries its own position along the curve
and nothing pins it. Log the result rather than raising on it. The exemption covers the labels and
nothing else.

### 2. drawTooth — the involute tooth, drawn at its final angle

1. **Sample the flank.** With `steps = InvoluteSteps`, sample `i = 0 … steps-1` at
   `r = BaseCircleRadius + (TipCircleRadius - BaseCircleRadius) * i / (steps - 1)`, so the first
   sample radius is exactly `BaseCircleRadius` and the last is exactly `TipCircleRadius`. Do **not**
   clamp the start to `max(BaseCircleRadius, RootCircleRadius)`: the flank is sampled from the base
   circle even when the base circle sits inside the root circle. Each sample is
   `self.calculateInvolutePoint(BaseCircleRadius, r)`; drop any that returns `None`.
2. **Mirror across +X** — negate `y` on every sample — before rotating. The standard parametric
   involute spirals so its angular position *grows* with radius, which as a left flank gives a
   tooth wider at the tip than at the root.
3. **Rotate so the tooth is symmetric about +X.** Compute the pitch crossing **analytically**, not
   by interpolating between samples: with
   `(px, py) = self.calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`,
   `rotate_angle = math.pi / (2 * ToothNumber) - math.atan2(-py, px)`. The `-py` is step 2's mirror
   applied to the analytic point; `math.atan2(py, px)` is the wrong sign.
4. **Rotate the mirrored samples by `rotate_angle`** to get the **left** flank, then mirror that
   result across the X axis to get the **right** flank. **Then rotate BOTH flank collections by the
   `angle` argument**, and seed the tooth-top point and every rib midpoint at their `angle`-rotated
   positions too. `[SPUR-F-ROTATE-CONFIRM]`: the geometry is drawn already rotated *and* the
   confirming angular dimension is set at the end — the two are not alternatives. Drawing the tooth
   flat and swinging it with the dimension lets Fusion pick a branch about 180 degrees off, which
   ruins the helical loft. At `angle = 0` this rotation is a no-op. Because the bottom and top
   profiles share one `rotate_angle` baseline and differ by exactly `angle`, the loft twists by
   exactly the helix angle.
5. **Draw each flank as a `SketchFittedSpline`** through its point collection:
   collect the points into `adsk.core.ObjectCollection.create()` and call
   `sketch.sketchCurves.sketchFittedSplines.add(pointCollection)`.

### 3. Tooth-top arc

The arc caps the tooth at the tip circle (`[SPUR-F-TOOTHTOP-ARC]`), so it *is* part of that circle
and must bulge outward.
Exactly these four things, and nothing else:

1. Materialize a **tooth-top point** — a `SketchPoint` at
   `(TipCircleRadius * math.cos(angle), TipCircleRadius * math.sin(angle))`, rotated by `angle` to
   match the flanks — and constrain it coincident to the **tip circle** with
   `sketch.geometricConstraints.addCoincident(toothTopPoint, tipCircle)`.
2. Create the arc as
   `sketch.sketchCurves.sketchArcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)`,
   passing the two flank splines' **end `SketchPoint`s directly** — the arc shares those, so they
   need no coincidences. The start is the **right** flank's `endSketchPoint` and the end is the
   **left** flank's, in that argument order.
3. **Then tie the centre back:**
   `sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, localOrigin)`.
   `addByCenterStartEnd` shares the start and end points but **copies the centre** into a fresh
   `SketchPoint` (`[PB-SHARE-XOR-COINCIDENT]`), so passing `localOrigin` as the first argument fixes
   nothing. This is the one arc here whose centre must be coincident rather than shared, and it is
   not the redundant double-bind that rule otherwise forbids. Without it the centre is a free point
   carrying only the arc's equal-radius relation to the two flank ends — 2 free degrees of freedom
   — and it stays behind when the tooth is dragged onto the anchor. Measured in Fusion 2026-09-02
   on a default 31/31 bevel pair: a 0.5743 mm tooth-top radius on the pinion and 17.0204 mm on the
   driving gear where both should have been 22.5 mm.
4. Add **no diameter dimension**. A free centre plus a diameter fixes the size but not which way the
   arc curves: the same radius through the same two ends can bulge inward, so the sketch reaches
   DOF 0 with two valid answers. Putting the centre on the origin removes the choice.

Putting the centre on the origin also makes the **last rib's perpendicular redundant**; the rib
recipe below omits it, and keeping both throws `VCS_SKETCH_OVER_CONSTRAINTS`.

### 4. Spine, +X reference and angular pin

Draw the spine (`[SPUR-F-SPINE]`) as a construction line
`sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, toothTopPoint)` — pass **both**
existing `SketchPoint`s directly so the line shares them — and set `spine.isConstruction = True`.
Do not create it from `.geometry`, do not add a separate start-coincident to the origin (sharing
already ties it, and the extra coincident makes the solver fail), and do not constrain the spine's
end onto the arc.

Build the **+X reference construction line for every `angle`, including 0**:

1. Add a far endpoint at `(TipCircleRadius, 0)` with
   `sketch.sketchPoints.add(adsk.core.Point3D.create(tipRadius, 0, 0))` and pin it with **two axis
   dimensions from the local origin**:
   `sketch.sketchDimensions.addDistanceDimension(localOrigin, referenceEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
   with value `TipCircleRadius`, and the same call with
   `adsk.fusion.DimensionOrientations.VerticalDimensionOrientation` with value `0`. Both values are
   non-negative magnitudes and the endpoint is seeded on the +X side
   (`[PB-DIM-VALUE-SEMANTICS]`). Pin it this way rather than with a coincidence to the tip circle:
   a point on a circle has two answers, and pinning `x` at the tip radius instead touches the circle
   at its extreme, where the numbers go unstable.
2. Draw the reference line from the origin to that endpoint with `addByTwoPoints` and mark it
   construction.
3. Add the angular dimension **from the reference to the spine, in that argument order**:
   `sketch.sketchDimensions.addAngularDimension(referenceLine, spine, textPoint)`. Place its
   `textPoint` on the **bisector of the intended angle**, at
   `(R * math.cos(angle / 2), R * math.sin(angle / 2))` for a small `R`, so Fusion selects `angle`
   and not its supplement (`[PB-ANGULAR-DIM]`). Keep the returned dimension; its value is set as the
   very last action of `draw` (below).

Do **not** use a plain `addHorizontal` on the spine for the `angle = 0` case: horizontal fixes the
line's direction but says nothing about which way it points, so the tooth top can settle at either
end of the tip circle and the tooth comes out 180 degrees around. The angular dimension against a
+X-pinned reference is what says which way, and it is used for every angle so spur, helical,
herringbone and the bevel virtual tooth stay on one path.

### 5. Ribs, exact order

One rib per fit-point index `i` (`[SPUR-F-RIBS]`), for **all N indices, endpoints included** — the
base-circle pair
`i = 0` and the tip pair `i = N-1` both get one. The fit points carry no other constraint, so an
omitted endpoint rib leaves the sketch under-constrained. Per rib, in exactly this order; a
different order throws `VCS_SKETCH_OVER_CONSTRAINTS` (`[PB-NO-OVERCONSTRAIN]`):

1. `rib = sketch.sketchCurves.sketchLines.addByTwoPoints(leftSpline.fitPoints.item(i), rightSpline.fitPoints.item(i))`
   — pass the two fit-point `SketchPoint`s **directly** so the rib shares them
   (`[SPUR-F-SHARED-ADJACENCY]`); mark it construction.
2. Dimension the rib with an **axis** dimension, never an aligned one:
   `sketch.sketchDimensions.addDistanceDimension(leftFitPoint, rightFitPoint, orientation, textPoint)`.
   Choose the orientation so the rib takes the axis **across** the spine and the midpoint chain
   takes the one **along** it: when `abs(math.cos(angle)) >= abs(math.sin(angle))` the rib is
   `VerticalDimensionOrientation` and the chain is `HorizontalDimensionOrientation`; otherwise swap
   both. That reduces to vertical-rib / horizontal-chain at `angle = 0`, and a tooth at 90 degrees
   fails without it. Create the dimension with the fit points already at their seeded positions and
   leave its value at the measured magnitude — the direction is captured from the seed at creation
   (`[PB-DIM-VALUE-SEMANTICS]`). An aligned dimension gives only the length, which the left and
   right flanks satisfy equally well swapped over, so the tooth can come out mirrored.
3. Add a fresh `SketchPoint` for the midpoint, created **already on the spine**. With
   `t = fitX * math.cos(angle) + fitY * math.sin(angle)` taken from the **left** fit point, the seed
   is `(t * math.cos(angle), t * math.sin(angle))`. At `angle = 0` that reduces to `(fitX, 0)`. Do
   not seed it at the rib's true 2-D midpoint, and do not seed it at `(fitX, 0)` for a rotated
   tooth (`[PB-SEED-NEAR]`).
4. `sketch.geometricConstraints.addCoincident(midPoint, spine)` — pin the point onto the spine
   **first**.
5. `sketch.geometricConstraints.addMidPoint(midPoint, rib)` — then make it the rib's midpoint.
6. `sketch.geometricConstraints.addPerpendicular(spine, rib)` — then make the rib perpendicular to
   the spine. **Skip this for the last rib only.** That rib joins the two flank tips, which the
   tooth-top arc already holds at equal radius either side of the spine, so its perpendicular says
   nothing new and Fusion rejects it with `VCS_SKETCH_OVER_CONSTRAINTS`.

Then dimension each rib's midpoint from the **previous** rib's midpoint with an axis dimension along
the spine direction (horizontal at `angle = 0`, per the swap rule above), **starting the chain at the
local origin**: for the first rib the dimension runs from the local origin to its midpoint. Without
that origin-to-first dimension the whole chain has one residual degree of freedom — it slides along
the spine as a unit — and the sketch never fully constrains. The axis dimension's direction, captured
from the seeded midpoints, makes the chain run outward; an aligned dimension is equally happy running
the other way, which is one of the ways the tooth ends up reversed. Per rib this is exactly
determined: any further constraint, a different order, or an off-spine midpoint seed over-constrains
it.

### 6. Flank-to-root lines and the embedded test

**The embedded test is strict `<`.** With `firstRadius` the distance from the local origin to the
**left** flank's first fit point, `embedded = firstRadius < RootCircleRadius`
(`[SPUR-F-FLANK-ROOT]`), comparing raw values with no tolerance. Exact equality therefore counts as **non**-embedded and draws a zero-length stub.
Keep the strict comparison; do not "improve" it to `<=` or add a tolerance.

When the flank starts **outside** the root circle, draw one short radial line per side. Build each
as `sketch.sketchCurves.sketchLines.addByTwoPoints(rootEndGeometry, flankStartFitPoint)` — pass the
flank spline's **start `SketchPoint` directly** as the far endpoint, so the line shares it and needs
no separate coincident. Seed the root end at its exact computed position **before** creating any
dimension, then place it with **exactly these two axis dimensions from the local origin, and no
others**:

- `sketch.sketchDimensions.addDistanceDimension(localOrigin, rootEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
- the same call with `adsk.fusion.DimensionOrientations.VerticalDimensionOrientation`

Each dimension captures its direction from the seed and is created already at the exact magnitude.
Set values only to `abs(dx)` and `abs(dy)` — **never the axis-signed deltas: a negative
`parameter.value` flips the point to the other side of the origin** (`[PB-DIM-VALUE-SEMANTICS]`;
this exact flip mirrored the right-hand root end and left the tooth loop open, found in Fusion
2026-08-24). Together the two dimensions take the root end from 2 free degrees of freedom to 0, and
their captured directions say which side of the gear centre it sits on.

Do **not** place it instead with "root end on the root circle" plus "local origin on the line":
those two are satisfied by **two** points, because the line through the flank start and the centre
meets the root circle again on the far side, and the sketch then reaches DOF 0 with both answers
available.

The two shapes and their curve counts, which S9 and S10 select on:

- **Non-embedded** (the common case): the tooth loop has **6 curves** — 2 splines, 2 flank-to-root
  lines, 2 arcs (the tooth top, and the piece of the root circle between the two stub feet).
- **Embedded**: no flank-to-root line is drawn and the loop has **4 curves** — 2 splines and
  2 arcs. This happens above `2.5 / (1 - cos(PressureAngle))` teeth, which is 41.5 at 20 degrees,
  78.5 at 14.5 degrees and 26.7 at 25 degrees, so a larger pressure angle brings it on sooner.

**Embedded-flag mechanism.** The tooth generator has no `ctx`, so `drawTooth` sets the boolean on
its parent generator: `self.parent._lastToothEmbedded = True` or `False`. `buildSketches`, which
does hold `ctx`, copies it across with `ctx.toothProfileIsEmbedded = self._lastToothEmbedded`. Do
not try to set `ctx.toothProfileIsEmbedded` from inside the tooth generator. `bevelgear` reads the
same `_lastToothEmbedded` slot off its proxy after `draw()` returns, so keep that write in place.

### 7. Anchor the sketch, then confirm the rotation

Project the Tools-sketch anchor into this sketch —
`projectedAnchor = sketch.project(ctx.anchorPoint)` — and add
`sketch.geometricConstraints.addCoincident(self.anchorPoint, projectedAnchor)`, where
`self.anchorPoint` is the tooth generator's local origin, **not** `sketch.originPoint`. Because
every piece of geometry above is constrained relative to the local origin, this one coincidence
drags the whole tooth profile onto the user's anchor as a unit.

This anchoring happens **inside `draw()`**, not in `buildSketches` after `draw()` returns: helical
and herringbone build their twisted profile by calling the generator's `draw` directly and rely on
that one call to anchor the sketch.

Then, as the **very last action after the entire constraint network exists**, and only when
`angle != 0`, set the spine's angular dimension value:
`if angle != 0: spineAngularDimension.parameter.value = angle` (`[SPUR-F-ROTATE-CONFIRM]`). The
angular dimension itself exists for every angle including 0; at 0 it is created at 0 and there is
nothing to set.

### What the proof establishes

`stepGearProfileSketch` rebuilds this sketch in the sketch engine and gates it on
`sketch.VerificationReport.Check` with nothing waived — fully constrained, no redundant or
conflicting constraint, well-conditioned, valid profiles, and no discrete ambiguity — across the
regime the spec declares the scheme must hold across: several Module and Tooth Number pairs; the
whole signed range of `angle` including a negative one, a quarter turn either way where the rib and
chain dimensions swap axis, and the half turn the bevel virtual tooth draws; the low end of the rib
count as well as the standard 15; and both routes into the embedded shape, a high tooth count at 20
degrees and a moderate tooth count at a large pressure angle. It also asserts the two regions and
their curve counts on the sketch it actually drew, which is what S9 and S10 match on.

<!-- check-step-calls: ignore addHorizontal -->
`addHorizontal` is named only to forbid it on the spine; the module must not call it there.

**From:** `spec/spurgear/instructions.md` L108-124, L254-326, L384-387, L411-453, L504-513, L515-553, L554-559; `spec/spurgear/fusion.md` L17-44, L45-61, L62-68, L69-105, L106-132, L133-174, L175-216; `.claude/skills/generate-gear/PLAYBOOK.md` L438-475, L491-523, L581-601, L602-603, L615-623, L649-659.

## S8 `[PROSE]` Sketch-only short-circuit

If the `SketchOnly` parameter reads true —
`self.getParameterAsBoolean(PARAM_SKETCH_ONLY)` — make the Gear Profile sketch visible
(`ctx.gearProfileSketch.isVisible = True`) and stop: `buildMainGearBody` returns before the tooth
extrude, so S9 through S13 do not run. The bore steps and the chamfer step still get called
from `generate` and return early themselves, and S17 runs unconditionally.

This step builds no geometry, so no proof function realises it. Its effect on the later steps is
carried in their own early returns.

**From:** `spec/spurgear/instructions.md` L60, L560-562, L362-371.

## S9 `[GO]` Extrude the tooth

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeTooth, assertExtrudeTooth) -->

`buildTooth(ctx)` owns this step; helical overrides it to loft instead, so keep the boundary.

Find the single tooth cross-section in the Gear Profile sketch with the framework helper — do not
re-implement the loop search, which rejects loops whose counts do not match and raises when nothing
matches (`[PB-PROFILE-MATCH]`):

`find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`

The profile has 2 NURBS (the two flanks), 2 arcs (the tooth top and the root arc between them) and,
unless the profile is embedded, 2 short line segments (the flank-to-root lines).

Extrude it from the target plane to the Extrusion End Plane as a **New Body**:

```python
extrudeFeatures = self.getComponent().features.extrudeFeatures
extrudeInput = extrudeFeatures.createInput(toothProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
extrude = extrudeFeatures.add(extrudeInput)
```

Written out as calls this step must make:
`extrudeFeatures.createInput(toothProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`extrudeFeatures.add(extrudeInput)`.

Name the feature `Extrude tooth` and store the resulting body as `ctx.toothBody`. This step applies
no chamfer.

`stepExtrudeTooth` builds the tooth section and sweeps it, and `assertExtrudeTooth` checks it comes
out as one new body, exactly `Thickness` tall — which is what ending on the Extrusion End Plane
means — and reaching the tip circle.

**From:** `spec/spurgear/instructions.md` L292-300, L384-391, L564-568; `.claude/skills/generate-gear/PLAYBOOK.md` L151-158, L639-648.

## S10 `[GO]` Extrude the gear body

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeBody, assertExtrudeBody) -->

`buildBody(ctx)` owns this step.

Find the gear-body profile — the solid disc inside the root circle, whose boundary is **exactly 2
arcs**, the two pieces the tooth's flank-to-root lines cut the root circle into:
`find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)`. It is **not** an annulus and the tip
circle is not part of it: the tip circle is construction geometry and construction geometry bounds
no profile.

Extrude it from the target plane to the Extrusion End Plane as a **New Body**, with
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)` and
`adsk.fusion.ExtentDirections.PositiveExtentDirection`. Name the feature `Extrude body` and the
resulting body `Gear Body`.

While iterating the new body's faces — `extrude.bodies.item(0).faces` — classify each by
`face.geometry.surfaceType` and capture two references. `SurfaceTypes` lives in **`adsk.core`**, not
`adsk.fusion` (`[PB-ADSK-MODULES]`):

- **`Gear Center` construction axis** — from any face whose `surfaceType` is
  `adsk.core.SurfaceTypes.CylinderSurfaceType`. Build it with
  `axisInput = self.getComponent().constructionAxes.createInput()`, then
  `axisInput.setByCircularFace(cylindricalFace)`, then
  `ctx.centerAxis = self.getComponent().constructionAxes.add(axisInput)`
  (`[PB-CONSTRUCTION-AXES]`). Name it `Gear Center` and set `isLightBulbOn = False`.
- **`ctx.extrusionExtent`** — the far end-cap face the bore cut ends on. Among faces whose
  `surfaceType` is `adsk.core.SurfaceTypes.PlaneSurfaceType`, take the one parallel to but **not**
  coplanar with the sketch plane. Test it with the plane-geometry API rather than a hand-rolled dot
  product: with `sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`, pick the face where
  `sketchPlane.isParallelToPlane(face.geometry)` and not `sketchPlane.isCoPlanarTo(face.geometry)`.
  The near cap is coplanar, so `isCoPlanarTo` rules it out, and the cylindrical and side faces are
  not planar.

Raise if either reference is not found; a face search that finds nothing must not fall through
(`[PB-EMPTY-RESULT]`, `[PB-SELF-DIAGNOSING]`). Finally store `ctx.gearBody`.

`stepExtrudeBody` sweeps the root disc and `assertExtrudeBody` checks its volume is the whole root
disc's, `pi * RootCircleRadius^2 * Thickness`, which a ring bounded by the tip circle could not be.

**From:** `spec/spurgear/instructions.md` L292-300, L336-338, L364-365, L570-579; `.claude/skills/generate-gear/PLAYBOOK.md` L430-437, L707-728, L729-736, L754-757.

## S11 `[GO]` Pattern the teeth

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepPatternTeeth, assertPatternTeeth) -->

`patternTeeth(ctx)` owns this step and the next one. Circular-pattern `ctx.toothBody` around the
`Gear Center` axis. Pin all three pattern inputs explicitly rather than relying on Fusion's defaults
(`[PB-CIRCULAR-PATTERN]`):

```python
bodies = adsk.core.ObjectCollection.create()
bodies.add(ctx.toothBody)
circularPatternFeatures = self.getComponent().features.circularPatternFeatures
patternInput = circularPatternFeatures.createInput(bodies, ctx.centerAxis)
patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
patternInput.isSymmetric = False
pattern = circularPatternFeatures.add(patternInput)
```

Written out as calls this step must make: `adsk.core.ObjectCollection.create()`,
`bodies.add(ctx.toothBody)`, `circularPatternFeatures.createInput(bodies, ctx.centerAxis)`,
`adsk.core.ValueInput.createByString('360 deg')`, `circularPatternFeatures.add(patternInput)`.
`quantity` is Tooth Number, `totalAngle` is the fixed string expression `'360 deg'`, and
`isSymmetric` is `False`. Those three together are the placement: Tooth Number copies over a full
turn, running one way from the seed, so copy `k` sits at `2 * pi * k / ToothNumber`.

`stepPatternTeeth` proves that placement. `assertPatternTeeth` checks the copy count against Tooth
Number, that every copy carries the seed tooth's own volume, and that copy `k`'s centroid sits at
`2 * pi * k / ToothNumber` around the gear axis, measured one way from a seed that is itself at 0 —
which a symmetric pattern or a different total angle would not produce. The proof file records that
the copies are measured one document at a time, and why the harness will not hold them all at once.

**From:** `spec/spurgear/instructions.md` L366, L581-583; `.claude/skills/generate-gear/PLAYBOOK.md` L665-670.

## S12 `[GO]` Combine the patterned teeth into the Gear Body

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCombineTeeth, assertCombineTeeth) -->

Still inside `patternTeeth(ctx)`, and a second timeline entry: one Combine-Join of the patterned
tooth bodies into `Gear Body`.

`pattern.bodies` already holds the seed tooth plus the copies (`[PB-PATTERN-BODIES]`), so feed it
to the combine as-is and do not re-add the seed — but copy it into a fresh
`adsk.core.ObjectCollection.create()` first, item by item through `pattern.bodies.item(i)`, because
`pattern.bodies` is a `BRepBodies` and `combineFeatures.createInput` rejects it:

```python
tools = adsk.core.ObjectCollection.create()
combineFeatures = self.getComponent().features.combineFeatures
combineInput = combineFeatures.createInput(ctx.gearBody, tools)
combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
combineFeatures.add(combineInput)
```

Written out as calls this step must make: `tools.add(pattern.bodies.item(i))`,
`combineFeatures.createInput(ctx.gearBody, tools)`, `combineFeatures.add(combineInput)`.

`stepCombineTeeth` builds **one** Combine-Join, because the harness will not chain them: decad
admits a single analytic union of two prisms, and the result then reroutes the next union on the
same lineage to a mesh path that refuses two prisms swept from a common plane. `assertCombineTeeth`
checks that the united body is the Gear Body disc plus exactly the material of one tooth outside it,
and then that the whole gear, extruded from one outline, has the volume of the disc plus Tooth
Number teeth — which a join that dropped or doubled a tooth, or a pattern whose copies overlapped,
could not satisfy. The proof file records the substitutions and what they cost.

**From:** `spec/spurgear/instructions.md` L366, L583-585; `.claude/skills/generate-gear/PLAYBOOK.md` L660-664.

## S13 `[GO]` Root fillets

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepFilletRoots, assertFilletRoots) -->

`createFillets(ctx)` owns this step. It runs only when `FilletRadius > 0`, and it reads the
`FilletRadius` parameter's numeric `.value` — it does **not** call
`filletHelixFactorExpression()`, whose only consumer is the parameter registration in S2.

Round the corner where the root valley floor meets each tooth flank: the sharp inside corner running
the full thickness of the gear, parallel to its main axis. That is where bending stress
concentrates at the tooth root. It is **not** the front or back rim, which is a cosmetic rounding
the user does not want here.

Two things make the edge selection fiddly:

- After the pattern and combine, the root cylinder is usually split into one patch per valley rather
  than one continuous surface. Collect **every** cylindrical face whose radius equals
  `RootCircleRadius`, not just the first one found. Floating-point radii differ in the last bits, so
  the test carries a tolerance and it is `0.0001` cm:
  `abs(face.geometry.radius - rootRadius) <= 0.0001`. That is the same default
  `utilities.find_circle_by_radius` uses, so the two ways of finding a circle in this codebase
  agree.
- On each such face keep the **axial straight edges** — the two valley-floor-to-tooth-flank corners
  on that patch. Filter first to edges whose `edge.geometry.curveType` is
  `adsk.core.Curve3DTypes.Line3DCurveType` (`[PB-PROFILE-MATCH]` gives the exact member names; the
  constants end in `...CurveType`). Take each line's direction from its **geometry endpoints** —
  `direction = edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)` — then
  `direction.normalize()`, and keep it when
  `abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01`, where `axisNormal` is the target
  plane's normal from `get_normal(self.plane)`. **Use exactly that 0.01 tolerance**: a tighter test
  such as `> 0.999` can drop valid axial edges that are slightly off from tessellation, leaving root
  fillets missing. Drop the *circular* edges that wrap the circumference at the front and back end
  caps; those are end-cap rims, not structural root corners.

Do **not** read the direction with `edge.evaluator.getTangent(0)`: parameter `0` is not guaranteed
to lie inside the edge's parameter range and Fusion raises
`RuntimeError: invalid argument parameter`.

Apply the fillet with the edge set added on the input **itself** (`[PB-FILLET-CHAMFER]`):

```python
filletFeatures = self.getComponent().features.filletFeatures
filletInput = filletFeatures.createInput()
filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), False)
filletFeatures.add(filletInput)
```

Written out as calls this step must make: `filletFeatures.createInput()`,
`filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), False)`,
`filletFeatures.add(filletInput)`.

`isTangentChain` must be `False` — the collected edges are exactly the axial root corners, and
tangent-chaining would let Fusion pull in tangent-adjacent edges and round more than intended. Do
not route the edge set through `filletInput.edgeSetInputs`; that is the chamfer-side shape and
reaching for it on a fillet input raises `AttributeError`.

If the edge collection ends up **empty**, silently skip the fillet: return without creating the
feature and raise nothing. An empty edge set must never reach `filletFeatures.add`
(`[PB-EMPTY-RESULT]`).

`stepFilletRoots` selects the same corners in the predicates decad has — concave, parallel to the
gear's main axis, exactly two per valley — and `assertFilletRoots` checks that a fillet in a concave
corner adds material, that no circular end-cap rim reads as parallel to the axis, and that a
selection matching nothing is a real outcome the empty-collection guard has to handle.

<!-- check-step-calls: ignore edgeSetInputs getTangent -->
`filletInput.edgeSetInputs` and `edge.evaluator.getTangent` are named only to forbid them; the
module must call neither.

**From:** `spec/spurgear/instructions.md` L86-89, L126-131, L367, L392-396, L587-596; `.claude/skills/generate-gear/PLAYBOOK.md` L151-158, L437, L536-542, L639-648.

## S14 `[GO]` Bore Profile sketch

<!-- proof-run: proofkit.Run(boreProfileCases, stepBoreProfileSketch) -->

`buildBore(ctx)` runs **unconditionally** from `generate()`, after `buildMainGearBody`, so it must
early-return in two cases of its own, before drawing anything:

- when **SketchOnly** is set. This guard is essential: in sketch-only mode `buildMainGearBody`
  short-circuits before `buildBody`, so `ctx.gearBody` and `ctx.extrusionExtent` are never set and
  the cut in the next step would dereference `None`. Do not rely on the bore diameter being 0 in
  that mode — the user may have set both.
- when **Bore Diameter is 0 or less**, the shipped default, which means no bore at all. Neither this
  sketch nor the cut after it is created.

Otherwise, on the full-build path with Bore Diameter above 0, create a separate sketch named
`Bore Profile` on the target plane —
`self.boreSketch = self.createSketchObject('Bore Profile', plane=self.plane)` — and draw the bore
circle by instantiating the tooth generator on that sketch,
`SpurGearInvoluteToothDesignGenerator(self.boreSketch, self)`, and calling
`toothGen.drawBore(ctx.anchorPoint, boreDiameter)`. `drawBore(anchorPoint, diameter)` takes the
anchor entity and the bore diameter in cm; it projects the anchor into the sketch with
`sketch.project(anchorPoint)`, draws the circle of that diameter centred on the projection with
`sketch.sketchCurves.sketchCircles.addByCenterRadius(projectedAnchor, diameter / 2)` and a
**driving** diameter dimension, and returns the circle. The circle is solid, not construction.

The tooth generator's **constructor** always adds its local-origin `(0, 0, 0)` `SketchPoint`
(`[SPUR-F-LOCAL-ORIGIN]`), so the Bore Profile sketch carries one stray unused sketch point at the
origin. That is faithful behaviour — do not suppress it — but it must be grounded, exactly as S7
grounds the Gear Profile's: add
`sketch.geometricConstraints.addCoincident(toothGen.anchorPoint, projectedAnchor)` using the same
projection `drawBore` already made. With no grounding at all the point is free in two directions and
the sketch never reaches `isFullyConstrained` (`[PB-FULL-CONSTRAINT]`).

Do **not** ground it on `boreSketch.originPoint` instead: that pins the point to the plane rather
than to the gear, and `[PB-CIRCLE-CENTER]` records a solver failure from constraining to
`originPoint`.

### What the proof establishes

`stepBoreProfileSketch` draws this sketch and gates it fully constrained across the bore sizes the
dialog accepts above zero, with the anchor both on the sketch origin and off it. It asserts the
sketch closes exactly one region, the bore disc of `pi * (BoreDiameter / 2)^2`, which is what the
cut consumes, and it rebuilds the same sketch with the stray point left ungrounded and requires it
to hold the two degrees of freedom the grounding removes.

The alternative the spec forbids is not reachable. Grounding on the sketch's own origin point
reaches DOF 0 in the engine exactly as grounding on the projection does; the engine has no notion of
a constraint that solves but tracks the wrong thing, and `[PB-CIRCLE-CENTER]`'s solver failure is a
Fusion observation only. The proof file records that beside the check.

**From:** `spec/spurgear/instructions.md` L54, L320-322, L368, L430-435, L598-602; `spec/spurgear/fusion.md` L26-32; `.claude/skills/generate-gear/PLAYBOOK.md` L438-454.

## S15 `[GO]` Bore cut

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepBoreCut, assertBoreCut) -->

Still inside `buildBore(ctx)`, and a second timeline entry: extrude-cut the Bore Profile region from
the target plane to `ctx.extrusionExtent`, the far end-cap face captured in S10, affecting only
`ctx.gearBody`. It is skipped whenever the previous step was, on the same two guards.

```python
extrudeFeatures = self.getComponent().features.extrudeFeatures
extrudeInput = extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
extrudeInput.participantBodies = [ctx.gearBody]
extrudeFeatures.add(extrudeInput)
```

Written out as calls this step must make:
`extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)`,
`extrudeFeatures.add(extrudeInput)`.

`extrudeInput.participantBodies` is set to `[ctx.gearBody]` so the cut reaches that body and nothing
else. Ending on the far end-cap face is what guarantees the bore goes all the way through whatever
Thickness is.

`stepBoreCut` proves both sides of the Bore-Diameter branch: at 0 the gear body comes back whole,
and above 0 `assertBoreCut` checks exactly the bore cylinder's volume is gone and the body is still
the full Thickness tall. The proof file records that its receiver is the Gear Body cylinder rather
than the completed gear, and why.

**From:** `spec/spurgear/instructions.md` L54, L338, L368, L598-602; `.claude/skills/generate-gear/PLAYBOOK.md` L438-454.

## S16 `[GO]` Chamfer the completed gear

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepChamferTeeth, assertChamferTeeth) -->

`chamferTeeth(ctx)` runs from `generate` after `buildBore`, so it sees the patterned and joined
teeth, the root fillets and an optional bore. It is shared unchanged by spur, helical and
herringbone. It returns early in **SketchOnly** mode and when **Apply-chamfer-to-teeth is zero**,
the shipped default.

Walk every planar face of `ctx.gearBody` parallel to the Gear Profile sketch plane — both end caps —
and add each edge of those faces once, de-duplicating with `edge.tempId`. This includes the tooth
flanks, the tooth tops and the root-radius arcs; those are deliberately in.

Exclude only a `adsk.core.Curve3DTypes.Circle3DCurveType` edge whose radius is the positive Bore
Diameter divided by two, within `0.001` cm, so a bore never receives a chamfer.

Raise when no end-cap face is found or no chamfer edge remains; do not create a partial chamfer
(`[PB-EMPTY-RESULT]`, `[PB-SELF-DIAGNOSING]`).

Apply the set with the edge set added on the input's **`chamferEdgeSets`** collection — the mirror
image of the fillet side (`[PB-FILLET-CHAMFER]`):

```python
chamferFeatures = self.getComponent().features.chamferFeatures
chamferInput = chamferFeatures.createInput2()
chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(chamferTooth), False)
chamferFeatures.add(chamferInput)
```

Written out as calls this step must make: `chamferFeatures.createInput2()`,
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(chamferTooth), False)`,
`chamferFeatures.add(chamferInput)`.

The earlier tooth-cap edge-count predicates are gone: `4` aborted the feature and `6` chamfered only
the bottom cap because it required coplanarity with the base sketch plane, both confirmed in Fusion
on 2026-08-30 (`[HELI-F-CHAMFER-COUNT]`). The completed-gear selection above is what replaced them,
and it remains pending Fusion verification.

`stepChamferTeeth` proves the one selection rule the spec states — a bore never receives a chamfer —
by chamfering the end-cap loop of a bored gear body and measuring that exactly the outer rim's
material is gone. `assertChamferTeeth` checks that against the closed form for an equal-distance
chamfer of a circular rim. The proof file records its two forced substitutions: the receiver is the
gear body's own end cap rather than the toothed one, because a cap-loop chamfer of the toothed
section builds but leaves decad's volume reading beyond tolerance and the solid gate refuses that
rather than waive it; and only one cap is chamfered, because decad will not compose a second modify
operation onto a cap-loop chamfer result.

**From:** `spec/spurgear/instructions.md` L58, L388-390, L369, L604-619; `spec/helicalgear/fusion.md` L69-80; `.claude/skills/generate-gear/PLAYBOOK.md` L437, L536-542, L639-648, L729-736.

## S17 `[PROSE]` Cleanup

`cleanup(ctx)` is the **very last action of `generate()`** — after `chamferTeeth`, not inside
`buildMainGearBody` — and it is called **unconditionally**, in both modes. The SketchOnly
distinction lives inside it, not at the call site. Placement after `buildBore` matters because
`buildBore` re-projects `ctx.anchorPoint` out of the Tools sketch and projection fails once that
sketch is hidden, so the Tools sketch must stay visible through the bore and the chamfer.

Hide each entity with the right property (`[SPUR-F-CLEANUP]`), and never cross them
(`[PB-HIDE-AFTER-USE]`): `isLightBulbOn = False` for construction planes and axes,
`isVisible = False` for sketches.

- The construction-plane and axis hiding **always runs, in both modes**, so no stray plane floats:
  the `Extrusion End Plane`, the `Gear Center` axis, and the normalized target plane if S4 created
  one.
- The **sketch** hiding runs **only on the full-build path**. In Generate-Sketches-Only mode the
  Tools and Gear Profile sketches are left visible for inspection, which is the whole point of that
  mode. On the full-build path hide the Tools, Gear Profile and Bore Profile sketches, so only the
  finished gear body shows.

Guard each entity individually and hide it only if it was actually created: the `Gear Center` axis
and the Bore Profile sketch do not exist in sketch-only mode.

This step changes visibility only and builds no geometry, so no proof function realises it.

**From:** `spec/spurgear/instructions.md` L313-315, L370-381, L562; `spec/spurgear/fusion.md` L217-230; `.claude/skills/generate-gear/PLAYBOOK.md` L626-638.
