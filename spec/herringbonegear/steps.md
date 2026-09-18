# Herringbone Gear — compiled step list

The proof for this step list is `proof/herringbonegear/sketches_test.go`, `proof/herringbonegear/solids_test.go` and the generated registration file `proof/herringbonegear/zz_registrations_test.go`.

Herringbone is a thin specialization of helical, which is itself a thin specialization of spur. It
adds no dialog input, no user parameter and no sketch, and it re-implements nothing: the module is
three subclasses, two of them empty, and five overridden methods. Steps HB1–HB4 are that surface.
Steps HB5–HB9 are the two behavioural deltas — the mid-body plane and the loft-mirror-combine
chevron — and each is checked by the proof. Step HB10 is the inherited pipeline that must stay
inherited.

Every string, id and number this module needs is written out below. `/emit-gear` reads this file,
the proof and a playbook extract and nothing else, so a value that is only cited here is a value
that reaches the transcriber as nothing at all.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/herringbonegear/instructions.md` | `0ca177828c06bbf8472200559b5dd47b8cb359a9` |
| `spec/herringbonegear/fusion.md` | `62755ad2d376481cf449893ab2a445772357d218` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `spec/helicalgear/instructions.md` | `71b7993007b17cc4948569313c23614b8fbd246c` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9ee2dcbaed7b5480aa69e9295e8b61acaea081f3` |

## HB1 `[PROSE]` Module layout, imports and exported surface

Write `lib/geargen/herringbonegear.py` as a module that defines three classes and nothing else: no
module-level constant, no helper function, no dialog input, no user parameter. Herringbone adds
none of those — `[SPUR-EXPORTED-CONSTANTS]` names the constants it imports rather than declares.

Imports, explicitly and with no `import *` (PLAYBOOK "Module layout & imports"):

```python
import adsk.core
import adsk.fusion
from .base import GenerationContext
from .spurgear import PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS
from .helicalgear import (
    PARAM_HELIX_ANGLE,
    HelicalGearCommandConfigurator,
    HelicalGearGenerationContext,
    HelicalGearGenerator,
)
```

`GenerationContext` is imported from `.base` because the spec's Architecture section lists it among
herringbone's imports; it is the base type of the context hierarchy and nothing else in this module
names it. `adsk.core` carries `ValueInput` and `ObjectCollection`; `adsk.fusion` carries the feature
collections — get the module right, since the wrong one is a runtime `AttributeError` rather than a
parse error (`[PB-ADSK-MODULES]`, and look every call up before writing it, `[PB-API-LOOKUP]`).

The three class names are reproduced surface: `commands/herringbonegear/entry.py` binds two of them
by name, so a rename is a breaking change.

**From:** `spec/herringbonegear/instructions.md` L1–6, L27–41, L93–104; `.claude/skills/generate-gear/PLAYBOOK.md` L17–41, L226–243

## HB2 `[PROSE]` `HerringboneGearCommandConfigurator` — helical's dialog, unchanged

```python
class HerringboneGearCommandConfigurator(HelicalGearCommandConfigurator):
    pass
```

A trivial `pass` subclass. Herringbone's dialog **is** helical's: the same ten spur inputs in spur's
order, with helical's Helix Angle appended last by the inherited configurator
(`[SPUR-SUBCLASS-INPUT]`). Do not add an input, do not re-add helical's, and do not reorder.

The inherited dialog is reproduced here as reference, because nothing downstream can look it up.
The module writes none of these ids; they are the surface the inherited configurator and
`processInputs` already carry:

| # | Dialog label | input id | registered user-parameter |
|---|---|---|---|
| 1 | Target Plane (selection) | `plane` | — |
| 2 | Anchor Point (selection) | `anchorPoint` | — |
| 3 | Module | `module` | `Module` |
| 4 | Tooth Number | `toothNumber` | `ToothNumber` |
| 5 | Pressure Angle | `pressureAngle` | `PressureAngle` |
| 6 | Bore Diameter | `boreDiameter` | `BoreDiameter` |
| 7 | Thickness | `thickness` | `Thickness` |
| 8 | Apply chamfer to teeth | `chamferTooth` | `ChamferTooth` |
| 9 | Generate sketches, but do not build body | `sketchOnly` | `SketchOnly` |
| 10 | Parent Component (selection) | `parentComponent` | — |
| 11 | Helix Angle | `helixAngle` | `HelixAngle` |

The derived parameters the inherited pipeline registers, unchanged, are `PitchCircleDiameter`,
`PitchCircleRadius`, `BaseCircleDiameter`, `BaseCircleRadius`, `RootCircleDiameter`,
`RootCircleRadius`, `TipCircleDiameter`, `TipCircleRadius`, `InvoluteSteps`,
`ToothSpaceAngleAtRoot`, `ToothSpaceArcAtRoot`, `FilletClearance` and `FilletRadius`. Helix Angle
defaults to 14.5 degrees and is registered in radians; every registration is the inherited
pipeline's work (`[SPUR-EXTRA-PARAMS]`, `[PB-DIALOG-DEFAULT-UNITS]`, `[PB-INPUT-READ]`,
`[PB-SELECTION-DECL]`).

**From:** `spec/herringbonegear/instructions.md` L16–25, L32–33; `spec/helicalgear/instructions.md` L51–70, L215; `spec/spurgear/instructions.md` L90–106, L182–193

## HB3 `[PROSE]` `HerringboneGearGenerationContext` — helical's context, one field repurposed

```python
class HerringboneGearGenerationContext(HelicalGearGenerationContext):
    pass
```

Another trivial `pass` subclass: the same fields, initialised by the inherited `__init__`. Do not
declare a field, and do not re-declare helical's `helixPlane` or `twistedGearProfileSketch`.

What changes is meaning, not structure. Because HB5 halves the offset, `ctx.helixPlane` is now the
**mid-body** plane, and it is the plane HB8 mirrors across. `ctx.twistedGearProfileSketch` is the
loft's top section sitting on that mid-body plane — the profile itself is full size; only its plane
moves.

**From:** `spec/herringbonegear/instructions.md` L34–35, L43–48, L103

## HB4 `[PROSE]` `HerringboneGearGenerator` — the three identity overrides

```python
class HerringboneGearGenerator(HelicalGearGenerator):
    def newContext(self) -> HerringboneGearGenerationContext:
        return HerringboneGearGenerationContext()

    def prefixBase(self) -> str:
        return 'HerringboneGear'

    def generateName(self) -> str:
        module = self.getParameter(PARAM_MODULE)
        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)
        thickness = self.getParameter(PARAM_THICKNESS)
        helixAngle = self.getParameter(PARAM_HELIX_ANGLE)
        return 'Herringbone Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(
            module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)
```

Three overrides, none of them more than a lookup and a return:

- `newContext` returns `HerringboneGearGenerationContext()`.
- `prefixBase` returns the string `'HerringboneGear'`, which becomes the `<prefix>_` on every user
  parameter this gear registers.
- `generateName` returns the component name, built from the **four parameters' `.expression`
  strings** — not their values — with `.format(module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)`,
  so units show through as the user typed them. The format string is
  `'Herringbone Gear (M={}, Tooth={}, Thickness={}, Angle={})'`, verbatim, four placeholders in that
  order.

All three carry a return annotation, because the base declares one and a subclass that drops it
turns the parent's return into the literal it happens to return. `newContext` is annotated with this
gear's own context type; the other two return `str`.

The four parameters are fetched by name through the inherited `getParameter`, with the constants
imported in HB1: `PARAM_MODULE`, `PARAM_TOOTH_NUMBER` and `PARAM_THICKNESS` from `.spurgear`, and
`PARAM_HELIX_ANGLE` from `.helicalgear`. The spec names the four `.expression` strings and the
imports but not the lookups between them; this is the only reading of it that uses exactly those
imports.

These three methods exist for the framework to call, so nothing in this module calls them.

<!-- check-step-calls: ignore newContext prefixBase generateName -->

**From:** `spec/herringbonegear/instructions.md` L36, L55–58; `spec/helicalgear/instructions.md` L111–114; `spec/spurgear/instructions.md` L108–124, L405–409

## HB5 `[GO]` `helicalPlaneOffset` — put the twisted profile's plane at half the thickness

This is herringbone's first delta, and the whole of it is one division:

```python
    def helicalPlaneOffset(self):
        thickness = self.getParameter(PARAM_THICKNESS).value
        return adsk.core.ValueInput.createByReal(thickness / 2)
```

No return annotation: spur's annotation contract names the five methods that must carry one, and
this hook is not among them, so annotating it here would add surface the spec does not declare.

Read the thickness with `self.getParameter(PARAM_THICKNESS)` and take its `.value`, then return
`adsk.core.ValueInput.createByReal(thickness / 2)` — a fresh `ValueInput` built from a raw number.
Helical's hook returns `getParameterAsValueInput(PARAM_THICKNESS)` instead, the whole thickness; do
not call that here.

Two things about that difference are load-bearing:

- **Both hooks are numeric snapshots** (`[PB-NUMERIC-SNAPSHOT]`), taken at generation time. Editing
  the `<prefix>_Thickness` parameter afterwards does not move this plane; the user regenerates.
- **Herringbone's number corresponds to no registered parameter.** Half the thickness is computed in
  Python and handed straight to `createByReal`, so there is no `<prefix>_` parameter holding it and
  nothing in the parameter table to find it under. That is faithful and deliberate.

The plane itself is built by the inherited `buildSketches` from whatever this hook returns
(`[HELI-F-TWIST-PLANE]`), and it lands at mid-body: the loft then spans the bottom half and the
mirror completes the top half. Keep this a method of its own — do not inline the offset into
`buildSketches`, which is the hook helical exposes for exactly this override. The value is in
Fusion's internal units, centimetres, because `createByReal` always is.

The proof function `stepMidBodyPlane` builds the half tooth this offset decides and reads the two
faces it stands between: the base plane at 0 and the mid-body plane at half the thickness, over a
thickness sweep from 2 mm to 40 mm so a constant offset could not pass as a halved one.

<!-- check-step-calls: ignore helicalPlaneOffset getParameterAsValueInput buildSketches -->
<!-- proof-run: proofkit3d.RunSolid(solidCases, stepMidBodyPlane, assertMidBodyPlane) -->

**From:** `spec/herringbonegear/instructions.md` L3–6, L59–68, L100–104; `spec/herringbonegear/fusion.md` L9–12, L46–47; `spec/helicalgear/fusion.md` L9–27

## HB6 `[GO]` The inherited `buildSketches` draws the twisted profile onto that plane

Herringbone overrides nothing here. The inherited `buildSketches` calls its own `super()`, which
draws the bottom Gear Profile at angle 0, then creates the offset construction plane from
`helicalPlaneOffset`, stores it as `ctx.helixPlane`, creates the hidden `'Twisted Gear Profile'`
sketch on it and runs the spur tooth generator into that sketch at `angle=helixAngle`
(`[HELI-F-TWIST-PLANE]`). Do not re-implement any of it, and do not add cleanup for the plane: the
helix plane is left visible after generation, a declared delta from `[PB-HIDE-AFTER-USE]` that a
regeneration must reproduce rather than tidy away.

What herringbone changes is where the sketch lands, and what that makes the section: drawn once on
the mid-body plane and consumed twice, as the top section of the lofted half and as the plane the
mirror reflects across. So the section is the chevron's apex, and the two facts the chevron rests on
are proven there:

- **The tooth loop closes with six curves** — 2 splines, 2 arcs, 2 flank-to-root lines — which is
  what the inherited `loftTooth` searches for with its fixed `nurbs=2, arcs=2, lines=2`
  (`[PB-PROFILE-MATCH]`, `[HELI-F-LOFT]`). The disc inside the root circle is the second region.
- **The tooth is drawn already rotated by the helix angle**, and the spine's angular dimension
  confirms it as the very last action (`[SPUR-F-ROTATE-CONFIRM]`, `[SPUR-F-SPINE]`). That rotation
  is the apex angle of the chevron.

The constraint scheme is spur's throughout — the anchor chain and local origin
(`[SPUR-F-ANCHOR-CHAIN]`, `[SPUR-F-LOCAL-ORIGIN]`), shared adjacency rather than doubled
coincidences (`[PB-SHARE-XOR-COINCIDENT]`), driving dimensions (`[PB-DRIVING-DIM]`), the tooth-top
arc centred on the local origin (`[SPUR-F-TOOTHTOP-ARC]`), the rib chain in its exact order
(`[SPUR-F-RIBS]`), and the flank-to-root stubs with exactly two axis dimensions each
(`[SPUR-F-FLANK-ROOT]`) — and it is proven to reach DOF 0 with nothing waived (`[PB-SKETCH-FIRST]`,
`[PB-FULL-CONSTRAINT]`). In Fusion the same sketch carries four along-path circle labels
(`[PB-SKETCH-TEXT]`), so it may read `isFullyConstrained == False` purely because it is labelled
(`[PB-TEXT-HOLDS-DOF]`); neither helical nor herringbone registers a runtime full-constraint gate,
and this module must not add one.

The proof function `stepMidBodyTwistedProfile` rebuilds that section from the same recipes, sweeps
the signed helix range including a quarter turn in each direction, both hands, the low rib counts
and both routes into the embedded shape, and asserts the loop counts, the disc area and the drawn
twist.

The names this step mentions are the inherited pipeline's own, not calls this module makes:
herringbone does NOT override `buildSketches`, so it neither defines nor calls it, and the plane and
the sketch that inherited body creates through `createSketchObject` and `setByOffset` are created
on herringbone's behalf.

<!-- check-step-calls: ignore buildSketches createSketchObject setByOffset -->
<!-- proof-run: proofkit.Run(profileCases, stepMidBodyTwistedProfile) -->

**From:** `spec/herringbonegear/instructions.md` L8–14, L43–48, L72–75, L77–91; `spec/herringbonegear/fusion.md` L10–12, L34–36; `spec/helicalgear/fusion.md` L9–42; `spec/helicalgear/instructions.md` L126–128, L165–174, L181–190

## HB7 `[GO]` `buildTooth`, first action — loft the bottom half tooth

`buildTooth` is herringbone's second delta and its only other override. Its first action is the
inherited loft, called and not reimplemented:

```python
    def buildTooth(self, ctx: GenerationContext):
        assert isinstance(ctx, HerringboneGearGenerationContext)
        self.loftTooth(ctx)
```

**The parameter is annotated `GenerationContext`, and narrowed by the assertion on the next line.**
That is the only context type this module imports (HB1), and the import list is exact: herringbone
imports neither `SpurGearGenerationContext`, which is what the inherited signature declares, nor its
own context type into the annotation position. A wider parameter annotation is compatible with the
override — what a type checker rejects in an override is a NARROWED parameter — and the
`isinstance` assertion against `HerringboneGearGenerationContext` narrows the name for every field
read below it, so `ctx.toothBody` and `ctx.helixPlane` are read against a class that declares them
and `self.loftTooth(ctx)` is handed the narrowed type. Write the assertion before the first field
read, as the first statement of the body. Do not add an import to annotate this differently: the
import list in HB1 is reproduced surface.

Helical's own spec forbids `GenerationContext` for ITS three `ctx`-taking overrides, and that rule
does not carry over: helical imports the spur context and calls `super().buildSketches(ctx)`, which
is what a widened annotation breaks there, while herringbone imports neither that type nor calls a
`super()` with `ctx` at all. Its one `ctx` method is this one, and the assertion is what types it.

`self.loftTooth(ctx)` lofts the bottom Gear Profile's tooth loop to the twisted loop on
`ctx.helixPlane`, in that order — bottom section first — and leaves the new body on `ctx.toothBody`
named `'Tooth Body'` (`[HELI-F-LOFT]`, `[PB-LOFT]`). Because HB5 halved the offset, that body spans
the **bottom half** of the gear rather than the whole of it, which is the only thing herringbone
changes about the loft.

The proof function `stepLoftToothHalf` builds that half between chorded copies of the two sections —
decad's loft pairs recorded segments and refuses a free-form pair, so the flanks are walked as
polylines, and the proof file says so next to the substitution — and asserts its exact bounding box,
which is the twist's own footprint, and its volume against the ruled solid's prismatoid volume.

<!-- check-step-calls: ignore buildTooth -->
<!-- proof-run: proofkit3d.RunSolid(solidCases, stepLoftToothHalf, assertLoftToothHalf) -->

**From:** `spec/herringbonegear/instructions.md` L69–70, L77–85, L100–104; `spec/herringbonegear/fusion.md` L9–14; `spec/helicalgear/instructions.md` L129–130, L136–156, L176–179; `spec/helicalgear/fusion.md` L46–65

## HB8 `[GO]` `buildTooth`, second action — mirror the lofted half across `ctx.helixPlane`

Still inside `buildTooth`, and still exactly as the current code does it
(`[HERR-F-MIRROR-COMBINE]`):

```python
        # Mirror the lofted half across the mid-body helix plane to form the other half.
        entities = adsk.core.ObjectCollection.create()
        entities.add(ctx.toothBody)
        mirrorInput = self.getComponent().features.mirrorFeatures.createInput(entities, ctx.helixPlane)
        mirrorResult = self.getComponent().features.mirrorFeatures.add(mirrorInput)
        mirrorResult.bodies.item(0).name = 'Tooth Body (Mirrored)'
```

The calls, in order: `adsk.core.ObjectCollection.create()` for a fresh collection,
`entities.add(ctx.toothBody)` to put the lofted half in it,
`self.getComponent().features.mirrorFeatures.createInput(entities, ctx.helixPlane)` for the input,
`self.getComponent().features.mirrorFeatures.add(mirrorInput)` to create the feature, and
`mirrorResult.bodies.item(0)` to reach the new body, whose `name` is set to the string
`'Tooth Body (Mirrored)'`.

Two operands, each named exactly:

- The **entity mirrored is `ctx.toothBody`** — the body `loftTooth` just produced — and it goes in
  through an `ObjectCollection`, which is what `createInput` takes. `ObjectCollection` is
  `adsk.core`; `mirrorFeatures` is `adsk.fusion` (`[PB-ADSK-MODULES]`).
- The **mirror plane is `ctx.helixPlane`**, the mid-body plane HB5 placed, not a fresh plane and not
  the base plane. That plane is left visible after generation, and this step adds no cleanup for it
  (`[HELI-F-TWIST-PLANE]`, `[PB-HIDE-AFTER-USE]`).

The mirrored body is renamed immediately, because the combine in HB9 looks its target up by name and
two bodies called `'Tooth Body'` would make that lookup ambiguous.

The proof function `stepMirrorToothHalf` substitutes a second loft for the mirror, since decad has
no mirror feature: it builds the upper half travelling the same way the lower one does, from the far
face's untwisted section to the same twisted section on the mid plane, which is the reflection. It
then asserts that the two halves' volumes agree as readings and that the mirrored half stands on the
mid plane and reaches the far face.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepMirrorToothHalf, assertMirrorToothHalf) -->

**From:** `spec/herringbonegear/fusion.md` L9–21, L33–36, L39–42; `spec/herringbonegear/instructions.md` L69–70, L81–82, L103–104

## HB9 `[GO]` `buildTooth`, third action — combine the mirrored half into `'Tooth Body'`

The last action of `buildTooth`, and the end of herringbone's own code
(`[HERR-F-MIRROR-COMBINE]`):

```python
        # Combine the mirrored half into the original so the pattern/combine steps in the
        # inherited spur pipeline operate on a single body.
        entities = adsk.core.ObjectCollection.create()
        entities.add(mirrorResult.bodies.item(0))
        combineInput = self.getComponent().features.combineFeatures.createInput(
            self.getComponent().bRepBodies.itemByName('Tooth Body'),
            entities,
        )
        self.getComponent().features.combineFeatures.add(combineInput)
```

The calls, in order: a second `adsk.core.ObjectCollection.create()`, rebinding the same local name,
then `entities.add(mirrorResult.bodies.item(0))` to put the mirrored half in it as the **tool**,
then `self.getComponent().features.combineFeatures.createInput(target, entities)` whose **target**
is `self.getComponent().bRepBodies.itemByName('Tooth Body')` — the lofted half, looked up by that
exact name rather than carried in a variable — and finally
`self.getComponent().features.combineFeatures.add(combineInput)`.

Three details are deliberate and must not be tidied:

- **The operation is left at its API default.** `CombineFeatureInput.operation` defaults to
  `JoinFeatureOperation`, and this code never assigns it. Do not set the operation explicitly.
- **The target is found by name.** `itemByName('Tooth Body')` is the call, with the string spelled
  exactly as `loftTooth` named the body.
- **The tool is the mirrored body**, reached again through `mirrorResult.bodies.item(0)`, not the
  renamed handle read back from the component.

After this call there is a single `'Tooth Body'` spanning the full thickness. That is what the
inherited `patternTeeth` circular-patterns and joins into the gear body, and it is why the combine
has to happen here rather than being left to the pattern.

The proof function `stepCombineToothHalves` joins the two halves and asserts what the join has to
produce: one solid body, spanning the base plane to the far face, whose volume is twice a half and
whose centroid sits on the mid plane, which is the chevron's own symmetry. decad's union refuses two
solids that meet exactly on a shared face, so the proof overlaps the mirrored half one hundredth of
the thickness past the mid plane and records what that substitution costs; at zero helix angle even
the overlap leaves coplanar walls and the case is skipped in the proof with its reason.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCombineToothHalves, assertCombineToothHalves) -->

**From:** `spec/herringbonegear/fusion.md` L9–12, L23–32, L37–42; `spec/herringbonegear/instructions.md` L69–70, L81–85

## HB10 `[PROSE]` Everything after the tooth is inherited — do not re-implement it

The rest of the build is spur's, reached through helical, and herringbone overrides none of it. Do
not write any of these methods in this module: `addExtraPrimaryParameters`,
`filletHelixFactorExpression`, `buildSketches`, `loftTooth`, `processInputs`, `prepareTools`,
`buildMainGearBody`, `buildBody`, `patternTeeth`, `createFillets`, `buildBore`, `chamferTeeth`,
`cleanup`, and the whole tooth generator.

What they do, so the omission is a decision rather than a gap:

- `buildBody` extrudes the gear body across the **full** thickness. Only the tooth is built
  half-then-mirrored; the body is not halved, and the mid-body plane does not enter it.
- `patternTeeth` circular-patterns the single combined `'Tooth Body'` around the `Gear Center` axis,
  quantity Tooth Number, and joins the result into the gear body. The pattern's own body collection
  already includes the seed (`[PB-PATTERN-BODIES]`, `[PB-CIRCULAR-PATTERN]`).
- `createFillets` rounds the axial root corners with `FilletRadius`, whose expression already
  carries helical's `cos(<prefix>_HelixAngle)` factor from the inherited
  `filletHelixFactorExpression` (`[PB-FILLET-CHAMFER]`).
- `buildBore` cuts the optional bore, and `chamferTeeth` chamfers the completed gear's end caps
  after the pattern, the fillets and the bore, root-radius arcs included and bore edges excluded
  (`[HELI-F-CHAMFER-COUNT]`).
- `cleanup` runs last, unconditionally, and hides only the entities spur itself created
  (`[SPUR-F-CLEANUP]`). It does not touch `ctx.helixPlane`, which stays visible.

<!-- check-step-calls: ignore addExtraPrimaryParameters filletHelixFactorExpression processInputs prepareTools buildMainGearBody buildBody patternTeeth createFillets buildBore chamferTeeth cleanup -->

**From:** `spec/herringbonegear/instructions.md` L50–53, L72–85; `spec/herringbonegear/fusion.md` L1–7, L43–47; `spec/helicalgear/instructions.md` L106–110, L132–134, L158–163; `spec/helicalgear/fusion.md` L67–80; `spec/spurgear/instructions.md` L344–371, L581–586
