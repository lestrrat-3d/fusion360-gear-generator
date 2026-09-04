# Herringbone Gear — compiled step list

The proof for these steps is `proof/herringbonegear/sketches_test.go`,
`proof/herringbonegear/solids_test.go` and the generated
`proof/herringbonegear/zz_registrations_test.go`.

Herringbone is a subclass gear. Its module writes three classes and five method bodies; every
other timeline entry the gear produces is inherited code that this module must not restate. Steps
H1 to H3 are the module surface, H4 to H7 are the entries herringbone's own overrides put in the
timeline, and H8 to H9 are what the inherited pipeline does afterwards and what it deliberately
leaves behind.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/herringbonegear/instructions.md` | `45c8836ef944f9c0e4a188572b4a0ffc83250f1e` |
| `spec/herringbonegear/fusion.md` | `62755ad2d376481cf449893ab2a445772357d218` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `spec/helicalgear/instructions.md` | `6c1d3b4d7aa824d90f9f0f851d4115179e754707` |
| `spec/spurgear/instructions.md` | `8cb886a7827d6745fde7c876475066918c328283` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `1b3078d6767d6a3f56c228e1e934c82ccfbf53fe` |

## H1 `[PROSE]` Module layout, imports and constants

Write `lib/geargen/herringbonegear.py` as a subclass module: it declares three classes and
nothing else at module level. It adds **no** module-level constant of its own — no input id, no
parameter name — because it adds no dialog input and no user parameter.

Import explicitly; no `import *` in a gear module. These are the imports, and each name is used:

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

`adsk.core` is used for `adsk.core.ValueInput.createByReal(thickness / 2)` in H4 and
`adsk.core.ObjectCollection.create()` in H6 and H7. `adsk.fusion` is used for the type of the
mirror and combine feature collections reached through the component. `GenerationContext` is the
annotation on `buildTooth`'s `ctx` parameter (H5); the four spur and helical constants are read by
`generateName` and `helicalPlaneOffset`. `ValueInput` and `ObjectCollection` are **`adsk.core`**
types, not `adsk.fusion` ones, and `mirrorFeatures` / `combineFeatures` are reached off the
component's `features`, which is `adsk.fusion` — get each module right at authoring time
([PB-ADSK-MODULES]), and confirm every member against the API database before writing it
([PB-API-LOOKUP]).

**From:** `spec/herringbonegear/instructions.md` L27–41, `.claude/skills/generate-gear/PLAYBOOK.md` L17–41 L432–436

## H2 `[PROSE]` Dialog inputs and user parameters — inherited whole, none added

Herringbone's dialog **is** helical's, which is spur's plus one input. This module registers no
parameter and adds no input: `HerringboneGearCommandConfigurator` is a `pass` subclass, and
`addExtraPrimaryParameters` is **not** overridden, so helical's registration of `HelixAngle` is
the last word.

Nothing in this module may add, rename or reorder any of these. They are reproduced so the
transcriber can see there is nothing to write, and so the four constants H3 and H4 read are
unambiguous:

| Dialog input | input id | registered user-parameter |
|---|---|---|
| Target Plane (selection) | `plane` | — |
| Anchor Point (selection) | `anchorPoint` | — |
| Module | `module` | `Module` |
| Tooth Number | `toothNumber` | `ToothNumber` |
| Pressure Angle | `pressureAngle` | `PressureAngle` |
| Bore Diameter | `boreDiameter` | `BoreDiameter` |
| Thickness | `thickness` | `Thickness` |
| Apply chamfer to teeth | `chamferTooth` | `ChamferTooth` |
| Generate sketches, but do not build body | `sketchOnly` | `SketchOnly` |
| Parent Component (selection) | `parentComponent` | — |
| Helix Angle | `helixAngle` | `HelixAngle` |

<!-- check-step-calls: ignore configure -->

Helix Angle is added by helical's configurator **after** `super().configure(cmd)`, so it appears
last in the dialog, after Parent Component. Its dialog unit is degrees and its default is 14.5°;
the `HelixAngle` user parameter is registered in radians. Herringbone inherits that arrangement
unchanged and must not try to move the input earlier.

The constants this module imports carry these exact string values: `PARAM_MODULE` is `'Module'`,
`PARAM_TOOTH_NUMBER` is `'ToothNumber'`, `PARAM_THICKNESS` is `'Thickness'` and
`PARAM_HELIX_ANGLE` is `'HelixAngle'`. Read parameters through the constants, never through a
literal.

`configure` is marked ignored above because this module never calls it: the mention names what
helical's configurator does, and `HerringboneGearCommandConfigurator` is a `pass` subclass that
defines no `configure` of its own.

**From:** `spec/herringbonegear/instructions.md` L16–25, `spec/helicalgear/instructions.md` L51–70, `spec/spurgear/instructions.md` L90–106 L182–193

## H3 `[PROSE]` The three classes and the trivial overrides

Three classes, each extending its helical counterpart. The names are reproduced surface:
`commands/herringbonegear/entry.py` binds two of them by name.

```python
class HerringboneGearCommandConfigurator(HelicalGearCommandConfigurator):
    pass


class HerringboneGearGenerationContext(HelicalGearGenerationContext):
    pass


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
            module.expression, toothNumber.expression, thickness.expression,
            helixAngle.expression)
```

The component name string is exactly `'Herringbone Gear (M={}, Tooth={}, Thickness={}, Angle={})'`
and the four substituted values are the parameters' `.expression` strings, not their `.value`
numbers, so units show through as they do for helical.

`newContext`, `prefixBase` and `generateName` carry return annotations because the spur base
declares them for exactly this reason: an unannotated parent reads as returning the literal it
happens to return, and a subclass annotating the wider type is then reported as an incompatible
override.

The context is a `pass` subclass: it adds no field. What changes is the meaning of one inherited
field — `ctx.helixPlane` is now the **mid-body** plane, and therefore the plane the mirror in H6
reflects across — and that follows from H4 alone, with no code in this class.

The generation context fields this module reads later are `ctx.helixPlane` and `ctx.toothBody`,
both declared by helical and spur; the framework helpers it uses are `self.getComponent()` and
`self.getParameter(name)`.

**From:** `spec/herringbonegear/instructions.md` L27–41 L43–48 L50–58, `spec/spurgear/instructions.md` L108–124

## H4 `[GO]` Mid-body helix plane and the Twisted Gear Profile sketch

This is the first timeline entry herringbone changes, and the change is one number.
`helicalPlaneOffset` is the hook the inherited `buildSketches` asks for the offset of the
twisted-profile plane; helical returns the whole `Thickness` and herringbone returns **half** of
it, which is what puts `ctx.helixPlane` at mid-body so the loft spans the bottom half and the
mirror completes the top half.

Write exactly this override, and nothing else in this step:

```python
def helicalPlaneOffset(self):
    thickness = self.getParameter(PARAM_THICKNESS).value
    return adsk.core.ValueInput.createByReal(thickness / 2)
```

The two calls it makes are `self.getParameter(PARAM_THICKNESS)` and
`adsk.core.ValueInput.createByReal(thickness / 2)`. Note what this is **not**: helical snapshots
the parameter with the framework's getParameterAsValueInput helper, while herringbone computes a
raw value, `thickness / 2`, that corresponds to no registered parameter. Both are numeric
snapshots taken at generation time rather than live expressions ([PB-NUMERIC-SNAPSHOT]), so
editing `<prefix>_Thickness` afterwards moves neither plane.

`buildSketches` is **not** overridden here. The inherited helical `buildSketches` creates the
construction plane at the offset this hook returns, creates a sketch named `'Twisted Gear Profile'`
on it, and draws the tooth into it with the spur tooth generator at
`angle=self.getParameter(PARAM_HELIX_ANGLE).value` — the twist is delivered as the tooth
generator's own `draw()` angle, so the tooth is drawn already rotated and the spine's angular
dimension confirms it. That inherited code is quoted here only so the offset's effect is legible;
none of it is written into this module:

```python
constructionPlaneInput = self.getComponent().constructionPlanes.createInput()
constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())
plane = self.getComponent().constructionPlanes.add(constructionPlaneInput)
ctx.helixPlane = plane
loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)
SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(
    ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)
ctx.twistedGearProfileSketch = loftSketch
```

The sketch that lands on this plane is the spur tooth at a non-zero angle, and the scheme that
draws it is the one the playbook's sketch-first gate applies to
([PB-SKETCH-FIRST], [PB-FULL-CONSTRAINT]): a movable local origin coincident with the projected
anchor, four circles sharing that origin, two fitted-spline flanks, a tooth-top arc whose centre
is tied back to the origin, a spine pinned against a +X reference line by a signed angular
dimension, one rib per involute sample with its midpoint on the spine and the last rib's
perpendicular omitted, and — when the flank starts outside the root circle — two flank-to-root
stubs each placed by two axis dimensions whose captured directions say which side of the centre
they sit on ([PB-DIM-VALUE-SEMANTICS], [PB-SHARE-XOR-COINCIDENT], [SPUR-F-SPINE], [SPUR-F-RIBS],
[SPUR-F-TOOTHTOP-ARC], [SPUR-F-FLANK-ROOT], [SPUR-F-ROTATE-CONFIRM], [HELI-F-TWIST-PLANE]).

`stepTwistedGearProfileSketch` proves that scheme at the twist herringbone delivers, on both
signs and at a quarter turn, at the low end of the rib count as well as the standard fifteen
samples, and through both routes into the embedded shape. It also checks the plane this step's
override produces lands at half the thickness, and it reads the two curve counts the later steps
select on off the regions the sketch actually closes: the six-curve tooth loop the loft's profile
finder keys on with nurbs=2, arcs=2, lines=2, and the disc inside the root circle
([PB-PROFILE-MATCH]).

<!-- proof-run: proofkit.Run(twistedCases, stepTwistedGearProfileSketch) -->

The inherited code above is quoted in a fenced block rather than in call spans, because this
module makes none of those calls: `buildSketches` is helical's and stays helical's. The one call
this step requires of the module is the pair inside `helicalPlaneOffset`.

**From:** `spec/herringbonegear/instructions.md` L59–68 L77–85, `spec/helicalgear/fusion.md` L9–43, `spec/helicalgear/instructions.md` L120–128 L165–175, `.claude/skills/generate-gear/PLAYBOOK.md` L352–360

## H5 `[GO]` `buildTooth` — loft the bottom half of the tooth

`buildTooth` is the one method herringbone rewrites rather than inherits, and its first action is
the loft helical already performs. Declare it with the context parameter annotated as the base
type the imports name, and narrow it with an assertion before touching any field:

```python
def buildTooth(self, ctx: GenerationContext):
    assert isinstance(ctx, HerringboneGearGenerationContext)
    self.loftTooth(ctx)
```

H5, H6 and H7 are the three consecutive statements of this one `buildTooth` body, written in that
order inside the method H5 declares; the fenced blocks in H6 and H7 carry the body's indentation to
show it. The call this step requires is `self.loftTooth(ctx)`. It is the inherited helical `loftTooth`,
which finds the tooth loop in the bottom Gear Profile sketch and in the mid-body Twisted Gear
Profile sketch, adds the bottom section to the loft first and the top second, and leaves the
result in `ctx.toothBody` named `'Tooth Body'` ([PB-LOFT], [HELI-F-LOFT]). Because
`helicalPlaneOffset` now returns half the thickness, that body spans the **bottom half** of the
gear rather than the whole of it. Do not re-implement `loftTooth`, do not extrude, and do not
chamfer here.

The annotation is `ctx: GenerationContext` — the base type — and never a narrowing to
`HerringboneGearGenerationContext`, since narrowing a parameter in an override is its own error.
The assertion does the narrowing, holds at runtime because `newContext` returns this gear's
context, and fails loudly and immediately if a caller ever passes something else.

`stepLoftToothHalf` builds that loft from the two sections at the target plane and the mid-body
plane and proves the body is a sound solid — one lump, watertight, manifold, no
self-intersection — spanning exactly the bottom half of the thickness, with the drawn tooth
section at both ends. `assertLoftToothHalf` carries the measurements.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepLoftToothHalf, assertLoftToothHalf) -->

**From:** `spec/herringbonegear/instructions.md` L69–71 L79–85, `spec/herringbonegear/fusion.md` L9–14, `spec/helicalgear/fusion.md` L46–66, `spec/helicalgear/instructions.md` L129–156

## H6 `[GO]` `buildTooth` — mirror the lofted half across `ctx.helixPlane`

The second action of `buildTooth`, and the first of the two features herringbone adds to the
timeline. Mirror the lofted half across the mid-body plane to form the other half:

```python
    entities = adsk.core.ObjectCollection.create()
    entities.add(ctx.toothBody)
    mirrorInput = self.getComponent().features.mirrorFeatures.createInput(entities, ctx.helixPlane)
    mirrorResult = self.getComponent().features.mirrorFeatures.add(mirrorInput)
    mirrorResult.bodies.item(0).name = 'Tooth Body (Mirrored)'
```

The calls this step requires, each written as the module must make it:
`adsk.core.ObjectCollection.create()`, `entities.add(ctx.toothBody)`,
`self.getComponent().features.mirrorFeatures.createInput(entities, ctx.helixPlane)`,
`self.getComponent().features.mirrorFeatures.add(mirrorInput)` and
`mirrorResult.bodies.item(0)`.

Three things are pinned and none of them may drift:

- The mirrored entity is **`ctx.toothBody`** — the lofted half — added to a fresh
  `ObjectCollection`, which is what `createInput` takes as its `inputEntities`. A collection is
  required; a bare body is not accepted.
- The mirror plane is **`ctx.helixPlane`**, the mid-body plane H4 placed, and never a fresh plane
  built for the purpose. `createInput`'s second argument is that planar entity.
- The mirrored body is renamed to exactly `'Tooth Body (Mirrored)'`, through
  `mirrorResult.bodies.item(0)`, because H7 finds the two bodies by name and the names must
  differ. Do not read `mirrorResult.bodies` without an index and do not assume a second body
  exists.

`stepMirrorToothHalf` proves the reflected half: it occupies the top half of the thickness, its
footprint is the same as the lofted half's to a part in a million, it carries the same section at
both ends, and it has the same volume. `assertMirrorToothHalf` carries those measurements. The
proof builds the reflection as a second loft rather than as a transform, because the solid
engine's placement verbs carry a body under a rigid motion and a reflection is not one; the proof
file states that substitution and what it costs.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepMirrorToothHalf, assertMirrorToothHalf) -->

**From:** `spec/herringbonegear/instructions.md` L69–71, `spec/herringbonegear/fusion.md` L9–21 L34–36

## H7 `[GO]` `buildTooth` — combine the mirrored half into `'Tooth Body'`

The third action of `buildTooth` and the last timeline entry herringbone adds. Combine the
mirrored half back into the original so the inherited pattern and combine steps operate on a
single body:

```python
    entities = adsk.core.ObjectCollection.create()
    entities.add(mirrorResult.bodies.item(0))
    combineInput = self.getComponent().features.combineFeatures.createInput(
        self.getComponent().bRepBodies.itemByName('Tooth Body'),
        entities,
    )
    self.getComponent().features.combineFeatures.add(combineInput)
```

The calls this step requires: `adsk.core.ObjectCollection.create()`,
`entities.add(mirrorResult.bodies.item(0))`,
`self.getComponent().bRepBodies.itemByName('Tooth Body')`,
`self.getComponent().features.combineFeatures.createInput(...)` and
`self.getComponent().features.combineFeatures.add(combineInput)`.

What is pinned:

- The **target** is looked up **by name**: `self.getComponent().bRepBodies.itemByName('Tooth Body')`,
  with that exact string. It is not `ctx.toothBody`, even though that is the same body.
- The **tool** is the mirrored half, `mirrorResult.bodies.item(0)`, in a fresh
  `ObjectCollection` — `createInput` takes a `BRepBody` target and an `ObjectCollection` of tools,
  in that order.
- `combineInput.operation` is **left at its API default**, which is `JoinFeatureOperation`. No
  operation is assigned. This is deliberate and faithful; do not "fix" it by setting the operation
  explicitly.
- A second `ObjectCollection` is created for the tools rather than reusing the one the mirror was
  given. Reuse would hand the combine the original body as a tool as well.

These four statements close the `buildTooth` H5 opened; the method returns nothing and applies no
chamfer. After the combine there is a single `'Tooth Body'` spanning the **full** thickness — the chevron —
which the inherited `patternTeeth` then circular-patterns and joins into the gear body.

`stepCombineToothHalves` proves what the Join has to leave behind. The solid engine refuses the
boolean itself: two bodies meeting exactly on a shared face come within its chord tolerance
without provably interpenetrating deeper than it, so it declines to decide whether their surfaces
touch or cross — measured on the exact contact, on an overlapped pair, and on two straight prisms
stacked the same way, so it is the contact and not the twist. The step therefore attempts the
union, keeps the result when one comes back, and otherwise proves the fact a Join of these two
bodies rests on: that the faces they meet on are one region in one plane, with the same area and
the same centroid, and that together the two halves span the full thickness.
`assertCombineToothHalves` carries those measurements, and the proof file records the refusal in
the engine's own words next to the geometry it could not join ([PB-EMPTY-RESULT] is not the case
here: the refusal is explicit and typed, and the step fails loudly on any other error).

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCombineToothHalves, assertCombineToothHalves) -->

**From:** `spec/herringbonegear/instructions.md` L69–71 L79–85, `spec/herringbonegear/fusion.md` L23–42

## H8 `[PROSE]` The inherited pipeline, before and after `buildTooth`

Everything else in the timeline is spur's and helical's code, reached through the inherited call
graph `generate → processInputs → prepareTools → buildMainGearBody(buildSketches → buildTooth →
buildBody → patternTeeth → createFillets) → buildBore → chamferTeeth → cleanup`. Herringbone
overrides `newContext`, `prefixBase`, `generateName`, `helicalPlaneOffset` and `buildTooth`, and
**nothing else**. Do not re-implement, and do not call from this module:

- `addExtraPrimaryParameters` and `filletHelixFactorExpression` — helical's versions stand, so
  `HelixAngle` is registered once and the root-fillet radius keeps its `cos(HelixAngle)` factor.
- `buildSketches` and `loftTooth` — helical's, consuming the overridden offset from H4.
- `prepareTools`, `buildBody`, `patternTeeth`, `createFillets`, `buildBore`, `chamferTeeth` and
  `cleanup` — spur's, unchanged, along with the whole tooth generator.

Two consequences of that inheritance are worth stating because they look like defects and are not.
The gear **body** is still extruded across the **full** thickness by `buildBody`; only the tooth is
built half-then-mirrored. And the inherited `chamferTeeth` runs from `generate` after the optional
bore, so it sees the patterned, root-filleted, optionally bored gear; it chamfers both end caps,
includes the root-radius arcs, and excludes the bore's two circular cap edges by the positive bore
radius. Herringbone inherits that selection without an override.

**From:** `spec/herringbonegear/instructions.md` L72–76 L77–85, `spec/herringbonegear/fusion.md` L43–47, `spec/helicalgear/instructions.md` L104–110 L158–164, `spec/spurgear/instructions.md` L344–371

## H9 `[PROSE]` Visibility and cleanup — what is deliberately left alone

Herringbone adds no cleanup of its own, and that leaves two visible facts a regeneration must
reproduce rather than tidy away:

- The mid-body helix `ConstructionPlane` is **left visible** after generation. Spur's `cleanup`
  hides only the entities spur itself created, and neither helical nor herringbone adds cleanup for
  this plane. Do not add one. Hiding a construction plane would in any case be
  `isLightBulbOn = False`, never `isVisible = False`, which has no effect on construction geometry
  ([PB-HIDE-AFTER-USE]).
- The `'Twisted Gear Profile'` sketch stays **hidden its whole life**. The framework's sketch
  helper returns a hidden sketch and nothing ever shows it — not `buildSketches`, not spur's
  `cleanup` — and the loft's profile finding works on it anyway. This is the declared exception to
  the draw-then-hide discipline ([PB-HIDE-AFTER-USE]), not a licence to generalise it.

In sketch-only mode the same holds: the helix plane is still created and still left visible, the
twisted sketch is drawn and stays hidden, and only the bottom Gear Profile is shown, so the
twisted profile is not inspectable there.

**From:** `spec/herringbonegear/fusion.md` L34–36, `spec/helicalgear/fusion.md` L26–42, `spec/helicalgear/instructions.md` L215–222, `spec/spurgear/fusion.md` L219–229
