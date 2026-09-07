# Helical gear — compiled step list

The proof for this step list is `proof/helicalgear/geometry_test.go`, `proof/helicalgear/sketches_test.go`,
`proof/helicalgear/solids_test.go` and the generated `proof/helicalgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/helicalgear/instructions.md` | `f24b67c01eecd0c193319f192e73e48ac62e7ed5` |
| `spec/helicalgear/fusion.md` | `c636a3b7bb6fd13cd8a4153fe63a123137d32262` |
| `spec/helicalgear/contract.json` | `b76c17b25ecc90caad199e10f4bb35308899ee79` |
| `spec/spurgear/fusion.md` | `933fe3b43c7d74696313b460cdf921a367e779a6` |
| `spec/spurgear/instructions.md` | `4a0bca4ab7b0fd275571f3408de06d013738ca53` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `bee2bf5c5d058504d0284fea00f6a4cd74ce2a36` |

## Compilation contract

```json
{
  "_comment": "Machine-readable mirror of the spec's contract sections, checked by .claude/skills/generate-gear/check_contract.py. The spec prose (instructions.md/fusion.md) is authoritative — a mismatch between this file and the spec is a spec bug; fix both together. Helical is subclass-only: it pins its own three classes and the override surface named in instructions.md 'Method contract', and inherits everything else from spur, which its own manifest pins. module_constants pins the two identifiers herringbone imports AND their exact string values. source_guards pins the two recipes the spec chose over an alternative that also builds a solid — reverting either renames nothing, so nothing else here would see it.",
  "classes": {
    "HelicalGearCommandConfigurator": {
      "bases": [
        "SpurGearCommandInputsConfigurator"
      ],
      "methods": [
        "configure"
      ]
    },
    "HelicalGearGenerationContext": {
      "bases": [
        "SpurGearGenerationContext"
      ],
      "ctx_fields": [
        "helixPlane",
        "twistedGearProfileSketch"
      ],
      "methods": [
        "__init__"
      ]
    },
    "HelicalGearGenerator": {
      "bases": [
        "SpurGearGenerator"
      ],
      "methods": [
        "newContext",
        "prefixBase",
        "generateName",
        "addExtraPrimaryParameters",
        "filletHelixFactorExpression",
        "helicalPlaneOffset",
        "buildSketches",
        "buildTooth",
        "loftTooth"
      ]
    }
  },
  "module": "lib/geargen/helicalgear.py",
  "module_constants": {
    "INPUT_ID_HELIX_ANGLE": "helixAngle",
    "PARAM_HELIX_ANGLE": "HelixAngle"
  },
  "source_guards": [
    {
      "file": "lib/geargen/helicalgear.py",
      "in_function": "buildSketches",
      "required": [
        "SpurGearInvoluteToothDesignGenerator\\(",
        "angle\\s*=",
        "PARAM_HELIX_ANGLE"
      ],
      "why": "[SPUR-F-ROTATE-CONFIRM] and [SPUR-F-SPINE]: the twist is delivered as the tooth generator's own draw() angle, which rotates the tooth in its point math and confirms the rotation with the spine's angular dimension. Drawing the tooth flat and rotating the sketch geometry afterward also produces a twisted profile, but leaves the spine dimension measuring the unrotated angle, so the sketch no longer proves its own twist."
    },
    {
      "file": "lib/geargen/helicalgear.py",
      "in_function": "loftTooth",
      "required": [
        "loftSections\\.add\\(bottomToothProfile\\)[\\s\\S]*loftSections\\.add\\(topToothProfile\\)",
        "find_profile_by_curve_counts\\("
      ],
      "why": "[HELI-F-LOFT]: the bottom section is added to loftSections before the top. Adding them in the other order also lofts a valid solid, and the handedness does NOT flip — the proof builds both orders and reads the same twist, same sign, off each centroid. What changes is the ruled walls, which are built outward from the FROM section, and with them about 12% of the volume at a 14.5 degree helix. The swap is therefore silent in the one reading a caller is most likely to check, which is why the order is pinned here rather than left to the implementation."
    }
  ]
}
```

## H1 `[PROSE]` Module layout, imports and constants

`lib/geargen/helicalgear.py` is a thin specialization of the spur gear: three classes, each
extending its spur counterpart, and no standalone generator. Nothing spur builds is re-implemented
here; the module adds one dialog input, one user parameter, one construction plane, one sketch and
one loft, and inherits everything else.

Imports are explicit, no `import *`. Import exactly these names from their home modules:

- `import math`
- `import adsk.core, adsk.fusion`
- from `.spurgear`: `PARAM_MODULE`, `PARAM_TOOTH_NUMBER`, `PARAM_THICKNESS`,
  `SpurGearCommandInputsConfigurator`, `SpurGearGenerationContext`, `SpurGearGenerator`,
  `SpurGearInvoluteToothDesignGenerator` (all public API of spur, [SPUR-EXPORTED-CONSTANTS])
- from `.base`: `get_value`
- from `.utilities`: `find_profile_by_curve_counts`

`GenerationContext` is **not** imported: the three `ctx`-taking overrides annotate
`ctx: SpurGearGenerationContext` (see H6), so nothing in this module names the base type.

Module-level constants, exact strings, both imported by name by herringbone:

| constant | value |
|---|---|
| `PARAM_HELIX_ANGLE` | `'HelixAngle'` |
| `INPUT_ID_HELIX_ANGLE` | `'helixAngle'` |

The class names are the reproduced surface: `commands/helicalgear/entry.py` binds
`HelicalGearCommandConfigurator` and `HelicalGearGenerator` by name, and herringbone subclasses all
three classes.

**From:** `spec/helicalgear/instructions.md` L3–8 L70 L72–92; `spec/spurgear/instructions.md` L226–243;
`.claude/skills/generate-gear/PLAYBOOK.md` L17–40

## H2 `[PROSE]` Dialog: `HelicalGearCommandConfigurator.configure` adds Helix Angle last

`class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator)` with
`@classmethod def configure(cls, cmd)`. The body is two statements, in this order:

1. `super().configure(cmd)` — spur adds its ten inputs, Parent Component last ([SPUR-SUBCLASS-INPUT]).
2. `cmd.commandInputs.addValueInput('helixAngle', 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(14.5)))`
   — id `helixAngle` (the value of `INPUT_ID_HELIX_ANGLE`), label `Helix Angle`, unit string
   `'deg'`, default 14.5° written in Fusion's internal angle unit, radians, because a
   `createByReal` default is always in internal units whatever the unit string
   ([PB-DIALOG-DEFAULT-UNITS]).

Because spur's `configure` already added Parent Component last, Helix Angle lands **last in the
dialog, after Parent Component**. This is the current behaviour and is reproduced exactly; the input
is appended after `super().configure(cmd)` and nothing tries to insert it earlier in the list. The
whole dialog therefore reads, in display order (the ten spur rows are added by spur's own
`configure` and are listed here only so the position of the eleventh is unambiguous):

| # | dialog label | input id | kind |
|---|---|---|---|
| 1 | Target Plane | `plane` | selection |
| 2 | Anchor Point | `anchorPoint` | selection |
| 3 | Module | `module` | value, unitless |
| 4 | Tooth Number | `toothNumber` | value, unitless |
| 5 | Pressure Angle | `pressureAngle` | value, `deg` |
| 6 | Bore Diameter | `boreDiameter` | string value |
| 7 | Thickness | `thickness` | value, `mm` |
| 8 | Apply chamfer to teeth | `chamferTooth` | value, `mm` |
| 9 | Generate sketches, but do not build body | `sketchOnly` | boolean |
| 10 | Parent Component | `parentComponent` | selection |
| 11 | Helix Angle | `helixAngle` | value, `deg` — added here |

The Helix Angle input accepts a negative value: negative is a left-hand helix. No range is enforced
and none is added — no clamp, no warning, no documented maximum.

**From:** `spec/helicalgear/instructions.md` L29–49 L51–64 L78–79 L215; `spec/spurgear/instructions.md` L182–212 L245–252;
`.claude/skills/generate-gear/PLAYBOOK.md` L128–136

## H3 `[PROSE]` Generation context: `HelicalGearGenerationContext` adds two fields

`class HelicalGearGenerationContext(SpurGearGenerationContext)` whose `__init__(self)` calls
`super().__init__()` and then initialises exactly two new fields to a cast-None, in this order:

- `self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)` — the offset `ConstructionPlane` the
  twisted top profile is drawn on (also the mirror plane herringbone reflects across).
- `self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)` — the second, "Twisted Gear
  Profile" sketch: the top loft section.

Both classes live in `adsk.fusion` ([PB-ADSK-MODULES]). Spur's fields — `plane`, `anchorPoint`,
`extrusionEndPlane`, `gearProfileSketch`, `toothBody`, `gearBody`, `centerAxis`, `extrusionExtent`,
`toothProfileIsEmbedded` — are inherited unchanged and not restated.

**From:** `spec/helicalgear/instructions.md` L80–83 L94–102; `spec/spurgear/instructions.md` L327–342

## H4 `[PROSE]` Generator identity: `newContext`, `prefixBase`, `generateName`

`class HelicalGearGenerator(SpurGearGenerator)`. Three identity overrides, each a one-line return
carrying a return annotation, since spur's base methods are annotated and a subclass override
without one draws a type-checker complaint:

| method | annotation | returns |
|---|---|---|
| `def newContext(self)` | `-> HelicalGearGenerationContext` | `HelicalGearGenerationContext()` |
| `def prefixBase(self)` | `-> str` | `'HelicalGear'` |
| `def generateName(self)` | `-> str` | the string below |

`generateName` returns
`'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)`
where `module = self.getParameter(PARAM_MODULE)`, `toothNumber = self.getParameter(PARAM_TOOTH_NUMBER)`,
`thickness = self.getParameter(PARAM_THICKNESS)` and `helixAngle = self.getParameter(PARAM_HELIX_ANGLE)`:
the four parameters' **`.expression`** strings, never `.value`, so units show through (for example
`Helical Gear (M=1, Tooth=17, Thickness=10 mm, Angle=14.5 deg)`).

These three, and the two hooks in H5, are called by spur's inherited `generate` and
`processInputs`; the module defines them and never calls them itself.
<!-- check-step-calls: ignore newContext prefixBase generateName addExtraPrimaryParameters filletHelixFactorExpression buildTooth -->

**From:** `spec/helicalgear/instructions.md` L84 L104–114; `spec/spurgear/instructions.md` L108–124 L405–409

## H5 `[PROSE]` Parameters: `addExtraPrimaryParameters`, `filletHelixFactorExpression`, `helicalPlaneOffset`

Three more overrides on `HelicalGearGenerator`, all hooks spur's inherited `processInputs`,
`registerDerivedParameters` and `buildSketches` call at fixed points ([SPUR-EXTRA-PARAMS]):

1. `def addExtraPrimaryParameters(self, inputs)` — registers the one new primary parameter, between
   spur's input-sourced parameters and its derived ones. Body, in order:
   - `helixAngle = get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')` — the input was declared with
     `addValueInput`, so it is read with `get_value` ([PB-INPUT-READ]); the unit string `'rad'` has
     `get_value` evaluate the degree expression to internal radians and return a `ValueInput` ready
     to register ([PB-GET-VALUE-CONTRACT]).
   - `self.addParameter(PARAM_HELIX_ANGLE, helixAngle, 'rad', 'Helix angle for the helical gear')`
     — user-parameter name `HelixAngle`, units **`'rad'`** (the dialog is degrees; the parameter is
     radians), comment string exactly `Helix angle for the helical gear`.
2. `def filletHelixFactorExpression(self) -> str` returns
   `f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'`. Spur splices this string in as the last factor
   of the live `FilletRadius` expression, `(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>`,
   so the root-fillet radius is multiplied by `cos(HelixAngle)` and reads correctly on the tilted
   tooth's transverse plane. Nothing else reads this hook.
3. `def helicalPlaneOffset(self)` returns `self.getParameterAsValueInput(PARAM_THICKNESS)`: the offset
   of the twisted-profile plane from the base plane, as a `ValueInput`, and for helical the full
   `Thickness`. It is a **numeric snapshot**: `getParameterAsValueInput` returns
   `ValueInput.createByReal(param.value)`, the `Thickness` value at generation time, not a live
   reference ([PB-NUMERIC-SNAPSHOT]). Keep it its own method — herringbone re-points it to half the
   thickness — and have `buildSketches` call it (H7) rather than computing the offset in place.

The `HelixAngle` value is passed straight through to the tooth generator's `draw()` `angle`
argument in H8, so its sign is the hand of the helix and nothing rescales it: the twist between the
two loft sections **is** the Helix Angle, not a lead angle derived from it, so `Thickness` does not
enter the twist.

**From:** `spec/helicalgear/instructions.md` L29–40 L65–70 L115–125; `spec/spurgear/instructions.md` L86–88 L391–404;
`.claude/skills/generate-gear/PLAYBOOK.md` L103–126 L196–228

## H6 `[PROSE]` Inherited timeline entries 1–5, reached through `super().buildSketches(ctx)`

Spur's call graph is inherited whole and no work moves across its boundaries:
`generate → processInputs → prepareTools → buildMainGearBody(buildSketches → buildTooth → buildBody →
patternTeeth → createFillets) → buildBore → chamferTeeth → cleanup`. The inherited `prepareTools`
creates the `Tools` sketch with the projected anchor `ctx.anchorPoint` and the `Extrusion End Plane`
(spur steps 1–2), and the inherited spur `buildSketches` creates the `Gear Profile` sketch and runs
the tooth generator at angle 0 (spur steps 3–5), leaving `ctx.gearProfileSketch` and
`ctx.toothProfileIsEmbedded`.

Helical overrides `def buildSketches(self, ctx: SpurGearGenerationContext):` and its body opens with
exactly two statements before anything helical:

1. `assert isinstance(ctx, HelicalGearGenerationContext)` — the parameter is annotated with the
   inherited signature's type, `SpurGearGenerationContext`, and narrowed by this assertion; every
   read or write of `ctx.helixPlane` or `ctx.twistedGearProfileSketch` comes after it. The
   annotation is neither widened to `GenerationContext` nor narrowed to
   `HelicalGearGenerationContext` (narrowing a parameter in an override is its own type error).
   `buildTooth` and `loftTooth` (H9) carry the same annotation and the same opening assertion.
   <!-- check-compile: ignore isinstance -->
2. `super().buildSketches(ctx)` — draws the bottom Gear Profile and runs the spur tooth generator at
   angle 0. The bottom tooth loop it leaves is the non-embedded 6-curve tooth (2 splines, 2 arcs,
   2 flank-to-root lines); the embedded 4-curve shape is unsupported by helical (H9).

Then the two deltas follow, H7 and H8, in that order, still inside `buildSketches`.

**From:** `spec/helicalgear/instructions.md` L17–21 L104–109 L126–128 L132–156 L158–167; `spec/spurgear/instructions.md` L344–390 L494–558

## H7 `[PROSE]` Helix plane: offset construction plane at `helicalPlaneOffset()` from `self.plane`

Inside `buildSketches`, after `super().buildSketches(ctx)`, create the plane on the gear's own
component ([HELI-F-TWIST-PLANE], [PB-CONSTRUCTION-PLANES]), in this order:

1. `constructionPlaneInput = self.getComponent().constructionPlanes.createInput()`
2. `constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())` — the planar entity
   is `self.plane`, the normalised target `ConstructionPlane` spur set in its step 1; the offset is
   the `ValueInput` returned by `self.helicalPlaneOffset()` (H5), the full `Thickness` for helical.
   The offset argument is a `ValueInput`, never a bare number.
3. `plane = self.getComponent().constructionPlanes.add(constructionPlaneInput)`
4. `ctx.helixPlane = plane`

**Visibility, a deliberate fact to reproduce:** the helix `ConstructionPlane` is left visible after
generation. Nothing sets its `isLightBulbOn` off — spur's inherited `cleanup` hides only the entities
spur itself created (Extrusion End Plane, normalised plane, `Gear Center` axis), and helical adds no
cleanup of its own. This holds in SketchOnly mode too: the plane is still created and still left lit.
This is a declared delta from [PB-HIDE-AFTER-USE]'s construction-geometry rule.

The plane is proven inside `stepLoftTooth` (H9), which draws the twisted section on a plane offset
by `Thickness` and reads the offset back off the lofted body's extent; a plane on its own leaves no
body for the solid harness to gate.

**From:** `spec/helicalgear/instructions.md` L100–101 L120–125 L165–168 L217–222; `spec/helicalgear/fusion.md` L9–16 L21–27 L35–42;
`.claude/skills/generate-gear/PLAYBOOK.md` L626–638 L742–753

## H8 `[GO]` Twisted Gear Profile sketch on the helix plane, `stepTwistedGearProfile`

Still inside `buildSketches`, after H7 ([HELI-F-TWIST-PLANE]):

1. `loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)` — the sketch is
   named exactly `Twisted Gear Profile` and is created on `plane`, the helix plane from H7.
   `createSketchObject` returns it hidden.
2. `toothGenerator = SpurGearInvoluteToothDesignGenerator(loftSketch, self)` — the spur tooth
   generator, constructed with its default `angle=0`, on the new sketch with this generator as its
   parent.
3. `toothGenerator.draw(ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)` — the
   anchor is `ctx.anchorPoint`, the Tools-sketch projection of the user's anchor, which `draw()`
   re-projects into this sketch and constrains the local origin to ([SPUR-F-ANCHOR-CHAIN],
   [SPUR-F-LOCAL-ORIGIN]); the twist is delivered as the `draw()` `angle` argument, the raw
   `.value` of the `HelixAngle` parameter in radians, sign included. The tooth generator draws the
   whole tooth already rotated by that angle in its point math and, as its very last action after
   the anchoring, sets the spine's confirming angular dimension to it ([SPUR-F-ROTATE-CONFIRM],
   [SPUR-F-SPINE]). The tooth is never drawn flat and rotated afterwards: that leaves the spine
   dimension measuring the unrotated angle and lets the solver pick the ~180°-off branch, which
   sends the loft through the gear centre.
4. `ctx.twistedGearProfileSketch = loftSketch`

That is all the helical code in this step. The constraint scheme it relies on is spur's, run on its
angle ≠ 0 path — the +X reference line whose far end is pinned by two axis dimensions and the
angular dimension from reference to spine ([SPUR-F-SPINE]); one rib per involute fit point with
the across-spine and along-spine axis dimensions swapping when |sin| > |cos| ([SPUR-F-RIBS]);
the tooth-top arc centred on the local origin with no diameter dimension ([SPUR-F-TOOTHTOP-ARC]);
the two flank-to-root lines placed by exactly two axis dimensions each ([SPUR-F-FLANK-ROOT]); every
axis dimension a magnitude whose side is set by the seed ([PB-DIM-VALUE-SEMANTICS]). It reaches
DOF 0 with no redundant or conflicting constraint and no mirrored or rotated alternative
([PB-FULL-CONSTRAINT]), which is what the proof below shows before any Fusion code is written
([PB-SKETCH-FIRST]). Helical adds no runtime full-constraint gate, and neither does spur.

**Visibility, a deliberate fact to reproduce:** the Twisted Gear Profile sketch stays hidden its whole
life. `createSketchObject` returns it hidden and nothing shows it — not `buildSketches`, not spur's
`cleanup`, and not SketchOnly mode, where the twisted profile is therefore not inspectable. The
loft's profile search in H9 works on the hidden sketch; this is the one verified exception
[PB-HIDE-AFTER-USE] records, and helical declares it as a sketch-discipline delta.

**What the proof pins.** `stepTwistedGearProfile` draws this sketch — the four circles, the two
involute flank splines from `proof/involute`, the tooth-top arc, spine, reference, ribs and
flank-to-root lines, anchored to a reference point standing for the projected anchor — at the case's
helix angle, and then counts the closed regions: exactly two, the tooth loop closing as 2 splines +
2 arcs + 2 lines (one of the arcs being the piece of the solid root circle the lines split off), and
the disc inside the root circle bounded by the root circle alone. In the embedded regime it pins the
4-curve loop and the absence of any 6-curve loop, which is what makes the H9 search fail there. The
cases sweep sizes, both signs of the helix angle including a quarter turn each way, 4 and 15
involute steps, and both routes into the embedded shape.

<!-- proof-run: proofkit.Run(twistedProfileCases, stepTwistedGearProfile) -->

**From:** `spec/helicalgear/instructions.md` L10–15 L35–40 L102 L126–128 L169–174 L181–190 L203–208 L217–222;
`spec/helicalgear/fusion.md` L16–24 L29–34 L40–42; `spec/spurgear/instructions.md` L411–429 L515–558;
`spec/spurgear/fusion.md` L19–31 L47–60 L69–173 L175–215; `.claude/skills/generate-gear/PLAYBOOK.md` L230–242 L350–419 L438–490 L626–638

## H9 `[GO]` Loft the tooth from the bottom loop to the twisted loop, `stepLoftTooth`

`def buildTooth(self, ctx: SpurGearGenerationContext):` opens with
`assert isinstance(ctx, HelicalGearGenerationContext)` and then does one thing: `self.loftTooth(ctx)`.
It does not extrude and it does not chamfer; the inherited `chamferTeeth` runs later on the completed
gear (H10).

`def loftTooth(self, ctx: SpurGearGenerationContext):` opens with the same assertion and lofts the
two tooth loops into `ctx.toothBody` ([HELI-F-LOFT], [PB-LOFT]), in this order:

1. `lofts = self.getComponent().features.loftFeatures`
2. `bottomToothProfile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)`
   — the bottom tooth loop in the Gear Profile sketch, found by its curve counts and nothing else
   ([PB-PROFILE-MATCH]); the framework helper raises when no loop matches.
3. `topToothProfile = find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)`
   — the same loop in the hidden Twisted Gear Profile sketch.
4. `loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)` — a new
   body; `FeatureOperations` lives in `adsk.fusion` ([PB-ADSK-MODULES]).
5. `loftInput.loftSections.add(bottomToothProfile)` — the bottom section **first**.
6. `loftInput.loftSections.add(topToothProfile)` — then the top. The order of `loftSections.add`
   is the loft order. Adding them the other way also lofts a valid solid with the same hand, but the
   ruled walls are built outward from the first section and about 12% of the volume moves at a
   14.5° helix, silently, which is why the order is pinned.
7. `loftResult = lofts.add(loftInput)`
8. `ctx.toothBody = loftResult.bodies.item(0)`
9. `ctx.toothBody.name = 'Tooth Body'`

Both sections pass a fixed `nurbs=2, arcs=2, lines=2`, the non-embedded 6-curve tooth. `loftTooth`
does not read `ctx.toothProfileIsEmbedded` and has no embedded branch: an embedded low-tooth-count
helical gear (flank starting inside the root circle, `lines=0`) fails to find the profile and raises.
This is the current behaviour, a documented limitation reproduced as is.

**What the proof pins.** `stepLoftTooth` draws the bottom tooth at angle 0 on the base plane and the
twisted tooth at the helix angle on a plane offset by `Thickness`, finds the tooth loop in each,
and lofts bottom → top into one body. `assertLoftTooth` then reads: the body spans exactly
`Thickness` along the plane normal (the H7 offset); its centroid sits at half that and, in the
plane, at polar angle `HelixAngle/2` with the helix angle's sign — the twist reading that tells a
left-hand tooth from a right-hand one; its centroid radius lies between the root circle drawn in by
`cos(HelixAngle/2)` and the tip circle, so a twist that drags the walls through the gear centre
fails; and its volume sits in a loose bracket around the straight prism of the same loop. The
sections are drawn loft-ready — flanks, tooth top and root piece chorded on the same constrained
points — because the solid engine pairs only line and circular segments and refuses the volume bound
its own chording of a circular pair produces; the proof file says what that costs. The cases sweep
sizes, thicknesses, 4 and 15 involute steps, both signs of the helix angle out to the bound the
proof measures per sign (recorded in the proof, not here), and the embedded shape, where the proof
pins that no 6-curve loop exists and records the step as the raise it is in Fusion.

<!-- proof-run: proofkit3d.RunSolid(loftCases, stepLoftTooth, assertLoftTooth) -->

**From:** `spec/helicalgear/instructions.md` L129–131 L136–147 L176–179 L187–188 L216–217; `spec/helicalgear/fusion.md` L46–65;
`spec/helicalgear/contract.json` L57–65; `spec/spurgear/instructions.md` L387–388 L564–568;
`.claude/skills/generate-gear/PLAYBOOK.md` L435 L639–648 L691–695

## H10 `[PROSE]` Inherited remainder: SketchOnly, body, pattern, fillets, bore, chamfer, cleanup

Everything after the loft is spur's code, inherited unchanged, and the module defines none of it:
not `processInputs`, `prepareTools`, `buildMainGearBody`, `buildBody`, `patternTeeth`, `createFillets`,
`buildBore`, `chamferTeeth` or `cleanup`, and not the tooth generator
`SpurGearInvoluteToothDesignGenerator`. What runs, for the record of the timeline it produces:

- Spur step 6, the SketchOnly short-circuit inside the inherited `buildMainGearBody`: shows the Gear
  Profile sketch and stops before `buildTooth`. Helical's H7 plane and H8 sketch have already been
  created by then, so in this mode the helix plane stands lit and the twisted sketch stays hidden
  (H7, H8).
- Spur step 9, `buildBody`: extrudes the 2-arc disc to the Extrusion End Plane as `Gear Body`, and
  captures the `Gear Center` axis and `ctx.extrusionExtent`.
- Spur step 10, `patternTeeth`: circular-patterns the lofted `ctx.toothBody` about `Gear Center`,
  quantity Tooth Number, and combine-joins the pattern into `Gear Body`.
- Spur step 11, `createFillets`: the root fillets, whose radius is the `FilletRadius` parameter
  already carrying the `cos(HelixAngle)` factor from H5.
- Spur step 12, `buildBore`: the optional bore, skipped in SketchOnly mode and at a zero diameter.
- Spur step 13, `chamferTeeth`: the optional completed-gear chamfer over every end-cap edge of
  `ctx.gearBody`, bore edges excluded ([HELI-F-CHAMFER-COUNT]); helical inherits it with no
  override, and its final selection is pending Fusion verification.
- Spur `cleanup`: hides spur's own planes, axis and sketches only. The helix plane is not among them
  and stays visible (H7).

None of these is helical geometry, so the helical proof does not build them; `proof/spurgear` does.
The proof file records why, beside the loft it does build.

**From:** `spec/helicalgear/instructions.md` L132–134 L158–163 L176–179 L210–222; `spec/helicalgear/fusion.md` L69–80;
`spec/spurgear/instructions.md` L560–619; `spec/spurgear/fusion.md` L219–229
