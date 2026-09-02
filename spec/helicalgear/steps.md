# Helical Gear — compiled step list

The proof for these steps is `proof/helicalgear/sketches_test.go` and
`proof/helicalgear/solids_test.go`, registered by the generated
`proof/helicalgear/zz_registrations_test.go`.

Helical is a thin specialization of the spur gear. It subclasses spur's three classes, inherits the
whole spur build pipeline, and changes three things: one extra dialog input, a second twisted
profile sketch, and a loft in place of the tooth extrude. The steps below are helical's own work
only. Spur's steps belong to spur's step list and helical must not re-implement any of them — H10
says which ones and why.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/helicalgear/instructions.md` | `21ad4667c0a7ebefab1d2d4fb7145512b66a8869` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `spec/spurgear/fusion.md` | `7cd4e5b0fa38dcd39cbd1b5bad1cf8489e2bc2ae` |
| `spec/spurgear/instructions.md` | `560549a350c789b6598ee49b72d3cd7b4460e214` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `aa97523b214dbc98b679e8eea73c56c5115c4e54` |

## H1 `[PROSE]` Module layout, imports and exported constants

`lib/geargen/helicalgear.py` imports explicitly, with no `import *`. From `.spurgear`:
`PARAM_MODULE`, `PARAM_TOOTH_NUMBER`, `PARAM_THICKNESS`, `SpurGearCommandInputsConfigurator`,
`SpurGearGenerationContext`, `SpurGearGenerator`, `SpurGearInvoluteToothDesignGenerator`. From
`.base`: `GenerationContext`, `get_value`. From `.utilities`: `find_profile_by_curve_counts`. Plus
`math`, `adsk.core` and `adsk.fusion`.

The module defines exactly two module-level constants, and both are public API that
`herringbonegear.py` imports by name, which is the exported-constants convention
`[SPUR-EXPORTED-CONSTANTS]` sets for this family:

- `PARAM_HELIX_ANGLE = 'HelixAngle'`
- `INPUT_ID_HELIX_ANGLE = 'helixAngle'`

Every `adsk` name this module writes belongs to a specific submodule and the wrong one is a runtime
`AttributeError` rather than a parse error, so resolve each before writing it (`[PB-ADSK-MODULES]`):
`ValueInput` is `adsk.core`, while `ConstructionPlane`, `Sketch` and `FeatureOperations` are
`adsk.fusion`. Look up any call whose spelling, submodule or signature is not certain rather than
guessing it (`[PB-API-LOOKUP]`).

`GenerationContext` is imported for one reason: the three `ctx`-taking overrides annotate their
parameter as `ctx: GenerationContext`, matching the inherited signatures. Keep the annotation or
the import goes unused.

**From:** `spec/helicalgear/instructions.md` L1-8, L70, L86-90, L134-137;
`spec/spurgear/instructions.md` L152-169; `.claude/skills/generate-gear/PLAYBOOK.md` L17-40

## H2 `[PROSE]` HelicalGearCommandConfigurator — the Helix Angle dialog input

`HelicalGearCommandConfigurator` extends `SpurGearCommandInputsConfigurator`. Its
`configure(cls, cmd)` classmethod calls `super().configure(cmd)` first, then appends one value
input. That is the configurator extension seam `[SPUR-SUBCLASS-INPUT]`, and appending after the
`super()` call is the whole of it:

`cmd.commandInputs.addValueInput('helixAngle', 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(14.5)))`

The display unit is `'deg'` and the default is passed in Fusion's internal unit for an angle, which
is radians — hence `math.radians(14.5)` rather than a bare `14.5`. That is
`[PB-DIALOG-DEFAULT-UNITS]`: the unit string governs display and expression parsing only, never how
`createByReal` reads its argument, so a bare `14.5` would ship a 14.5-radian default in a field
labelled degrees.

Because spur's `configure` has already added Parent Component last, the Helix Angle input lands
**after** Parent Component in the dialog. That is the consequence `[SPUR-SUBCLASS-INPUT]` records
for any subclass input. That is the current behaviour and it is reproduced
exactly; do not try to insert the input earlier in the list.

The dialog accepts a negative value, and a negative helix angle is a left-hand helix. Nothing
clamps the input and nothing warns about a large one. Do not add a clamp, a warning or a documented
maximum: what Fusion does at a large helix angle is unverified, and the bound the proof measures is
a property of the proof's own evaluator, recorded in `proof/helicalgear/solids_test.go` and
deliberately not promoted to a rule here.

**From:** `spec/helicalgear/instructions.md` L36-49, L51-64, L78-79, L196;
`spec/spurgear/instructions.md` L131-138, L171-178;
`.claude/skills/generate-gear/PLAYBOOK.md` L128-136

## H3 `[PROSE]` HelicalGearGenerationContext — spur's fields plus two

`HelicalGearGenerationContext` extends `SpurGearGenerationContext`. Its `__init__(self)` calls
`super().__init__()` and then initialises exactly two new fields, each to a cast-None:

- `self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)` — the offset construction plane the
  twisted top profile is drawn on, and the plane herringbone later mirrors across.
- `self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)` — the second sketch, the top loft
  section.

Both cast targets are `adsk.fusion` names, not `adsk.core` ones (`[PB-ADSK-MODULES]`). H7 and H8 are
where the two fields are filled, under `[HELI-F-TWIST-PLANE]`.

Every spur context field is inherited unchanged and none is restated here.

**From:** `spec/helicalgear/instructions.md` L80-83, L92-100;
`spec/spurgear/instructions.md` L253-268; `spec/helicalgear/fusion.md` L9-19

## H4 `[PROSE]` HelicalGearGenerator identity — context, prefix and component name

`HelicalGearGenerator` extends `SpurGearGenerator` and overrides three identity members:

- `newContext` returns a `HelicalGearGenerationContext`.
- `prefixBase` returns `'HelicalGear'`, so the user parameters register under `HelicalGear<N>_`.
- `generateName` returns
  `'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)`,
  reading the four parameters through `self.getParameter(PARAM_MODULE)` and its siblings and taking
  each one's `.expression` string, never its `.value`, so units show through.

These three are overrides the inherited `base.Generator` calls; the module defines them and never
calls them itself, which is why they are exempted from the call check below.

<!-- check-step-calls: ignore newContext prefixBase generateName -->

**From:** `spec/helicalgear/instructions.md` L84, L109-112;
`spec/spurgear/instructions.md` L331-336; `.claude/skills/generate-gear/PLAYBOOK.md` L75-101

## H5 `[PROSE]` Register the HelixAngle user parameter

`addExtraPrimaryParameters(self, inputs)` overrides spur's no-op hook `[SPUR-EXTRA-PARAMS]`, which
`processInputs` calls between the input-sourced parameters and the derived ones — so `HelixAngle`
exists before `FilletRadius` refers to it in H6. The body is two calls:

- `get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')`
- `self.addParameter(PARAM_HELIX_ANGLE, helixAngle, 'rad', 'Helix angle for the helical gear')`

The input was declared with `addValueInput`, so the helper that reads it is `get_value` and not
`get_boolean` (`[PB-INPUT-READ]`). What `get_value` hands back is already a `ValueInput` fit to pass
straight to `addParameter`, and it raises on an invalid expression rather than returning nothing, so
add no ok-flag handling and no wrapping of your own around it (`[PB-GET-VALUE-CONTRACT]`).

The dialog input is in degrees and the user parameter is registered in `'rad'`. The value is
signed, and the sign is the hand of the helix; nothing rescales it and `Thickness` never enters it.

The hook is called by spur's inherited `processInputs`, not by this module, so it is exempted below.

<!-- check-step-calls: ignore addExtraPrimaryParameters -->

**From:** `spec/helicalgear/instructions.md` L28-40, L65-70, L113-114;
`spec/spurgear/instructions.md` L323-330;
`.claude/skills/generate-gear/PLAYBOOK.md` L103-118, L196-218

## H6 `[PROSE]` filletHelixFactorExpression — cos(HelixAngle)

`filletHelixFactorExpression(self)` returns the f-string `f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'`,
where the spur base returns `'1'`.

The return value is an expression string, not a number, and `createFillets` never reads it. Spur's
`registerDerivedParameters` splices it in as the last factor of the live `FilletRadius` expression
`(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>`, so the root fillet reads correctly on the
tilted tooth's transverse plane. `createFillets` then reads only the resulting parameter's numeric
`.value`, which is the numeric-snapshot rule at work (`[PB-NUMERIC-SNAPSHOT]`, spur's application
`[SPUR-F-SNAPSHOT]`): the fillet is cut at the value the parameter held at generation time, and
editing `HelixAngle` afterwards does not move an existing gear.

The hook is called by spur's inherited parameter registration, never by this module.

<!-- check-step-calls: ignore filletHelixFactorExpression -->

**From:** `spec/helicalgear/instructions.md` L31-33, L115-117;
`spec/spurgear/instructions.md` L86-88, L317-322

## H7 `[PROSE]` Offset construction plane for the twisted profile

`buildSketches(self, ctx: GenerationContext)` first calls `super().buildSketches(ctx)`, which draws
the bottom Gear Profile and runs the spur tooth generator at angle 0. It then creates one
construction plane, which is this step's single timeline entry. The recipe is
`[HELI-F-TWIST-PLANE]`:

- `self.getComponent().constructionPlanes.createInput()`
- `constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())`
- `self.getComponent().constructionPlanes.add(constructionPlaneInput)`, stored as `ctx.helixPlane`

The offset comes from its own overridable hook. `helicalPlaneOffset(self)` returns
`self.getParameterAsValueInput(PARAM_THICKNESS)` — the full `Thickness`. Keep it a separate method:
herringbone re-points this one hook at half the thickness so its mirror plane lands mid-body, and
inlining the offset into `buildSketches` would take that seam away. Note what the helper returns: a
`ValueInput.createByReal(param.value)`, a numeric snapshot of `Thickness` at generation time, not a
live parameter reference (`[PB-NUMERIC-SNAPSHOT]`, spur's application `[SPUR-F-SNAPSHOT]`). The
offset argument must be that `ValueInput`; a bare number is a runtime `TypeError`, and
`[PB-CONSTRUCTION-PLANES]` carries the `setByOffset` signature this call is written against.

The plane is left visible. Spur's `cleanup` switches the light bulb off only on the entities spur
itself created (`[SPUR-F-CLEANUP]`), and helical adds no cleanup of its own, so this plane stays lit
after the build. That is a declared delta from `[PB-HIDE-AFTER-USE]`, deliberate and faithful; a
regeneration must not add cleanup for it, and if it ever did the property would be
`isLightBulbOn = False`, never `isVisible = False`, since a construction plane does not answer to
the latter. The same holds in SketchOnly mode: the plane is still created and still left visible.

Nothing in the sketch or solid harness holds a Fusion construction plane, so no proof case reaches
this step directly. The one consequence that does reach geometry is the offset distance, and
`stepLoftTooth` measures it back off the built solid as the body's extent along the gear axis —
which is the reading that would tell a full `Thickness` from herringbone's half.

**From:** `spec/helicalgear/instructions.md` L118-126, L146-149;
`spec/helicalgear/fusion.md` L9-27, L29-42;
`spec/spurgear/instructions.md` L422; `.claude/skills/generate-gear/PLAYBOOK.md` L711-722

## H8 `[GO]` Twisted Gear Profile sketch

Still inside `buildSketches`, create the second sketch on `ctx.helixPlane` and draw the tooth into
it at the helix angle. The recipe is the second half of `[HELI-F-TWIST-PLANE]`:

- `self.createSketchObject('Twisted Gear Profile', plane=plane)`
- `SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)`
- store the sketch as `ctx.twistedGearProfileSketch`

The twist is delivered as the tooth generator's own `draw()` angle argument, read as a raw `.value`
in radians. The generator draws the whole tooth already rotated by it, in its point math, and then
confirms that rotation with the spine's angular dimension as its very last action. Those two actions
are both required and neither substitutes for the other (`[SPUR-F-ROTATE-CONFIRM]`), and the
angular dimension they turn on is the one `[SPUR-F-SPINE]` builds against a pinned +X reference for
every angle including zero. Do not draw the tooth flat and rotate the sketch geometry afterwards:
the spine dimension would then measure the unrotated angle, the sketch would no longer prove its own
twist, and Fusion can settle the tooth on a branch about 180 degrees away, which sends the loft
through the gear centre.

Helical adds no constraint of its own here, and must add none: the tooth-top arc
(`[SPUR-F-TOOTHTOP-ARC]`), the rib chain (`[SPUR-F-RIBS]`) and the flank-to-root stubs
(`[SPUR-F-FLANK-ROOT]`) are the generator's own construction, reached through this one `draw()`
call. What is new is that this is the spur generator's angle-not-zero path, which spur itself never
runs, so this is where that path is first held to full constraint. That full constraint is a
design-time property, proven on the bench before any code is written (`[PB-SKETCH-FIRST]`,
`[PB-FULL-CONSTRAINT]`); spur registers no runtime full-constraint gate and helical adds none.

The sketch closes two regions and their curve counts are a contract the next step matches on: the
tooth loop is 2 splines, 2 arcs and 2 lines, and the disc inside the root circle is 2 arcs. Both
loops exist only because every adjacency in the tooth profile is a shared `SketchPoint` rather than
two coincident ones (`[SPUR-F-SHARED-ADJACENCY]`, the profile-loop application of
`[PB-SHARE-XOR-COINCIDENT]`); a loop built from fresh points at matching coordinates is not
recognised as closed and the next step finds nothing. `draw()` also performs the step-5 anchoring
itself — it projects `ctx.anchorPoint` in and makes the sketch's own movable local origin coincident
with it, which is the projection chain `[SPUR-F-ANCHOR-CHAIN]` and the local origin
`[SPUR-F-LOCAL-ORIGIN]` — so this single call is what leaves the twisted sketch fully constrained
and riding on the user's anchor.

The sketch is created hidden by `createSketchObject` and is never shown, not by `buildSketches` and
not by spur's `cleanup`, which touches only its own three sketches (`[SPUR-F-CLEANUP]`). Reading
`sketch.profiles` for the loft works on it anyway. This is a declared, verified exception to
`[PB-HIDE-AFTER-USE]` rather than a general one — there is no shown-then-hidden phase at all — and
it holds in SketchOnly mode too, where the twisted profile is therefore not inspectable.

Proven by `stepTwistedGearProfileSketch`, which builds this sketch across sizes, rib counts, both
signs of the helix angle, a quarter turn either way, an anchor dragged off the sketch origin, and
both routes into the embedded profile — then holds the solved result to where the tooth points, to
the tooth-top arc really being a piece of the tip circle, and to the two loops' curve counts.

<!-- proof-run: proofkit.Run(sketchCases, stepTwistedGearProfileSketch) -->

**From:** `spec/helicalgear/instructions.md` L4-8, L150-155, L162-171, L199-203;
`spec/helicalgear/fusion.md` L16-19, L21-24, L29-42;
`spec/spurgear/instructions.md` L441-478, L480-484;
`spec/spurgear/fusion.md` L45-60, L69-104, L106-131, L133-173, L175-215;
`.claude/skills/generate-gear/PLAYBOOK.md` L595-607

## H9 `[GO]` Loft the tooth

`buildTooth(self, ctx: GenerationContext)` does one thing: it delegates to `self.loftTooth(ctx)`.
It does not extrude and it does not chamfer. `buildTooth` itself is called by spur's inherited
`buildMainGearBody`, never by this module, so it is exempted from the call check.

`loftTooth(self, ctx: GenerationContext)` finds the tooth loop in each sketch and lofts bottom to
top into a new body, which is this step's timeline entry. The recipe is `[HELI-F-LOFT]` and the
loft API shape it is written against is `[PB-LOFT]` — `createInput(operation)`, then one
`loftSections.add(...)` per section in loft order, then `add(input)`:

- `self.getComponent().features.loftFeatures`
- `find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)`
- `find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)`
- `lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`
- `loftInput.loftSections.add(bottomToothProfile)`
- `loftInput.loftSections.add(topToothProfile)`
- `lofts.add(loftInput)`
- `ctx.toothBody = loftResult.bodies.item(0)`, then `ctx.toothBody.name = 'Tooth Body'`

The bottom section is added first and the top second. The order is not cosmetic: the ruled walls
are built outward from the section the loft starts at, so swapping the two builds a different
solid — about 2.6 percent less volume at the default 14.5 degree helix, and 16 percent at a quarter
turn — while the twist, its sign and the centroid all come back looking right. Use the framework
helper for the profile search; do not re-implement the loop search. The helper is the shared
application of `[PB-PROFILE-MATCH]`: a profile is found by the count and type of the curves on its
loop, never by index, and the helper raises rather than falling back to a wrong profile when nothing
matches.

Both searches pass a fixed `nurbs=2, arcs=2, lines=2`, the non-embedded six-curve tooth. This
implementation never reads `ctx.toothProfileIsEmbedded` and has no embedded branch, so an embedded
low-tooth-count gear, whose flanks start inside the root circle and whose tooth loop therefore has
no lines at all, would fail to find a profile. That is faithful to the current behaviour and is a
documented limitation, not a defect to fix here.

<!-- check-step-calls: ignore buildTooth -->

Proven by `stepLoftTooth`, with `assertLoftTooth` reading the measurements back off the body: the
section spacing as the solid's extent along the gear axis, and the twist as the angle between the
two caps, which carries the sign of the helix. Three thicknesses at one helix angle hold the spec's
statement that nothing rescales the twist. The embedded cases are unmodelled there for exactly the
reason above.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepLoftTooth, assertLoftTooth) -->

**From:** `spec/helicalgear/instructions.md` L4-8, L127-128, L157-160, L168-169, L197-198;
`spec/helicalgear/fusion.md` L44-65;
`spec/spurgear/instructions.md` L218-226, L313-314;
`.claude/skills/generate-gear/PLAYBOOK.md` L151-155, L608-617, L660-664

## H10 `[PROSE]` The inherited pipeline helical must not re-implement

Helical keeps spur's entire call graph and every override boundary in it:

```
generate → processInputs → prepareTools → buildMainGearBody(buildSketches → buildTooth →
buildBody → patternTeeth → createFillets) → buildBore → chamferTeeth → cleanup
```

Only the members named in H2 to H9 are overridden. Everything else runs spur's code untouched and
must not be re-implemented, re-ordered or moved across a boundary: `processInputs`, `prepareTools`,
`buildMainGearBody`, `buildBody`, `patternTeeth`, `createFillets`, `buildBore`, `chamferTeeth`,
`cleanup`, and the whole of `SpurGearInvoluteToothDesignGenerator`. In timeline terms that is the
Tools sketch and the Extrusion End Plane, the bottom Gear Profile sketch, the body extrude, the
circular pattern and its combine, the root fillets, the optional bore cut and the optional
completed-gear chamfer. Each of those is one timeline entry owned by spur's step list and proven by
spur's proof; a helical step list that restated them would invite a re-implementation the spec
forbids.

Two of them are worth naming for what helical does to them and not for anything helical writes. The
root fillet radius carries helical's `cos(HelixAngle)` factor from H6, spliced in during spur's
parameter registration. The completed-gear chamfer scans every planar face of the finished
`ctx.gearBody` parallel to the Gear Profile plane and adds every unique boundary edge once,
excluding a bore's two circular cap edges by the positive bore radius; helical inherits that
selection with no override, and no tooth-cap edge count is involved (`[HELI-F-CHAMFER-COUNT]`). Its
final behaviour on a completed gear remains pending a Fusion session. The end-of-build cleanup is
spur's recipe unchanged (`[SPUR-F-CLEANUP]`), which is why H7's helix plane is left lit: that recipe
hides only the entities spur itself created.

None of these names is a call this module makes, so all of them are exempted from the call check.

<!-- check-step-calls: ignore processInputs prepareTools buildMainGearBody buildBody patternTeeth createFillets buildBore chamferTeeth cleanup -->

**From:** `spec/helicalgear/instructions.md` L17-21, L102-107, L130-132, L139-144;
`spec/helicalgear/fusion.md` L67-80;
`spec/spurgear/instructions.md` L270-306, L530-545;
`.claude/skills/generate-gear/PLAYBOOK.md` L256-280
