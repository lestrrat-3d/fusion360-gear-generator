# Helical gear — compiled step list

The proof for this step list is `proof/helicalgear/geometry_test.go`,
`proof/helicalgear/sketches_test.go`, `proof/helicalgear/solids_test.go` and the generated
`proof/helicalgear/zz_registrations_test.go`.

Helical is a thin specialization of the spur gear. It subclasses the spur family and reuses the
whole spur build pipeline; the steps below are only what `lib/geargen/helicalgear.py` itself
contains. Everything the spur base does — the Tools sketch, the bottom Gear Profile, the body
extrude, the circular pattern, the root fillets, the optional bore, the completed-gear chamfer and
the cleanup — is inherited and is not restated here.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/helicalgear/instructions.md` | `6c1d3b4d7aa824d90f9f0f851d4115179e754707` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `spec/spurgear/fusion.md` | `7cd4e5b0fa38dcd39cbd1b5bad1cf8489e2bc2ae` |
| `spec/spurgear/instructions.md` | `560549a350c789b6598ee49b72d3cd7b4460e214` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `aa97523b214dbc98b679e8eea73c56c5115c4e54` |

## H-01 `[PROSE]` Module layout, imports and the two exported constants

Write `lib/geargen/helicalgear.py` as a subclass module. Import explicitly — no `import *` — and
import exactly the names the module uses: from `.spurgear`, `PARAM_MODULE`, `PARAM_TOOTH_NUMBER`,
`PARAM_THICKNESS`, `SpurGearCommandInputsConfigurator`, `SpurGearGenerationContext`,
`SpurGearGenerator` and `SpurGearInvoluteToothDesignGenerator`; from `.base`, `get_value`; from
`.utilities`, `find_profile_by_curve_counts`; plus `math`, `adsk.core` and `adsk.fusion`.

**Do not import `GenerationContext`.** The overrides annotate their parameter as
`SpurGearGenerationContext` and narrow with an assertion, so nothing in this module names the base
type, and importing it would leave an unused name that invites the wrong annotation (see H-08).

Declare two module-level constants, whose exact string values herringbone imports by name:
`PARAM_HELIX_ANGLE = 'HelixAngle'` and `INPUT_ID_HELIX_ANGLE = 'helixAngle'`.

The module defines exactly three classes, each extending its spur counterpart:
`HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator)`,
`HelicalGearGenerationContext(SpurGearGenerationContext)` and
`HelicalGearGenerator(SpurGearGenerator)`. There is no standalone generator.

Use `adsk.core` for value and geometry types and `adsk.fusion` for feature and topology types
([PB-ADSK-MODULES]); this module needs `adsk.core.ValueInput` in H-02 and
`adsk.fusion.FeatureOperations` in H-10.

**From:** `spec/helicalgear/instructions.md` L1–8, L70–92; `spec/helicalgear/fusion.md` L1–5;
`.claude/skills/generate-gear/PLAYBOOK.md` L17–40, L435.

## H-02 `[PROSE]` Add the Helix Angle dialog input

`HelicalGearCommandConfigurator.configure` is a `@classmethod` taking `(cls, cmd)`. It calls
`super().configure(cmd)` first, so every spur input is added, and then appends its own input:

`cmd.commandInputs.addValueInput('helixAngle', 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(14.5)))`

The display unit is degrees and the default is 14.5°, but a `createByReal` default is read in
Fusion's internal units, so the default is written as radians ([PB-DIALOG-DEFAULT-UNITS]).

⚠️ Because spur's `configure` already added Parent Component last, the Helix Angle input
necessarily lands **last in the dialog, after Parent Component**. That is the current behavior and
it is reproduced exactly; do not try to insert Helix Angle earlier in the list. This is the
subclass extension seam ([SPUR-SUBCLASS-INPUT]): the configurator adds the dialog input, and the
hook in H-05 registers the matching user parameter.

**From:** `spec/helicalgear/instructions.md` L51–64, L74–79, L210–215;
`spec/spurgear/instructions.md` L171–178;
`.claude/skills/generate-gear/PLAYBOOK.md` L103–119, L128–137.

## H-03 `[PROSE]` Extend the generation context with two fields

`HelicalGearGenerationContext.__init__(self)` calls `super().__init__()` and then initialises the
two fields helical adds, each to a cast-`None` so the field exists and carries its type:

- `self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)` — the offset construction plane the
  twisted top profile is drawn on, and the plane herringbone later mirrors across.
- `self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)` — the second, "Twisted Gear
  Profile" sketch, which is the top loft section.

Every spur context field is inherited unchanged and none is restated here; subclasses read these
two by name, so neither may be renamed.

**From:** `spec/helicalgear/instructions.md` L80–83, L94–102.

## H-04 `[PROSE]` Generator identity: context, prefix and component name

`HelicalGearGenerator` overrides three identity hooks. The spur base calls each of them; this
module only defines them.

<!-- check-step-calls: ignore newContext prefixBase generateName -->

- `newContext()` returns a `HelicalGearGenerationContext()`, which is what makes the narrowing
  assertion in H-08 and H-10 hold at runtime.
- `prefixBase()` returns `'HelicalGear'`, so every registered user parameter reads
  `HelicalGear<id>_…`.
- `generateName()` returns
  `'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)`
  — the four parameters read with `self.getParameter(PARAM_MODULE)` and its siblings, and rendered
  from each parameter's **`.expression`** string rather than its `.value`, so units show through.
  The fourth term is spur's three-term name extended with `HelixAngle`.

`newContext`, `prefixBase` and `generateName` are named here as the methods this module defines for
the inherited pipeline to call, not as calls this module makes, so they carry the exemption above.

**From:** `spec/helicalgear/instructions.md` L84–85, L110–114;
`spec/spurgear/instructions.md` L331–335.

## H-05 `[PROSE]` Register the HelixAngle user parameter

Override the `addExtraPrimaryParameters(self, inputs)` hook, which is a no-op on the spur base and
which `processInputs` calls between the input-sourced parameters and the derived ones
([SPUR-EXTRA-PARAMS]). Read the dialog input and register the parameter:

<!-- check-step-calls: ignore addExtraPrimaryParameters -->

`helixAngle = get_value(inputs, 'helixAngle', 'rad')` then
`self.addParameter('HelixAngle', helixAngle, 'rad', 'Helix angle for the helical gear')`.

The input was declared with `addValueInput`, so it is read with `get_value`, which already returns
a `ValueInput` ready to hand straight to `addParameter` ([PB-INPUT-READ], [PB-GET-VALUE-CONTRACT]).
The dialog is in degrees and the parameter is registered in **`'rad'`**.

The value is **signed, and the sign is the hand of the helix**: a negative value is a left-hand
helix, the dialog accepts it, and nothing rescales it. No range is enforced and none is asserted —
do not add a clamp, a warning or a documented maximum. Registering the parameter here, before the
derived parameters, is what lets H-06 name it inside a live expression.

`addExtraPrimaryParameters` is the method this module defines for `processInputs` to call, not a
call this module makes, so it carries the exemption above.

**From:** `spec/helicalgear/instructions.md` L23–49, L53–58, L65–68, L115–116;
`spec/spurgear/instructions.md` L323–330;
`.claude/skills/generate-gear/PLAYBOOK.md` L103–127.

## H-06 `[PROSE]` Multiply the root-fillet radius by cos(HelixAngle)

Override `filletHelixFactorExpression(self)` to return
`f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'`, where the spur base returns `'1'`.

<!-- check-step-calls: ignore filletHelixFactorExpression -->

This is an expression string, not a number, and it is consumed exactly once — in
`registerDerivedParameters`, spliced in as the last factor of the live `FilletRadius` expression
`(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>`. The root fillet then reads correctly on
the transverse plane of a tilted tooth. `createFillets` never calls this hook; it reads only the
resulting `FilletRadius` parameter's numeric value, so nothing else in helical touches the fillets.

`filletHelixFactorExpression` is the method this module defines for the inherited parameter
registration to call, not a call this module makes, so it carries the exemption above.

**From:** `spec/helicalgear/instructions.md` L31–33, L117–119;
`spec/spurgear/instructions.md` L86–88, L317–322.

## H-07 `[PROSE]` The twisted-profile plane offset hook

Define `helicalPlaneOffset(self)` returning `self.getParameterAsValueInput(PARAM_THICKNESS)` — the
distance from the base plane to the plane the twisted profile is drawn on, as a `ValueInput`.
Helical returns the **full** thickness.

**Keep this its own method.** It is a distinct overridable hook — herringbone re-points it to half
the thickness so its mirror plane lands mid-body — so the offset must not be inlined into H-08.

The returned value is a **numeric snapshot**, not a live parameter reference:
`getParameterAsValueInput` wraps the `Thickness` parameter's value at generation time with
`adsk.core.ValueInput.createByReal(param.value)`. Editing `Thickness` afterwards does not move the
plane; the gear is regenerated instead ([PB-NUMERIC-SNAPSHOT], [SPUR-F-SNAPSHOT]).

**From:** `spec/helicalgear/instructions.md` L120–125; `spec/helicalgear/fusion.md` L21–27;
`spec/spurgear/fusion.md` L231–238;
`.claude/skills/generate-gear/PLAYBOOK.md` L220–229.

## H-08 `[PROSE]` Offset construction plane for the twisted profile

This is the first half of the `buildSketches(self, ctx)` override and one entry in the Fusion
timeline. Declare the parameter as `ctx: SpurGearGenerationContext`, which is what the inherited
signature declares, and narrow it at the top of the body with an `assert` on
`HelicalGearGenerationContext`:

```python
def buildSketches(self, ctx: SpurGearGenerationContext):
    assert isinstance(ctx, HelicalGearGenerationContext)
```

⚠️ Do not annotate the parameter as `GenerationContext`, which is wider than the inherited
signature, and do not narrow the annotation to `HelicalGearGenerationContext`, which is narrowing a
parameter in an override. The annotation matches the base and the assertion does the narrowing;
every read and write of `ctx.helixPlane` or `ctx.twistedGearProfileSketch` happens after it.

Call `super().buildSketches(ctx)` **first**, which draws the bottom Gear Profile and runs the spur
tooth generator at angle 0. Then create the offset plane on the gear's own component:

- `constructionPlaneInput = self.getComponent().constructionPlanes.createInput()`
- `constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())` — the offset argument
  is a `ValueInput`, never a bare number ([PB-CONSTRUCTION-PLANES]).
- `plane = self.getComponent().constructionPlanes.add(constructionPlaneInput)`
- store it as `ctx.helixPlane`.

⚠️ **The helix plane is left visible after generation, deliberately.** Spur's cleanup switches off
only the entities spur itself created, and helical adds no cleanup of its own, so this plane stays
lit. A regeneration must not add cleanup for it ([HELI-F-TWIST-PLANE]). In
Generate-Sketches-Only mode the plane is still created and still left visible.

**From:** `spec/helicalgear/instructions.md` L126–128, L136–156, L158–169, L216–222;
`spec/helicalgear/fusion.md` L7–20, L28–42; `spec/spurgear/instructions.md` L310–312;
`.claude/skills/generate-gear/PLAYBOOK.md` L711–722.

## H-09 `[GO]` Twisted Gear Profile sketch

The second half of `buildSketches`, and one entry in the Fusion timeline. On the plane H-08 built,
create the sketch and draw the tooth into it already rotated by the helix angle:

- `loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)`
- `SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)`
- store the sketch as `ctx.twistedGearProfileSketch`.

The twist is delivered as the tooth generator's own `draw()` `angle` argument, read as a raw
radian `.value`. The generator draws the whole tooth already rotated by it in its point math and
then confirms that rotation with the spine's angular dimension ([SPUR-F-ROTATE-CONFIRM],
[SPUR-F-SPINE]). Drawing the tooth flat and rotating the sketch geometry afterwards also produces a
twisted profile, but leaves the spine dimension measuring the unrotated angle, so the sketch no
longer proves its own twist — and it lets Fusion pick the half-turn-off branch, which sends the
loft through the gear centre. Helical does nothing else here: it passes the angle and the shared
generator does the rest, including the anchoring, which happens inside `draw()` itself.

⚠️ **This sketch stays hidden its whole life.** `createSketchObject` returns a hidden sketch and
nothing ever shows it — not this method, and not spur's cleanup, which touches only its own three
sketches. Reading `profiles` off it for the loft in H-10 works anyway. This is a declared delta
from [PB-HIDE-AFTER-USE]: there is no "shown, then hidden after use" phase, because it is never
shown at all, including in Generate-Sketches-Only mode, where the twisted profile is therefore not
inspectable ([HELI-F-TWIST-PLANE]).

⚠️ **No runtime full-constraint gate.** Spur registers none and helical adds none. That the
twisted profile fully constrains is a design-time property, proved by `stepTwistedGearProfile` and
by `spec/helicalgear/sketch/`, not asserted in the generated code.

Proved by `stepTwistedGearProfile`, which draws this sketch in the sketch engine across both hands
of the helix, the sizes, the sample counts and both routes into the embedded shape, and holds it to
the harness's whole verdict — DOF 0, no redundant or conflicting constraint, valid profiles, a
system that is not near-singular and no discrete ambiguity, with nothing waived. It reads the twist
back off the solved sketch as the signed angle from the +X reference to the spine, and it counts
the curves on the two loops the sketch closes, because those counts are what H-10 selects on.

<!-- proof-run: proofkit.Run(twistedProfileCases, stepTwistedGearProfile) -->

**From:** `spec/helicalgear/instructions.md` L126–128, L165–174, L180–190, L218–222;
`spec/helicalgear/fusion.md` L9–27, L29–42; `spec/spurgear/fusion.md` L45–60, L106–131;
`spec/spurgear/instructions.md` L337–355, L441–478, L480–484;
`.claude/skills/generate-gear/PLAYBOOK.md` L595–607.

## H-10 `[GO]` Loft the tooth body

`buildTooth(self, ctx)` is overridden to call `self.loftTooth(ctx)` and nothing else — helical does
not extrude and does not chamfer here. Both methods take `ctx: SpurGearGenerationContext` and
narrow it with the same assertion as H-08.

<!-- check-step-calls: ignore buildTooth -->

`loftTooth(self, ctx)` is one entry in the Fusion timeline. Find the tooth loop in each sketch and
loft the bottom one to the twisted top one into a new body:

- `lofts = self.getComponent().features.loftFeatures`
- `bottomToothProfile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)`
- `topToothProfile = find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)`
- `loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`
- `loftInput.loftSections.add(bottomToothProfile)`, then `loftInput.loftSections.add(topToothProfile)`
- `loftResult = lofts.add(loftInput)`
- `ctx.toothBody = loftResult.bodies.item(0)` and name it `'Tooth Body'`.

**Add the bottom section first, then the top.** Adding them in the other order also lofts a valid
solid and does not flip the hand — the same twist, with the same sign, comes back off either body —
but it rebuilds the ruled walls from the other section and changes the volume with them. The order
is silent in the one reading a caller is most likely to check, which is why it is pinned here
rather than left to the implementation.

Use the framework helper `find_profile_by_curve_counts` and do not re-implement the loop search
([PB-PROFILE-MATCH]). The loft input shape is [PB-LOFT]: create the input with the operation, add
each section in order, then add the input.

⚠️ **Non-embedded only.** Both sections pass a fixed `nurbs=2, arcs=2, lines=2`, the six-curve
tooth loop. This implementation does not read `ctx.toothProfileIsEmbedded` and has no embedded
branch, so a low-module, high-tooth-count gear whose flank starts inside the root circle draws no
flank-to-root lines and the profile search finds nothing. That is faithful to the current code and
a documented limitation, not a bug to fix here ([HELI-F-LOFT]).

`buildTooth` is the method this module defines for the inherited `buildMainGearBody` to call, not a
call this module makes, so it carries the exemption above.

Proved by `stepLoftTooth`, which lofts the two sections in `decad` and returns the Tooth Body, and
by `assertLoftTooth`, which measures it. The gate is the bounded-solid one: the body has to come
back solid, watertight, manifold, free of self-intersection, one lump, no voids. The assertion pins
the span from the base plane to the helix plane at the full Thickness, so H-07's hook is checked
rather than assumed; it pins the centroid at half the helix angle, sign included, which is where
the twist and its hand are read back; and it builds the loft a second time with the sections
swapped, to show that the swap keeps the twist and moves the volume. The embedded case and the
twists past the proof's own measured loft bound are skipped by name, and the bound is recorded per
sign in the proof file, because the two signs do not fail alike.

<!-- proof-run: proofkit3d.RunSolid(loftCases, stepLoftTooth, assertLoftTooth) -->

**From:** `spec/helicalgear/instructions.md` L129–130, L176–179, L186–188, L216–217;
`spec/helicalgear/fusion.md` L44–65; `spec/spurgear/instructions.md` L313–314, L490–494;
`.claude/skills/generate-gear/PLAYBOOK.md` L608–617, L660–664.

## H-11 `[PROSE]` Everything else is inherited, and nothing else is cleaned up

Helical overrides the methods in H-04 through H-10 and nothing more. Do **not** re-implement
`processInputs`, `prepareTools`, `buildMainGearBody`, `buildBody`, `patternTeeth`, `createFillets`,
`buildBore`, `chamferTeeth`, `cleanup`, or any part of `SpurGearInvoluteToothDesignGenerator`, and
do not move work across the inherited call-graph boundaries: `generate → processInputs →
prepareTools → buildMainGearBody(buildSketches → buildTooth → buildBody → patternTeeth →
createFillets) → buildBore → chamferTeeth → cleanup`.

<!-- check-step-calls: ignore processInputs prepareTools buildMainGearBody buildBody patternTeeth createFillets buildBore chamferTeeth cleanup -->

The completed-gear chamfer is inherited without an override. The shared `chamferTeeth` runs after
patterning, the root fillets and the optional bore; it scans every planar face of the completed
gear body parallel to the Gear Profile plane and adds every unique boundary edge once, root-radius
arcs included, excluding a bore's two circular cap edges by the positive bore radius. Its earlier
tooth-cap edge counts are gone. That selection is still pending Fusion verification
([HELI-F-CHAMFER-COUNT]).

Helical adds no cleanup. The two visibility facts that follow are deliberate and must be
reproduced, not tidied up: the helix construction plane stays visible, and the Twisted Gear Profile
sketch stays hidden from creation onwards.

The nine method names above are named as the inherited work this module must leave alone, not as
calls it makes, so they carry the exemption above.

**From:** `spec/helicalgear/instructions.md` L17–21, L104–109, L132–134, L158–163, L192–208,
L210–222; `spec/helicalgear/fusion.md` L29–42, L67–80; `spec/spurgear/instructions.md` L279–316,
L530–545; `spec/spurgear/fusion.md` L217–229.
