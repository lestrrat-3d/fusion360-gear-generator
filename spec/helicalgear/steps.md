# Helical gear — compiled step list

The proof for these steps is `proof/helicalgear/tooth_test.go`, `proof/helicalgear/sketches_test.go`,
`proof/helicalgear/solids_test.go` and the generated `proof/helicalgear/zz_registrations_test.go`.

Helical is a subclass-only module. It inherits spur's whole build — the Tools sketch and anchor
chain, the Gear Profile sketch, the body extrude, the circular pattern, the root fillets, the bore
and the cleanup — and changes three things: it adds a Helix Angle input, draws a second "Twisted
Gear Profile" sketch on an offset plane, and lofts the bottom profile to that twisted top profile
instead of extruding. Steps H1 to H5 build the module surface and register the one new parameter;
H6 to H9 are the four timeline-visible changes.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/helicalgear/instructions.md` | `08d1959f0fd39d217c6a2bafe4f5421ff5e2e273` |
| `spec/helicalgear/fusion.md` | `83fac920272341e3c4584f16031478a69b7472e7` |
| `spec/spurgear/fusion.md` | `ea678245854cfec80055d67c46a8788772b0f9d4` |
| `spec/spurgear/instructions.md` | `f5ffe3451454bb3b187b1318e47b92281d9f0bb0` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `dadae022d2220a73b25e07b24ef99075a8be23a5` |

## H1 `[PROSE]` Module surface: imports, constants and the three subclasses

Write `lib/geargen/helicalgear.py` as a subclass module. It defines no standalone generator, no
tooth generator of its own, and no `generate`; every method not listed in H2 to H9 is inherited
from spur unchanged and must NOT be re-implemented — `processInputs`, `prepareTools`,
`buildMainGearBody`, `buildBody`, `patternTeeth`, `createFillets`, `chamferTooth`, `buildBore`,
`cleanup`, and the entire `SpurGearInvoluteToothDesignGenerator`.

Module-level constants, both public API:

- `PARAM_HELIX_ANGLE = 'HelixAngle'`
- `INPUT_ID_HELIX_ANGLE = 'helixAngle'`

Imports are explicit, never `import *` (PLAYBOOK module-layout rule). From `.spurgear`:
`PARAM_MODULE`, `PARAM_TOOTH_NUMBER`, `PARAM_THICKNESS`, `SpurGearCommandInputsConfigurator`,
`SpurGearGenerationContext`, `SpurGearGenerator`, `SpurGearInvoluteToothDesignGenerator`. From
`.base`: `GenerationContext`, `get_value`. From `.utilities`: `find_profile_by_curve_counts`. Plus
`math`, `adsk.core`, `adsk.fusion`.

Three classes, each extending its spur counterpart. Herringbone subclasses all three and
`commands/helicalgear/entry.py` binds two by name, so these names are the reproduced surface:

1. `HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator)` — H2.
2. `HelicalGearGenerationContext(SpurGearGenerationContext)` — H4.
3. `HelicalGearGenerator(SpurGearGenerator)` — H3 to H9.

`GenerationContext` is imported because the three `ctx`-taking overrides annotate their parameter
as `ctx: GenerationContext`, matching the inherited signatures; keep the annotation so the import
is used.

No API call is required of this step. The names above are declarations, not calls.

**From:** `spec/helicalgear/instructions.md` L3–8, L54, L56–74, L118–125, L160–175;
`.claude/skills/generate-gear/PLAYBOOK.md` L17–41, L42–74, L256–280

## H2 `[PROSE]` Helix Angle dialog input

`HelicalGearCommandConfigurator.configure` is a `@classmethod` taking `(cls, cmd)`. It calls
`super().configure(cmd)` first, then appends one value input:

`cmd.commandInputs.addValueInput('helixAngle', 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(14.5)))`

The unit string is `'deg'` and the default is passed in Fusion internal units, which for an angle
is radians — `math.radians(14.5)`, not `14.5` ([PB-DIALOG-DEFAULT-UNITS]). Writing the bare number
ships a 14.5-radian default in a dialog that displays degrees, and no gate catches it.

Because `super().configure(cmd)` has already added Parent Component last, Helix Angle necessarily
lands **after** Parent Component in the dialog. That is the current behaviour and is reproduced
exactly: do not try to insert Helix Angle earlier ([SPUR-SUBCLASS-INPUT]).

`configure` is named here on two sides. The module both defines it and calls the base class's, and
only the call is a requirement; the definition is the extension seam.

<!-- check-compile: ignore configure -->

**From:** `spec/helicalgear/instructions.md` L39–48, L62–64, L182;
`spec/spurgear/instructions.md` L108–129, L171–178;
`.claude/skills/generate-gear/PLAYBOOK.md` L128–136

## H3 `[PROSE]` Register the HelixAngle user parameter

`HelicalGearGenerator` overrides `addExtraPrimaryParameters(self, inputs)`, which spur's base
declares as a no-op and `processInputs` calls between the input-sourced parameters and the derived
ones ([SPUR-EXTRA-PARAMS]). The override reads the dialog input and registers the parameter:

- `get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')`
- `self.addParameter(PARAM_HELIX_ANGLE, helixAngle, 'rad', 'Helix angle for the helical gear')`

The dialog is in degrees and the parameter is registered in **radians**. The read helper is
`get_value` because the input was declared with `addValueInput` ([PB-INPUT-READ]).

The hook must be registered here rather than anywhere later, because the derived parameters
registered after it reference `HelixAngle` — see H5.

`addExtraPrimaryParameters` is a method this module defines for the inherited `processInputs` to
call. It is named so the emitted module declares it, not so the module calls it.

<!-- check-compile: ignore addExtraPrimaryParameters -->
<!-- check-step-calls: ignore addExtraPrimaryParameters -->

**From:** `spec/helicalgear/instructions.md` L29–33, L49–52, L97–98;
`spec/spurgear/instructions.md` L324–331;
`.claude/skills/generate-gear/PLAYBOOK.md` L104–118

## H4 `[PROSE]` Generation context, component name and parameter prefix

`HelicalGearGenerationContext.__init__` calls `super().__init__()` and then initialises the two
new fields to a cast-None:

- `adsk.fusion.ConstructionPlane.cast(None)` → `ctx.helixPlane`, the offset plane the twisted top
  profile is drawn on, and the plane herringbone later mirrors across.
- `adsk.fusion.Sketch.cast(None)` → `ctx.twistedGearProfileSketch`, the top loft section.

Every spur context field is inherited unchanged; do not restate the list, which drifts.

`HelicalGearGenerator` overrides three naming hooks:

- `newContext()` returns a `HelicalGearGenerationContext()`.
- `prefixBase()` returns `'HelicalGear'`.
- `generateName()` returns
  `'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)`
  — the four parameters' `.expression` strings, never `.value`, so units show through. This is
  spur's rule extended with `HelixAngle`.

`newContext`, `prefixBase` and `generateName` are methods this module defines for the inherited
`generate` to call, so they are named here as declarations rather than as calls the module makes.
`__init__` is the same: the module defines it and calls the base class's.

<!-- check-compile: ignore newContext prefixBase generateName __init__ -->
<!-- check-step-calls: ignore newContext prefixBase generateName -->

**From:** `spec/helicalgear/instructions.md` L65–67, L76–85, L93–96;
`spec/spurgear/instructions.md` L253–268, L332–336

## H5 `[PROSE]` Root-fillet transverse factor

`HelicalGearGenerator` overrides `filletHelixFactorExpression(self)` to return
`f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'`, where spur's base returns `'1'`.

The string is not read by `createFillets`. It is consumed exactly once, in
`registerDerivedParameters`, spliced in as the last factor of the live `FilletRadius` expression
`(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>`. `createFillets` then reads only the
resulting parameter's numeric `.value`. Multiplying by `cos(HelixAngle)` is what makes the root
fillet read correctly on the transverse plane of a tilted tooth.

This step has no geometry of its own to build. What it produces is one factor inside a Fusion
parameter expression, evaluated by Fusion's own expression engine; there is no sketch and no solid
for either harness to hold it to. The value it scales — the fillet radius on the finished gear —
is applied by `createFillets`, which helical inherits unchanged and does not modify.

`filletHelixFactorExpression` is a method this module defines for spur's parameter registration to
call.

<!-- check-compile: ignore filletHelixFactorExpression -->
<!-- check-step-calls: ignore filletHelixFactorExpression -->

**From:** `spec/helicalgear/instructions.md` L31–33, L99–101;
`spec/spurgear/instructions.md` L86–88, L316–323

## H6 `[GO]` Twisted-profile construction plane

The first of `buildSketches`'s two additions, and one timeline entry: an offset construction plane
on the gear's own component, created after `super().buildSketches(ctx)` has drawn the bottom Gear
Profile and run the tooth generator at angle 0.

- `self.getComponent().constructionPlanes.createInput()`
- `constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())`
- `self.getComponent().constructionPlanes.add(constructionPlaneInput)`

Store the result as `ctx.helixPlane`.

The offset comes from `helicalPlaneOffset(self)`, a **distinct overridable hook** that helical
implements as `self.getParameterAsValueInput(PARAM_THICKNESS)` — the full Thickness. Keep it its
own method and do not inline the offset: herringbone re-points this same hook at half the
thickness so its mirror plane lands mid-body, and an inlined offset gives herringbone nothing to
override.

The offset is a **numeric snapshot**, not a live parameter reference:
`self.getParameterAsValueInput(PARAM_THICKNESS)` hands back
`adsk.core.ValueInput.createByReal(param.value)`, the Thickness as it stood at generation time
([PB-NUMERIC-SNAPSHOT], [SPUR-F-SNAPSHOT]). Editing the parameter afterwards does not move the
plane; regenerate. The second argument of `setByOffset` is a `ValueInput`, never a bare number
([PB-CONSTRUCTION-PLANES]).

The plane is **left visible** after the build. Spur's `cleanup` turns the light bulb off only on
the entities spur itself created — the Extrusion End Plane, the normalized target plane and the
`Gear Center` axis — and helical adds no cleanup of its own, so the offset plane stays lit. This is
deliberate and reproduced, not tidied up ([HELI-F-TWIST-PLANE]). In SketchOnly mode the plane is
still created and still left lit.

The proof function is `stepHelixPlane`. decad has no construction plane to hand a gate and a
proofkit3d build must return bodies, so the plane is proven by what is built on it: the twisted
section is extruded a fixed witness height off the plane and the body's near face reports where the
plane sits. `assertHelixPlane` requires that face at the full Thickness and says explicitly that
half the Thickness — herringbone's offset — is not it. The cases sweep the thickness, including the
dialog default of 10 mm, at values whose halves are distinct. What the substitution costs is the
plane's lifecycle: its visibility after the build has no counterpart in decad and is unproven here.

<!-- proof-run: proofkit3d.RunSolid(helixPlaneCases, stepHelixPlane, assertHelixPlane) -->

`helicalPlaneOffset` is named twice over: the module defines it, and `buildSketches` calls it. Only
the call is a requirement. `buildSketches` itself is H7's.

<!-- check-compile: ignore helicalPlaneOffset -->

**From:** `spec/helicalgear/instructions.md` L83–84, L103–112, L133–137, L190–193;
`spec/helicalgear/fusion.md` L9–27, L29–42;
`.claude/skills/generate-gear/PLAYBOOK.md` L220–228, L699–710

## H7 `[GO]` Twisted Gear Profile sketch

The second addition inside `buildSketches`, and one timeline entry: the sketch named
`'Twisted Gear Profile'`, drawn on `ctx.helixPlane` and stored as `ctx.twistedGearProfileSketch`.

`buildSketches(self, ctx: GenerationContext)` calls `super().buildSketches(ctx)` first — that is
what draws the bottom Gear Profile and runs the spur tooth generator at angle 0 — then:

- `self.createSketchObject('Twisted Gear Profile', plane=plane)`
- `SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)`

The twist is delivered as `draw()`'s `angle` argument and nothing else. The spur tooth generator
rotates the whole tooth by that angle in its own point math and then confirms the rotation with the
spine's angular dimension; do not draw the tooth flat and rotate it afterwards
([SPUR-F-ROTATE-CONFIRM], [SPUR-F-SPINE]). The angle is read as a raw `.value`, which is radians.
The anchor passed in is `ctx.anchorPoint`, the Tools-sketch projection, so this sketch joins the
same projection chain every other sketch is on and the twisted profile follows the user's anchor
([SPUR-F-ANCHOR-CHAIN]). `draw()` performs the anchoring itself, which is why the single call is
enough.

The sketch **stays hidden its whole life**. `createSketchObject` returns a hidden sketch and
nothing ever shows it — not `buildSketches`, not spur's `cleanup`, and not SketchOnly mode, where
the twisted profile is therefore not inspectable. Reading `sketch.profiles` for the loft works on
the hidden sketch. This is a declared delta from [PB-HIDE-AFTER-USE] — there is no shown-then-
hidden phase, it is never shown at all ([HELI-F-TWIST-PLANE]).

Both this sketch and the bottom Gear Profile close the **non-embedded six-curve tooth**: 2 NURBS
flanks, 2 arcs, 2 flank-to-root lines. That count is the key H8's profile search matches on, so it
is a contract of this step and not a detail of it.

The proof function is `stepTwistedGearProfileSketch`. It reproduces the whole constraint scheme —
the four circles sharing the local origin, the fitted-spline flanks drawn already rotated, the
tooth-top arc centred on the origin, the spine with its +X reference and confirming angular
dimension, one rib per fit-point index in [SPUR-F-RIBS] order, and the two flank-to-root lines —
and gates it on the engine's own verdict, which asks for more than DOF 0: no redundant or
conflicting constraint, valid profiles, a system that is not near-singular, and no discrete
ambiguity. It then counts the curves on the loop the sketch actually closed, with the root circle
solid and split by the stubs, and requires nurbs=2, arcs=2, lines=2.

The case table is spur's own regime, because the twisted profile IS the spur tooth at a non-zero
angle: several module and tooth-count pairs; the whole signed range of the angle, with a left-hand
(negative) helix beside its positive twin, zero as the spur baseline, and a quarter turn each way
where the rib and chain dimensions swap axes; a four-sample case at the low end of the rib count;
and both routes into the embedded shape, by tooth count and by pressure angle, where the loop comes
out four curves with no lines and H8's fixed six-curve search can match nothing.

<!-- proof-run: proofkit.Run(twistedProfileCases, stepTwistedGearProfileSketch) -->

`buildSketches` is named on both sides: the module defines the override, and the override calls the
base class's. Only the call is a requirement.

<!-- check-compile: ignore buildSketches -->

**From:** `spec/helicalgear/instructions.md` L84, L110–112, L122–125, L133–142, L148–157, L188–193;
`spec/helicalgear/fusion.md` L9–27, L29–42;
`spec/spurgear/instructions.md` L180–251, L306–312, L338–379, L442–479, L481–485;
`spec/spurgear/fusion.md` L19–31, L33–43, L47–60, L69–87, L89–114, L116–156, L158–198;
`.claude/skills/generate-gear/PLAYBOOK.md` L350–428, L583–595

## H8 `[GO]` Loft the tooth

`buildTooth(self, ctx: GenerationContext)` replaces spur's tooth extrude. It calls
`self.loftTooth(ctx)` and then, as its last action, `self.chamferTooth(ctx)` — spur's boundary,
which `buildMainGearBody` relies on because it does not chamfer separately. `buildTooth` does not
extrude anything.

`loftTooth(self, ctx: GenerationContext)` is one timeline entry:

- `find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)` — the bottom
  section.
- `find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)` — the top
  section.
- `lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`
- `loftInput.loftSections.add(bottomToothProfile)` then `loftInput.loftSections.add(topToothProfile)`
- `lofts.add(loftInput)`
- `loftResult.bodies.item(0)` → `ctx.toothBody`, named `'Tooth Body'`.

The collection is `self.getComponent().features.loftFeatures` ([PB-LOFT]). Add the **bottom**
section first and the top second: `loftSections.add` order is the loft order, and reversing it
turns a right-hand helix into a left-hand one.

Use the framework helper for both searches; do not re-implement the loop search
([PB-PROFILE-MATCH]).

Both searches pass a fixed `nurbs=2, arcs=2, lines=2`. This implementation does **not** read
`ctx.toothProfileIsEmbedded` and has no embedded branch, so an embedded low-tooth-count gear — one
whose flanks start inside the root circle and whose loop therefore has four curves and no lines —
fails to find a profile. That is a documented limitation faithful to the current code, not a bug to
fix here ([HELI-F-LOFT]).

The proof function is `stepLoftTooth`. `assertLoftTooth` measures the three things the loft
produces: a body spanning the target plane at z=0 to the helix plane at the full Thickness, two cap
sections that correspond edge for edge, and a top section turned relative to the bottom by exactly
the helix angle, signed — so a reversed section order fails. The gate additionally requires the
body to be a solid, watertight, manifold, non-self-intersecting single lump with no voids. The
cases sweep the sign and size of the twist including zero, three sizes and two thicknesses.

Both loft sections in the proof are polylines. decad's Loft pairs recorded segments and admits only
two LineSegs, two ArcSegs or two CircleSegs, so a fitted-spline flank cannot cross it at all, and
an arc pair's internally chorded walls leave a volume bound the solid gate refuses. What the
substitution costs is the flank and tip surfaces: the loft proof pins the twist, the height, the
section correspondence and the topology, and not the shape of the walls. The flank shape is pinned
instead by H7, which draws the real fitted spline, and the six-curve count both sections are found
by is asserted there.

<!-- proof-run: proofkit3d.RunSolid(loftCases, stepLoftTooth, assertLoftTooth) -->

`loftTooth` and `buildTooth` are each named on both sides: the module defines them, and
`buildTooth` calls `loftTooth` and `chamferTooth`. The two calls are requirements; the two
definitions are not.

<!-- check-compile: ignore loftTooth buildTooth chamferTooth -->
<!-- check-step-calls: ignore buildTooth -->

**From:** `spec/helicalgear/instructions.md` L6–8, L113–116, L122–125, L144–146, L154–155, L187–188;
`spec/helicalgear/fusion.md` L46–65;
`spec/spurgear/instructions.md` L313–318, L491–495;
`.claude/skills/generate-gear/PLAYBOOK.md` L596–605, L648–652

## H9 `[GO]` Chamfer front-face edge count

`chamferWantEdges(self)` returns `4`, where spur's base returns `6`. Helical does **not** override
`chamferTooth` itself and adds no content filter of its own; it changes only this number.

The inherited `chamferTooth` runs from the end of H8 and picks the tooth's front face by a single
conjunction on the same face: `face.edges.count == chamferWantEdges()` **and** the face is coplanar
with the gear's sketch plane. If no face satisfies both, it raises; there is no partial-match
fallback.

**This value is flagged in the spec as asserted and unverified, and is reproduced verbatim rather
than fixed.** The tooth helical lofts is the non-embedded six-curve profile, so its front cap face
carries six edges, and a wanted count of four matches nothing. The concrete failure mode: with a
chamfer value above 0 on a default helical gear, `chamferTooth` raises the front-face-not-found
error, the command's execute handler catches it and calls `deleteComponent()`, and the whole new
component is rolled back — an abort, not a skipped chamfer. Helical chamfer is therefore exercised
only at the default chamfer of 0. Do not soften the raise into a skip
([HELI-F-CHAMFER-COUNT]).

The proof function is `stepChamferFrontFace`. It builds the tooth whose front face that search runs
over and `assertChamferFrontFace` counts the face's edges: six on every non-embedded case, which is
the shape helical actually lofts, and four on the embedded cases, which is the shape a wanted count
of four does fit and the one H8's profile search can never find. That pairing is the measurement
behind the flag.

The body is an extrude of the real six-curve tooth rather than the loft, because the count means
something only while each flank is a single edge, which H8's chorded sections give up. decad
refuses the same tooth past eight involute samples with a fixed free-form work budget, so the cases
run at four to eight samples against a shipped `InvoluteSteps` of 15; the cap-face edge count does
not vary with the sample count. What no harness here reaches is the Fusion control flow around the
raise — the caught exception and the component rollback. What is proven is the premise it rests on,
that the two counts do not match.

<!-- proof-run: proofkit3d.RunSolid(chamferCases, stepChamferFrontFace, assertChamferFrontFace) -->

`chamferWantEdges` is a method this module defines for the inherited `chamferTooth` to call, so it
is named as a declaration and not as a call the module makes. `deleteComponent` is named only to
describe what the command entry point does when the raise reaches it; helical never calls it.

<!-- check-compile: ignore chamferWantEdges -->
<!-- check-step-calls: ignore chamferWantEdges deleteComponent -->

**From:** `spec/helicalgear/instructions.md` L102, L113–115, L184–186;
`spec/helicalgear/fusion.md` L67–84;
`spec/spurgear/instructions.md` L497–503
