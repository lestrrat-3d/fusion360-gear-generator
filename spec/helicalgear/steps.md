# Helical Gear — compiled step list

The proof for these steps is `proof/helicalgear/tooth_test.go`, `proof/helicalgear/sketches_test.go`,
`proof/helicalgear/solids_test.go` and the generated `proof/helicalgear/zz_registrations_test.go`.

Helical is a subclass module: it inherits spur's whole build pipeline and adds three timeline
entries of its own — the helix construction plane (step 8), the Twisted Gear Profile sketch
(step 9) and the loft (step 10). Steps 1–7 are the module surface those three are reached
through, and steps 11–12 are the boundaries helical must not move. Everything spur already owns
is inherited unchanged and is not restated here.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/helicalgear/instructions.md` | `21ad4667c0a7ebefab1d2d4fb7145512b66a8869` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `spec/spurgear/fusion.md` | `ea678245854cfec80055d67c46a8788772b0f9d4` |
| `spec/spurgear/instructions.md` | `560549a350c789b6598ee49b72d3cd7b4460e214` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `dadae022d2220a73b25e07b24ef99075a8be23a5` |

## 1 `[PROSE]` Module surface: imports and the two exported constants

Write `lib/geargen/helicalgear.py` with explicit imports — no `import *`, per the playbook's
module-layout rule. Import from `.spurgear`: `PARAM_MODULE`, `PARAM_TOOTH_NUMBER`,
`PARAM_THICKNESS`, `SpurGearCommandInputsConfigurator`, `SpurGearGenerationContext`,
`SpurGearGenerator`, `SpurGearInvoluteToothDesignGenerator`; from `.base`: `GenerationContext`,
`get_value`; from `.utilities`: `find_profile_by_curve_counts`; plus `math`, `adsk.core` and
`adsk.fusion`.

Two module-level constants are public API — herringbone imports both by name, and the contract
manifest pins their exact string values:

- `PARAM_HELIX_ANGLE = 'HelixAngle'`
- `INPUT_ID_HELIX_ANGLE = 'helixAngle'`

`GenerationContext` is imported for the parameter annotations in steps 8, 10 and 11, so keep those
annotations or the import goes unused.

**From:** `spec/helicalgear/instructions.md` L72–90 (Architecture, import list), L51–70 (exact input
ids and parameter-name strings); `spec/helicalgear/contract.json` L4–7 (`module_constants`);
`.claude/skills/generate-gear/PLAYBOOK.md` L18–40 (module layout and imports), L151–157 (the shared
helper library `find_profile_by_curve_counts` comes from).

## 2 `[PROSE]` Add the Helix Angle dialog input

`HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator)` defines
`@classmethod def configure(cls, cmd)`, which calls `super().configure(cmd)` first and then
appends one value input:

`cmd.commandInputs.addValueInput('helixAngle', 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(14.5)))`

The display unit is `'deg'` and the default is given in Fusion's internal unit for an angle, so the
14.5° default is written `math.radians(14.5)` — a bare `createByReal(14.5)` would ship a
14.5-radian default that the dialog still renders as degrees.

Because spur's `configure` has already added Parent Component last, the Helix Angle input lands
**after** Parent Component in the dialog. That is the current behaviour and it is reproduced
deliberately; do not try to insert it earlier.

<!-- check-step-calls: ignore configure -->
`configure` is named here as the method this module defines for the framework to invoke, not as a
call helicalgear.py makes; the call it does make, `super().configure(cmd)`, is required above.

**From:** `spec/helicalgear/instructions.md` L51–64 (dialog input, default, position), L78–79
(configurator class and base), L199 (Helix Angle sits last — faithful-but-flagged);
`spec/spurgear/instructions.md` L108–129 (spur's fixed dialog order), L171–178
(`[SPUR-SUBCLASS-INPUT]`), L132–138 (`addValueInput` defaults are in internal units);
`.claude/skills/generate-gear/PLAYBOOK.md` L128–136 (`[PB-DIALOG-DEFAULT-UNITS]`), L53–60 (the
four-class pattern's configurator).

## 3 `[PROSE]` Extend the generation context with two fields

`HelicalGearGenerationContext(SpurGearGenerationContext)` defines `__init__(self)`, which calls
`super().__init__()` and then declares the two fields helical adds, each cast-None initialised:

- `self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)` — the offset construction plane the
  twisted profile is drawn on, and the plane herringbone later mirrors across.
- `self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)` — the second sketch, the loft's
  top section.

Spur's own context fields are inherited unchanged and are not restated; both new names are read by
name from outside this module, so they are public API.

**From:** `spec/helicalgear/instructions.md` L80–83 (the class and its two cast-None fields), L92–101
(Generation Context — spur's, plus two); `spec/helicalgear/contract.json` L17–28 (`ctx_fields`);
`spec/spurgear/instructions.md` L253–268 (spur's canonical field names, inherited).

## 4 `[PROSE]` Generator identity: context, prefix, component name

`HelicalGearGenerator(SpurGearGenerator)` overrides three members that only say which gear this is:

- `newContext(self)` returns `HelicalGearGenerationContext()`.
- `prefixBase(self)` returns `'HelicalGear'`.
- `generateName(self)` returns
  `'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)`
  — spur's rule with `HelixAngle` appended, reading each parameter's `.expression` string, not its
  `.value`, so units show through. The four parameters come from `self.getParameter(PARAM_MODULE)`,
  `self.getParameter(PARAM_TOOTH_NUMBER)`, `self.getParameter(PARAM_THICKNESS)` and
  `self.getParameter(PARAM_HELIX_ANGLE)`.

<!-- check-step-calls: ignore newContext prefixBase generateName -->
The three names above are methods this module defines for the inherited pipeline to call, not calls
it makes.

**From:** `spec/helicalgear/instructions.md` L84 (the generator class and base), L109–112
(`newContext`, `prefixBase`, `generateName`); `spec/helicalgear/contract.json` L29–44 (the
generator's pinned method list); `spec/spurgear/instructions.md` L332–336 (`generateName` returns
`.expression` strings); `.claude/skills/generate-gear/PLAYBOOK.md` L88–91 (`prefixBase` is the
per-gear override on `base.Generator`).

## 5 `[PROSE]` Register the HelixAngle user parameter

Override spur's no-op hook: `addExtraPrimaryParameters(self, inputs)` reads the dialog input and
registers the parameter, in radians:

- `helixAngle = get_value(inputs, 'helixAngle', 'rad')`
- `self.addParameter('HelixAngle', helixAngle, 'rad', 'Helix angle for the helical gear')`

The dialog is in degrees and the parameter is in radians; `get_value` returns a `ValueInput` ready
to hand straight to `addParameter`. The hook is called by the inherited `processInputs` between the
input-sourced parameters and the derived ones, which is what lets step 6's derived `FilletRadius`
expression reference `HelixAngle`.

<!-- check-step-calls: ignore addExtraPrimaryParameters processInputs -->
`addExtraPrimaryParameters` is defined here for `processInputs` to call, and `processInputs` is
inherited unchanged — neither is a call this module makes.

**From:** `spec/helicalgear/instructions.md` L65–68 (the hook body, verbatim), L113–114 (the override
list entry); `spec/spurgear/instructions.md` L324–331 (`[SPUR-EXTRA-PARAMS]`, and the call site's
position between primary and derived parameters);
`.claude/skills/generate-gear/PLAYBOOK.md` L102–126 (`[PB-INPUT-READ]`,
`[PB-GET-VALUE-CONTRACT]`), L196–218 (the `processInputs` ordering the hook sits inside).

## 6 `[PROSE]` Point the fillet factor hook at cos(HelixAngle)

`filletHelixFactorExpression(self)` returns `f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'`, where
the spur base returns `'1'`. The string is spliced in once, as the last factor of the live
`FilletRadius` expression at parameter-registration time, so the root fillet reads correctly on the
tilted tooth's transverse plane. `createFillets` never calls this hook; it reads only the resulting
`FilletRadius` parameter's numeric value.

<!-- check-step-calls: ignore filletHelixFactorExpression createFillets registerDerivedParameters -->
The hook is defined for the inherited parameter registration to call; `createFillets` and
`registerDerivedParameters` are inherited and are named only to say where the value is and is not
consumed.

**From:** `spec/helicalgear/instructions.md` L30–33 (the `cos(HelixAngle)` correction), L115–117 (the
override); `spec/spurgear/instructions.md` L86–88 (the `* 1` hook in `FilletRadius`), L316–323 (which
member consumes the hook, and where).

## 7 `[PROSE]` The helix plane offset hook

`helicalPlaneOffset(self)` returns `self.getParameterAsValueInput(PARAM_THICKNESS)` — the full
Thickness for helical. Keep it a method of its own: it is the seam herringbone re-points at half
the thickness so its mirror plane lands mid-body, and inlining the offset into `buildSketches`
removes that seam.

What it returns is a numeric snapshot, not a live parameter reference:
`getParameterAsValueInput` wraps `param.value` in `ValueInput.createByReal`, so the plane is placed
at the Thickness that held at generation time and editing the parameter afterwards does not move
it.

<!-- check-step-calls: ignore helicalPlaneOffset -->
The hook is defined here and called in step 8; this mention is the definition.

**From:** `spec/helicalgear/instructions.md` L122–126 (the hook, the snapshot note, and the
do-not-inline rule); `spec/helicalgear/fusion.md` L21–27 (`[HELI-F-TWIST-PLANE]`, the offset and its
snapshot semantics); `.claude/skills/generate-gear/PLAYBOOK.md` L220–228 (`[PB-NUMERIC-SNAPSHOT]`);
`spec/spurgear/fusion.md` L216–221 (`[SPUR-F-SNAPSHOT]`).

## 8 `[GO]` Create the helix construction plane

`buildSketches(self, ctx: GenerationContext)` first calls `super().buildSketches(ctx)`, which draws
the bottom Gear Profile and runs the spur tooth generator at angle 0. Then it creates the offset
plane the twisted profile will be drawn on, on the gear's own component:

- `constructionPlaneInput = self.getComponent().constructionPlanes.createInput()`
- `constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())`
- `plane = self.getComponent().constructionPlanes.add(constructionPlaneInput)`
- `ctx.helixPlane = plane`

The offset argument is a `ValueInput`, never a bare number. The plane is left visible when the
build finishes: spur's cleanup hides only the entities spur created, and helical adds no cleanup of
its own, so this plane stays lit. That is deliberate — do not add cleanup for it.

The proof builds the plane at the offset and proves the offset is what it claims to be, by lofting
an untwisted section on the base plane to the same section on this plane: the resulting prism's
axial extent is exactly the offset, and its volume is the section area times the offset. A plane
placed at anything other than Thickness fails both readings.

Proof function: `stepHelixPlane`, with `assertHelixPlane`.

<!-- proof-run: proofkit3d.RunSolid(helixPlaneCases, stepHelixPlane, assertHelixPlane) -->

**From:** `spec/helicalgear/instructions.md` L128–129 (the `buildSketches` override), L144–159
(Generation Order, Delta 1 steps 1–2), L98–99 (`ctx.helixPlane`), L206–210 (the plane is left
visible); `spec/helicalgear/fusion.md` L9–27 (`[HELI-F-TWIST-PLANE]`, the exact three calls),
L29–42 (the two visibility facts); `spec/spurgear/instructions.md` L421–423 (the offset argument is a
`ValueInput`, not a bare number); `.claude/skills/generate-gear/PLAYBOOK.md` L699–710
(`[PB-CONSTRUCTION-PLANES]`).

## 9 `[GO]` Draw the Twisted Gear Profile sketch

Still inside `buildSketches`, after the plane exists: create the second sketch on it and draw the
tooth into it at the helix angle.

- `loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)`
- `SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)`
- `ctx.twistedGearProfileSketch = loftSketch`

The twist is delivered as the tooth generator's own `draw()` angle. The generator rotates the whole
tooth by it in the Python point math and then confirms the rotation with the spine's angular
dimension, so the sketch says which way it is turned. Drawing the tooth flat and rotating the
geometry afterwards also produces a twisted profile, but leaves the spine dimension measuring the
unrotated angle, and Fusion is then free to settle the tooth on the far solver branch — which is
what sends the loft through the gear centre. The angle is read as a raw radian `.value`, and it is
signed: a negative value is a left-hand helix and the dialog accepts one. Nothing rescales it —
the twist between the two loft sections is the Helix Angle itself, so Thickness does not enter.

The sketch is created hidden and is never shown, in either mode, including SketchOnly. This is a
declared delta from the hide-after-use rule: profile finding for the loft works on it hidden.

The proof rebuilds this sketch in the sketch engine under the same constraint recipes the tooth
generator uses — the projected anchor as a reference point with the local origin constrained onto
it, the solid root circle against three construction circles, the shared-centre tooth-top arc, the
+X reference line with its two axis pins and the angular dimension, the per-sample ribs in their
exact order, and the two flank-to-root lines — and gates it on the engine's full verdict, so a
scheme that reached DOF 0 while still admitting a mirrored answer fails there. It then reads two
things off the solved drawing: the tooth-top point sits at exactly the helix angle from +X, sign
included, and the tooth loop closes with the curve counts step 10 searches by — 2 splines, 2 arcs
and 2 lines, or 2 splines and 2 arcs where the profile is embedded.

Proof function: `stepTwistedGearProfile`.

<!-- proof-run: proofkit.Run(twistedProfileCases, stepTwistedGearProfile) -->

**From:** `spec/helicalgear/instructions.md` L3–8 (what helical changes), L34–49 (the signed angle,
no rescaling, no enforced range, and the proof's own bound), L100 (`ctx.twistedGearProfileSketch`),
L150–159 (Delta 1 steps 2–4), L165–174 (Sketch-discipline deltas), L209–210 (the sketch stays hidden);
`spec/helicalgear/fusion.md` L9–24 (`[HELI-F-TWIST-PLANE]`, the sketch and draw call), L29–42
(visibility); `spec/helicalgear/contract.json` L46–56 (the `buildSketches` source guard and its
reason); `spec/spurgear/instructions.md` L180–251 (Sketch Discipline and the regime the scheme must
hold across), L218–226 (the two loops' curve counts as a contract), L442–479 (step 4, the involute
tooth and where the angle is applied), L481–485 (step 5, the anchoring inside `draw`), L338–379 (the
tooth generator's reproduced surface and the exact involute math);
`spec/spurgear/fusion.md` L19–43 (`[SPUR-F-ANCHOR-CHAIN]`, `[SPUR-F-LOCAL-ORIGIN]`,
`[SPUR-F-SHARED-ADJACENCY]`), L47–60 (`[SPUR-F-ROTATE-CONFIRM]`), L69–87 (`[SPUR-F-TOOTHTOP-ARC]`),
L89–114 (`[SPUR-F-SPINE]`), L116–156 (`[SPUR-F-RIBS]`), L158–198 (`[SPUR-F-FLANK-ROOT]`);
`.claude/skills/generate-gear/PLAYBOOK.md` L230–242 (`[PB-DIM-VALUE-SEMANTICS]`), L350–428
(`[PB-SKETCH-FIRST]` and the Fusion-to-engine constraint mapping), L583–595
(`[PB-HIDE-AFTER-USE]`, including the loft-on-a-hidden-sketch exception this gear declares).

## 10 `[GO]` Loft the tooth body

`loftTooth(self, ctx: GenerationContext)` finds the tooth loop in each sketch and lofts bottom to
top into a new body:

- `lofts = self.getComponent().features.loftFeatures`
- `bottomToothProfile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)`
- `topToothProfile = find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)`
- `loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`
- `loftInput.loftSections.add(bottomToothProfile)`
- `loftInput.loftSections.add(topToothProfile)`
- `loftResult = lofts.add(loftInput)`
- `ctx.toothBody = loftResult.bodies.item(0)`, then name the body `'Tooth Body'`

The bottom section is added before the top. Use the framework's profile finder rather than
re-implementing the loop search.

Both searches pass a fixed `lines=2`, the non-embedded six-curve tooth, and neither reads
`ctx.toothProfileIsEmbedded`. An embedded low-tooth-count gear's loop has four curves, so the
search finds nothing and the build fails there. That is the current implementation's documented
limitation, reproduced as it stands — do not add an embedded branch.

The proof lofts a section standing in for that six-curve loop — the tooth's six corners taken from
the same involute samples, joined by six straight segments, because this evaluator refuses a
free-form section pair outright and its chorded circular pair leaves the volume reading outside the
gate's tolerance. It gates the result as a solid: watertight, manifold, one lump, no voids. Then it
reads the twist back off the body. A tooth twisted uniformly between its two ends is symmetric
about the half-twist plane, so its centroid sits at exactly half the helix angle from +X — which
recovers both the size of the twist and its hand, and fails if either is rescaled or dropped. The
axial extent is checked against the plane offset of step 8. It also builds the reversed section
order and reads it: the order is observable — reversing it moves every ruled wall's diagonal and
changes the volume — but the twist the body carries is unchanged, sign and size, so the reversed
build is not the mirror-hand gear the contract manifest's guard says it is. The guard stands; its
stated reason does not.

Proof function: `stepLoftTooth`, with `assertLoftTooth`.

<!-- proof-run: proofkit3d.RunSolid(loftToothCases, stepLoftTooth, assertLoftTooth) -->

**From:** `spec/helicalgear/instructions.md` L133 (the `loftTooth` override), L161–163 (Delta 2),
L171–172 (both profiles are the non-embedded six-curve tooth), L204–205 (embedded helical is
unsupported); `spec/helicalgear/fusion.md` L46–65 (`[HELI-F-LOFT]`, the exact calls, the section
order and the non-embedded limitation); `spec/helicalgear/contract.json` L57–65 (the `loftTooth`
source guard); `spec/spurgear/instructions.md` L491–495 (spur step 7, the curve counts the finder
matches), L218–226 (the counts as a contract);
`.claude/skills/generate-gear/PLAYBOOK.md` L648–652 (`[PB-LOFT]`, the API shape and that section
order is loft order), L596–605 (`[PB-PROFILE-MATCH]`), L151–157 (the helper's contract).

## 11 `[PROSE]` Loft the tooth

`buildTooth(self, ctx: GenerationContext)` is `self.loftTooth(ctx)`, and nothing else. It does not
extrude or chamfer. The inherited spur pipeline applies `chamferTeeth` after the full gear is
patterned, root-filleted, and optionally bore-cut.

The loft uses the non-embedded six-curve profile. Embedded helical gears remain unsupported because
`loftTooth` still requires two flank-to-root lines.

<!-- check-step-calls: ignore buildTooth buildMainGearBody -->
`buildTooth` is defined here for the inherited `buildMainGearBody` to call. The required
`self.loftTooth(ctx)` call is above.

**From:** `spec/helicalgear/instructions.md` L127–132, L160–164;
`spec/spurgear/instructions.md` L306–319, L547–560.

## 12 `[PROSE]` Leave the rest of the pipeline alone

Nothing else is written. `processInputs`, `prepareTools`, `buildMainGearBody`, `buildBody`,
`patternTeeth`, `createFillets`, `buildBore`, `chamferTeeth`, `cleanup` and the whole
`SpurGearInvoluteToothDesignGenerator` are inherited and must not be re-implemented, and the call
graph and override boundaries must not move: `generate → processInputs → prepareTools →
buildMainGearBody(buildSketches → buildTooth → buildBody → patternTeeth → createFillets) →
buildBore → chamferTeeth → cleanup`.

Two visibility facts follow from adding no cleanup, and both are deliberate:

- the helix `ConstructionPlane` is left visible after generation, because spur's cleanup switches
  off only the light bulbs of entities spur itself created;
- the Twisted Gear Profile sketch stays hidden its whole life — created hidden, never shown, in
  SketchOnly mode too, where the twisted profile is therefore not inspectable.

A regeneration must reproduce both rather than tidy them up.

<!-- check-step-calls: ignore prepareTools buildBody patternTeeth buildBore cleanup generate -->
Every name in that list is inherited from spur; they are named only to forbid re-implementing them,
not as calls this module makes.

**From:** `spec/helicalgear/instructions.md` L17–21 (Component Setup is inherited), L102–107 (the
inherited call graph and the do-not-move rule), L135–142 (the inherited-unchanged list and the `ctx`
annotations), L176–192 (Dependencies — the borrowed spur surface), L206–210 (the two visibility
facts); `spec/helicalgear/fusion.md` L29–42 (`[HELI-F-TWIST-PLANE]`'s visibility rules);
`spec/spurgear/instructions.md` L270–305 (the call graph and `cleanup`'s position);
`spec/spurgear/fusion.md` L202–212 (`[SPUR-F-CLEANUP]`);
`.claude/skills/generate-gear/PLAYBOOK.md` L256–280 (the dependent-gear contract).
