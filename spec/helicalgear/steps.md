# Helical Gear — compiled step list

The proof for this step list is `proof/helicalgear/sketches_test.go` and
`proof/helicalgear/solids_test.go`, registered by the generated
`proof/helicalgear/zz_registrations_test.go`.

Helical is a **subclass-only module**. It inherits spur's whole build pipeline and changes three
things: one dialog input, one extra pair of timeline entries inside `buildSketches` (an offset
construction plane and a second, twisted sketch), and one replacement of spur's tooth extrude with a
loft. Steps 1–7 and 11 are the Python surface the module must reproduce; steps 8, 9 and 10 are the
three Fusion timeline entries helical itself adds. Everything spur builds — the Tools sketch, the
bottom Gear Profile sketch, the body extrude, the circular pattern, the root fillets, the optional
bore, the completed-gear chamfer and the cleanup — is inherited unchanged and appears here only as
step 12, which says what must **not** be re-implemented.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/helicalgear/instructions.md` | `6c1d3b4d7aa824d90f9f0f851d4115179e754707` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `spec/spurgear/fusion.md` | `7cd4e5b0fa38dcd39cbd1b5bad1cf8489e2bc2ae` |
| `spec/spurgear/instructions.md` | `486f78e9844f07a1ab7ebf4af110260aafac6c99` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `1b3078d6767d6a3f56c228e1e934c82ccfbf53fe` |

## 1 `[PROSE]` Module surface: imports and the two exported constants

**From:** `spec/helicalgear/instructions.md` L70, L86–92, L194–208;
`.claude/skills/generate-gear/PLAYBOOK.md` L17–40

Write `lib/geargen/helicalgear.py` with exactly these module-level constants, whose string values
are public API (herringbone imports both by name):

| constant | value |
|---|---|
| `PARAM_HELIX_ANGLE` | `'HelixAngle'` |
| `INPUT_ID_HELIX_ANGLE` | `'helixAngle'` |

Import explicitly — **no `import *`** in a gear module (the playbook's "Module layout & imports"
section, which star-imports have broken before by hiding a real dependency):

```python
import math
import adsk.core, adsk.fusion
from .base import get_value
from .utilities import find_profile_by_curve_counts
from .spurgear import (PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS,
                       SpurGearCommandInputsConfigurator, SpurGearGenerationContext,
                       SpurGearGenerator, SpurGearInvoluteToothDesignGenerator)
```

`GenerationContext` is **not** imported. The three `ctx`-taking overrides annotate their parameter as
`SpurGearGenerationContext`, which is what the inherited signatures declare, so nothing in this
module names the base type.

The three spur constants imported here are the only ones this module uses: `PARAM_MODULE`,
`PARAM_TOOTH_NUMBER` and `PARAM_THICKNESS` ([SPUR-EXPORTED-CONSTANTS]).

## 2 `[PROSE]` The Helix Angle dialog input

**From:** `spec/helicalgear/instructions.md` L25–49, L51–70, L78–79, L215;
`spec/spurgear/instructions.md` L90–160, L186–193;
`.claude/skills/generate-gear/PLAYBOOK.md` L128–136

`class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator)` with
`@classmethod def configure(cls, cmd)`. It calls `super().configure(cmd)` first, then appends one
value input and nothing else ([SPUR-SUBCLASS-INPUT]):

```python
class HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator):
    @classmethod
    def configure(cls, cmd):
        super().configure(cmd)
        cmd.commandInputs.addValueInput(
            INPUT_ID_HELIX_ANGLE, 'Helix Angle', 'deg',
            adsk.core.ValueInput.createByReal(math.radians(14.5)))
```

The call is `cmd.commandInputs.addValueInput('helixAngle', 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(14.5)))`:
input id `helixAngle`, label `Helix Angle`, display unit string `'deg'`, and a default of **14.5
degrees passed in radians**, because a `createByReal` default is always in Fusion's internal units
whatever the display unit says ([PB-DIALOG-DEFAULT-UNITS]).

**The dialog's full input list, in the order `configure()` adds them.** Spur's ten come from
`super().configure(cmd)`; helical's is the eleventh and lands **after** Parent Component, because
spur already added Parent Component last. ⚠️ This is the actual current behavior — reproduce it; do
not try to insert Helix Angle earlier.

| # | dialog label | input id | added with | unit | default | user parameter |
|---|---|---|---|---|---|---|
| 1 | Target Plane | `plane` | `addSelectionInput` (filters `ConstructionPlanes`, `PlanarFaces`; `setSelectionLimits(1, 1)`) | — | — | — |
| 2 | Anchor Point | `anchorPoint` | `addSelectionInput` (filters `ConstructionPoints`, `SketchPoints`; `setSelectionLimits(1, 1)`) | — | — | — |
| 3 | Module | `module` | `addValueInput` | `''` | `createByReal(1)` | `Module` |
| 4 | Tooth Number | `toothNumber` | `addValueInput` | `''` | `createByReal(17)` | `ToothNumber` |
| 5 | Pressure Angle | `pressureAngle` | `addValueInput` | `'deg'` | `createByReal(math.radians(20))` | `PressureAngle` |
| 6 | Bore Diameter | `boreDiameter` | `addStringValueInput` | — | `'0 mm'` | `BoreDiameter` |
| 7 | Thickness | `thickness` | `addValueInput` | `'mm'` | `createByReal(to_cm(10))` | `Thickness` |
| 8 | Apply chamfer to teeth | `chamferTooth` | `addValueInput` | `'mm'` | `createByReal(0)` | `ChamferTooth` |
| 9 | Generate sketches, but do not build body | `sketchOnly` | `addBoolValueInput` | — | `false` | `SketchOnly` |
| 10 | Parent Component | `parentComponent` | `addSelectionInput` (filters `Occurrences`, `RootComponents`; `setSelectionLimits(1, 1)`; pre-selects the root component) | — | — | — |
| 11 | **Helix Angle** | **`helixAngle`** | **`addValueInput`** | **`'deg'`** | **`createByReal(math.radians(14.5))`** | **`HelixAngle`** |

Rows 1–10 are spur's and this module writes none of them; they are reproduced here because row 11's
position is stated relative to them and because the ids are the surface a subclass must not collide
with.

**The value is signed and the sign is the hand of the helix.** Negative is a **left-hand** helix, and
the dialog accepts a negative value. **No range is enforced** — no clamp, no warning, no documented
maximum. What Fusion does at a large helix angle is unverified, so do not add one.

<!-- check-step-calls: ignore configure addSelectionInput addSelectionFilter setSelectionLimits addStringValueInput addBoolValueInput to_cm -->
`configure` is defined here for the framework to call, not called by this module, and the six other
names above belong to spur's inherited `configure`, which this module does not write.

## 3 `[PROSE]` The generation context: spur's fields plus two

**From:** `spec/helicalgear/instructions.md` L80–83, L94–102, L136–156;
`spec/spurgear/instructions.md` L268–283

`class HelicalGearGenerationContext(SpurGearGenerationContext)`. Its `__init__` calls
`super().__init__()` and then initialises exactly two new fields, each to a cast-`None`:

```python
class HelicalGearGenerationContext(SpurGearGenerationContext):
    def __init__(self):
        super().__init__()
        self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)
        self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)
```

The two calls are `adsk.fusion.ConstructionPlane.cast(None)` and `adsk.fusion.Sketch.cast(None)`.

- **`ctx.helixPlane`** — the offset `ConstructionPlane` the twisted top profile is drawn on. It is
  also the plane herringbone mirrors across, so the field name is public API.
- **`ctx.twistedGearProfileSketch`** — the second, `'Twisted Gear Profile'` sketch: the loft's top
  section.

Spur's own context fields are inherited unchanged and are not restated in this module.

## 4 `[PROSE]` Generator identity: `newContext`, `prefixBase`, `generateName`

**From:** `spec/helicalgear/instructions.md` L84, L110–114;
`spec/spurgear/instructions.md` L346–350

`class HelicalGearGenerator(SpurGearGenerator)` overrides three identity methods:

- `newContext` returns `HelicalGearGenerationContext()`.
- `prefixBase` returns the string `'HelicalGear'`. The framework builds the user-parameter prefix as
  `HelicalGear_<component id without dashes>` from it.
- `generateName` returns
  `'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)`
  — the four parameters' **`.expression`** strings, never `.value`, so units show through. Read each
  with `self.getParameter(PARAM_MODULE)`, `self.getParameter(PARAM_TOOTH_NUMBER)`,
  `self.getParameter(PARAM_THICKNESS)` and `self.getParameter(PARAM_HELIX_ANGLE)`. This extends
  spur's three-parameter rule with `HelixAngle` as the fourth.

<!-- check-step-calls: ignore newContext prefixBase generateName -->
All three are methods this module DEFINES for `base.Generator` to call; the module never calls them
itself, so naming them here is a mention, not a required call. `HelicalGearGenerationContext()`,
`self.getParameter(...)` and `.format(...)` inside them are required.

## 5 `[PROSE]` Register the `HelixAngle` user parameter

**From:** `spec/helicalgear/instructions.md` L29–34, L65–68, L115–116;
`spec/spurgear/instructions.md` L338–345;
`.claude/skills/generate-gear/PLAYBOOK.md` L103–126, L196–218

Override spur's no-op hook `addExtraPrimaryParameters(self, inputs)` ([SPUR-EXTRA-PARAMS]).
`processInputs` calls it **between** the input-sourced parameters and the derived ones, which is why
`FilletRadius` can reference `HelixAngle` in step 6.

```python
def addExtraPrimaryParameters(self, inputs):
    helixAngle = get_value(inputs, INPUT_ID_HELIX_ANGLE, 'rad')
    self.addParameter(PARAM_HELIX_ANGLE, helixAngle, 'rad', 'Helix angle for the helical gear')
```

The two calls are `get_value(inputs, 'helixAngle', 'rad')` and
`self.addParameter('HelixAngle', helixAngle, 'rad', 'Helix angle for the helical gear')`. The
comment string is `'Helix angle for the helical gear'` verbatim.

**The dialog is degrees; the parameter is radians.** The input was declared with `addValueInput`, so
it is read with `get_value` ([PB-INPUT-READ]), which returns a `ValueInput` ready to pass straight to
`addParameter` ([PB-GET-VALUE-CONTRACT]) — no `ok` flag, no re-wrapping.

<!-- check-step-calls: ignore addExtraPrimaryParameters processInputs -->
`addExtraPrimaryParameters` is defined here for `processInputs` to call, and `processInputs` is
inherited from spur and not written by this module.

## 6 `[PROSE]` The root-fillet transverse correction

**From:** `spec/helicalgear/instructions.md` L31–33, L117–119;
`spec/spurgear/instructions.md` L86–88, L332–337

Override `filletHelixFactorExpression(self)` to return the **expression string**
`f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'` — the spur base returns `'1'`.

It is **not** read by `createFillets`. `registerDerivedParameters` splices it in as the last factor
of the live `FilletRadius` expression, `(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>`, so
the root fillet reads correctly on the tilted tooth's transverse plane. `createFillets` then reads
only the resulting `FilletRadius` parameter's numeric `.value`.

The call this makes is `self.parameterName(PARAM_HELIX_ANGLE)`, which yields the prefixed name
(`HelicalGear_<id>_HelixAngle`) Fusion's expression engine resolves.

<!-- check-step-calls: ignore filletHelixFactorExpression createFillets registerDerivedParameters -->
`filletHelixFactorExpression` is defined here for `registerDerivedParameters` to call, and both
`createFillets` and `registerDerivedParameters` are inherited from spur and not written here.

## 7 `[PROSE]` The helix plane's offset hook

**From:** `spec/helicalgear/instructions.md` L120–125;
`spec/helicalgear/fusion.md` L21–27;
`spec/spurgear/fusion.md` L233–238

Define `helicalPlaneOffset(self)` as its own method, returning
`self.getParameterAsValueInput(PARAM_THICKNESS)` — the **full** `Thickness` as a `ValueInput`.

Keep it a separate overridable hook. Herringbone re-points it at half the thickness so its mirror
plane lands mid-body; inlining the offset into `buildSketches` removes the seam.

The returned value is a **numeric snapshot**, not a live parameter reference:
`getParameterAsValueInput` returns `ValueInput.createByReal(param.value)`, the `Thickness` value at
generation time ([PB-NUMERIC-SNAPSHOT], [SPUR-F-SNAPSHOT]). Editing `Thickness` afterwards does not
move the plane; regenerate.

## 8 `[GO]` Create the helix construction plane

**From:** `spec/helicalgear/instructions.md` L126–128, L165–174, L219–222;
`spec/helicalgear/fusion.md` L9–27, L36–42;
`.claude/skills/generate-gear/PLAYBOOK.md` L742–753

Proof function: `stepHelixPlane`, asserted by `assertHelixPlane`.

<!-- proof-run: proofkit3d.RunSolid(planeCases, stepHelixPlane, assertHelixPlane) -->

Override `buildSketches(self, ctx)`. Annotate the parameter `ctx: SpurGearGenerationContext` — the
inherited signature's own type — and narrow it with an assertion at the top of the body:

```python
def buildSketches(self, ctx: SpurGearGenerationContext):
    assert isinstance(ctx, HelicalGearGenerationContext)
    super().buildSketches(ctx)
```

⚠️ Do **not** annotate the parameter as `GenerationContext` (wider than the inherited signature) and
do **not** narrow it to `HelicalGearGenerationContext` (narrowing a parameter in an override is its
own error). The annotation matches the base and the assertion does the narrowing; every read and
write of `ctx.helixPlane` and `ctx.twistedGearProfileSketch` happens after it.

`super().buildSketches(ctx)` runs first and draws the bottom Gear Profile sketch, running the spur
tooth generator at angle 0. Then, on the gear's **own** component, create the offset plane
([HELI-F-TWIST-PLANE]):

```python
constructionPlaneInput = self.getComponent().constructionPlanes.createInput()
constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())
plane = self.getComponent().constructionPlanes.add(constructionPlaneInput)
ctx.helixPlane = plane
```

The calls are `self.getComponent().constructionPlanes.createInput()`,
`constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())` and
`self.getComponent().constructionPlanes.add(constructionPlaneInput)`. The plane is offset from
`self.plane` — the generator's normalised target plane, the same plane the bottom Gear Profile
sketch sits on — and by nothing else. The offset argument is a `ValueInput`, never a bare number
([PB-CONSTRUCTION-PLANES]).

**Create it on `self.getComponent()`, and do not activate anything.** [PB-CONSTRUCTION-NEEDS-ACTIVE]
says construction geometry raises `RuntimeError: 3 : Environment is not supported` on a component
that is not the activated one, and [PB-NEVER-ACTIVATE] says never to activate an occurrence. The
recipe here is the spec's and it is what the shipped add-in does — spur creates its Extrusion End
Plane the same way — so transcribe it as written: no `activate()` call, and no substitute for the
construction plane.

<!-- check-step-calls: ignore activate -->
`activate()` is named only to forbid it; this module must not call it.

**The plane is left visible after generation.** Spur's `cleanup` switches the light bulb off only on
the entities spur created — the Extrusion End Plane, the normalized target plane, the `Gear Center`
axis — and helical adds no cleanup of its own, so this plane stays lit. This is a declared,
deliberate delta from [PB-HIDE-AFTER-USE]; a regeneration must **not** add cleanup for it.

In **SketchOnly** mode the plane is still created, and still left visible.

**What the proof measures.** `stepHelixPlane` builds the loft across this plane and
`assertHelixPlane` reads the gap it spans: the body starts on the gear's own plane and ends exactly
`Thickness` away, which is the one number separating helical's hook from herringbone's half. The
build also carries the zero-offset control — with both sections on the gear's own plane the loft is
refused as degenerate — so the plane is proven load-bearing rather than assumed to be.

## 9 `[GO]` Draw the Twisted Gear Profile sketch

**From:** `spec/helicalgear/instructions.md` L4–8, L36–40, L165–174, L181–190, L216–222;
`spec/helicalgear/fusion.md` L9–24, L29–42;
`spec/spurgear/instructions.md` L325–327, L352–393, L445–499;
`spec/spurgear/fusion.md` L19–43, L47–60, L69–215

Proof function: `stepTwistedGearProfile`.

<!-- proof-run: proofkit.Run(twistedCases, stepTwistedGearProfile) -->

Still inside `buildSketches`, after the plane exists, create the second sketch on it and run the
**inherited** spur tooth generator into it at the helix angle ([HELI-F-TWIST-PLANE]):

```python
loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)
SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(
    ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)
ctx.twistedGearProfileSketch = loftSketch
```

The calls are `self.createSketchObject('Twisted Gear Profile', plane=plane)`,
`SpurGearInvoluteToothDesignGenerator(loftSketch, self)`,
`.draw(ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)` and
`self.getParameter(PARAM_HELIX_ANGLE)`. The sketch name is the exact string
`'Twisted Gear Profile'`.

**The twist is delivered as the `draw()` `angle` argument, read as a raw `.value` in radians.** The
spur generator rotates the whole tooth by that angle in its own point math and then confirms the
rotation with the spine's angular dimension as its very last action
([SPUR-F-ROTATE-CONFIRM], [SPUR-F-SPINE]). Do **not** draw the tooth flat and rotate the geometry
afterwards: the confirming dimension would then measure the unrotated angle, the solver can settle
on the branch about 180 degrees away, and the loft passes through the gear centre.

**Helical does nothing else to this sketch.** Its four circles, two involute flank splines,
tooth-top arc, spine, +X reference line, rib chain and flank-to-root stubs are all spur's
construction and are not re-implemented here — [SPUR-F-ANCHOR-CHAIN], [SPUR-F-LOCAL-ORIGIN],
[SPUR-F-SHARED-ADJACENCY], [SPUR-F-TOOTHTOP-ARC], [SPUR-F-SPINE], [SPUR-F-RIBS],
[SPUR-F-FLANK-ROOT], with [PB-SHARE-XOR-COINCIDENT], [PB-DRIVING-DIM] and [PB-SKETCHCURVES]
underneath them. `draw()` also performs the step-5 anchoring itself, projecting `ctx.anchorPoint` in
and constraining it to the sketch's own local origin, which is why one `draw()` call is enough to
tie this sketch to the user's anchor.

**Both loft sections are the non-embedded six-curve tooth** — 2 splines, 2 arcs, 2 flank-to-root
lines — which is the count step 10 searches on ([PB-PROFILE-MATCH]).

**The sketch stays hidden its whole life.** `createSketchObject` returns a hidden sketch and nothing
ever shows it: not `buildSketches`, and not spur's `cleanup`, which touches only its own three
sketches. The loft's profile search works on the hidden sketch. This is a declared delta from
[PB-HIDE-AFTER-USE] — there is no "shown, then hidden after use" phase at all. In **SketchOnly**
mode the same holds, so the twisted profile is not inspectable there.

**No runtime full-constraint gate.** Spur registers none and helical adds none. The twisted sketch's
full constraint is a design-time property, proven on the bench ([PB-SKETCH-FIRST]) and by
`stepTwistedGearProfile`, not asserted in the generated code. It could not be asserted anyway:
spur's `drawCircles` labels each circle with along-path sketch text, and text carries its own
unpinned position, so a labelled sketch never reports `isFullyConstrained` ([PB-TEXT-HOLDS-DOF]).

**What the proof measures.** `stepTwistedGearProfile` rebuilds this sketch — spur's scheme with a
non-zero angle — and gates it on the engine's full verdict: DOF 0, no redundant or conflicting
constraint, well conditioned, valid profiles, no discrete ambiguity. The table sweeps sizes, both
signs of the helix angle including a quarter turn each way, a low rib count, and both routes into
the embedded shape. It then counts the curves on the two loops the sketch closes and holds them to
the contract step 10 keys on.

## 10 `[GO]` Loft the tooth

**From:** `spec/helicalgear/instructions.md` L129–130, L176–179, L186–188, L216–217;
`spec/helicalgear/fusion.md` L46–65;
`spec/spurgear/instructions.md` L328–329, L505–509;
`.claude/skills/generate-gear/PLAYBOOK.md` L151–155, L691–695

Proof function: `stepLoftTooth`, asserted by `assertLoftTooth`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepLoftTooth, assertLoftTooth) -->

Override `buildTooth(self, ctx)` so that it does exactly one thing — `self.loftTooth(ctx)` — and
then write `loftTooth(self, ctx)`. `buildTooth` does **not** extrude and does **not** chamfer. Both
carry the same annotation and narrowing assertion as step 8.

<!-- check-step-calls: ignore buildTooth -->
`buildTooth` is a method this module DEFINES for the inherited `buildMainGearBody` to call; the
module never calls it itself, so naming it here is a mention, not a required call. `loftTooth` in
the same sentence IS required, because `buildTooth`'s body calls it.

```python
def buildTooth(self, ctx: SpurGearGenerationContext):
    assert isinstance(ctx, HelicalGearGenerationContext)
    self.loftTooth(ctx)

def loftTooth(self, ctx: SpurGearGenerationContext):
    assert isinstance(ctx, HelicalGearGenerationContext)
    lofts = self.getComponent().features.loftFeatures
    bottomToothProfile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)
    topToothProfile    = find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)
    loftInput = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    loftInput.loftSections.add(bottomToothProfile)
    loftInput.loftSections.add(topToothProfile)
    loftResult = lofts.add(loftInput)
    ctx.toothBody = loftResult.bodies.item(0)
    ctx.toothBody.name = 'Tooth Body'
```

The calls are `self.loftTooth(ctx)`, `self.getComponent().features.loftFeatures`,
`find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)`,
`find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)`,
`lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`loftInput.loftSections.add(bottomToothProfile)`, `loftInput.loftSections.add(topToothProfile)`,
`lofts.add(loftInput)` and `loftResult.bodies.item(0)`.

**The named operands, exactly.** The bottom section is found in **`ctx.gearProfileSketch`** — the
Gear Profile sketch spur's `super().buildSketches` drew on `self.plane`. The top section is found in
**`ctx.twistedGearProfileSketch`** — the Twisted Gear Profile sketch from step 9, on `ctx.helixPlane`.
The two are not interchangeable: swapping them inverts the twist.

**Add the bottom section first, then the top.** `loftSections.add` order is the loft order
([PB-LOFT]). The other order also lofts a valid solid and reads the same twist, so nothing downstream
raises; what changes is the ruled walls, which are built outward from the FROM section.

**Both searches pass a fixed `nurbs=2, arcs=2, lines=2`.** Use the framework helper
`find_profile_by_curve_counts` and do not re-implement the loop search; it raises rather than
falling back to a wrong profile ([PB-PROFILE-MATCH]). ⚠️ **Non-embedded only, by construction.** This
implementation never reads `ctx.toothProfileIsEmbedded` and has **no** embedded branch, so an
embedded gear — the flank starting inside the root circle, a four-curve loop with `lines=0`, which
happens above `2.5 / (1 - cos(PressureAngle))` teeth — fails to find the profile. That is faithful
to the current code and a documented limitation, not a bug to fix here ([HELI-F-LOFT]).

The resulting body is `ctx.toothBody`, named with the exact string `'Tooth Body'`.

Helical replaces spur's tooth extrude entirely, so this module never calls
`extrudeFeatures.addSimple(...)` and never builds a `setOneSideToExtent(...)` extent for a tooth.

<!-- check-step-calls: ignore addSimple setOneSideToExtent -->
Both are named only to forbid them; neither is a call this module makes.

**What the proof measures.** `stepLoftTooth` lofts a chorded stand-in for the two sections — decad
refuses a free-form segment pair, so the flanks and both arcs are chorded through the same sample
points — and `assertLoftTooth` reads the twist off the two cap sections, requiring it to equal the
Helix Angle in size **and sign** and to be unchanged when `Thickness` changes. It also proves the
start cap is the bottom section, which is the `loftSections.add` order above, and that a zero-twist
loft is exactly the section's prism. The proof file records what the chording costs, and the
measured twist bound, per sign, past which this harness stops verifying.

## 11 `[PROSE]` The method contract helical must keep

**From:** `spec/helicalgear/instructions.md` L104–134, L158–164, L192–208

The whole call graph is spur's and the override boundaries are public API. Helical overrides
**exactly** these nine methods and no others: `newContext`, `prefixBase`, `generateName`,
`addExtraPrimaryParameters`, `filletHelixFactorExpression`, `helicalPlaneOffset`, `buildSketches`,
`buildTooth`, `loftTooth`.

Do not move work across the inherited boundaries:

```
generate → processInputs → prepareTools
        → buildMainGearBody(buildSketches → buildTooth → buildBody → patternTeeth → createFillets)
        → buildBore → chamferTeeth → cleanup
```

**Inherited unchanged — do NOT re-implement:** `processInputs`, `prepareTools`,
`buildMainGearBody`, `buildBody`, `patternTeeth`, `createFillets`, `buildBore`, `chamferTeeth`,
`cleanup`, and the entire `SpurGearInvoluteToothDesignGenerator`.

Helical constructs `SpurGearInvoluteToothDesignGenerator` directly (step 9) but subclasses none of
it, and it subclasses `SpurGearCommandInputsConfigurator`, `SpurGearGenerationContext` and
`SpurGearGenerator`. It leans on the inherited framework helpers `getComponent`,
`createSketchObject`, `getParameter`, `getParameterAsValueInput` and `parameterName`, and on the
context fields `ctx.anchorPoint`, `ctx.gearProfileSketch`, `ctx.toothBody`,
`ctx.toothProfileIsEmbedded` and `self.plane`.

<!-- check-step-calls: ignore generate processInputs prepareTools buildMainGearBody buildBody patternTeeth createFillets buildBore chamferTeeth cleanup -->
Every name in that list is inherited from spur and named here only to forbid re-implementing it;
this module writes none of them.

## 12 `[PROSE]` The inherited completed-gear chamfer

**From:** `spec/helicalgear/fusion.md` L69–80; `spec/spurgear/instructions.md` L545–560

Helical inherits `chamferTeeth` with **no override**. `generate` calls it after the optional bore, so
it runs on the patterned, filleted, optionally bored gear. It uses **no tooth-cap edge count**: the
earlier `4`-edge predicate aborted the feature and the later `6`-edge one chamfered only the bottom
cap. It scans every planar face of the completed `ctx.gearBody` parallel to the Gear Profile plane
and adds every unique boundary edge once, root-radius arcs included, excluding a circular edge whose
radius is the positive bore radius ([HELI-F-CHAMFER-COUNT]).

Add nothing to this module for it. The final completed-gear selection remains pending Fusion
verification, and that verdict belongs in the shared pipeline's own spec, not in a helical override.
