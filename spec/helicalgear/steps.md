# Helical Gear — compiled step list

The proof for these steps is `proof/helicalgear/sketches_test.go`, `proof/helicalgear/solids_test.go`
and the generated registration file `proof/helicalgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/helicalgear/instructions.md` | `71b7993007b17cc4948569313c23614b8fbd246c` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `spec/spurgear/fusion.md` | `5dccd871606c3709ecfa07c05f58c126369f2927` |
| `spec/spurgear/instructions.md` | `2a98a801da25e77958488252bc87b499475ac95d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9ee2dcbaed7b5480aa69e9295e8b61acaea081f3` |

## H1 `[PROSE]` Module layout, imports and the two module constants

Helical is a **thin specialization of the spur gear**. It subclasses the spur family and reuses the
entire spur build pipeline verbatim — the tools sketch, the gear profile, the body extrude, the
pattern, the fillets, the bore, the cleanup and the whole tooth generator. It changes exactly three
things: it adds one **Helix Angle** input (H2), it draws a second **Twisted Gear Profile** sketch
(H8 and H9), and it **lofts** the bottom profile to that twisted top profile instead of extruding
(H10).

Write `lib/geargen/helicalgear.py` with explicit imports and no `import *`:

```python
import math
import adsk.core, adsk.fusion
from .base import get_value
from .spurgear import (PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS,
                       SpurGearCommandInputsConfigurator, SpurGearGenerationContext,
                       SpurGearGenerator, SpurGearInvoluteToothDesignGenerator)
from .utilities import find_profile_by_curve_counts
```

`GenerationContext` is **not** imported. The overrides annotate `ctx: SpurGearGenerationContext` and
narrow with an assertion, so nothing in this module names the base type.

Two module-level constants, with exactly these values — herringbone imports both by name:

| constant | value |
|---|---|
| `PARAM_HELIX_ANGLE` | `'HelixAngle'` |
| `INPUT_ID_HELIX_ANGLE` | `'helixAngle'` |

The module defines three classes, each extending its spur counterpart, and these names are the
reproduced surface: herringbone subclasses all three, and `commands/helicalgear/entry.py` binds two
of them by name.

1. `HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator)` — H2.
2. `HelicalGearGenerationContext(SpurGearGenerationContext)` — H3.
3. `HelicalGearGenerator(SpurGearGenerator)` — H4 through H10.

**From:** `spec/helicalgear/instructions.md` L1-8, L70, L72-92; `spec/spurgear/instructions.md` L226-243; `.claude/skills/generate-gear/PLAYBOOK.md` L17-40, L42-74, L265-289.

## H2 `[PROSE]` The Helix Angle dialog input — `HelicalGearCommandConfigurator`

`HelicalGearCommandConfigurator` subclasses `SpurGearCommandInputsConfigurator` and overrides the
`@classmethod` `configure(cls, cmd)`. It calls `super().configure(cmd)` first, so every spur input is
added exactly as spur adds it, then appends its own one input:

`cmd.commandInputs.addValueInput('helixAngle', 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(14.5)))`

This is the whole table of what helical adds to the dialog. Reproduce every string in it:

| dialog label | input id | display unit | default | user parameter | parameter units |
|---|---|---|---|---|---|
| Helix Angle | `helixAngle` | `'deg'` | `ValueInput.createByReal(math.radians(14.5))` — 14.5 degrees | `HelixAngle` | `'rad'` |

The default is passed as a **real in Fusion's internal units**, which for an angle is radians, even
though the input displays degrees (`[PB-DIALOG-DEFAULT-UNITS]`) — hence `math.radians(14.5)` rather
than `14.5`. The value input is registered in H5.

⚠️ **The Helix Angle input necessarily appears LAST in the dialog, after Parent Component.** Spur's
`configure` already added Parent Component last, and this is the configurator extension seam
(`[SPUR-SUBCLASS-INPUT]`): a subclass appends after `super()`, so its input lands below. This is the
actual, current behavior — reproduce it exactly, and do not attempt to insert Helix Angle earlier in
the list.

**Signed, and the sign is the hand of the helix.** The dialog accepts a negative value: negative is
a **left-hand** helix. **No range is enforced**, and none is added here — no clamp, no warning, no
documented maximum. What Fusion does at a large helix angle is unverified, and until a Fusion
session settles it the dialog takes whatever the user types.

**From:** `spec/helicalgear/instructions.md` L26-34, L36-40, L42-49, L51-64, L78-80, L210-216; `spec/spurgear/instructions.md` L182-203, L205-212, L245-252; `.claude/skills/generate-gear/PLAYBOOK.md` L128-136.

## H3 `[PROSE]` The generation context — spur's fields, plus two

`HelicalGearGenerationContext(SpurGearGenerationContext)` declares `__init__(self)`, which calls
`super().__init__()` and then initialises exactly two new fields, each to a cast-`None`:

- `ctx.helixPlane` = `adsk.fusion.ConstructionPlane.cast(None)` — the offset `ConstructionPlane` the
  twisted top profile is drawn on, and the plane herringbone later reflects across.
- `ctx.twistedGearProfileSketch` = `adsk.fusion.Sketch.cast(None)` — the second, `Twisted Gear
  Profile` sketch: the top loft section.

Every spur context field is inherited unchanged; do not restate spur's list here, and do not rename
either of these two — herringbone reads both by name.

Seed each field with the cast of the class the field actually holds. `adsk.core.Base.cast` is
declared by the API database and the stubs but is **not** defined by the Fusion runtime, which
raises `AttributeError` on it; every concrete subclass does have `cast`.

**From:** `spec/helicalgear/instructions.md` L80-83, L94-103; `spec/spurgear/instructions.md` L327-342; `.claude/skills/generate-gear/PLAYBOOK.md` L42-74.

## H4 `[PROSE]` The generator's identity — `newContext`, `prefixBase`, `generateName`

`HelicalGearGenerator(SpurGearGenerator)` keeps spur's entire call graph and every override
boundary: `generate → processInputs → prepareTools → buildMainGearBody(buildSketches → buildTooth →
buildBody → patternTeeth → createFillets) → buildBore → chamferTeeth → cleanup`. **Do not move work
across those boundaries.** Three of its overrides only name the gear:

<!-- check-step-calls: ignore newContext prefixBase generateName -->
These three are methods the module DEFINES for the inherited pipeline to call; the module never
calls any of them itself, so they are named here and not required as calls.

- `newContext` returns a `HelicalGearGenerationContext()`, annotated `-> HelicalGearGenerationContext`.
- `prefixBase` returns the string `'HelicalGear'`, annotated `-> str`.
- `generateName` returns, annotated `-> str`:
  `'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)`
  where the four values are the `.expression` strings — not `.value` — of the parameters
  `self.getParameter(PARAM_MODULE)`, `self.getParameter(PARAM_TOOTH_NUMBER)`,
  `self.getParameter(PARAM_THICKNESS)` and `self.getParameter(PARAM_HELIX_ANGLE)`, so units show
  through. This is spur's rule extended with `HelixAngle`.

Spur annotates five methods' returns precisely so a subclass may narrow on them; carry the
annotations above for the same reason.

**From:** `spec/helicalgear/instructions.md` L84-85, L104-114, L132-134; `spec/spurgear/instructions.md` L108-124, L344-351, L405-409.

## H5 `[PROSE]` Register the `HelixAngle` user parameter — `addExtraPrimaryParameters`

<!-- check-step-calls: ignore addExtraPrimaryParameters -->
`addExtraPrimaryParameters(self, inputs)` is the hook spur's `processInputs` calls between
registering the input-sourced parameters and the derived ones (`[SPUR-EXTRA-PARAMS]`). It is a no-op
on the spur base and the module only defines it, never calls it, so it is exempt above.

The override reads the dialog input and registers the parameter, with exactly these strings:

`helixAngle = get_value(inputs, 'helixAngle', 'rad')`

`self.addParameter('HelixAngle', helixAngle, 'rad', 'Helix angle for the helical gear')`

The dialog input is **degrees** and the user parameter is **radians**: that is deliberate, not a
mismatch. `get_value` returns a `ValueInput` ready to pass straight to `addParameter`, and raises on
an invalid expression, so there is no `ok`-flag handling and no wrapping to do
(`[PB-GET-VALUE-CONTRACT]`); read a value input with `get_value` and nothing else
(`[PB-INPUT-READ]`). The fourth argument is the Comment column Fusion shows beside the parameter —
it is what the user reads, so write it exactly as given.

Registering here, before the derived parameters, is what lets H6's expression reference
`HelixAngle`.

**From:** `spec/helicalgear/instructions.md` L65-68, L115-116; `spec/spurgear/instructions.md` L133-137, L397-404; `.claude/skills/generate-gear/PLAYBOOK.md` L103-118, L120-126, L205-227.

## H6 `[PROSE]` The root fillet's transverse correction — `filletHelixFactorExpression`

<!-- check-step-calls: ignore filletHelixFactorExpression -->
`filletHelixFactorExpression(self)` returns an expression **string**, annotated `-> str`:

`f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'`

The spur base returns `'1'`. The module defines this method for the inherited pipeline and never
calls it, so it is exempt above; `parameterName` is a call the override itself makes.

Nothing reads the returned string except `registerDerivedParameters`, which splices it in as the
last factor of the live `FilletRadius` expression, `(ToothSpaceArcAtRoot / 2) * FilletClearance *
<factor>`. The inherited root-fillet step then reads only the resulting `FilletRadius` parameter's
numeric value. Multiplying by `cos(HelixAngle)` is what makes the fillet radius read correctly on
the transverse plane of a tilted tooth.

The proof does not reach this step. The factor changes one parameter expression and no geometry: the
root fillet itself is spur's inherited feature, proved in `proof/spurgear`, and what helical changes
about it is a number Fusion's expression engine evaluates rather than a shape a solid engine can
build.

**From:** `spec/helicalgear/instructions.md` L31-33, L117-119; `spec/spurgear/instructions.md` L86-88, L391-396.

## H7 `[PROSE]` The twisted-profile plane offset — `helicalPlaneOffset`

`helicalPlaneOffset(self)` returns the offset of the twisted profile's plane from the base plane, as
a `ValueInput`. Helical returns the **full thickness**:

`self.getParameterAsValueInput(PARAM_THICKNESS)`

Keep this its own method and **do not inline the offset into `buildSketches`**: it is a distinct
overridable hook, and herringbone re-points it to half the thickness so its mirror plane lands
mid-body. H8 calls it.

Note this is a **numeric snapshot**, not a live parameter reference: `getParameterAsValueInput`
returns `ValueInput.createByReal(param.value)`, the `Thickness` value at generation time. Editing the
parameter afterwards does not move the plane; regenerate (`[PB-NUMERIC-SNAPSHOT]`,
`[SPUR-F-SNAPSHOT]`).

That the offset is the whole thickness is asserted on the built solid in H10, where the lofted tooth
is measured from the target plane to the twisted profile's plane.

**From:** `spec/helicalgear/instructions.md` L120-125; `spec/spurgear/fusion.md` L235-240; `.claude/skills/generate-gear/PLAYBOOK.md` L229-237.

## H8 `[PROSE]` The helix construction plane — `buildSketches`, after `super()`

`buildSketches(self, ctx)` annotates its parameter `ctx: SpurGearGenerationContext`, which is what
the inherited signature declares, and narrows it at the top of the body:

```python
def buildSketches(self, ctx: SpurGearGenerationContext):
    assert isinstance(ctx, HelicalGearGenerationContext)
```

⚠️ Do **not** annotate the parameter as `GenerationContext`, which is wider than the inherited
signature, and do **not** narrow it to `HelicalGearGenerationContext`, which is an error of its own.
The annotation matches the base and the assertion does the narrowing; `newContext` returns the
helical context, so the assertion always holds at runtime. Every read and write of `ctx.helixPlane`
or `ctx.twistedGearProfileSketch` happens after it.

The override's first action is `super().buildSketches(ctx)`, which draws the bottom `Gear Profile`
sketch and runs the spur tooth generator at angle 0. Then it creates the offset plane, on the gear's
own component (`[HELI-F-TWIST-PLANE]`, `[PB-CONSTRUCTION-PLANES]`):

`constructionPlaneInput = self.getComponent().constructionPlanes.createInput()`

`constructionPlaneInput.setByOffset(self.plane, self.helicalPlaneOffset())`

`plane = self.getComponent().constructionPlanes.add(constructionPlaneInput)`

Store it as `ctx.helixPlane`. The offset argument is a `ValueInput`, never a bare number.

⚠️ **The helix construction plane is left VISIBLE after generation.** It is never light-bulbed off:
spur's cleanup hides only the entities spur itself created — the Extrusion End Plane, the normalized
plane and the `Gear Center` axis — and helical adds no cleanup of its own. That is faithful,
deliberate behavior; a regeneration must **not** add cleanup for it. The same holds in SketchOnly
mode, where the plane is still created and still left lit.

No proof function realises this step on its own: it creates a construction plane and no measurable
geometry. What the plane is for is measured in H10, where the lofted tooth's height is read against
the offset this plane was created at.

**From:** `spec/helicalgear/instructions.md` L126-128, L136-156, L158-174, L217-222; `spec/helicalgear/fusion.md` L9-27, L29-42; `spec/spurgear/instructions.md` L384-387; `.claude/skills/generate-gear/PLAYBOOK.md` L659-671, L775-786.

## H9 `[GO]` The Twisted Gear Profile sketch

<!-- proof-run: proofkit.Run(twistedProfileCases, stepTwistedGearProfileSketch) -->

<!-- check-compile: ignore draw -->
Still inside `buildSketches`, and still after the assertion in H8, create the second sketch on the
plane H8 built and draw the tooth into it at the helix angle:

`loftSketch = self.createSketchObject('Twisted Gear Profile', plane=plane)`

`toothGenerator = SpurGearInvoluteToothDesignGenerator(loftSketch, self)`

`toothGenerator.draw(ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)`

Then store the sketch as `ctx.twistedGearProfileSketch`. The sketch's name is exactly
`'Twisted Gear Profile'`. `draw` is the spur tooth generator's own entry point, which no Fusion API
database carries, hence the exemption above; it is still a call this module must make.

The twist is delivered as the **`draw()` `angle` argument**, read as a raw `.value` in radians off
the `HelixAngle` parameter. The generator draws the whole tooth already rotated by that angle in its
own point math, and then, as the very last action after the entire constraint network exists, sets
the confirming angular dimension to the same angle (`[SPUR-F-ROTATE-CONFIRM]`, `[SPUR-F-SPINE]`).
**Do not draw the tooth flat and rotate it afterward**, and do not rely on the dimension alone to
swing it into place: that lets the solver pick the branch about 180 degrees away, and the loft in
H10 then passes through the gear centre. Helical does nothing else special here — it passes the
angle, and the generator builds the fully constrained rotated tooth, including its own projection of
`ctx.anchorPoint` and the coincidence that anchors the sketch.

`SpurGearInvoluteToothDesignGenerator(sketch, parent)` with `draw(anchorPoint, angle=0)` is borrowed
surface that must exist unchanged on spur. The generator is constructed with the default `angle=0`,
so its stored `self.toothAngle` is 0 and the live rotation comes from the `draw()` argument.

⚠️ **The Twisted Gear Profile sketch stays hidden its whole life.** `createSketchObject` returns a
hidden sketch and nothing ever shows it — not `buildSketches`, and not spur's cleanup, which touches
only its own three sketches. The loft's profile-finding works on the hidden sketch, and that is a
declared delta from `[PB-HIDE-AFTER-USE]`: there is no "shown, then hidden after use" phase, because
it is never shown at all. In SketchOnly mode the twisted profile is therefore not inspectable. A
regeneration must reproduce this rather than clean it up.

### What the proof establishes

`stepTwistedGearProfileSketch` rebuilds this sketch in the sketch engine and gates it on the engine's
own verdict (`[PB-SKETCH-FIRST]`, `[PB-FULL-CONSTRAINT]`): DOF 0, no conflicting or redundant
constraint, valid profiles, a system that is not near-singular, and no discrete ambiguity. It sweeps
the whole signed range the dialog accepts — the 14.5 degree default both ways, zero, a quarter turn
each way where the rib and chain dimensions swap axes, several sizes, the rib count down to two
samples, both routes into the embedded shape, and the anchor on and off the sketch origin.

Three things are asserted on the solved sketch. The tooth top lands on the tip circle at exactly the
helix angle, measured from the anchor the whole sketch was dragged onto — with the table's own
zero-angle case as the bottom section's baseline, that is the statement that the angle between the
two loft sections **is** the Helix Angle, not a lead angle derived from it, and that `Thickness`,
which does not appear in this sketch at all, cannot enter it. The confirming angular dimension is
asked whether it is satisfied rather than recomputed. And the sketch closes exactly two regions: the
tooth loop of 2 NURBS, 2 arcs and 2 lines that H10's fixed key matches, and the disc inside the root
circle, of the root circle's own area.

The embedded cases are where the loft's limitation is proved rather than asserted: there the tooth
loop has no stubs, so no loop in the sketch matches the fixed `lines=2` key, and an embedded helical
gear fails in the profile search before it can reach a loft.

The proof records three things it cannot reach, each beside the thing it cannot reach: the four
circle labels are sketch text, which the engine has no notion of and which in Fusion leaves
`isFullyConstrained` unreliable on any labelled sketch (`[PB-TEXT-HOLDS-DOF]`); the projection chain
of `[SPUR-F-ANCHOR-CHAIN]`, since the engine refuses another sketch's point as a foreign handle, so
the sketch carries its own local endpoint of the chain; and the tooth loop's trimmed boundary
parameters, which the engine withdraws from every partial edge in a scene holding a free-form entity,
so the loop is held to its curve counts rather than to its cuts.

**From:** `spec/helicalgear/instructions.md` L10-15, L126-128, L165-174, L181-190, L192-208, L217-222; `spec/helicalgear/fusion.md` L9-27, L29-42; `spec/spurgear/instructions.md` L254-325, L411-452, L554-558; `spec/spurgear/fusion.md` L19-31, L47-60, L69-106, L108-133, L135-175, L177-217; `.claude/skills/generate-gear/PLAYBOOK.md` L359-431, L441-493, L517-532, L614-634, L659-671.

## H10 `[GO]` Loft the tooth — `buildTooth` and `loftTooth`

<!-- proof-run: proofkit3d.RunSolid(loftCases, stepLoftTooth, assertLoftTooth) -->

<!-- check-step-calls: ignore buildTooth -->
`buildTooth(self, ctx)` replaces spur's tooth extrude. Its whole body is `self.loftTooth(ctx)`. It
does **not** extrude and it applies **no** chamfer — the inherited `chamferTeeth` runs later, on the
completed gear, after the pattern, the root fillets and the optional bore. `buildTooth` is called by
the inherited `buildMainGearBody` rather than by this module, so it is exempt above; `loftTooth` is
a call this module makes.

`loftTooth(self, ctx)` takes the same `ctx: SpurGearGenerationContext` annotation and the same
`isinstance` assertion as its narrowing, exactly as H8 writes it, then lofts bottom to top
(`[HELI-F-LOFT]`, `[PB-LOFT]`):

```python
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

Written out as the calls this step must make:
`find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)`,
`find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)`,
`lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`loftInput.loftSections.add(bottomToothProfile)`,
`loftInput.loftSections.add(topToothProfile)`,
`lofts.add(loftInput)`,
`loftResult.bodies.item(0)`.

The resulting body is `ctx.toothBody` and its name is exactly `'Tooth Body'`.

**Add the bottom section first, then the top.** Both sections are found with the framework helper;
do not re-implement the loop search, which rejects loops whose curve counts do not match and raises
when nothing matches (`[PB-PROFILE-MATCH]`).

⚠️ **Non-embedded only.** Both searches pass a fixed `nurbs=2, arcs=2, lines=2` — the non-embedded
six-curve tooth. This implementation does **not** read `ctx.toothProfileIsEmbedded` and has **no**
embedded branch, so an embedded low-tooth-count helical gear, whose flanks start inside the root
circle and whose loop therefore has `lines=0`, fails to find its profile. That is faithful to the
current code: a documented limitation, not a bug to fix here.

### What the proof establishes

`stepLoftTooth` builds the two sections on two planes of one world — the bottom tooth at angle 0 on
the target plane, the twisted tooth at the helix angle on a plane offset by the full `Thickness` —
and lofts them bottom first, exactly as this step does. `assertLoftTooth` then reads the solid.

The tooth starts on the target plane and is exactly `Thickness` tall, which is the reading that
holds `helicalPlaneOffset` (H7) at the full thickness rather than a fraction of it. Its twist is
read off its own centroid, which for a solid ruled between a tooth and the same tooth rotated sits
at exactly half the helix angle: that one reading says the twist is the Helix Angle rather than a
lead angle, that Thickness does not enter it — two cases differ only in Thickness — and that the
sign of the input is the hand of the built helix, since a left-hand helix that came out right-handed
would read at the opposite angle.

The pinned section order is proved by building the other one. The reversed loft is a valid solid of
the same handedness twisted by the same angle, so the swap is **silent** in the reading a caller is
most likely to check, which is why the order has to be pinned here rather than left to the
implementation. What it changes is the ruled walls: each is a quadrilateral that a twist makes
non-planar and that is built across one of its two diagonals, and reversing the sections picks the
other. The two solids straddle the true ruled volume and their mean is it exactly, which is asserted
against a closed form of the same two outlines; the gap between them is asserted to be larger than
either reading's own bound, and its measured size is printed per case — 6.8 percent of the ruled
volume at the 14.5 degree default, rising to 40.2 percent at a quarter turn. At zero twist there is
no diagonal to choose and the two orders are required to agree outright.

Two substitutions are recorded in the proof file, and both are forced. The flanks are chorded because
the engine refuses a free-form segment pairing, which two fitted splines are. The tooth-top and root
arcs are chorded too, because the bound the engine proves on an arc-walled loft's volume reading
lands outside its own relative tolerance at gear-tooth scale — measured, 0.0321 mm³ against a 0.0300
mm³ tolerance on a 1 module, 12 tooth, 10 mm tooth at 14.5 degrees — and the solid gate this proof is
held to admits such a reading for an area or a centroid but not for a volume. A section of lines
carries no such wall. What the chording costs is each chord's sagitta, so the lofted tooth is
slightly smaller than the real one; no assertion compares it against a closed-form involute tooth,
so it never enters one.

The proof also records where the ruled walls stop being buildable, measured **per sign**, because
this gear's spec deliberately quotes no range and the bound belongs to the proof's own modelling
rather than to the gear. Measured at three sizes on 2026-09-18, a positive twist builds to +95
degrees and is refused from +100, while a negative twist builds to -179 degrees, the widest tried.
The table carries a case past the positive bound, which skips with the engine's own refusal, so
every run says where that bound still is.

**From:** `spec/helicalgear/instructions.md` L42-49, L104-109, L129-131, L176-179, L181-190, L217-219; `spec/helicalgear/fusion.md` L46-65; `spec/spurgear/instructions.md` L292-300, L387-390, L564-568; `.claude/skills/generate-gear/PLAYBOOK.md` L145-158, L672-681, L724-728.

## H11 `[PROSE]` Everything else is spur's, unchanged

Helical overrides the methods in H4 through H10 and **nothing else**. Do not re-implement
`processInputs`, `prepareTools`, `buildMainGearBody`, `buildBody`, `patternTeeth`, `createFillets`,
`buildBore`, `chamferTeeth`, `cleanup`, or any part of `SpurGearInvoluteToothDesignGenerator`. Spur's
generation order runs untouched around helical's two deltas: the body extrude, the pattern and
combine, the root fillets, the optional bore, the completed-gear chamfer and the cleanup are spur's
code, and the component setup — the gear occurrence, the Tools sketch and anchor chain, the Gear
Profile sketch — is spur's too.

Helical adds **no cleanup of its own**, and the two visibility facts that follow from that are
deliberate: the helix construction plane is left visible (H8), and the Twisted Gear Profile sketch
stays hidden its whole life (H9). Reproduce both.

Helical also adds **no runtime full-constraint gate**. Spur registers none and helical adds none; the
twisted sketch's full constraint is a design-time property, proved by H9's bench rather than asserted
in the generated module. Nothing in this module calls `settle_sketch_display` either — the shared
command wrapper does that once, for every gear.

The completed-gear chamfer is shared and inherited without an override. It no longer uses a tooth-cap
edge count: it scans every planar face of the completed gear body parallel to the Gear Profile plane
and adds every unique boundary edge once, root-radius arcs included, excluding a bore's two circular
cap edges by the positive bore radius. That selection remains pending Fusion verification
(`[HELI-F-CHAMFER-COUNT]`).

**From:** `spec/helicalgear/instructions.md` L17-21, L23-25, L104-109, L132-134, L158-164, L186-190, L210-222; `spec/helicalgear/fusion.md` L67-80; `spec/spurgear/instructions.md` L373-381, L604-619; `spec/spurgear/fusion.md` L221-231; `.claude/skills/generate-gear/PLAYBOOK.md` L534-555.
