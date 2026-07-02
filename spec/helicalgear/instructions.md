# Helical Gear Creation Instructions

Helical is a **thin specialization of the spur gear.** It **subclasses `SpurGearGenerator`** and
reuses the entire spur build pipeline verbatim (tools sketch, gear profile, body extrude, pattern,
fillets, bore, cleanup, and the whole tooth generator). It changes exactly three things: it adds one
**Helix Angle** input, draws a **second "Twisted Gear Profile" sketch** (the spur tooth generator run
at `angle=helixAngle`), and **lofts** the bottom profile to that twisted top profile instead of
extruding.

**Read `spec/spurgear/instructions.md` + `spec/spurgear/fusion.md` first.** This spec states only
helical's *deltas* and cites spur's contract by anchor (`[SPUR-F-…]`) and section name; it does not
restate spur's geometry. The Fusion-API mechanics unique to helical (the offset plane, the loft, the
chamfer edge count) live in `fusion.md` (`[HELI-F-…]`). The twisted profile's constraint scheme — the
spur tooth at a non-zero angle, i.e. the `[SPUR-F-SPINE]` angle≠0 path — is proven to fully constrain
in `spec/helicalgear/sketch/` (the sketch-first gate, `[PB-SKETCH-FIRST]`).

## Component Setup

Identical to spur (inherited). The gear occurrence, the Tools sketch and anchor chain, the Gear
Profile sketch, the body, the circular pattern, the root fillets, the optional bore, and cleanup are
all produced by the inherited `SpurGearGenerator` pipeline unchanged.

## Variables

**All spur inputs and derived parameters, inherited verbatim** (same input ids, same
`SpurGear<N>_`-style names, same formulas — see `spec/spurgear/instructions.md` Variables). Helical
adds **one** input and **one** derived parameter:

Helix Angle: user-specified angle. Default **14.5°**. The angle by which the top gear profile is
twisted relative to the bottom; the tooth is formed by lofting between them. Stored in the user
parameter **`HelixAngle`**, registered in **radians** (from a **degree** dialog input). It also drives
the root-fillet transverse correction via `filletHelixFactorExpression()` = `cos(HelixAngle)` (spur's
`* 1` fillet-factor hook — see spur Variables "Fillet Radius").

### Exact input ids and parameter-name strings

Inherits every spur input id and user-parameter name unchanged, and adds exactly one of each:

| Dialog label | input id       | user-parameter name |
|--------------|----------------|---------------------|
| Helix Angle  | `helixAngle`   | `HelixAngle`        |

- **Dialog input** — `addValueInput('helixAngle', 'Helix Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(14.5)))`:
  a **degree** value input defaulting to 14.5°. Added by `HelicalGearCommandConfigurator.configure`
  **after** `super().configure(cmd)` (spur `[SPUR-SUBCLASS-INPUT]`). Because spur's `configure` already
  added Parent Component **last**, the Helix Angle input necessarily appears **last in the dialog,
  after Parent Component.** ⚠️ This is the actual/current behavior — reproduce it exactly; do not
  attempt to insert Helix Angle earlier in the list.
- **User parameter** — registered by the overridden `addExtraPrimaryParameters` hook (spur
  `[SPUR-EXTRA-PARAMS]`): `helixAngle = get_value(inputs, 'helixAngle', 'rad')` then
  `self.addParameter('HelixAngle', helixAngle, 'rad', 'Helix angle for the helical gear')`. Registered
  in **`'rad'`** (the dialog is degrees; the parameter is radians).

Module-level constants: `PARAM_HELIX_ANGLE = 'HelixAngle'`, `INPUT_ID_HELIX_ANGLE = 'helixAngle'`.

## Architecture

Helical **subclasses the spur family** — three classes, each extending its spur counterpart. There is
no standalone generator; the spur pipeline is inherited. These names are the reproduced surface
(herringbone subclasses all three, and `commands/helicalgear/entry.py` binds two by name):

1. **`HelicalGearCommandConfigurator(SpurGearCommandInputsConfigurator)`** — `@classmethod def
   configure(cls, cmd)` that calls `super().configure(cmd)` then appends the Helix Angle value input.
2. **`HelicalGearGenerationContext(SpurGearGenerationContext)`** — `__init__(self)` calls
   `super().__init__()` then initializes the two new fields to a cast-None:
   `self.helixPlane = adsk.fusion.ConstructionPlane.cast(None)`,
   `self.twistedGearProfileSketch = adsk.fusion.Sketch.cast(None)`.
3. **`HelicalGearGenerator(SpurGearGenerator)`** — overrides the methods in Method contract below.

Imports (explicit, no `import *`, per the PLAYBOOK Module-layout rule): from `.spurgear` —
`PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS, SpurGearCommandInputsConfigurator,
SpurGearGenerationContext, SpurGearGenerator, SpurGearInvoluteToothDesignGenerator`; from `.base` —
`GenerationContext, get_value`; from `.utilities` — `find_profile_by_curve_counts`; plus `math`,
`adsk.core`, `adsk.fusion`.

## Generation Context — spur's, plus two fields

Cite spur's `SpurGearGenerationContext` field list (inherited unchanged — `ctx.plane`,
`ctx.anchorPoint`, `ctx.extrusionEndPlane`, `ctx.gearProfileSketch`, `ctx.toothBody`, `ctx.gearBody`,
`ctx.centerAxis`, `ctx.extrusionExtent`, `ctx.toothProfileIsEmbedded`). Helical adds two:

- **`ctx.helixPlane`** — the offset `ConstructionPlane` the twisted top profile is drawn on. (Also the
  mirror plane herringbone reflects across.)
- **`ctx.twistedGearProfileSketch`** — the second, "Twisted Gear Profile" sketch: the top loft section.

## Method contract — overrides only (spur's call graph is inherited)

Helical keeps spur's entire call graph and override boundaries (`spec/spurgear/instructions.md` Method
contract): `generate → processInputs → prepareTools → buildMainGearBody(buildSketches → buildTooth →
buildBody → patternTeeth[→createFillets]) → buildBore → cleanup`. **Do not move work across those
boundaries.** Helical overrides exactly these methods:

- **`newContext()`** → `HelicalGearGenerationContext()`.
- **`prefixBase()`** → `'HelicalGear'`.
- **`generateName()`** → `'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)`
  — the four parameters' **`.expression`** strings (spur's rule, extended with `HelixAngle`).
- **`addExtraPrimaryParameters(self, inputs)`** → registers `HelixAngle` (see Exact input ids). Overrides
  spur's no-op (`[SPUR-EXTRA-PARAMS]`).
- **`filletHelixFactorExpression(self)`** → `f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'` (spur base
  returns `'1'`). Multiplies the root-fillet radius by `cos(HelixAngle)` so it reads correctly on the
  tilted tooth's transverse plane.
- **`chamferWantEdges(self)`** → `4` (spur base returns 6). See `fusion.md` `[HELI-F-CHAMFER-COUNT]`
  (reproduce verbatim; it is flagged there).
- **`helicalPlaneOffset(self)`** → the offset of the twisted-profile plane from the base plane, as a
  `ValueInput`. Helical returns the full thickness: `self.getParameterAsValueInput(PARAM_THICKNESS)`.
  **This is a distinct overridable hook** (herringbone re-points it to half-thickness); keep it its own
  method — do not inline the offset into `buildSketches`.
- **`buildSketches(self, ctx)`** → calls `super().buildSketches(ctx)` (which draws the bottom Gear
  Profile and runs the spur tooth generator at angle 0), then draws the twisted top profile
  (`[HELI-F-TWIST-PLANE]`; see Generation Order).
- **`buildTooth(self, ctx)`** → `self.loftTooth(ctx)` then `self.chamferTooth(ctx)`. **Must end by
  calling `chamferTooth`** (spur's boundary — `buildMainGearBody` does not chamfer separately). Does
  **not** extrude.
- **`loftTooth(self, ctx)`** → lofts the two profiles into `ctx.toothBody` (`[HELI-F-LOFT]`).

**Inherited unchanged — do NOT re-implement:** `processInputs`, `prepareTools`, `buildMainGearBody`,
`buildBody`, `patternTeeth`, `createFillets`, `chamferTooth`, `buildBore`, `cleanup`, and the entire
`SpurGearInvoluteToothDesignGenerator`.

The three `ctx`-taking overrides — `buildSketches`, `buildTooth`, `loftTooth` — **annotate the
parameter as `ctx: GenerationContext`** (the base type imported from `.base`), matching the inherited
signatures. This is why `GenerationContext` is in the import list; keep the annotation so the import
is used.

## Generation Order — spur's 12 steps, with two deltas

Cite `spec/spurgear/instructions.md` Generation Order. Helical changes exactly two steps; everything
else (body extrude step 9, pattern+combine step 10, fillets step 11, bore step 12, cleanup) runs
spur's code untouched.

**Delta 1 — `buildSketches` (extends spur steps 3–5).** After `super().buildSketches(ctx)` has drawn
the bottom Gear Profile and run the tooth generator (at angle 0):
1. create an offset construction plane at `self.helicalPlaneOffset()` from `self.plane`, store it as
   `ctx.helixPlane`;
2. create a sketch named `'Twisted Gear Profile'` on that plane;
3. draw the tooth into it with `SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=self.getParameter(PARAM_HELIX_ANGLE).value)`
   — the **twist is delivered as the `draw()` `angle`** (spur's tooth generator rotates the whole
   tooth by `angle`; do not draw flat and rotate afterward — `[SPUR-F-ROTATE-CONFIRM]`);
4. store the sketch as `ctx.twistedGearProfileSketch`.
See `[HELI-F-TWIST-PLANE]`.

**Delta 2 — `buildTooth` (replaces spur step 7's extrude).** Loft the bottom Gear Profile tooth loop
to the top twisted tooth loop → `ctx.toothBody` named `'Tooth Body'`, then `self.chamferTooth(ctx)`.
See `[HELI-F-LOFT]`.

## Sketch-discipline deltas

- **The twisted profile is drawn at `angle=helixAngle`** — the spur tooth generator's **angle≠0**
  path applies verbatim (`[SPUR-F-SPINE]`: pre-rotated geometry + a +X horizontal reference line whose
  far end is pinned on the tip circle + a confirming angular dimension; `[SPUR-F-ROTATE-CONFIRM]`).
  Helical does nothing special here — it just passes `angle=helixAngle` to `draw()`; the generator
  builds the fully-constrained rotated tooth. This angle≠0 path is proven in `spec/helicalgear/sketch/`.
- **Both profiles are the non-embedded 6-curve tooth** (2 splines + 2 arcs + 2 lines). Helical
  **requires non-embedded** — see `[HELI-F-LOFT]` and its limitation note.
- **No runtime full-constraint gate.** Spur registers none, and helical adds none; the twisted
  sketch's full-constraint is a design-time property, proven by the sketch bench, not asserted in code.

## Dependencies

Helical **subclasses `SpurGearGenerator`, `SpurGearCommandInputsConfigurator`, and
`SpurGearGenerationContext`** (from `.spurgear`), and additionally constructs
`SpurGearInvoluteToothDesignGenerator` directly for the twisted profile. Generation reads
`spec/spurgear/instructions.md` + `spec/spurgear/fusion.md` and the current `lib/geargen/spurgear.py`
surface. The borrowed surface that must exist unchanged on spur:

- **The three base classes and their method boundaries** (spur Method contract): the full call graph
  and the overridable hooks `addExtraPrimaryParameters` (`[SPUR-EXTRA-PARAMS]`), `chamferWantEdges`,
  `filletHelixFactorExpression`, and the configurator extension seam `[SPUR-SUBCLASS-INPUT]`.
- **The tooth generator** `SpurGearInvoluteToothDesignGenerator(sketch, parent)` with
  `draw(anchorPoint, angle=0)` — the `angle` rotates the whole tooth (`[SPUR-F-ROTATE-CONFIRM]`).
- **Context fields** `ctx.anchorPoint`, `ctx.gearProfileSketch`, `ctx.toothBody`,
  `ctx.toothProfileIsEmbedded`, and `self.plane`.
- **Inherited framework helpers**: `getComponent()`, `createSketchObject(name, plane=…)`,
  `getParameter(name)`, `getParameterAsValueInput(name)`, `parameterName(name)`.

## Honesty note — faithful-but-flagged behaviors

Reproduced verbatim from `lib/geargen/helicalgear.py`; **do not "fix" them in the spec** (fixing
changes behavior and belongs in a separate, deliberate change once verified in Fusion):

- **Helix Angle sits last in the dialog**, after Parent Component (`[SPUR-SUBCLASS-INPUT]` consequence).
- **`chamferWantEdges()` returns 4** for a non-embedded 6-curve lofted tooth — see
  `[HELI-F-CHAMFER-COUNT]`; helical chamfer is likely fragile for the default tooth and effectively
  exercised only at chamfer 0.
- **Embedded (low-tooth-count) helical is unsupported** — `loftTooth` hardcodes `lines=2` and never
  reads `ctx.toothProfileIsEmbedded` (`[HELI-F-LOFT]`).
