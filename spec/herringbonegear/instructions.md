# Herringbone Gear Creation Instructions

Herringbone is a **thin specialization of the helical gear** — a double-helical (chevron) tooth. It
**subclasses `HelicalGearGenerator`** and reuses everything helical does, changing exactly two things:
it puts the twisted-profile plane at **mid-body** (half thickness) instead of the far face, and its
`buildTooth` **lofts one half, then mirrors and combines** to form the symmetric chevron.

**Read `spec/helicalgear/instructions.md` (and, through it, `spec/spurgear/`) first.** This spec states
only herringbone's deltas over helical and cites those specs by anchor/section. Herringbone adds **no
new dialog input and no new user parameter** — its dialog and parameters are exactly helical's (Helix
Angle and all inherited spur parameters). The tooth profile is helical's twisted profile at the same
`angle=helixAngle`, already proven fully-constrained in `spec/helicalgear/sketch/`; herringbone's only
extra work — a mirror and a combine — is a **solid-body** operation, not a sketch, so it needs no
separate sketch-first proof.

## Component Setup / Variables

Identical to helical (inherited). Same inputs (including Helix Angle, default 14.5°), same derived
parameters, same input ids and parameter names. No additions.

## Architecture

Three classes extending their helical counterparts (names are the reproduced surface;
`commands/herringbonegear/entry.py` binds two by name):

1. **`HerringboneGearCommandConfigurator(HelicalGearCommandConfigurator)`** — trivial `pass`
   subclass (herringbone's dialog is helical's).
2. **`HerringboneGearGenerationContext(HelicalGearGenerationContext)`** — trivial `pass` subclass
   (same context fields; `ctx.helixPlane` is now the mid-body mirror plane).
3. **`HerringboneGearGenerator(HelicalGearGenerator)`** — overrides the methods in Method contract.

Imports (explicit, no `import *`): from `.base` — `GenerationContext`; from `.spurgear` —
`PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS`; from `.helicalgear` — `PARAM_HELIX_ANGLE,
HelicalGearCommandConfigurator, HelicalGearGenerationContext, HelicalGearGenerator`; plus `adsk.core`,
`adsk.fusion`.

## Generation Context

Helical's context unchanged (`HerringboneGearGenerationContext` is a `pass` subclass). `ctx.helixPlane`
is repurposed as the **mid-body mirror plane** (a consequence of the `helicalPlaneOffset` override
below); `ctx.twistedGearProfileSketch` is the (half-height) top loft section.

## Method contract — overrides only (helical's/spur's call graph is inherited)

Herringbone keeps the full inherited call graph (`spec/helicalgear/instructions.md` +
`spec/spurgear/instructions.md`). It overrides exactly:

- **`newContext()`** → `HerringboneGearGenerationContext()`.
- **`prefixBase()`** → `'HerringboneGear'`.
- **`generateName()`** → `'Herringbone Gear (M={}, Tooth={}, Thickness={}, Angle={})'.format(module.expression, toothNumber.expression, thickness.expression, helixAngle.expression)`
  — the four parameters' `.expression` strings (helical's form with the herringbone label).
- **`helicalPlaneOffset(self)`** → **half** the thickness, as a fresh `ValueInput`:
  ```python
  thickness = self.getParameter(PARAM_THICKNESS).value
  return adsk.core.ValueInput.createByReal(thickness / 2)
  ```
  (Helical returned the full thickness.) This places `ctx.helixPlane` at mid-body so the loft spans the
  bottom half and the mirror completes the top half. Note the value is a **raw computed offset**
  (`createByReal(thickness/2)`), not a parameter reference.
- **`buildTooth(self, ctx)`** → loft one half, mirror it across `ctx.helixPlane`, combine, then chamfer
  (`[HERR-F-MIRROR-COMBINE]`).

**Inherited unchanged — do NOT re-implement:** `addExtraPrimaryParameters`, `chamferWantEdges` (→4),
`filletHelixFactorExpression`, `buildSketches` (draws the twisted profile using the overridden
half-thickness `helicalPlaneOffset`), `loftTooth`, and the entire spur pipeline
(`prepareTools`/`buildBody`/`patternTeeth`/`createFillets`/`buildBore`/`cleanup`) and tooth generator.

## Generation Order — helical's, with one delta

Cite `spec/helicalgear/instructions.md` Generation Order. The `buildSketches` twisted-profile step is
unchanged in code but its plane now lands at **half thickness** (via the overridden
`helicalPlaneOffset`). The only code delta is **`buildTooth`**: instead of helical's `loftTooth` +
`chamferTooth`, herringbone does loft → mirror → combine → chamfer (`[HERR-F-MIRROR-COMBINE]`). Body
extrude, pattern+combine, fillets, bore, and cleanup remain spur's, unchanged. (The gear body is still
extruded across the **full** thickness by the inherited `buildBody`; only the lofted tooth is built
half-then-mirrored.)

## Dependencies

Herringbone **subclasses `HelicalGearGenerator`, `HelicalGearCommandConfigurator`, and
`HelicalGearGenerationContext`** (from `.helicalgear`), and transitively depends on spur. Generation
reads `spec/helicalgear/instructions.md` + `spec/helicalgear/fusion.md`, `spec/spurgear/*`, and the
current `lib/geargen/helicalgear.py` surface. Borrowed surface that must exist unchanged:

- **`HelicalGearGenerator`** and its overridable methods — `loftTooth(ctx)` (called first in
  `buildTooth`), `helicalPlaneOffset()` (overridden here), `chamferTooth(ctx)` (called last),
  `buildSketches` (inherited; consumes the overridden offset), and `PARAM_HELIX_ANGLE`.
- **Context fields** `ctx.helixPlane` and `ctx.toothBody` (produced by `loftTooth`).
- **Inherited framework helpers** `getComponent()`, `getParameter(name)`.
