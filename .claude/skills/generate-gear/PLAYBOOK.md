# Gear Generator Playbook — framework conventions (gear-agnostic)

This playbook captures the **repo scaffolding and Fusion-API conventions** shared by every gear
generator in `lib/geargen/`. It is the *how*. A per-gear spec (e.g. `lib/geargen/spurgear.md`)
supplies the *what* — the geometry. A code generator is given **both** the spec and this
playbook, and must not need anything else.

Deliberately, this file contains **no gear-specific geometry** (no involute math, no
tooth/root/flank construction). That belongs in the spec, so that regenerating a `.py` from its
spec is an honest test of the spec's completeness.

## Module layout & imports

Each gear lives in `lib/geargen/<gear>.py` and imports exactly:

```python
import math
from ...lib import fusion360utils as futil
from .misc import *        # to_cm, to_mm, get_design, get_ui
from .base import *        # Generator, GenerationContext, get_value/get_boolean/get_selection, ParamNamePrefix, ComponentCleaner
from .utilities import *   # get_normal
```

Bevel additionally does `import adsk.core, adsk.fusion`; spur relies on the star-imports
pulling `adsk` in transitively. Match the reference file's import list.

Module-level constants name the recurring parameters and dialog input ids, e.g.
`PARAM_MODULE = 'Module'`, `PARAM_TOOTH_NUMBER = 'ToothNumber'`,
`INPUT_ID_PARENT = 'parentComponent'`, `INPUT_ID_PLANE = 'plane'`,
`INPUT_ID_ANCHOR_POINT = 'anchorPoint'`. Subclasses (helical) add their own,
e.g. `PARAM_HELIX_ANGLE`.

## The four classes

A gear module is built from four cooperating classes. Names are part of the public API —
subclasses in `helicalgear.py` / `herringbonegear.py` bind to them **by name** (see Contract).

1. **`<Gear>CommandInputsConfigurator`** — a class with `@classmethod def configure(cls, cmd)`
   that adds the dialog inputs to `cmd.commandInputs`. Selection inputs use
   `addSelectionInput(id, label, tooltip)` + `addSelectionFilter(...)` + `setSelectionLimits(1)`;
   the parent selection pre-selects `get_design().rootComponent`. Value inputs use
   `addValueInput(id, label, unit, ValueInput.createByReal(default))`; booleans
   `addBoolValueInput(...)`; expression-style fields `addStringValueInput(...)`.

2. **`<Gear>GenerationContext(GenerationContext)`** — a plain data carrier whose `__init__`
   declares the fields passed between steps (each `cast(None)` initialised). These field names
   are public API; subclasses read and add to them.

3. **`<Gear>InvoluteToothDesignGenerator`** — constructed as `(sketch, parent)` (optionally an
   `angle`); draws the tooth/profile geometry into the given sketch via `.draw(anchorPoint,
   angle=0)` and related helpers. This is where the spec's geometry section is realised. The
   `parent` is the generator, used to read parameters via `parent.getParameter(...)` /
   `parent.parameterName(...)`.

4. **`<Gear>Generator(Generator)`** — the orchestrator. Subclass of `base.Generator`. Holds
   `processInputs`, `generate`, the parameter registration, and the per-step build methods.

## `base.Generator` — what you inherit (do not re-implement)

`lib/geargen/base.py` provides:

- `__init__(self, design)` — sets `self.design`, `self.parentComponent`, `self.occurrence`,
  `self.prefix`, `self.cleaner`.
- `getOccurrence()` — lazily creates the child occurrence under `self.parentComponent`, and on
  first creation builds the `ParamNamePrefix` as `f'{self.prefixBase()}_{component.id without
  dashes}'` and a `ComponentCleaner`. **Calling this (or anything that calls it transitively,
  including `parameterName()`/`addParameter()`) shifts Fusion's active component context.**
- `getComponent()` → `getOccurrence().component`.
- `addParameter(name, ValueInput, units, comment)` → `design.userParameters.add(parameterName(name), …)`.
- `getParameter(name)`, `getParameterAsValueInput(name)`, `getParameterAsBoolean(name)`.
- `parameterName(name)` → `f'{prefix}_{name}'` (creates the occurrence if needed).
- `prefixBase()` → default `'Gear'`; override per gear (`'SpurGear'`, `'HelicalGear'`, …).
- `deleteComponent()` — deletes the occurrence; used by the entry point on failure.
- `createSketchObject(name, plane=None)` — adds a sketch (defaults to XY plane), names it, sets
  `isVisible=False`, returns it. (Note: a sketch created this way starts hidden; make it visible
  while you draw/consume profiles, per the spec's sketch-discipline rules.)
- `@abstractmethod generate(self, inputs)` — every gear implements this.

Helpers (free functions in `base.py`): `get_selection(inputs, id)` → `(list_of_entities, ok)`;
`get_boolean(inputs, id)` → `(value, ok)`; `get_value(inputs, id, units)` → `(value, ok)`.

**`get_value` return contract — important, easy to get wrong.** It does NOT always return a
`ValueInput`:
- Numeric `ValueCommandInput`, valid expression → `(ValueInput.createByReal(evaluated), True)`.
- Numeric input, invalid expression → `(None, False)`.
- `StringValueCommandInput` whose text **is** an existing user-parameter name →
  `(ValueInput.createByString(text), True)`.
- `StringValueCommandInput` holding a literal expression (e.g. the bore field's default
  `'0 mm'`) → `(evaluated_number, False)` — a **raw float, not a `ValueInput`, and `ok` is False**.

`addParameter(name, value, units, comment)` calls `userParameters.add(...)`, whose value
argument **must be a `ValueInput`** — passing the raw float raises
`TypeError: ... argument 3 of type 'adsk::core::Ptr< adsk::core::ValueInput >'`. So whenever
`get_value` can return `ok=False` (i.e. any `addStringValueInput` field such as Bore Diameter),
the caller must wrap the returned value in a `ValueInput` before `addParameter`. **Wrap the
evaluated number `get_value` handed back — do not discard it to 0.** That returned float is
already in Fusion's internal units (cm), which is exactly what `ValueInput.createByReal` expects,
so wrapping it preserves the user's value; a literal like `'5 mm'` then actually takes effect.
(The stock generator wrote `createByReal(0)` here, which silently threw the value away — so a
typed Bore Diameter never cut a hole. Preserve the value instead.)

```python
(boreDiameter, ok) = get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm')
if not ok:
    # get_value returned the evaluated value as a raw number (internal cm).
    # Wrap THAT — wrapping 0 would silently ignore a typed bore value.
    boreDiameter = adsk.core.ValueInput.createByReal(
        boreDiameter if isinstance(boreDiameter, (int, float)) else 0)
self.addParameter(PARAM_BORE_DIAMETER, boreDiameter, 'mm', 'Size of the bore')
```

(Numeric inputs — module, tooth number, pressure angle, thickness, chamfer — return `ok=True`
with a `ValueInput`, so they don't need this; only the string-value inputs do.)

## `processInputs` pattern

Order is load-bearing (see the spec's "Generation Order"): **pull every selection input out of
`inputs` and stash it on `self` before anything creates the occurrence.** Numeric/boolean inputs
are immune to the context shift, but they must be read and registered as parameters, which *does*
create the occurrence — so read selections first.

1. `get_selection` for parent → resolve `Occurrence.component` vs `Component` into
   `self.parentComponent`; raise on wrong count/type.
2. `get_selection` for plane and anchor point → `self.plane`, `self.anchorPoint`.
3. For each numeric/bool input: `get_value`/`get_boolean`, then `addParameter(...)` with the
   right units (`''`, `'mm'`, `'rad'`).
4. Call the `addExtraPrimaryParameters(inputs)` hook (subclasses inject primaries like
   `HelixAngle` here, **before** derived parameters reference them).
5. Register **derived** parameters as live Fusion expression strings via
   `ValueInput.createByString(f'{parameterName(...)} * …')` — e.g. PitchCircleDiameter,
   …Radius, Base/Root/Tip circles, InvoluteSteps. The exact formulas live in the spec's
   Variables section.
6. Parameters Fusion's expression engine can't evaluate (anything mixing the unitless `tan()`
   output with a radian value) are **pre-computed in Python** and registered with
   `ValueInput.createByReal(...)` — e.g. the root valley angle. The spec flags which ones.

Caveat to honor (already in the spec's Sketch Discipline): sketch **dimensions** and **feature
inputs** are set from the *current numeric value* of a parameter at generation time, not as live
links — editing a `<prefix>_…` parameter does not update an existing gear; regenerate.

## `generate` orchestration

`generate(inputs)`:
1. `processInputs(inputs)`.
2. `component = self.getComponent(); component.name = self.generateName()`.
3. Normalise the plane: if `self.plane` is not a `ConstructionPlane`, create a coplanar one via
   `constructionPlanes.add(createInput().setByOffset(self.plane, 0))` and replace `self.plane`.
4. `ctx = self.newContext()`.
5. Run the build steps in the order the spec's "Generation Order" lists. (The reference spur
   code groups them as prepareTools → buildMainGearBody → buildBore, with the tooth/body/pattern/
   fillet/chamfer sub-steps inside; an equivalent decomposition is fine as long as the order and
   the subclass-facing method names below are preserved.)

## Subclass-extension contract (MUST be preserved verbatim)

`helicalgear.py` and `herringbonegear.py` subclass the spur classes and override/call these by
name. Regenerated spur code that renames or drops any of them breaks those gears:

- Classes: `SpurGearCommandInputsConfigurator`, `SpurGearGenerationContext`,
  `SpurGearInvoluteToothDesignGenerator`, `SpurGearGenerator`.
- Overridable methods on the generator: `newContext(self)`, `prefixBase(self)`,
  `generateName(self)`, `addExtraPrimaryParameters(self, inputs)`,
  `filletHelixFactorExpression(self)`, `chamferWantEdges(self)`, `buildSketches(self, ctx)`,
  `buildTooth(self, ctx)`. (`buildSketches`/`buildTooth` are called via `super()` by helical, so
  they must exist as distinct, overridable steps.)
- Tooth generator entry point: `SpurGearInvoluteToothDesignGenerator(sketch, parent)` with
  `.draw(anchorPoint, angle=<radians>)` — helical passes a non-zero `angle`.
- Context fields consumed by subclasses: at minimum `ctx.anchorPoint`, `ctx.gearProfileSketch`,
  `ctx.toothBody` (helical also *adds* `ctx.helixPlane`, `ctx.twistedGearProfileSketch`). Keep
  every field the spec's "Generation Context — canonical field names" section lists.
- Inherited helpers relied on: `self.parameterName(...)`, `self.getParameter(...)`,
  `self.getComponent()`, `self.createSketchObject(...)`.

`filletHelixFactorExpression()` returns `'1'` for spur (helical returns
`f'cos({self.parameterName(PARAM_HELIX_ANGLE)})'`). `chamferWantEdges()` returns the expected
front-face edge count (spur tooth loop count; helical overrides for the lofted/embedded tooth).

## Command-entry wiring (`commands/<gear>/entry.py`)

Standard Fusion add-in command module (mostly boilerplate; match the reference):
`GEAR_TYPE`/`CMD_ID`/`CMD_NAME`; `start()` registers the button + dropdown and the
`commandCreated` handler; `command_created` calls
`geargen.<Gear>CommandInputsConfigurator.configure(args.command)` and wires the
execute/inputChanged/executePreview/validateInputs/destroy handlers via `futil.add_handler`;
`command_execute` does `g = geargen.<Gear>Generator(design); g.generate(inputs)` inside
`try/except` and calls `g.deleteComponent()` on failure (errors surfaced with
`futil.handle_error("Generation error", show_message_box=True)`).

## Fusion-API conventions & gotchas (cross-gear)

- **Driving vs driven dimensions:** `add*Dimension(...)` is *driving* by default; never pass the
  trailing `isDriven=True`/`True` (it inverts to a measured dimension and lets geometry float).
- **Selections drop on context shift:** stash selection entities before occurrence creation
  (above).
- **Hide a sketch only after consuming it:** projections/profile extraction stop working on an
  invisible sketch; draw → project → constrain → run features → then `isVisible=False`. Same for
  construction planes later features still consume.
- **Profiles are found by curve count/type**, not index — iterate `sketch.profiles` /
  `profile.profileLoops` and match the loop whose `profileCurves` have the expected
  count/type mix. Spec says which counts (and the embedded vs non-embedded variants).
  Read a curve's type with `curve.geometry.curveType` and compare against the
  `adsk.core.Curve3DTypes` enum. **The exact member names end in `...CurveType`** (this is
  easy to get wrong — `NurbsCurve3DType`/`Arc3DType` do NOT exist and raise `AttributeError`).
  The correct constants:
  - `adsk.core.Curve3DTypes.Line3DCurveType`
  - `adsk.core.Curve3DTypes.Arc3DCurveType`
  - `adsk.core.Curve3DTypes.Circle3DCurveType`
  - `adsk.core.Curve3DTypes.NurbsCurve3DCurveType`
  (The spur tooth loop is 2 `NurbsCurve3DCurveType` flanks + 2 `Arc3DCurveType` + 2
  `Line3DCurveType` flank-to-root lines when non-embedded, or 2 NURBS + 2 arcs when embedded;
  the annular body loop is exactly 2 `Arc3DCurveType`/`Circle3DCurveType`.)
- **Patterns return original + copies:** `CircularPatternFeature.bodies` already includes the
  seed body plus copies; feed the whole collection to Combine — don't re-add the seed.
- **Logging/robustness:** use `futil.log(...)` for step progress; let the entry point's
  try/except + `deleteComponent()` handle rollback rather than swallowing errors mid-step (match
  the reference's level of try/except — don't invent new silent failure paths).

## What may differ from the reference (still "logically equivalent")

Local variable names, comments, log strings, and how build steps are decomposed into private
helper methods. What may **not** differ: the contract names above, parameter names/ids, the
geometry the spec prescribes, feature order, and edge-case handling.
