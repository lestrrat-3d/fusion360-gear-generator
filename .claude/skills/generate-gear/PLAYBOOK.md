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

Some gears import `adsk.core, adsk.fusion` explicitly; others rely on the star-imports pulling
`adsk` in transitively. Use the same import list as the existing gear modules.

Module-level constants name the recurring parameters and dialog input ids, e.g.
`PARAM_MODULE = 'Module'`, `INPUT_ID_PARENT = 'parentComponent'`, `INPUT_ID_PLANE = 'plane'`,
`INPUT_ID_ANCHOR_POINT = 'anchorPoint'`. A subclass may add its own (e.g. a helix-angle param).

## The four-class pattern (involute family — one option, not mandatory)

This is the architecture the **involute gear family** (`spurgear` + its `helicalgear` /
`herringbonegear` subclasses) uses. It is **not** the only valid shape: a gear is free to use a
different class layout, skip `GenerationContext`, or not subclass `base.Generator` at all (e.g.
`bevelgear` does none of these). **The gear's own spec declares its architecture** (see the
skill's "Required spec sections"); use this four-class pattern only when that spec says to.

When a gear *does* use it, the four class names are part of the public API — subclasses bind to
them **by name** (see Contract), so reproduce them exactly:

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

Use this **when the gear's spec says it subclasses `base.Generator`** (the involute family does;
a gear with a standalone generator, e.g. `bevelgear`, manages its own occurrence/cleanup instead
and the spec describes that). When inherited, `lib/geargen/base.py` provides:

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
- `StringValueCommandInput` holding a literal expression (e.g. a default like `'0 mm'`) →
  `(evaluated_number, False)` — a **raw float, not a `ValueInput`, and `ok` is False**.

`addParameter(name, value, units, comment)` calls `userParameters.add(...)`, whose value
argument **must be a `ValueInput`** — passing the raw float raises
`TypeError: ... argument 3 of type 'adsk::core::Ptr< adsk::core::ValueInput >'`. So whenever
`get_value` can return `ok=False` (i.e. any `addStringValueInput` field), the caller must wrap
the returned value in a `ValueInput` before `addParameter`. **Wrap the evaluated number
`get_value` handed back — do not discard it to 0.** That returned float is already in Fusion's
internal units (cm), which is exactly what `ValueInput.createByReal` expects, so wrapping it
preserves the user's value; a literal like `'5 mm'` then actually takes effect. (Discarding it to
`createByReal(0)` silently throws the user's value away — e.g. a typed bore diameter that then
never cuts a hole.)

```python
# For any addStringValueInput field <FIELD>:
(value, ok) = get_value(inputs, INPUT_ID_<FIELD>, 'mm')
if not ok:
    # get_value returned the evaluated value as a raw number (internal cm).
    # Wrap THAT — wrapping 0 would silently ignore a typed value.
    value = adsk.core.ValueInput.createByReal(
        value if isinstance(value, (int, float)) else 0)
self.addParameter(PARAM_<FIELD>, value, 'mm', '…')
```

(Numeric inputs return `ok=True` with a `ValueInput`, so they don't need this wrap; only the
string-value inputs do.)

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
4. Call the `addExtraPrimaryParameters(inputs)` hook (subclasses inject their own primary
   parameters here, **before** derived parameters reference them).
5. Register **derived** parameters as live Fusion expression strings via
   `ValueInput.createByString(f'{parameterName(...)} * …')`. The exact parameters and formulas
   live in the spec's Variables section.
6. Any parameter Fusion's expression engine can't evaluate (e.g. one mixing the unitless `tan()`
   output with a radian value in a subtraction) is **pre-computed in Python** and registered with
   `ValueInput.createByReal(...)`. The spec flags which ones. Watch the registration **units**
   too: an angle (radian magnitude) that is later used as a bare multiplier of a length must be
   registered **unitless** (`''`), not `'rad'`, or Fusion rejects the dependent length expression
   (`mm·rad`) as invalid.

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
5. Run the build steps in the order the spec's "Generation Order" lists. The spec's "Method
   contract" section pins which build methods own which steps; preserve those method boundaries
   and names (subclasses override at them). Private-helper decomposition beyond that is free.

## Dependent-gear contract (preserve verbatim — when a spec declares one)

This applies **only when a gear's spec declares dependents** — another gear that either subclasses
its classes or imports one of them. For the involute gears, `helicalgear.py` and
`herringbonegear.py` subclass `spurgear`'s classes and override/call members **by name**; a gear
may also be borrowed by composition (e.g. `bevelgear` imports `spurgear`'s tooth generator). In
any such case the depended-on gear's public surface is a contract a regenerated `.py` must
preserve exactly, or the dependents break. That surface typically includes:

- the class names the dependent binds to;
- the overridable hook methods — kept as **distinct, overridable steps** (subclasses override them
  and call them via `super()`);
- the tooth/profile-generator entry point and its `draw(anchorPoint, angle=<radians>)` signature
  (a subclass may pass a non-zero `angle`);
- every `GenerationContext` field name the dependent reads or extends (for gears that use a
  context object);
- the inherited `base.Generator` helpers the dependent leans on — `parameterName`, `getParameter`,
  `getComponent`, `createSketchObject`.

**The concrete names, and whether any dependents exist at all, live in that gear's spec, not
here** — see the spec's "Method contract / call graph", "Generation Context", and "Dependencies"
sections, which enumerate the exact classes, hook methods, call graph, `ctx` fields, and
borrowed/inherited surface to reproduce. Treat them as fixed identifiers; don't rename or drop any.
(Hooks typically include ones a subclass uses to vary behaviour — a fillet-scaling expression, an
expected front-face edge count, etc.; the spec says what each returns for the depended-on gear.)

## Command-entry wiring (`commands/<gear>/entry.py`)

Standard Fusion add-in command module (mostly boilerplate; mirror the existing command modules):
`GEAR_TYPE`/`CMD_ID`/`CMD_NAME`; `start()` registers the button + dropdown and the
`commandCreated` handler; `command_created` calls
`geargen.<Gear>CommandInputsConfigurator.configure(args.command)` and wires the
execute/inputChanged/executePreview/validateInputs/destroy handlers via `futil.add_handler`;
`command_execute` does `g = geargen.<Gear>Generator(design); g.generate(inputs)` inside
`try/except` and calls `g.deleteComponent()` on failure (errors surfaced with
`futil.handle_error("Generation error", show_message_box=True)`).

## Fusion-API conventions & gotchas (cross-gear)

- **Sketch curve collections live under `sketch.sketchCurves`, never on the sketch directly.** Add
  geometry via `sketch.sketchCurves.sketchCircles`, `.sketchLines`, `.sketchArcs`,
  `.sketchFittedSplines`. `sketch.sketchCircles` (etc.) does NOT exist and raises `AttributeError:
  'Sketch' object has no attribute 'sketchCircles'`. (By contrast `sketch.sketchPoints`,
  `sketch.sketchDimensions`, `sketch.geometricConstraints`, `sketch.sketchTexts`, and
  `sketch.profiles` ARE direct members of the sketch — only the *curve* collections sit under
  `sketchCurves`.)
- **Selection filters take enum constants, not strings.** `selectionInput.addSelectionFilter(...)`
  expects `adsk.core.SelectionCommandInput.<Member>` (an int enum), e.g.
  `addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)`. Passing the string
  `'ConstructionPlanes'` raises a `TypeError` on the argument. The member names match the entity
  kinds (`ConstructionPlanes`, `PlanarFaces`, `ConstructionPoints`, `SketchPoints`, `Occurrences`,
  `RootComponents`, `Bodies`, …) — but they must be the enum attribute, not a quoted literal.
- **Fillet vs chamfer edge-sets are asymmetric — easy to get backwards:**
  - **Fillet:** add the edge set on the input **itself** —
    `filletInput.addConstantRadiusEdgeSet(edges, ValueInput, isTangentChain)`. There is no
    `filletInput.edgeSetInputs`; reaching for it raises `AttributeError`.
  - **Chamfer:** add the edge set on the input's **`chamferEdgeSets`** collection —
    `chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, ValueInput, isFlipped)`.
  Do not mirror one onto the other.
- **Driving vs driven dimensions:** `add*Dimension(...)` is *driving* by default; never pass the
  trailing `isDriven=True`/`True` (it inverts to a measured dimension and lets geometry float).
- **Selections drop on context shift:** stash selection entities before occurrence creation
  (above).
- **Hide a sketch only after consuming it:** projections/profile extraction stop working on an
  invisible sketch; draw → project → constrain → run features → then `isVisible=False`. Same for
  construction planes later features still consume.
- **Profiles are found by curve count/type**, not index — iterate `sketch.profiles` /
  `profile.profileLoops` and match the loop whose `profileCurves` have the expected count/type
  mix (the spec states the expected counts per loop, including any variant cases). Read a curve's
  type with `curve.geometry.curveType` and compare against the `adsk.core.Curve3DTypes` enum.
  **The exact member names end in `...CurveType`** (easy to get wrong — `NurbsCurve3DType` /
  `Arc3DType` do NOT exist and raise `AttributeError`). The correct constants:
  - `adsk.core.Curve3DTypes.Line3DCurveType`
  - `adsk.core.Curve3DTypes.Arc3DCurveType`
  - `adsk.core.Curve3DTypes.Circle3DCurveType`
  - `adsk.core.Curve3DTypes.NurbsCurve3DCurveType`
- **Along-path sketch text** (e.g. labelling a circle) has a fixed three-call shape — use it
  exactly; don't guess or wrap it defensively, this is the working API:
  ```python
  textInput = sketch.sketchTexts.createInput2(text, height)   # height in cm (internal units)
  textInput.setAsAlongPath(curve, True,
                           adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
  sketch.sketchTexts.add(textInput)
  ```
  `createInput2(text, height)` takes the string and a text height; `setAsAlongPath(curve,
  isAbovePath, horizontalAlignment, offset)` lays it along the curve. The spec gives the text
  content and the height to use.
- **Patterns return original + copies:** `CircularPatternFeature.bodies` already includes the
  seed body plus copies; feed the whole collection to Combine — don't re-add the seed.
- **Logging/robustness:** use `futil.log(...)` for step progress; let the entry point's
  try/except + `deleteComponent()` handle rollback rather than swallowing errors mid-step — don't
  invent new silent failure paths.

## What the spec need not pin (free to vary)

Local variable names, comments, log strings, and how build steps are decomposed into private
helper methods. What the spec **does** pin and the output must reproduce: the contract surface (the
classes, hook methods, `ctx` fields, and call graph the spec declares), parameter names/ids, the
geometry the spec prescribes, feature order, and edge-case handling.
