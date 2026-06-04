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
- **`sketch.modelToSketchSpace(p)` / `sketch.sketchToModelSpace(p)` are point-transforming
  METHODS, not matrices.** Each takes a `Point3D` (world resp. sketch) and returns the transformed
  `Point3D`: `local = sketch.modelToSketchSpace(worldPoint)`. Do NOT do `m = sketch.modelToSketchSpace`
  then `worldPoint.transformBy(m)` — passing a bound method where `Point3D.transformBy` expects a
  `Matrix3D` raises `TypeError: … argument 2 of type 'Matrix3D'`. (To map a world point into a
  sketch — e.g. an apex computed from a plane normal — call the method directly.)
- **Read SOLVED `.geometry` for any computed-from-geometry bound — not the seed coordinates.** When
  a value is derived from the positions of constrained sketch points (a clearance, a max-offset
  bound, a measured distance), read each point's `.geometry` *after* the constraints that locate it
  have been added (Fusion solves incrementally, so the point is already in its solved position).
  Do NOT reuse the raw `Point3D` seed coordinates you passed to `addByTwoPoints` — seeds can diverge
  from the solved positions (markedly so for asymmetric / non-symmetric configurations), making a
  seed-based bound wrong even though the same formula on solved geometry is correct.
- **A sketch entity you'll read `.worldGeometry` from must be CONSTRAINED, not free.** If you later
  take `someSketchLine.startSketchPoint.worldGeometry` (or a sketch point's `worldGeometry`) to build
  a world-space axis/origin/tool, that point must be pinned (shared with, or coincident to, fixed
  geometry). A **fully-unconstrained** sketch line/point has an **undefined `worldGeometry`** — Fusion
  resolves its free DOF against a default/world frame, so the value silently comes back wrong and any
  feature built from it (a rotation axis, a move) lands the body in the wrong place. Drawing a line
  from raw `.geometry` coordinates *without* coincident-pinning its endpoints is the classic trap:
  it looks placed, but its `worldGeometry` is not trustworthy.
- **Seed coordinates near the solved position — the solver is seed-sensitive.** When you create a
  curve/point from raw `Point3D` coordinates that constraints will then pin, place the seed close to
  where it will end up and on the correct side. Fusion's sketch solver is an iterative numerical
  solver: a seed that starts far away, or on the wrong side of a parallel/offset target, can fail to
  converge with `RuntimeError … VCS_SKETCH_SOLVING_FAILED` *even though the constraint system is
  perfectly solvable*. (Seeds are not constraints — they don't pin anything — but a bad seed still
  breaks the solve. Especially: a point that must end up *on another line* should be seeded near
  that line, not near some unrelated point.)
- **Never double-bind a point — share it OR coincident a fresh one, never both.** To start/anchor
  a curve at an existing sketch point, choose ONE: (a) pass that `SketchPoint` object directly into
  the creation call — `addByTwoPoints(existingPoint, otherEnd)` — so the curve *shares* it, and add
  **no** coincident; or (b) create the curve from raw `Point3D` coordinates and add **exactly one**
  `addCoincident(curve.startSketchPoint, existingPoint)`. Doing **both** (passing the shared point
  *and* then adding a coincident to it) stacks a redundant self-coincident and the solver dies with
  `RuntimeError … VCS_SKETCH_SOLVING_FAILED`. Same trap: never `addCoincident` two points that are
  already the same shared point. (This is the single most common over-constraint failure; it bit
  both spur and bevel.)
- **Driving vs driven dimensions:** `add*Dimension(...)` is *driving* by default; never pass the
  trailing `isDriven=True`/`True` (it inverts to a measured dimension and lets geometry float).
- **Line-to-line / parallel-offset dimension:** to dimension the gap between two (parallel) lines —
  used for base heights, face widths, and similar offsets — use
  `sketch.sketchDimensions.addOffsetDimension(lineA, lineB, textPoint)` and set the value via the
  returned dimension's `.parameter.value = <number>`. It is *not* `addDistanceDimension`
  (point-to-point). The two lines must be parallel — but **only add an `addParallel` constraint if
  they are not already parallel.** If the two lines are already parallel *by construction* (e.g.
  both are perpendicular to a common third line), they're parallel already; adding a redundant
  `addParallel` over-constrains the sketch and throws `VCS_SKETCH_OVER_CONSTRAINTS`. Add `addParallel`
  only for a freshly-drawn line that isn't yet parallel to its target (e.g. a toe line you just drew
  at arbitrary angle); for a line whose direction the existing constraints already fix, call
  `addOffsetDimension` directly with no parallel constraint.
- **Angular dimension picks the wedge containing its text point:**
  `addAngularDimension(lineA, lineB, textPoint)` measures the angle on the side where `textPoint`
  lies. To get the intended angle (e.g. a shaft angle Σ rather than its supplement 180−Σ), place
  `textPoint` inside the wedge you mean.
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

## Solid-modelling features beyond extrude (used by revolved/cut gears, e.g. bevel)

The involute gears only extrude + pattern + combine + chamfer + fillet. Gears whose spec calls for
revolved bodies, lofted teeth, or split-by-face shaping (bevel) use these additional features. The
spec says *which* to use and *on what geometry*; this pins the *API shape*.

- **Revolve** (`component.features.revolveFeatures`): `createInput(profile, axis, operation)` →
  `setAngleExtent(False, ValueInput.createByString('360 deg'))` → `add(input)`. The `axis` is a
  sketch line or construction axis. **Hard failure to design around: the profile must NOT cross the
  axis of revolution** — if it does, Fusion aborts with `RuntimeError … ASM_WIRE_X_AXIS … the
  profile crosses the axis of revolution`. A spec that revolves a profile bounded by a
  geometrically-derived dimension (e.g. bevel's Face Width) must cap that dimension so the profile
  stays on one side of the axis; reproduce the cap exactly.
- **Loft, including point-to-profile** (`component.features.loftFeatures`): `createInput(operation)`
  → `loftSections.add(entity)` for each section in order → `add(input)`. A section may be a
  **single point** (a `SketchPoint`/`ConstructionPoint`) for a degenerate end — lofting an apex
  point to a profile yields a tapered/pointed body. Order of `loftSections.add(...)` is the loft
  order.
- **Symmetric through-cut extent:** for an extrude-cut that must pierce a body fully regardless of
  thickness, use `extentInput.setSymmetricExtent(ValueInput.createByReal(largeDist), False)` — the
  second arg `isFullLength=False` means `largeDist` is the half-length *per side*. Do not pass a
  third (taper) argument. Restrict the cut with `participantBodies = [theBody]`.
- **Split body by a face** (`component.features.splitBodyFeatures`): `createInput(bodyToSplit,
  splittingTool, isSplittingToolExtended)` → `add(input)`. The splitting tool can be a `BRepFace`
  (reuse an existing face of another body rather than building a fresh surface — building a surface
  from a sketch line in a sibling/parent component fails, see cross-component below). Pass
  `isSplittingToolExtended=True` to extend the tool surface to fully bisect the target. A split that
  doesn't intersect raises `RuntimeError … SPLIT_TARGET_TOOL_NOT_INTERSECT` (message text may be
  localized) — catch it where the spec says a cut may be a no-op.
- **Find a face by surface type, incl. cones** (cross-gear pattern, extends the profile/face-finding
  rule above): iterate `body.faces`, test `face.geometry.surfaceType` against
  `adsk.core.SurfaceTypes` (e.g. `ConeSurfaceType`, `CylinderSurfaceType`, `PlaneSurfaceType`).
  When several faces share a type, disambiguate by **world-coordinate containment**: project the
  point with `surface.evaluator.getParameterAtPoint` → `getPointAtParameter` and compare distance
  within a tolerance like `1e-2` cm. **But that distance is to the *infinite* surface, so it is NOT
  a reliable unique key** — several trimmed faces whose infinite surface passes through the same
  points tie at distance ≈0 (common with near-coaxial cones), and a min-distance pick can land on a
  trimmed face that isn't the one you want. When the face is going to be *used in an operation*
  (e.g. a split tool), **validate against the operation rather than trusting the distance**: collect
  the candidate faces by surface type, order them best-first by distance, and **try each in the
  actual operation, keeping the first that succeeds** (e.g. the first cone face that actually splits
  the target body into >1 piece). **Do not use distance as a hard filter** — `getParameterAtPoint`
  returns no-result (None) near a surface singularity (a cone's apex), so a point actually *on* the
  surface can report an unevaluable distance; gating on "distance < tol" then wrongly drops the right
  face. Treat an unevaluable distance as "try last," never as "disqualified." Never rely on
  `body.faces` enumeration order to break a distance tie — that is luck, not correctness.
  **Identify a revolved-edge face by the edge's MIDPOINT, not its endpoints.** An endpoint that sits
  near the surface's singularity (a cone's apex) makes `getParameterAtPoint` return `None` there, so
  endpoint-distance can't see the right face; the edge midpoint is clear of the apex (reliable) and
  uniquely picks the face that edge swept (different edges → different midpoints), so it
  distinguishes adjacent/coaxial cones that endpoint-distance can't.
- **Make runtime-topology failures self-diagnosing.** Steps whose outcome depends on geometry the
  parse/contract checks can't see (face-finding by containment, body splits, piece counts) must
  raise exceptions that **carry the measured quantities** — candidate face distances vs. tolerance,
  pieces-in/pieces-out per operation, which entity was selected — never a bare count or a silent
  no-op. In a headless/Fusion-add-in run the exception text is the only telemetry the caller gets,
  so "expected 3 pieces, got 2" is useless but "got 2 (cut#1 found 3 cone faces, dists …, produced
  2; cut#2 found 0 faces within tol 1e-2, dists …)" pinpoints the cause. Prefer raising a rich error
  over returning unchanged on a lookup miss.
- **Select/remove split pieces:** after a split, identify pieces with
  `body.pointContainment(worldPoint)` (returns `adsk.fusion.PointContainment.PointInside` /
  `PointOnPoint`(`PointOnPointContainment`)`/Outside`) and rank by
  `body.physicalProperties.volume`. Delete unwanted pieces with
  `component.features.removeFeatures.add(body)` (timeline-visible) — not a bare `deleteMe()`.
- **Construction planes beyond offset:** `constructionPlanes.createInput()` then
  `setByOffset(plane, ValueInput)` (coplanar/offset), `setByAngle(line, ValueInput, refPlane)`
  (a plane through a line tilted by an angle off a reference plane — e.g. a plane *perpendicular* to
  a sketch by passing `'90 deg'`), or `setByDistanceOnPath(curve, ValueInput)` (a plane normal to a
  curve at a fractional distance 0–1). The spec says which constructor and inputs.
  **Pass the `SketchLine` (or edge / construction axis) DIRECTLY** to `setByAngle` /
  `setByDistanceOnPath` — both accept a single linear/curve entity. Do **NOT** wrap it in
  `adsk.fusion.Path.create(curve, …)` first: `Path.create` on a sketch curve raises
  `RuntimeError … InternalValidationError : Utils::getObjectPath(sketchCurve, …, nullptr,
  contextPath)` whenever the curve's owner sketch isn't trivially resolvable in the current
  (multi-component) context. Avoid `Path.create` on sketch curves entirely; the plane/axis builders
  resolve the curve themselves.
- **Construction axes beyond circular-face:** `constructionAxes.createInput()` then
  `setByLine(infiniteLine)` (axis along a sketch/3D line), `setByCircularFace(face)` (off a
  cylinder/cone), or `setByTwoPlanes(planeA, planeB)` (their intersection — a usable workaround when
  `setByPerpendicularAtPoint` would need a `BRepFace` you don't have).
- **Move/rotate a body** (`component.features.moveFeatures`): `createInput2(bodyCollection)` →
  `defineAsFreeMove(matrix3D)` → `add(input)`, where the matrix is built with
  `Matrix3D.setToRotation(angleRadians, axisVector, originPoint)`. Use `defineAsFreeMove` with a
  matrix (not `defineAsRotate`, which rejects a `SketchLine` axis). Used for aesthetic/meshing
  rotations.

## Multi-component orchestration (a gear that builds several sibling components, e.g. a pair)

Single-gear modules create one occurrence (`base.Generator.getOccurrence()`). A gear that produces
a *set* of related components (bevel: a parent holding a shared **Design** component plus a
**body-per-gear** component) cannot use that helper and must manage the tree itself:

- Create each occurrence with `parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` and
  name `occurrence.component.name`. Typical tree: `<Pair> Gear` under the user's parent → `Design`
  sub-component (all sketches/construction geometry/features run **here**) + one sub-component per
  output gear (the finished bodies are relocated into these at the end).
- **NEVER call `occurrence.activate()`.** Build in a component by calling its own collections
  (`comp.sketches.add(...)`, `comp.features.*`) on the *non-activated* component — exactly as a
  single-gear generator does. Activating a sub-occurrence and then sketching on a plane/face the
  **user selected in another component** (root) makes Fusion resolve that external reference in the
  activated component's **local frame** (identity → world-XY-aligned), silently collapsing the
  build's orientation onto XY no matter what plane the user picked. This is a real, hard-to-see
  trap: the gear still builds, just flat on XY. A single-gear generator that respects arbitrary
  planes works precisely *because* it never activates — match that.
- **Fusion rejects cross-sibling sketch/`project`/`Path.create` references** (entities wrapped via
  `createForAssemblyContext` don't help). So: run **all** feature operations in the one Design
  component, then **relocate finished bodies** with `body.moveToComponent(targetOccurrence)`
  (`moveToComponent` preserves world position and needs no activation). Do not sketch or
  path-reference geometry that lives in a sibling. Note this does **not** require activating
  anything — building everything in one component already avoids cross-sibling references.
- Cleanup/rollback is hand-rolled: a recursive walk turning off `isLightBulbOn` on every sketch /
  construction plane / construction axis across the tree (dedupe by `entityToken`), and
  `deleteComponent()` calls `deleteMe()` on the top occurrence for error rollback.
- **Don't re-derive a user-selected plane/face inside your sub-component — use it directly.** When
  the user selects a target plane (a `ConstructionPlane` or `PlanarFace`) that lives in the root or
  another component, place sketches **directly** on that selected entity (`sketches.add(target)`)
  and pass the selected entity itself to plane builders (`setByAngle(line, angle, target)`). Do NOT
  "normalize" it by building a coplanar construction plane (`setByOffset(target, 0)`) inside your
  activated sub-component first: a construction plane created in a sub-occurrence, offset from a
  face in a *different* component, resolves in the sub-component's own coordinate frame (identity →
  effectively world XY for a freshly-added component) and **silently loses the selected plane's
  world orientation** — the whole build then lies flat on XY regardless of what the user picked.
  (A single-component generator can normalize safely because there's no cross-component frame
  mismatch; a multi-component one must not.)

## Parameters: live-expression mode vs all-Python-precomputed mode

The involute gears register Fusion **user parameters** as live expression strings (the
`processInputs` pattern above) so they appear in the parameter table. This is **one valid mode, not
mandatory.** A gear may instead **precompute every value in Python** (internal cm) and write
geometry numerically — setting sketch dimensions via `dimension.parameter.value = <number>` and
feature inputs via `ValueInput.createByReal(<number>)` — registering **no** named user parameters
(bevel does this). When a gear borrows another gear's tooth generator that expects
`parent.getParameter(name).value`, it can satisfy that interface with a lightweight **proxy object**
that returns precomputed values, rather than registering real parameters (see the borrowing gear's
spec Dependencies section). The spec declares which mode the gear uses; either is acceptable, but
the contract self-check only looks for the parameter names the spec actually declares.

## What the spec need not pin (free to vary)

Local variable names, comments, log strings, and how build steps are decomposed into private
helper methods. What the spec **does** pin and the output must reproduce: the contract surface (the
classes, hook methods, `ctx` fields, and call graph the spec declares), parameter names/ids, the
geometry the spec prescribes, feature order, and edge-case handling.
