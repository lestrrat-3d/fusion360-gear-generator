# Spur Gear Creation Instructions

This file is the **design & geometry intent** — what the spur gear is and how it is built. Its
**Fusion-API realization** (the constraint recipes, the anchor-tracking sketch design, the cleanup
recipe — the gear-specific *how*) lives in the sidecar **`fusion.md`** next to this file and is
cited here by anchor (`[SPUR-F…]`). Cross-gear Fusion conventions are cited as `[PB-…]` (in the
shared `PLAYBOOK.md`). Read all three together; the cited rules are as binding as this body.

## Component Setup

A spur gear is a single cylindrical body with straight teeth cut along the axis. Unlike the bevel generator there is no pairing — one invocation of the command produces exactly one gear. The new gear is added as a child occurrence of the user-selected Parent Component.

## Architecture

Spur **opts into the playbook's four-class pattern** (PLAYBOOK "The four-class pattern") and into
`base.Generator` / `GenerationContext`. The module defines exactly these four classes; the names
are public API (helical/herringbone subclass all of them by name, and `commands/spurgear/entry.py`
binds two):

1. **`SpurGearCommandInputsConfigurator`** — a plain class (no base) with `@classmethod def
   configure(cls, cmd)` that adds the dialog inputs (see "Exact input ids and parameter-name
   strings"). Extension seam for subclasses: `[SPUR-SUBCLASS-INPUT]`.
2. **`SpurGearGenerationContext(GenerationContext)`** — the data carrier whose `__init__` declares
   the fields in "Generation Context — canonical field names", each `cast(None)`-initialised
   (`toothProfileIsEmbedded` starts `False`).
3. **`SpurGearInvoluteToothDesignGenerator`** — a plain class (no base), constructed as
   `(sketch, parent, angle=0)`; draws the circles, the involute tooth, and the bore circle. Its
   reproduced surface is pinned in "Tooth generator … reproduced surface" below.
4. **`SpurGearGenerator(Generator)`** — the orchestrator, subclass of `base.Generator`. Holds
   `processInputs`, `generate`, parameter registration, and the per-step build methods (see
   "Method contract — call graph and override boundaries").

`GenerationContext` and `Generator` are imported from `.base`.

## Variables

User inputs are listed below in the order they appear in the command dialog. Derived (calculated) parameters are registered as Fusion user parameters under the `SpurGear<N>_` prefix so they are visible and editable after generation; they are listed after the inputs they depend on.

Target Plane: user-specified plane. The gear's front face is sketched on this plane; the gear body is extruded away from it by Thickness. Any selection that isn't already a `ConstructionPlane` (for example a planar face) is converted to a coplanar construction plane at generation time so sketch-profile detection is not confused by the selected face itself.

Anchor Point: user-specified point. The gear center is aligned with this point. May be a `ConstructionPoint` or a `SketchPoint`. The anchor point is projected into the Tools sketch on the target plane; the Gear Profile sketch then constrains its own local origin (0,0,0) to that projected point, so changing the anchor point downstream moves the gear.

Module: user-supplied number. Specifies the module of the gear. The dialog input is unitless
(`''`) with default `1`, and the `Module` user parameter is registered with units **`''`
(unitless) — NOT `'mm'`** — so `generateName` renders `M=1` (no unit suffix) and the derived
`mm`-registered expressions (`PitchCircleDiameter = Module * ToothNumber`, `RootCircleDiameter =
PitchCircleDiameter - 2.5 * Module`, …) read the unitless `Module` factor exactly as the proven
implementation does (Fusion accepts the unitless term inside those `mm` expressions).

Tooth Number: user-specified integer. Default 17.

Pressure Angle: user-specified angle. Default 20°. Stored in radians in the user parameter `PressureAngle`.

Bore Diameter: user-specified positive number in mm. Default 0mm (i.e. no bore). When > 0, a cylindrical hole is cut through the gear body, centered on the anchor point.

Thickness: user-specified positive number in mm. Default 10mm. Axial length of the gear body.

Apply chamfer to teeth: user-specified positive number in mm. Default 0mm (i.e. no chamfer). Distance of the equal-distance chamfer applied to the tooth edges on the front (sketch-plane) face.

Generate sketches, but do not build body: user-specified boolean, default `false`. When `true`, stop after the Gear Profile sketch is drawn (no extrude, no pattern, no fillet, no bore, no chamfer). Useful for inspecting the involute construction.

Parent Component: user-specified component. Defaults to the root component (pre-selected). Listed last in the dialog because the default is correct for most uses; the user only touches it when nesting the gear inside an existing assembly. The new gear occurrence lives as a child of this component.

Pitch Circle Diameter: calculated number. `Module * Tooth Number`.

Pitch Circle Radius: calculated number. `Pitch Circle Diameter / 2`.

Base Circle Diameter: calculated number. `Pitch Circle Diameter * cos(Pressure Angle)`. The circle the involute flank unrolls from.

Base Circle Radius: calculated number. `Base Circle Diameter / 2`.

Root Circle Diameter: calculated number. `Pitch Circle Diameter - 2.5 * Module`. The circle at the bottom of the tooth valleys (dedendum = 1.25 · Module).

Root Circle Radius: calculated number. `Root Circle Diameter / 2`.

Tip Circle Diameter: calculated number. `Pitch Circle Diameter + 2 * Module`. The circle that caps the tops of the teeth (addendum = 1.0 · Module).

Tip Circle Radius: calculated number. `Tip Circle Diameter / 2`.

Involute Steps: calculated integer. 15. The involute flank is drawn as a fitted spline through this many sampled involute points between the base and tip circles.

Tooth Space Angle At Root: the angular width of the valley between adjacent teeth at the root circle, in radians. Registered as a user parameter (so it stays visible and editable), but with a pre-evaluated numeric value rather than a live Fusion expression — Fusion's expression engine refuses to mix the unitless output of `tan()` with the radian-valued Pressure Angle in a subtraction, so we compute it in Python. The value is `π / Tooth Number − 2 · (tan(Pressure Angle) − Pressure Angle)`. **Register this parameter as unitless (units `''`), not `'rad'`** — even though it holds a radian value. The next parameter multiplies it by a length (`Root Circle Radius`), and Fusion only accepts that product as a length (`mm`) if this factor is unitless; registering it as `'rad'` makes the product `mm·rad` and Fusion rejects the dependent parameter with `RuntimeError: Invalid expression`. (Treating a radian magnitude as a unitless number is correct — radians are dimensionless.)

Tooth Space Arc At Root: calculated number, registered in `mm`. Live Fusion expression `Root Circle Radius * Tooth Space Angle At Root`. The arc length of that valley along the root circle. This is why the factor above must be unitless — so this product reads as a pure length.

Fillet Clearance: calculated number. 0.9. Fraction of the half-valley arc used for the fillet radius. A value of 1.0 would make fillets from adjacent flanks meet at the midpoint of the valley (fully rounded root); 0.9 leaves a small flat strip.

Fillet Radius: calculated number. `(Tooth Space Arc At Root / 2) * Fillet Clearance * 1`. The `* 1` factor is a hook used by helical/herringbone subclasses to multiply by `cos(Helix Angle)` so the fillet reads correctly on the transverse plane of a tilted tooth. For a spur gear the factor is always 1.

### Exact input ids and parameter-name strings

These literal strings are part of the reproduced surface (they appear in the dialog and the
post-generation user-parameter table, and saved designs reference them). Use them verbatim:

| Dialog input | input id | registered user-parameter |
|---|---|---|
| Parent Component (selection) | `parentComponent` | — |
| Target Plane (selection) | `plane` | — |
| Anchor Point (selection) | `anchorPoint` | — |
| Module | `module` | `Module` |
| Tooth Number | `toothNumber` | `ToothNumber` |
| Pressure Angle | `pressureAngle` | `PressureAngle` |
| Bore Diameter | `boreDiameter` | `BoreDiameter` |
| Thickness | `thickness` | `Thickness` |
| Apply chamfer to teeth | `chamferTooth` | `ChamferTooth` |
| Generate sketches, but do not build body | `sketchOnly` | `SketchOnly` |

**Dialog display order (the order `configure()` adds inputs) is fixed and must be exactly:**

1. Target Plane (`plane`)
2. Anchor Point (`anchorPoint`)
3. Module (`module`)
4. Tooth Number (`toothNumber`)
5. Pressure Angle (`pressureAngle`)
6. Bore Diameter (`boreDiameter`)
7. Thickness (`thickness`)
8. Apply chamfer to teeth (`chamferTooth`)
9. Generate sketches, but do not build body (`sketchOnly`)
10. Parent Component (`parentComponent`) — **last**

This is the order the inputs are listed in the Variables section above, and `configure()` must
call its `add*Input(...)` methods in exactly this sequence. **Do not reorder by input *type*** (e.g.
all selections together, all value inputs together). In particular: Target Plane and Anchor Point
are the **first two** inputs, and Parent Component is **last** (its default — the root component —
is correct for most uses, so it sits at the bottom). **This display order is independent of, and
must not be confused with, the `processInputs` *read* order** — `processInputs` reads the three
selection inputs first to dodge the occurrence-context shift (see Generation Order), but that
read-order has no bearing on where the inputs appear in the dialog. A generator that puts the
selections last in `configure()` because they are "read first" has the rule backwards.

Bore Diameter is added as a **string** value input (`addStringValueInput`, default `'0 mm'`) so it
accepts expressions; the rest are numeric `addValueInput`s. **`addValueInput` defaults are given in
Fusion's internal units (cm for lengths, radians for angles), regardless of the input's display
unit string.** Pin the two non-trivial ones: Pressure Angle is
`addValueInput(…, 'deg', ValueInput.createByReal(math.radians(20)))` — display unit `'deg'`,
default in radians — and Thickness is `addValueInput(…, 'mm', ValueInput.createByReal(to_cm(10)))`
— display unit `'mm'`, default in cm. (Module and Tooth Number are unitless
`createByReal(1)` / `createByReal(17)`; chamfer is `createByReal(0)`.)

The three selection inputs each carry exactly these filters, and each is limited to exactly one
selection with `setSelectionLimits(1, 1)`:

- Target Plane: `ConstructionPlanes` + `PlanarFaces`.
- Anchor Point: `ConstructionPoints` + `SketchPoints`.
- Parent Component: `Occurrences` + `RootComponents`; pre-selects `get_design().rootComponent`.

`SketchOnly` is persisted as a
real-valued user parameter (1 = true, 0 = false), since the framework only reads numeric
parameters as booleans (`getParameterAsBoolean`). The derived parameters in the list above
(Pitch/Base/Root/Tip circles, InvoluteSteps, ToothSpace…, Fillet…) keep exactly those names.

**[SPUR-EXPORTED-CONSTANTS] Module-level constants are public API.** Every input id and
user-parameter name above is exported as a module-level constant in `spurgear.py`, and dependent
modules import them by name — renaming any of these identifiers is a breaking change. The full
roster:

- Input ids: `INPUT_ID_PARENT`, `INPUT_ID_PLANE`, `INPUT_ID_ANCHOR_POINT`, `INPUT_ID_MODULE`,
  `INPUT_ID_TOOTH_NUMBER`, `INPUT_ID_PRESSURE_ANGLE`, `INPUT_ID_BORE_DIAMETER`,
  `INPUT_ID_THICKNESS`, `INPUT_ID_CHAMFER_TOOTH`, `INPUT_ID_SKETCH_ONLY`.
- Parameter names: `PARAM_MODULE`, `PARAM_TOOTH_NUMBER`, `PARAM_PRESSURE_ANGLE`,
  `PARAM_BORE_DIAMETER`, `PARAM_THICKNESS`, `PARAM_CHAMFER_TOOTH`, `PARAM_SKETCH_ONLY`,
  `PARAM_PITCH_DIAMETER`, `PARAM_PITCH_RADIUS`, `PARAM_BASE_DIAMETER`, `PARAM_BASE_RADIUS`,
  `PARAM_ROOT_DIAMETER`, `PARAM_ROOT_RADIUS`, `PARAM_TIP_DIAMETER`, `PARAM_TIP_RADIUS`,
  `PARAM_INVOLUTE_STEPS`, `PARAM_TOOTH_SPACE_ANGLE`, `PARAM_TOOTH_SPACE_ARC`,
  `PARAM_FILLET_CLEARANCE`, `PARAM_FILLET_RADIUS`.

`helicalgear.py` and `herringbonegear.py` each import `PARAM_MODULE, PARAM_TOOTH_NUMBER,
PARAM_THICKNESS` from `.spurgear` (helical additionally imports the four classes; see
"Dependencies and dependents").

**[SPUR-SUBCLASS-INPUT] Configurator extension seam (for subclasses).** `configure()` is a
`@classmethod` on `SpurGearCommandInputsConfigurator`. A subclass gear (helical/herringbone) adds its
own extra dialog input by **subclassing the configurator and appending after `super().configure(cmd)`**
— e.g. `HelicalGearCommandConfigurator.configure` calls `super().configure(cmd)` then
`cmd.commandInputs.addValueInput(...)` for its Helix Angle. Because `super().configure()` already
added Parent Component **last**, a subclass's extra input necessarily lands **after** Parent Component
in the dialog. (This is the actual behavior; the "Parent Component last" rule above is a spur-base
statement that a subclass's appended inputs sit below.)

## Sketch Discipline

A few rules apply across every sketch created below. They're not obvious from the step list, and
the whole construction falls apart without them. The Fusion-API mechanics are in `fusion.md` and
`PLAYBOOK.md`; this section states the *intent* and points to the binding rule for each.

The Gear Profile constraint scheme below is **proven to fully constrain** (`DOF == 0`, no
redundant/conflicting constraints, across a size sweep) on the bench in `spec/spurgear/sketch/`
before any Fusion code is generated — the sketch-first gate `[PB-SKETCH-FIRST]`. That proof is the
executable check that these rules add up to a fully-constrained sketch; run it (`./run.sh`) when
changing any of them.

- **Sketches follow the user's anchor through a projection chain.** The Tools-sketch projection of
  the anchor is the canonical handle; later sketches re-project it so the whole gear tracks the
  anchor if it moves — see `[SPUR-F-ANCHOR-CHAIN]`.
- **Each anchor-following sketch keeps its own movable local origin** (a fresh `(0,0,0)`
  `SketchPoint`, not `sketch.originPoint`); all geometry is drawn relative to it, then anchored in
  step 5 — see `[SPUR-F-LOCAL-ORIGIN]`.
- **Every adjacency in the tooth profile loop is a *shared* `SketchPoint`** (so the loop is
  recognised as a closed profile) — share the point object, never re-coincident a fresh one
  (`[PB-SHARE-XOR-COINCIDENT]`, applied in `[SPUR-F-SHARED-ADJACENCY]`).
- **A requested rotation is drawn *and* confirmed** — pre-rotate the geometry in Python *and*
  set the confirming angular dimension last; both are required — see `[SPUR-F-ROTATE-CONFIRM]`.
- **Hide each entity with the right property, after it's consumed** — `isVisible=False` for
  sketches, `isLightBulbOn=False` for construction planes/axes (`[PB-HIDE-AFTER-USE]`); the
  spur cleanup recipe (which entities, the per-mode split) is `[SPUR-F-CLEANUP]`.
- **Dimensions are driving by default** — never pass `isDriven=True` (`[PB-DRIVING-DIM]`). All
  diameter dimensions here (four gear circles, tooth-top arc, bore circle) must be driving.
- **Dimensions and feature inputs are numeric snapshots** — editing a `<prefix>_…` parameter does
  not change an existing gear; regenerate (`[PB-NUMERIC-SNAPSHOT]`, spur application
  `[SPUR-F-SNAPSHOT]`).

## Generation Context — canonical field names

The `SpurGearGenerationContext` object is passed between generation steps and read by subclasses (helical, herringbone). Subclasses reach in by name, so these field names are part of the public API of this module — don't rename them when reconstructing:

- `ctx.plane` — the `ConstructionPlane` all sketches are built on (normalised in step 1).
- `ctx.anchorPoint` — the `SketchPoint` that is the Tools-sketch projection of the user's anchor. Later sketches project *this* in again to chain to the user's original anchor entity.
- `ctx.extrusionEndPlane` — the offset construction plane used as the `to-entity` target for the tooth and body extrudes.
- `ctx.gearProfileSketch` — the sketch containing the tooth profile + four gear circles.
- `ctx.toothBody` — the single extruded tooth, before the circular pattern.
- `ctx.gearBody` — the cylindrical body the teeth are joined into.
- `ctx.centerAxis` — the `Gear Center` construction axis built off the body's cylindrical face.
- `ctx.extrusionExtent` — the far end-cap face of the gear body, used as the `to-entity` for the bore cut.
- `ctx.toothProfileIsEmbedded` — `True` iff the base circle sits inside the root circle (no flank-to-root stubs drawn); used by the tooth extrude step to pick the right profile-curve count.

(The active plane is also held on the generator as `self.plane` — normalised in step 1 — and
subclasses read `self.plane` directly; keep both available.)

## Method contract — call graph and override boundaries

These method names **and the boundaries between them** are public API: `helicalgear.py` and
`herringbonegear.py` subclass `SpurGearGenerator` and override specific methods, calling
`super()` at specific points. A reconstruction that merges or reorders these steps — even if it
draws an identical spur gear — breaks the helical and herringbone gears. Keep the call graph
exactly as below; only the *contents* of each method (local variables, comments, further private
helpers) may vary.

```
generate(inputs)
  → processInputs(inputs)
  → name the component (generateName)
  → normalize self.plane to a ConstructionPlane
  → ctx = newContext()
  → prepareTools(ctx)            # Tools sketch + ctx.anchorPoint + ctx.extrusionEndPlane (steps 1–2)
  → buildMainGearBody(ctx)
        → buildSketches(ctx)     # Gear Profile sketch; runs the tooth generator (steps 3–5)
        → if SketchOnly: show the Gear Profile sketch and stop (step 6)
          else:
            → buildTooth(ctx)    # extrude the tooth → ctx.toothBody, then call chamferTooth(ctx) (steps 7–8)
            → buildBody(ctx)     # extrude the annular body → ctx.gearBody, centerAxis, extrusionExtent (step 9)
            → patternTeeth(ctx)  # circular pattern + combine, then call createFillets(ctx) (steps 10–11)
  → buildBore(ctx)               # optional bore (step 12)
  → cleanup(ctx)                 # always: hide construction planes/axes; sketches hidden only when NOT SketchOnly
```

**`cleanup(ctx)` is the very last action of `generate()` — after `buildBore`, not inside
`buildMainGearBody`.** Call it **unconditionally** (in both modes); the SketchOnly distinction
lives *inside* `cleanup` — the recipe (which entities, the per-mode split) is owned by
`[SPUR-F-CLEANUP]`. Placement after
`buildBore` matters because `buildBore` re-projects `ctx.anchorPoint` from the Tools sketch and
projection fails once that sketch is hidden — so the Tools sketch must stay visible through the
bore. Do not move `cleanup` up into `buildMainGearBody`, and do not guard the *call* (guard the
sketch-hiding inside it instead).

Specific boundaries subclasses depend on (do not move the work elsewhere):

- **`buildSketches(ctx)`** owns creating the Gear Profile sketch and invoking the tooth
  generator. Helical overrides it, calls `super().buildSketches(ctx)`, then draws a *second*
  twisted profile sketch with `SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=helixAngle)`.
- **`buildTooth(ctx)`** owns turning the profile into `ctx.toothBody` **and must call
  `self.chamferTooth(ctx)` as its last action.** Helical overrides it to loft (instead of
  extrude) and herringbone to loft+mirror; both still end by calling `chamferTooth`. Because the
  chamfer is triggered from inside `buildTooth`, `buildMainGearBody` must **not** chamfer
  separately.
- **`chamferWantEdges()` / `filletHelixFactorExpression()`** are overridable hooks that must exist
  on the generator, but they are consumed at different points. `chamferTooth` reads
  `chamferWantEdges()` (spur base: `6`) when picking the front face. `filletHelixFactorExpression()`
  is **not** read by `createFillets`: it returns an **expression string** (spur base: `'1'`;
  helical: `'cos(<prefix>_HelixAngle)'`) that is consumed exactly once, in
  `registerDerivedParameters`, where it is spliced in as the last factor of the live `FilletRadius`
  parameter expression (`(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>`). `createFillets`
  then reads only the resulting `FilletRadius` parameter's numeric `.value`.
- **[SPUR-EXTRA-PARAMS] `addExtraPrimaryParameters(self, inputs)`** is an overridable hook, a **no-op
  on the spur base**, that `processInputs` calls **between** registering the input-sourced parameters
  and the derived ones. Subclasses override it to register their own primary user parameters from the
  extra dialog inputs they added (e.g. helical registers `HelixAngle` from the `helixAngle` input).
  It must exist (as a no-op) on the spur base so the call site in `processInputs` is present for
  subclasses to hook. Together with `[SPUR-SUBCLASS-INPUT]`, these are the two seams by which a
  subclass adds a parameter: the configurator adds the dialog *input*, this hook registers the
  *parameter*.
- **`generateName()`** returns the component name. For the spur base it is
  `'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression, thickness.expression)`
  — i.e. the `Module`, `ToothNumber`, and `Thickness` parameters' **`.expression`** strings (not
  `.value`), so units show through (e.g. `Spur Gear (M=1, Tooth=17, Thickness=10 mm)`). Subclasses
  override this to read their own parameters.

### Tooth generator (`SpurGearInvoluteToothDesignGenerator`) reproduced surface

- Constructor `(sketch, parent, angle=0)`. Store the constructor angle as `self.toothAngle =
  angle`. **This stored value is NOT what `drawTooth` rotates by** — see the next bullet. (It is
  retained only as an incidental field; the live rotation always comes from `draw()`'s runtime
  argument. Do not use `self.toothAngle` inside `drawTooth`.)
- The movable **local origin is a field named `self.anchorPoint`** — a fresh `SketchPoint` added
  at (0, 0, 0) in the constructor (see Sketch Discipline). Subclasses don't read it directly, but
  the spur base `buildSketches` and the `draw()` anchoring below depend on this exact name.
- `draw(anchorPoint, angle=0)` performs, in order: `drawCircles()`, `drawTooth(angle)`, then the
  **step-5 anchoring**, then — as the very last action, *after* the anchoring — sets the
  confirming angular dimension's value to `angle` (only when `angle != 0`; this last action is
  `[SPUR-F-ROTATE-CONFIRM]`'s "very last action, after the entire constraint network exists").
  **`drawTooth` MUST rotate by the
  `angle` argument that flows in from `draw()` at call time — NOT by the constructor-stored
  `self.toothAngle`.** This is load-bearing for subclasses: helical/herringbone construct the
  generator with the default `angle=0` (so `self.toothAngle == 0`) and then call
  `draw(ctx.anchorPoint, angle=helixAngle)`. If `drawTooth` used `self.toothAngle` it would draw a
  flat tooth and the helical loft would have no twist.
- Methods `drawCircles`, `drawTooth`, `drawBore`, and `calculateInvolutePoint(baseRadius,
  intersectionRadius)` must all exist. The tooth generator also exposes parameter accessors
  `getParameter(name)` and `getParameterValue(name)` (these names are part of the reproduced
  surface). `drawBore(anchorPoint, diameter)` takes the anchor entity and the bore diameter (cm),
  projects the anchor into the sketch, draws the bore circle of that diameter centered on the
  projection with a driving diameter dimension, and returns the circle.
- When a drawing step needs one of the four circles drawn by `drawCircles` (the tip circle for
  the tooth-top point, the root circle for the flank-to-root lines), either keep direct references
  from `drawCircles` or locate it with the framework's `find_circle_by_radius(sketch, radius)`
  (from `.utilities`) — never fall back to an arbitrary circle on a failed radius match.
- **`calculateInvolutePoint(baseRadius, intersectionRadius)` — exact math** (this fully pins the
  flank shape; do not infer it). Returns the point on the involute of `baseRadius` at the radius
  where the unrolled string reaches `intersectionRadius`; returns `None` when `intersectionRadius
  < baseRadius` (the sample sits inside the base circle — this is the "non-positive involute
  parameter" case the sampling loop drops):
  ```
  alpha = acos(baseRadius / intersectionRadius)
  t     = tan(alpha)          # the curve parameter is tan(alpha) — NOT inv(alpha)=tan(alpha)-alpha
  x = baseRadius * (cos(t) + t * sin(t))
  y = baseRadius * (sin(t) - t * cos(t))
  ```
  Using `inv(alpha) = tan(alpha) − alpha` as the parameter instead of `tan(alpha)` is a common
  mistake and produces a wrong (mis-parameterised) flank.

## Dependencies and dependents

Spur imports only the framework (`.base` — `Generator, GenerationContext, get_value, get_boolean,
get_selection`; `.utilities` — `get_normal, find_profile_by_curve_counts`; `.misc` — `to_cm,
get_design`). It depends on no other gear. Two dependents bind to its surface; regenerating spur
must not break either:

- **`helicalgear.py` / `herringbonegear.py` subclass the four classes.** Helical imports
  `SpurGearCommandInputsConfigurator`, `SpurGearGenerationContext`, `SpurGearGenerator`, and
  `SpurGearInvoluteToothDesignGenerator` (herringbone subclasses helical's versions), and both
  import the module-level constants `PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS`
  (`[SPUR-EXPORTED-CONSTANTS]`). Everything they lean on is pinned in "Method contract" and
  "Generation Context" above.
- **`bevelgear.py` borrows the tooth generator without the Fusion parameter table.** It constructs
  `SpurGearInvoluteToothDesignGenerator(toothSketch, proxy)` where `proxy` is a
  `spurproxy.VirtualSpurProxy` — a fake `parent` whose `getParameter(name)` serves precomputed
  values in internal cm (`[PB-PRECOMPUTED-MODE]`). **Borrowing constraint:** inside `drawCircles`,
  `drawTooth`, and `draw` (including the helpers they call, e.g. `_drawFlankToRoot`), parameters
  may be read ONLY from the key set `VirtualSpurProxy` serves — `Module`, `ToothNumber`,
  `PressureAngle`, `PitchCircleDiameter`, `PitchCircleRadius`, `BaseCircleDiameter`,
  `BaseCircleRadius`, `RootCircleDiameter`, `RootCircleRadius`, `TipCircleDiameter`,
  `TipCircleRadius`, `InvoluteSteps`. Reading any other key on those paths breaks the bevel build
  (the proxy raises `KeyError`). The proxy also carries the `_lastToothEmbedded` output slot: the
  tooth generator's `drawTooth` writes `self.parent._lastToothEmbedded` (see
  `[SPUR-F-FLANK-ROOT]`), and bevel reads it back off the proxy after `draw()` — keep that output
  write in place.

## Generation Order

The 12 steps below are preceded by a dialog-reading pass. The order matters for one specific reason: as soon as you call `parentComponent.occurrences.addNewComponent(...)` (directly via `getOccurrence()`, or indirectly via the first `addParameter()` / `parameterName()` call), Fusion's active component context shifts to the newly created occurrence. `SelectionCommandInput`s holding entities that live in a *different* component — for example a `SketchPoint` on a sketch in the root component, while the new gear is being added under the root — can drop their selections when that context shift happens. Numeric and boolean inputs are unaffected.

So the rule is: pull every selection input (Parent, Target Plane, Anchor Point) out of `inputs` and stash the entities on `self` *before* triggering occurrence creation. The order inside `generate()` is therefore:

1. Read Parent, Target Plane, Anchor Point from `inputs`. Don't call anything that touches the design yet.
2. Now `getOccurrence()` (or a parameter registration that calls it transitively).
3. Register input-sourced and derived user parameters from the still-live numeric inputs.
4. Run the 12 steps below in order.

## Instructions

### 1: Normalize the Target Plane

If the user-selected plane is not already a `ConstructionPlane` (for example they picked a planar face of an existing body), create a coplanar construction plane via `ConstructionPlaneInput.setByOffset(selectedPlane, 0)` and use that for all subsequent operations. This keeps the downstream profile-detection code from having to filter out the selected face's native profile.

### 2: Tools Sketch

Create a sketch named `Tools` on the target plane. Project the Anchor Point into it and keep the result as `ctx.anchorPoint` — this is the canonical handle every later sketch will re-project from (see Sketch Discipline). The sketch draws no geometry of its own; it exists to own this one reference. Leave it visible while the later sketches are still projecting from it; toggle `isVisible = False` once the gear is fully built.

Create an offset construction plane `Extrusion End Plane` at distance `Thickness` from the target plane. Its only purpose is to serve as the `to-entity` target for the tooth and body extrudes, so both extrudes end on the same well-defined face. It must be left visible while those extrudes run, then hidden at the very end of the build with `isLightBulbOn = False` (see Sketch Discipline — `isVisible = False` does **not** hide a construction plane). Keep a handle to it (`ctx.extrusionEndPlane`) so the final cleanup can switch its light bulb off.

### 3: Gear Profile Sketch

Create a sketch named `Gear Profile` on the target plane. Inside this sketch the Spur Gear tooth profile generator draws, in order:

1. **Root Circle** (solid, not construction) at radius `Root Circle Radius`.
2. **Tip Circle** (construction), `Tip Circle Radius`.
3. **Base Circle** (construction), `Base Circle Radius`.
4. **Pitch Circle** (construction), `Pitch Circle Radius`.

Center every circle on the local-origin `SketchPoint` by passing it **directly** as the center — `sketchCircles.addByCenterRadius(localOrigin, radius)` — so all four share that one point (see Sketch Discipline: share, don't re-coincident; do not pass `localOrigin.geometry` and then add a center coincident). Give each a driving diameter dimension. Each circle is also labeled with along-path sketch text (see the playbook for the exact `sketchTexts.createInput2(...)` + `setAsAlongPath(...)` call). The label string is `'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` — the circle's name, its radius, and `size`, all using the radii's internal `.value` (cm) — where `size = Tip Circle Radius − Root Circle Radius`. Pass that same `size` as the text **height** argument to `createInput2`.

### 4: Involute Tooth

Still inside the Gear Profile sketch, draw a single involute tooth centered on the +X direction:

1. Sample a sequence of points along the involute flank, starting on the base circle and walking outward toward the tip circle in equal radial steps (Involute Steps samples in total). The sampling is **endpoint-inclusive**: with `steps = Involute Steps`, sample `i` (for `i = 0 … steps−1`) sits at radius `r = Base Circle Radius + (Tip Circle Radius − Base Circle Radius) · i / (steps − 1)`, so the first sample radius is exactly `Base Circle Radius` and the **last sample is exactly `Tip Circle Radius`**. Do **not** clamp the start to `max(Base Circle Radius, Root Circle Radius)`; the flank is sampled from the base circle even when the base circle sits inside the root circle (the embedded case is detected later in step 9 from where the flank *start* lands, not by trimming the sampling). Each sample is `calculateInvolutePoint(Base Circle Radius, r)` for that step's radius `r` (the exact math is pinned in the tooth-generator surface section above). Drop any sample that returns `None` (i.e. whose radius is below the base circle) — those sit inside the base circle and have no valid involute.
2. **Watch the spiral direction.** A correctly-formed involute tooth narrows from base to tip, so the left flank's angular distance above +X must *decrease* as the radius grows. The standard parametric involute (`rb*(cos t + t sin t, sin t - t cos t)`) spirals the opposite way — its angular position *increases* with radius — and using it as a left flank directly produces a tooth that splays outward (wider at the tip than at the root). Before rotating, mirror the samples across the +X axis (negate y) so the spiral matches a left flank's shape. The rotation in the next step lifts the mirrored curve from −Y back up into +Y where the left flank belongs.
3. Decide how far to rotate the (mirrored) sequence so the tooth ends up symmetric about +X. Measure where the mirrored involute crosses the pitch circle, then rotate by exactly the amount that lands that pitch-circle crossing at angle `+π / (2 · ToothNumber)` above +X. (The angular width of a single tooth at the pitch circle is `π / Tooth Number`, so half that — `π / (2 · ToothNumber)` — is where the left flank's pitch crossing must end up.) Compute the pitch-circle crossing angle **analytically** — evaluate `calculateInvolutePoint(Base Circle Radius, Pitch Circle Radius)` and take its polar angle — rather than interpolating between the sampled flank points; the analytic value places the tooth at exactly the right angle regardless of how few involute samples are taken. **Exact expression (pin the sign — it interacts with the step-2 mirror):** with `(px, py) = calculateInvolutePoint(Base Circle Radius, Pitch Circle Radius)`, the *mirrored* pitch crossing sits at polar angle `atan2(−py, px)`, so `rotate_angle = π / (2 · ToothNumber) − atan2(−py, px)`. (The `−py` is the step-2 mirror applied to the analytic point; do not take `atan2(py, px)`.)
4. Rotate the (mirrored) sampled points by `rotate_angle`. This produces the **left** flank. Mirror that result across the X axis to produce the **right** flank. You now have a tooth symmetric about +X.
   **Then apply the requested `angle`.** The generator's `draw(anchorPoint, angle=0)` takes an `angle` (0 for spur; the helix angle for helical; 180° for the bevel virtual tooth) — the seed tooth must end up rotated by exactly that. Do this by rotating the **whole** +X-centered tooth by `angle` right here, in the same Python point math: rotate both flank point collections by `angle` (and, below, place the tooth-top point and seed the rib midpoints at the rotated positions too). Draw the tooth directly at its final angular position. Do **not** instead leave the tooth at +X and rely on the spine's angular dimension to swing it into place after the fact — the wrong-solver-branch failure this causes (and why it ruins the helical loft) is owned by `[SPUR-F-ROTATE-CONFIRM]`. Because both the bottom (`angle = 0`) and top (`angle = helixAngle`) profiles share the same `rotate_angle` baseline and differ by exactly `angle`, the loft twists by exactly the helix angle regardless of the absolute baseline. For `angle = 0` this whole step is a no-op (rotating by 0).
5. Draw the two flanks as `SketchFittedSpline`s through the point collections.
6. Draw the **tooth-top arc** — an arc through the two flank ends, capping the tooth at the tip
   circle. This step is constraint-sensitive (over-constraining it blows up later when the last
   rib's perpendicular is added). Use the exact minimal constraint set in `[SPUR-F-TOOTHTOP-ARC]`.
7. Draw the **spine** — a construction line from the local origin to the tooth-top point, defining
   the tooth's axis of symmetry — and pin its absolute rotation so the tooth sits at `angle` and the
   sketch is fully constrained. The exact construction (sharing the endpoints, the +X horizontal
   reference and its required end-pin on the `angle != 0` path, and the confirming angular
   dimension) is in `[SPUR-F-SPINE]`; the draw-and-confirm rule is `[SPUR-F-ROTATE-CONFIRM]`.
8. Draw a **rib** construction line between each matching pair of left/right flank fit-points, with
   a midpoint on the spine; the ribs lock the flanks to the spine so the tooth rebuilds cleanly when
   `Module` or `Tooth Number` changes, without pinning any point to an absolute coordinate. **Build a
   rib for *every* fit-point index — including the first (base-circle) pair and the last (tip) pair
   whose ends are also joined by the tooth-top arc; there is no exception for endpoints.** With N
   involute samples per flank you draw N ribs; the flank fit-points carry no other constraint, so a
   missing endpoint rib leaves that fit-point free and the sketch under-constrained. The construction
   is order-sensitive — follow the exact six-step order and the midpoint-chain rule (including the
   origin-to-first-rib dimension) in `[SPUR-F-RIBS]`.
9. Close the tooth at the root. If the flank's first point (on the base circle) lies **outside** the
   root circle, draw a short **radial** flank-to-root line on each side (exact two-constraint
   construction in `[SPUR-F-FLANK-ROOT]`); the tooth loop then has **6 curves** (2 splines + 2
   flank-to-root lines + 2 arcs). If the flank starts **inside** the root circle (low tooth-count /
   pressure-angle combinations), no flank-to-root line is drawn and the loop has **4 curves** (2
   splines + 2 arcs) — the profile is "embedded." Record which shape was drawn so the extrude step
   knows which edge count to expect; the embedded-flag mechanism (the tooth generator sets
   `self.parent._lastToothEmbedded`, copied to `ctx.toothProfileIsEmbedded` in `buildSketches`) is
   pinned in `[SPUR-F-FLANK-ROOT]`.

### 5: Anchor the Sketch

This is the step that slides the whole drawing onto the user's anchor. Project the Tools-sketch anchor into the Gear Profile sketch (this re-projection is what chains the two sketches together), then add a coincidence constraint between that freshly-projected point and the Gear Profile's local origin — the sketch point the tooth generator added at (0, 0, 0) in step 4 (the field `self.anchorPoint`), *not* `sketch.originPoint`. Because every piece of geometry above is constrained relative to the local origin, constraining it here drags the entire tooth profile onto the anchor as a unit.

**This anchoring happens inside the tooth generator's `draw()` method itself** — `draw()` does `drawCircles()`, then `drawTooth()`, then this projection-and-coincidence, then (for `angle != 0` only) sets the confirming angular dimension's value as its very last action, after the anchoring (`[SPUR-F-ROTATE-CONFIRM]`) — *not* a separate step the generator performs after `draw()` returns. This matters because helical/herringbone build their twisted loft profile by calling `SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=…)` directly and rely on that single call to anchor the sketch. If the anchoring were moved up into `buildSketches`, the twisted sketch would be left unconstrained and the loft would float off the anchor.

### 6: Sketch-Only Short-Circuit

If the Generate-Sketches-Only input is `true`, make the Gear Profile sketch visible and stop — skip the remaining body operations (tooth/body/pattern/fillet/bore). The end-of-build cleanup still runs in this mode; its per-mode split (planes/axes always hidden, sketches left visible for inspection — the whole point of this mode) is owned by `[SPUR-F-CLEANUP]`.

### 7: Extrude the Tooth

Find the single tooth cross-section profile in the Gear Profile sketch. The profile has 2 NURBS (the two flanks), 2 arcs (the tooth top and the root arc between them), and — unless `toothProfileIsEmbedded` — 2 short line segments (the flank-to-root lines). Find it with the framework helper — `find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)` (from `.utilities`) — do not re-implement the loop search; the helper rejects loops whose curve counts don't match and raises when nothing matches.

Extrude this profile from the target plane to the Extrusion End Plane (`ToEntityExtentDefinition`, PositiveExtentDirection) as a **New Body**. Name the feature `Extrude tooth`. Store the resulting body as `ctx.toothBody`.

### 8: Chamfer Tooth (optional)

If Apply-Chamfer-To-Teeth > 0, round off every edge of the tooth's front face *except* the arc shared with the root valley. The target face is found with a **single conjunction predicate**: walk `ctx.toothBody.faces` and take the first face for which **both** `face.edges.count == chamferWantEdges()` (6 for the spur base) **and** the face is coplanar with the gear's sketch plane (`sketchPlane.isCoPlanarTo(face.geometry)` with `sketchPlane = ctx.gearProfileSketch.referencePlane.geometry` — the same test step 9 uses, inverted). Both conditions are required of the *same* face — this is not an edge-count match followed by a coplanarity tiebreak. If **no** face satisfies both, raise (front face not found); do not fall back to a partial match. Skipping the root arc is the critical part: chamfering it would eat into the neighbouring tooth. (Known, accepted limitation: an **embedded** profile yields a 4-edge front face while `chamferWantEdges()` stays 6 for spur, so chamfering an embedded spur tooth raises the front-face-not-found error — users disable chamfer for such gears.)

**Identify the root arc by radius, not by relative size.** Walk the front face's edges and add each to the chamfer edge set, *except* skip any edge that is an `Arc3DCurveType` whose `edge.geometry.radius` equals `Root Circle Radius` (compare against the registered parameter's `.value`, tolerance `0.001` cm). That radius match is exact — the root arc is the only edge lying on the root circle — so it is more robust than "the smallest-radius arc." (This is the same radius-matching technique step 11 uses to find the root cylinders.) Everything else on the face — the two flanks, the tooth-top arc, and the two flank-to-root lines — gets chamfered. Apply with `chamferFeatures.createInput2()` → `chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, <ChamferTooth value>, False)`.

Helical and herringbone subclasses override only the expected edge count, via `chamferWantEdges()`; they change nothing else about this step. Their value (4) and its flagged caveat are owned by the subclass specs — see `spec/helicalgear/fusion.md` `[HELI-F-CHAMFER-COUNT]`; do not restate or second-guess them here.

### 9: Extrude the Body

Find the gear body profile — the annular loop bounded by **exactly 2 arcs** (the root circle and the tip circle): `find_profile_by_curve_counts(sketch, arcs=2)` (from `.utilities`). Extrude it from the target plane to the Extrusion End Plane (`ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`, `PositiveExtentDirection`) as a **New Body**. Name the feature `Extrude body` and the resulting body `Gear Body`.

While iterating the new body's faces (`extrude.bodies.item(0).faces`), capture two references needed later. Classify each face by `face.geometry.surfaceType`:

- **`Gear Center` construction axis** — from any face whose `surfaceType` is `CylinderSurfaceType`. Build it with `constructionAxes.createInput()` → `axisInput.setByCircularFace(cylindrical_face)` → `constructionAxes.add(axisInput)`. Name it `Gear Center`; set `isLightBulbOn = False`. Store on `ctx.centerAxis`.
- **`ctx.extrusionExtent`** (the far end-cap face, used later by the bore cut) — among faces whose `surfaceType` is `PlaneSurfaceType`, the one that is parallel to but **not** coplanar with the gear's sketch plane. Test it with the plane-geometry API rather than a hand-rolled dot-product: let `sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`, and pick the face where `sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry)`. (The near cap is coplanar with the sketch plane, so `isCoPlanarTo` rules it out; the cylindrical and side faces aren't planar.) Raise if either reference isn't found.

Finally store `ctx.gearBody` (the `Gear Body` body).

### 10: Pattern Teeth

Circular-pattern `ctx.toothBody` around the `Gear Center` axis, quantity = Tooth Number. Pin the two other pattern inputs: `patternInput.totalAngle = ValueInput.createByString('360 deg')` (a full turn, set as a string expression) and `patternInput.isSymmetric = False`. Combine the patterned tooth bodies into `Gear Body` via a single Combine-Join.

Feed the pattern's `bodies` collection to the combine as-is — it already includes the original tooth body, per `[PB-PATTERN-BODIES]`.

### 11: Fillets

If Fillet Radius > 0, round the corner where the root valley floor meets each tooth flank — the sharp inside corner that runs the full thickness of the gear, parallel to its main axis. This is where bending stress concentrates at the tooth root, so it's the structurally important fillet (not the front/back rim, which is a cosmetic rounding the user doesn't want here). Two things make picking the right edges fiddly:

- After the pattern-and-combine step, the root cylinder is usually split into one patch per valley rather than a single continuous surface. Collect *every* cylindrical face whose radius equals Root Circle Radius, not just the first one found.
- On each such face, keep the **axial straight edges** — the ones whose direction is parallel to the target plane's normal (i.e. parallel to the gear's main axis). Those are the two valley-floor-to-tooth-flank corners on each valley patch. Drop the *circular* edges that wrap around the circumference at the front and back end caps; those are end-cap rims, not the structural root corners. Filter first to `Line3DCurveType` edges, then take each line's direction from its **geometry endpoints** (`geometry.startPoint.vectorTo(geometry.endPoint)`), normalize, and keep it if it is parallel to the axis within tolerance: `abs(abs(dot(direction, axisNormal)) - 1.0) < 0.01`. (Use exactly this tolerance — a tighter test like `> 0.999` can drop valid axial edges that are slightly off due to tessellation, leaving root fillets missing.)

Apply the fillet with `filletFeatures.createInput()` → `filletInput.addConstantRadiusEdgeSet(edges, <Fillet Radius value>, isTangentChain=False)` — add the edge set on the input **itself**, per `[PB-FILLET-CHAMFER]` (do **not** route it through a `filletInput.edgeSetInputs` collection — that is the chamfer-side shape, and reaching for it on the fillet input fails). **`isTangentChain` must be `False`** — the collected edges are exactly the axial root corners; tangent-chaining (`True`) would let Fusion pull in tangent-adjacent edges and round more than the intended root corner. Do **not** read the direction via `edge.evaluator.getTangent(0)` — parameter `0` is not guaranteed to lie inside the edge's parameter range and Fusion raises `RuntimeError: invalid argument parameter`.

If the edge collection ends up **empty** (no axial root edge matched), silently skip the fillet — return without creating the fillet feature, no error. An empty edge set must not reach `filletFeatures.add`.

### 12: Bore (optional)

`buildBore` runs unconditionally from `generate()` (after `buildMainGearBody`), so it MUST itself early-return in two cases: when **SketchOnly** is set, and when **Bore Diameter ≤ 0**. The SketchOnly guard is essential — in sketch-only mode `buildMainGearBody` short-circuits before `buildBody`, so `ctx.gearBody` and `ctx.extrusionExtent` are never set; proceeding into the cut would dereference `None`. (Do not rely on the bore diameter being 0 in sketch-only mode — the user may have set both.)

Otherwise (full build, Bore Diameter > 0), create a separate `Bore Profile` sketch on the target plane and draw the bore circle **by instantiating the tooth generator on that sketch** — `SpurGearInvoluteToothDesignGenerator(boreSketch, self)` — and calling its `drawBore(ctx.anchorPoint, boreDiameter)`, which projects the anchor in and draws the construction-less circle of that diameter with a driving diameter dimension. Note the accepted side effect: the tooth generator's **constructor** always adds its local-origin `(0, 0, 0)` `SketchPoint` (see `[SPUR-F-LOCAL-ORIGIN]`), so the Bore Profile sketch carries one stray unused sketch point at (0,0,0) — faithful behavior, don't suppress it. Then extrude-cut the bore profile from the target plane to `ctx.extrusionExtent` (the far end-cap face), affecting only `ctx.gearBody`. The `ToEntityExtentDefinition` to the far face guarantees the bore goes all the way through regardless of Thickness.
