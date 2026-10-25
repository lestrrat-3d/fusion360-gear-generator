# Fusion playbook

This file records what Fusion does that matters to every gear: its units, its sketch solver, its features, and the
calls it refuses. Read it before you change the Fusion runtime `lib/geargen/decadfusion.py`, the translator in
`port/go2py` and `port/fusionexpr`, or a stored fragment under `port/<gear>/fragments/`. A behaviour of one gear goes in
`docs/fusion/<gear>.md` instead.

## Adding an entry

`CLAUDE.md` ("When Fusion gives a verdict") sends every Fusion finding that holds for all gears to this file.

- Start the paragraph with a new anchor in bold, `[PB-…]`. Code comments and the per-gear files cite anchors, so never
  rename or reuse one. To retire an entry, keep its heading and say in one sentence what replaced it.
- State what Fusion did, the date and Fusion version of the observation, and the error text when there was one.
- Say where the code relies on the rule, and cite the anchor in a comment at that place. Name the test that enforces
  it, if one does.
- Never put the fix in `lib/geargen/<gear>.py`: `gearport gen` writes that file. Change the runtime, the translator or
  the fragment, then regenerate.
- When code can express the rule, write the code and keep the entry short.

Entries carry a date or a Fusion version where one was recorded. An entry without either was observed in Fusion before
the gallery port and has not been checked since. API names and signatures were checked against the Fusion API
reference (FusionAPIReference commit 07814d1) with the `fusion:query-api` skill on 2026-10-25.

## How the add-in is put together

- `commands/_gear_command.py` (`GearCommand`) is the Fusion command of every gear. Each `commands/<gear>/entry.py`
  declares one `GearCommand` with the gear's configurator and generator class, and lists it in `commands/__init__.py`.
  The command refuses a design that is not a Hybrid or Assembly design, runs `generator_class(design).generate(inputs)`,
  and calls `deleteComponent()` when that raises. The error box carries a Build line ([PB-STALE-MODULE]).
- `port/cmd/gearport` writes each `lib/geargen/<gear>.py`. The dialog, the input reading, the parameter registration
  and the sketches the gallery does not model (the `Tools` anchor sketch, the spur's `Reference Circles`) come from the
  stored fragment `port/<gear>/fragments/<gear>.py.frag`. The build itself is the gallery model translated by
  `port/go2py`.
- The translated build calls the functions and classes in `lib/geargen/decadfusion.py`, one for each entry of the API
  table in `port/go2py/api.go`. `TestRuntimeDefinesEveryOperationTheTranslatorAccepts` (`port/spur/runtime_test.go`)
  fails when the table names something the runtime does not define.
- Every generator subclasses `base.Generator`. `getOccurrence()` adds the gear's component under the selected parent
  and sets the parameter prefix to `<prefixBase>_<component id without dashes>`. `addParameter` registers a user
  parameter under that prefix. `deleteComponent()` deletes those parameters and the occurrence.
- `HelicalGearGenerator` subclasses `SpurGearGenerator`, and `HerringboneGearGenerator` subclasses the helical one; the
  configurator and context classes follow the same chain. `TestGeneratedModuleKeepsTheSurfaceOtherGearsUse`
  (`port/spur/generate_test.go`) requires the three spur classes and every `PARAM_` and `INPUT_ID_` name they use.
- A gear module imports each name it uses explicitly. Only `lib/geargen/__init__.py` star-imports, to build the package.
- These helpers are in use: `base.get_value`, `base.get_boolean`, `base.get_selection`, `misc.settle_sketch_display`,
  `solids.name_feature` and `solids.hide_construction_geometry`. No gear calls the cone-cut, slicing and rotation
  helpers in `solids.py`, anything in `utilities.py`, or `spurproxy.py` and `legacyspur.py` any more.
- Every object a gear creates is named by its role. `tests/test_naming_by_role.py` fails on an unnamed creation in any
  `lib/geargen/*.py`.

## Dialog inputs

**[PB-INPUT-READ] Read each input with the helper that matches the type it was declared with.**

| declared in `configure()` | read with | reads |
|---|---|---|
| `addValueInput` / `addStringValueInput` | `get_value(inputs, id, units)` | `.expression` or `.value`, as a `ValueInput` |
| `addBoolValueInput` (checkbox) | `get_boolean(inputs, id)` | `.value` (bool) |
| `addSelectionInput` | `get_selection(inputs, id)` | `.selection(i).entity` |
| `addDropDownCommandInput` | `inputs.itemById(id).selectedItem` | the selected `ListItem` |

`BoolValueCommandInput` has no `expression` member, so `get_value` on a checkbox raises `AttributeError:
'BoolValueCommandInput' object has no attribute 'expression'` at generation time. The spur reads its Sketch Only box
with `get_boolean` and registers it as a `1`/`0` user parameter, which `getParameterAsBoolean` reads back. The bevel
and cycloidal fragments also read their numbers straight from `inputs.itemById(id).value` in `_params_of_inputs`, which
gives internal units ([PB-UNITS]). No test checks the pairing: `tests/test_addin_imports.py` stubs `adsk` with
`MagicMock`, which answers every name.

**[PB-GET-VALUE-CONTRACT]** `base.get_value` always returns a `ValueInput` that can go straight to `addParameter`. For a
string input whose text names an existing user parameter it returns `createByString(text)`, a live reference to that
parameter. For any other text, and for a numeric input, it evaluates the expression to internal units and returns
`createByReal`. It raises on an invalid expression and never returns `None`.

**[PB-UNITS] Fusion's internal units are the centimetre and the radian.** A `ValueInput.createByReal` number, a user
parameter's `.value`, a `ValueCommandInput`'s `.value`, sketch coordinates and a circle's radius are in those units,
whatever unit the dialog shows (API reference: `ValueCommandInput.value`, `SketchCircles.addByCenterRadius`,
`FilletEdgeSetInputs.addConstantRadiusEdgeSet`). `decadfusion.Runtime` takes a `scale`, the centimetres in one unit of
length of the translated code. The spur family runs in centimetres (1.0). The bevel and cycloidal gallery models use
absolute millimetres, so their fragments pass `MILLIMETRE = 0.1`. The runtime converts at every place a length enters
or leaves Fusion: `Runtime.to_world` and `to_local`, `Sketch._p3`, `value_input` and `Value.base`.

**[PB-DIALOG-DEFAULT-UNITS] A dialog default passed as `ValueInput.createByReal(...)` is in internal units, whatever
the input's unit string.** The API reference says a real initial value "is assumed to be in database units". A 10 mm
default is `createByReal(to_cm(10))` and a 20 degree default is `createByReal(math.radians(20))`; `createByReal(10)`
would show 100 mm. The unit string on `addValueInput` sets display and expression parsing only. The generators write
the conversion: `port/spur/generate.go` emits `to_cm(...)` and `math.radians(...)`, and `port/bevel` and
`port/cycloidal` emit `to_cm(...)` or a string such as `createByString('90 deg')`.
`TestDialogDefaultsComeFromTheGallery` (`port/bevel`, `port/cycloidal`) and
`TestHelixInputComesFromTheGalleryDefault` (`port/helical`) check the defaults.

**[PB-EVAL-EXPRESSION] `UnitsManager.evaluateExpression(expression, units)` returns internal units.** A `deg` field
comes back in radians, so convert with `math.degrees(...)` before a check on a range in degrees. The reference says it
returns -1 on an invalid expression, so `base.get_value` calls `isValidExpression` first and raises. Reading
`ValueCommandInput.value` gives the same internal-unit number.

**[PB-SELECTION-DECL] Each selection input's filters and `setSelectionLimits(min, max)` are part of the fragment, per
input.** The filters decide what the user can pick, and the limits decide when the dialog is valid.
`setSelectionLimits(1)` means at least one with no maximum, because a maximum of 0 means no limit. Every gear uses
`(1, 1)` for the Target Plane and the anchor point. Every gear pre-selects the root component as the parent. The
parent is `(1, 1)` in the spur family and the bevel, and `(0, 1)` in the cycloidal drive; the bevel and cycloidal
generators fall back to the root component when nothing is selected. Write filters as constants
([PB-SELECTION-FILTER-ENUM]).

**[PB-SELECTION-FILTER-ENUM] Write a selection filter as the named constant, never a quoted literal.** Use
`addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)`. The member names match the entity kinds
(`ConstructionPlanes`, `PlanarFaces`, `ConstructionPoints`, `SketchPoints`, `Occurrences`, `RootComponents`, `Bodies`).
The reference types the parameter as `addSelectionFilter(filter: str)`, and `SelectionCommandInput.ConstructionPlanes`
is the string `'ConstructionPlanes'`, so the constant and the literal pass the same value. An earlier version of this
entry said a literal raises `TypeError`; it does not. The constant is still the rule: a misspelt constant raises
`AttributeError` when `configure()` runs, while a misspelt literal only makes `addSelectionFilter` return `False`,
which no caller checks.

**[PB-SELECTION-STASH] Read every selection before anything creates the gear's occurrence.** Creating the occurrence
shifts Fusion's active context, and selections read after that come back empty. `getOccurrence()` creates it, and so
does anything that calls it, including `parameterName()` and `addParameter()`. The spur's `processInputs`, the bevel's
`generate` and the cycloidal drive's `processInputs` read their selections first.

**[PB-VALIDATE-INPUTS]** `GearCommand.command_validate_input` lets a gear disable OK and say why while the dialog stays
open. A generator class opts in with a static method `validate_inputs(inputs) -> list[str]`. It runs on every keystroke,
so it reads values straight from `inputs.itemById(id).value` (internal units), touches no parameter or document, and
returns one message per problem, best with the input's name and a numeric bound. An empty list means valid.
`GearCommand` then sets `args.areInputsValid = not problems`, and treats an exception from the hook (a half-typed
expression) as valid, so the generator's own check at execute time stays the backstop. To show the messages, add a
hidden read-only `addTextBoxCommandInput` after each editable input, with the input's id plus `'__status'`, and name a
fallback slot in the class attribute `DEFAULT_STATUS_INPUT_ID`. The handler writes the problems into the slot of the
input the user last changed, or the fallback for a selection or the first open, and hides the slot it showed before.
The cycloidal drive is the only gear that opts in. Its `validate_inputs` and its `processInputs` both call the
translated gallery `problems()`, so the two checks cannot drift; `TestTranslatedValidityChecksMatchTheGallery`
(`port/cycloidal`) compares that function with the gallery's.

**[PB-AUTOFOCUS-FIRST] Fusion focuses the first `SelectionCommandInput` of a dialog and ignores a later
`hasFocus = True`.** Add the selection that should open with focus first. Every gear adds its Target Plane first.

## Parameters

**[PB-NUMERIC-SNAPSHOT] Before the port, sketch dimensions and feature inputs were set from numbers, because live
links did not update reliably.** Setting `dim.parameter.expression` for a sketch dimension and
`ValueInput.createByString(...)` for a feature input had seemed to need the sketch reopened before downstream
geometry followed, and features picked up the expression only some of the time. The translated gears now link
some values: `decadfusion.Sketch._bind` sets a radius or diameter dimension's expression to the user parameter the
gallery binds it to, and `value_input` passes a `Value` as `createByString(<parameter name>)` to extrudes, fillets and
chamfers. Point dimensions, pattern counts, plane offsets and the spur's `Reference Circles` dimensions are still
numbers taken at generation time. No Fusion run has yet recorded whether editing a linked parameter updates a
translated gear; re-run the dialog to change a gear until one does.

**[PB-PARAM-EXPRESSIONS] What Fusion's parameter expressions accept.**
- Through the API, lowercase `pi` parses as a unit, not the constant. It evaluates to 0 in a unitless parameter and is
  rejected next to a length. Capital `PI` is the constant (Fusion 2705.1.25, 2026-10-23). `port/fusionexpr` writes `PI`,
  and `TestTranslateWritesPiInCapitals` checks it.
- Dividing an angle by `1 deg` gives a plain number (Fusion 2705.1.25). `port/fusionexpr` translates the gallery's
  `deg(a)` to `(a / 1 deg)`, and the spur's `InvoluteAngle` uses it to subtract an angle from `tan(...)`.
- An angle used as a bare factor of a length must be registered unitless (`''`), not `'rad'`. Otherwise the dependent
  length expression has the unit `mm·rad` and Fusion rejects it. The spur registers `InvoluteAngle` unitless.
- `port/fusionexpr` refuses a gallery function it has no rule for (`TestTranslateRefusesWhatItHasNoRuleFor`).

**[PB-PRECOMPUTED-MODE] A gear either registers Fusion user parameters or keeps its numbers in Python.** The spur
family and the cycloidal drive register user parameters, with derived ones written as expressions of the dialog's
values ([PB-PARAM-EXPRESSIONS]). The bevel pair registers none: its table lives in `decadfusion.ParamTable`
([PB-PARAM-TABLE]), and its fragment raises if the runtime asks for a Fusion parameter. A value Fusion cannot express
is computed in Python and registered with `createByReal`, as the cycloidal drive does for `PinRadius` and
`OutputHoleDiameter` (their comments say "numeric snapshot").

**[PB-PARAM-TABLE] A model that only reads its parameters back keeps them in Python.** The bevel gear's
`defineParameters` writes inputs and expressions into a table, and its build reads the results (cone angle, radii, face
width). No sketch binds a dimension to a parameter, so no Fusion user parameter is made. `decadfusion.ParamTable` holds
the values and evaluates the expressions with Python's arithmetic and the functions in `_EXPRESSION_FUNCTIONS` (`hypot`,
`atan2`, `atan`, `asin`, `acos`, `sin`, `cos`, `tan`, `sqrt`, `min`, `max`, `pow`, `abs`). `Runtime.param` reads from it
first.

## Sketches

**[PB-SKETCH-FIRST]** Retired with `proof/` in #192. The gallery proves each model's sketches with its own tests, and
nothing in this repository checks the degrees of freedom of the Fusion-only sketches (see
`docs/gallery-port-pipeline.md`, "Checks").

**[PB-FULL-CONSTRAINT] Leave no sketch under-constrained, and check it with `sketch.isFullyConstrained`.** Every point
either carries constraints or dimensions, or is driven by them (a projected point follows its source). A free degree
of freedom lets the geometry move between rebuilds. `decadfusion.sketchcheck_verify_fully_constrained` raises for a
translated sketch that is not fully constrained, and `unconstrained_entities` lists the curves and fit points it left
free, in mm. The fragments raise the same way for their `Tools` sketches. A labelled sketch is the one exception
([PB-TEXT-HOLDS-DOF]).
  - **[PB-REFLINE-DIRECTION] Fix reference-line directions.** A line created only with coincident or midpoint
    constraints and a length still rotates freely (1 DOF). Pin it with a sketch-local `addHorizontal`/`addVertical`,
    which holds on a tilted plane because it uses the sketch's own frame, or with an angle dimension. The bevel's
    `Tools` line uses `addHorizontal`.
  - **[PB-CIRCLE-CENTER] A circle's centre is free even when created at (0, 0, 0).**
    `sketchCircles.addByCenterRadius(Point3D.create(0, 0, 0), r)` does not reuse the sketch's `originPoint`; its
    `centerSketchPoint` is a new free point that sits at the origin. Pass an existing `SketchPoint` as the centre to
    share it (the reference allows either), or fix the new centre. `addCoincident(circle.centerSketchPoint,
    sketch.originPoint)` was observed to throw `VCS_SKETCH_SOLVING_FAILED` on a `setByDistanceOnPath` plane.
    `decadfusion.Sketch.create_circle` places a free centre with dimensions from the local origin.
  - **[PB-PROJECT-NOT-FIXED] Check each projected entity's constrained state.** `sketch.project(...)` brings geometry
    in associatively, but its displayed position does not prove the sketch is fully constrained. A projection from a
    constrained source anchors a point, as the spur `Tools` sketch shows by passing `isFullyConstrained`. A projection
    from geometry with free DOF can leave the destination under-constrained. For a free projection, either
    `addCoincident` it to a fixed point, or create new points with
    `sketch.sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))`, draw the curves that share them, and only
    then set `isFixed = True` on the line endpoints. Setting `isFixed` on a bare point before a line uses it did not
    leave the sketch fully constrained.
  - **[PB-NO-OVERCONSTRAIN] Do not over-constrain to get there.** A dimension on a length that perpendicular,
    collinear or closing constraints already drive throws `VCS_SKETCH_OVER_CONSTRAINTS`. Full constraint comes from the
    missing constraint, not from more dimensions.
  - **[PB-COLLINEAR-CHAIN] A collinear names the line the new line's start point actually sits on, never a line farther
    up the same chain.** `addCollinear` carries two point-on-line rows. When the new line starts at the reference
    line's own endpoint, one row is already satisfied and Fusion absorbs the other. When the start reaches the named
    line only through an earlier collinear, the two chains assert the same fact twice and the sketch over-determines.
    In a chain A→E then E→G, the second collinear names A→E, not the axis A→E is collinear with.
    - Measured 2026-09-03 on the bevel lattice: `addCollinear(E→G, Apex→A)` raised `RuntimeError: 3 : failed to
      create offset: VCS_SKETCH_OVER_CONSTRAINTS` at the second such call, and `addCollinear(E→G, A→E)` builds. Where
      both endpoints are already fixed, drop the collinear and pin the point with two point-on-line `addCoincident`
      calls.
    - The sketch library behind the gallery's tests counts a collinear as the same two rows, so a gallery test cannot
      tell the two forms apart. Only Fusion does.

**[PB-DIM-VALUE-SEMANTICS] A linear sketch dimension's value is a magnitude, with a direction captured at creation.**
`addDistanceDimension(...)` measures the geometry as it sits and creates its parameter at the absolute distance: a
point at Δy = −0.5 gives `parameter.value == +0.5`, and the dimension remembers which side the point was on. Assigning
a negative value moves the point to the opposite side (verified in Fusion: −0.5 moved the point to Δy = +0.5, and
+0.5 moved it back). So choose a point's side by seeding it on that side before the dimension exists, and assign only
`abs(Δ)`. A signed delta mirrors the point whenever it is negative; the sketch still solves, and the fault shows only
later as a missing profile. The gallery's sketch library takes signed horizontal and vertical distances.
`decadfusion.Sketch` seeds each point at the gallery's signed coordinates and lets `addDistanceDimension` measure, so it
never assigns a distance value.

**[PB-TEXT-HOLDS-DOF] Sketch text counts toward `isFullyConstrained`, so a labelled sketch may not report fully
constrained.** Text placed with `setAsAlongPath` ([PB-SKETCH-TEXT]) has its own position along the curve, and nothing
pins it. Measured 2026-09-02 on the spur tooth sketch: `False` with its four circle labels, `True` with the labels
deleted and no other change. The bevel tooth sketches, labelled the same way, read `False` in one run on 2026-09-12 and
`True` in the two runs after it, with identical counts (53 points, 47 constraints, 41 dimensions, 4 texts) and the same
coordinates to four decimals, so the reading is not stable ([PB-SETTLE-DISPLAY]). Never gate a labelled sketch on the
property; log it. This exempts the labels and nothing else. The spur now draws its labels in a `Reference Circles`
sketch of their own, which is not gated, and the translated sketches carry no text and are gated.

**[PB-SETTLE-DISPLAY] Fusion's browser shows a stale constraint icon for a sketch a generator authors, until something
makes it settle.** The icon reflects the sketch as it was when created, before its constraints existed, so a finished,
fully constrained sketch shows as unconstrained. Opening the sketch and closing it refreshes the icon.
  - The icon is not evidence. Measured 2026-09-12: `isFullyConstrained` returned `True` for the bevel's Anchor, Gear
    Profiles and both per-gear Profile sketches when each was built and after the last feature, while all four icons
    showed otherwise. Read the property, not the browser.
  - `commands/_gear_command.py` records the sketches that exist before `generate()`, and afterwards calls
    `geargen.settle_sketch_display` once. That toggles `isComputeDeferred` on each new sketch, reads
    `isFullyConstrained`, and calls `adsk.doEvents()`. Every gear goes through that one call, so no fragment or runtime
    function adds its own.
  - It changes only the display. Across three instrumented bevel builds the geometry, constraint and dimension counts
    were the same with and without it. It also refreshed a control sketch it was not applied to, which points at
    `adsk.doEvents()` as the part that does the work; both parts are kept because only the pair was measured.
  - Sketch-only spur, Fusion 2705.1.25: the first run showed the `Tools` and `Gear Profile` sketches as not fully
    constrained, and a later run showed both fully constrained without any action. The first reading was not
    reproduced. Nothing in the port changed the sketch calls, so no code changed.

**[PB-SKETCH-TEXT] Along-path sketch text takes three calls.**
```python
textInput = sketch.sketchTexts.createInput2(text, height)   # height in cm
textInput.setAsAlongPath(curve, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
sketch.sketchTexts.add(textInput)
```
`setAsAlongPath(path, isAbovePath, horizontalAlignment, characterSpacing)` lays the text along the curve; the last
argument is a percentage change in character spacing. The reference now lists only
`createInput3(expression, height: ValueInput)`, but Fusion still runs `createInput2(text, height)`: the spur's
`Reference Circles` sketch uses it, as the add-in loaded on 2026-10-23 did.

**[PB-SHARE-XOR-COINCIDENT] Never bind a point twice: share it or add one coincident, not both.** To start a curve at
an existing sketch point, either pass that `SketchPoint` into the creation call, `addByTwoPoints(existingPoint,
otherEnd)`, and add no coincident, or create the curve from a `Point3D` and add exactly one
`addCoincident(curve.startSketchPoint, existingPoint)`. Both together stack a redundant coincident and the solver fails
with `VCS_SKETCH_SOLVING_FAILED`. Never `addCoincident` two points that are already one shared point.
  - Not every creation call shares every point it is given. `sketchArcs.addByCenterStartEnd(centre, start, end)`
    shares the start and end but copies the centre into a new free `SketchPoint`, so the arc needs
    `addCoincident(arc.centerSketchPoint, existingPoint)`. That is the one case where passing a point and adding a
    coincident to it is right. The reference adds that an end point not on the arc is replaced by a new point that is.
    Compare `entityToken`s to confirm sharing. Found 2026-09-02: the bevel tooth-top arc's centre stayed 22.9 mm behind
    its origin, giving a 0.5743 mm arc where 22.5 mm was intended, on a sketch that raised no error.
    `decadfusion.Sketch._drawArcs` adds that coincident ([SPUR-F-ARC-CENTRE] in `docs/fusion/spurgear.md`).

**[PB-SEED-NEAR] Seed coordinates near the solved position; the solver depends on the seed.** Fusion's sketch solver is
iterative. A seed far from its solution, or on the wrong side of a parallel or offset target, can fail with
`VCS_SKETCH_SOLVING_FAILED` although the constraints can be met. A point that must end on a line should start near
that line. `decadfusion.Sketch` seeds every point at the gallery's solved coordinates.

**[PB-SOLVED-GEOMETRY] Read solved `.geometry` for any value computed from constrained points, not the seed
coordinates.** Fusion solves as each constraint is added, so a point's `.geometry` is its solved position once the
constraints that place it exist. The seeds passed to `addByTwoPoints` can differ from the solved positions, markedly so
for asymmetric configurations.

**[PB-WORLDGEO-CONSTRAINED] Read `.worldGeometry` only from a constrained sketch entity.** A point used to build a world
axis, origin or tool must be shared with or coincident to fixed geometry. A fully free sketch point gave a wrong
`worldGeometry` and the feature built from it landed in the wrong place. The case that showed this was not recorded,
so treat the mechanism as unverified; the rule costs nothing to follow.

**[PB-WORLD-FRAME] `.geometry` is in the sketch's frame and `.worldGeometry` in the design's; match the frame of what
you compare against.** Mixing a sketch-frame curve with a world axis returns wrong numbers with no error, for example a
wrong spiral twist that made meshing teeth interfere. The two frames agree only for a sketch on the world XY plane.
The runtime reads the anchor's `worldGeometry` and maps every position through `Runtime.to_world` and `to_local`.

**[PB-SPACE-METHODS] `sketch.modelToSketchSpace(p)` and `sketch.sketchToModelSpace(p)` are methods that transform a
point, not matrices.** Each takes a `Point3D` and returns one: `local = sketch.modelToSketchSpace(worldPoint)`. Passing
the bound method to `Point3D.transformBy`, which expects a `Matrix3D`, raises `TypeError`. `decad_scaled_at` uses
`modelToSketchSpace` to place its base point.

**[PB-POINT-HELPER]** Retired: it was advice for hand-written point helpers. The runtime builds every `Point3D` from
numbers in one place each (`Sketch._p3`, `Runtime.to_world`).

**[PB-DRIVING-DIM] Driving and driven dimensions.** Every `add*Dimension(...)` takes a trailing `isDriving`, default
`True`. Passing `False` makes a driven dimension, which constrains nothing.

**[PB-OFFSET-DIM] Line-to-line offset dimension.** To dimension the gap between two parallel lines, use
`sketch.sketchDimensions.addOffsetDimension(line, entityTwo, textPoint)`, where `entityTwo` is a parallel line or a
point, and set `.parameter.value`. It is not `addDistanceDimension`, which is point to point. Add `addParallel` only
when the lines are not already parallel by construction (for example both perpendicular to a third line); a redundant
`addParallel` throws `VCS_SKETCH_OVER_CONSTRAINTS`.

**[PB-ANGULAR-DIM] An angular dimension measures the quadrant that holds its text point.**
`addAngularDimension(lineA, lineB, textPoint)` dimensions the angle on the side where `textPoint` lies, so place it
inside the wedge you mean (a shaft angle Σ, not 180 − Σ).

**[PB-RADIAL-DIM] A radial or diameter dimension's text point must not be the centre.** `addRadialDimension(arc,
textPoint)` and `addDiameterDimension(circle, textPoint)` reject a text point at the centre with `RuntimeError: 3 : …
一部の入力引数が無効です` ("some input arguments are invalid"). `decadfusion.Sketch._dimensionCircle` and the spur's
`Reference Circles` put it at `(r, r)`.

**[PB-SINGLE-PROFILE] When a sketch has one closed loop, take `sketch.profiles.item(0)`.** A filter on the curve type
of the loop (`ProfileCurve.geometry.curveType == Line3DCurveType`) once rejected a valid all-line loop and made a
revolve fail with "could not find profile". Filter only when the sketch holds several profiles.

**[PB-PROFILE-MATCH] When a sketch holds several profiles, find one by its curves, not its index.** Iterate
`sketch.profiles` and `profile.profileLoops`, and match the loop whose `profileCurves` have the expected count and
types. Read a curve's type from `curve.geometry.curveType` and compare it with `adsk.core.Curve3DTypes`, whose members
end in `CurveType`: `Line3DCurveType`, `Arc3DCurveType`, `Circle3DCurveType`, `NurbsCurve3DCurveType`.
`NurbsCurve3DType` and `Arc3DType` do not exist. `decadfusion.select_embedded_tooth_profile` matches this way
([PB-EMBEDDED-TOOTH]).

**[PB-PROFILE-STALE] Adding a curve to a sketch invalidates the profiles already read from it.** A revolve whose axis
was added to the profile's own sketch after `sketch.profiles` was read failed with `invalid profile(s) for Revolve
Feature`. The axis goes in a sketch of its own, or the sketch draws it before the profiles are read.
`Runtime.axis_line` draws each axis in a Fusion-only sketch, and one line serves every revolve and pattern about the
same axis ([PB-FRAME-PLANE]).

**[PB-BAD-ENTITY] `Sketch.project` answers `Bad entity parameter` for an entity that is missing or stale.** Two causes
were seen. The entity was `None`, because a sketch took its anchor from a plane that has none. And a `SketchPoint`
that another sketch had projected is refused once that sketch gains curves and constraints. So the bevel projects the
point the user chose into its `Pair Tools` sketch, not the copy its `Tools` sketch holds. The spur and cycloidal
`Tools` sketches hold only the projected point, so the runtime projects that copy. `tests/test_bevel_runtime.py` covers
the missing-anchor case.

**[PB-POINT-POSITION] The position of a point has a different name on each kind of point.** A `SketchPoint` has
`geometry` in its sketch and `worldGeometry` in the design. A `ConstructionPoint` and a `BRepVertex` have `geometry`
only. `Runtime.build_frame` reads whichever exists for the user's anchor, and `tests/test_bevel_runtime.py` covers both
kinds.

**[PB-DIMENSION-COST] The cost of a sketch's dimensions grows with the cube of its points, so a repeated shape is
patterned.** `addDistanceDimension` took 0.003 s per call on a 40-point sketch and 0.06 s per call after 150 spline fit
points. The cycloidal drive's rotor profile had 457 dimensioned points and its dimensions took 359 s; the casing's 448
took 99 s. Deferring the solver (`isComputeDeferred`), hiding the sketch and switching off its display changed none of
this: the deferred solve took 0.2 s, so the time is in the calls. A shape of repeated lobes is therefore drawn once and
patterned, as the gallery model does ([PB-SPLINE-RING]).

**[PB-SPLINE-RING] A circular pattern of splines closes a ring with one coincident.** The gallery draws one lobe of
chained fitted splines, leaves the last end point free, and patterns the lobe about a centre point. In Fusion,
`GeometricConstraints.addCircularPattern` copies the splines in 0.02 s, and the copies already form one closed outline
with the seed, because Fusion connects curve ends that sit on one point. Only the seed's free end needs a constraint: a
coincident to the start of the first copy, found as the created spline whose first fit point lies one step around from
the seed's start. That leaves the sketch fully constrained. A coincident between every neighbouring pair is refused as
redundant for 44 of 45 junctions, and merging the end points works but is slower and no better.
The pattern goes in after the local origin is anchored. While the origin is a loose point, the copies' junction points
stay free: with the anchor coincident added after the pattern, the check failed with the end of each copy's last
spline and the point beside it free, and with the anchor added before the pattern it passed.
`decadfusion.Sketch.finish` therefore anchors between the dimensions and the patterns when the sketch has a pattern,
and last otherwise ([SPUR-F-LOCAL-ORIGIN] in `docs/fusion/spurgear.md`).

**[PB-EMBEDDED-TOOTH] The embedded virtual tooth is the region bounded by two flank splines, a tip arc and a root
arc.** Decad numbers the trim of each flank at the root circle. Fusion's regions carry their trimmed curves, so
`decadfusion.select_embedded_tooth_profile` picks the one valid region with no hole and four edges, two of them
splines, and raises when it finds any other number.

**[PB-HIDE-AFTER-USE] Hide a sketch only after the features that use it, and hide construction geometry with
`isLightBulbOn`.** Projection and profile reads failed on invisible sketches before the port, so a sketch stays visible
while it is used. `Runtime.create_sketch` makes every translated sketch visible, and `Runtime.hide_helpers` hides the
sketches and planes after the build. The failure depends on the operation: before the port the helical gear lofted the
profile of a sketch that was never shown. Construction planes and axes are hidden with `isLightBulbOn = False`;
`ConstructionPlane.isVisible` is read-only in the reference. A sketch has both a writable `isVisible` and
`isLightBulbOn`: the runtime and the spur set `isVisible = False`, and `solids.hide_construction_geometry`
([PB-TREE-CLEANUP]) turns off `isLightBulbOn` on every sketch, plane and axis of the bevel and cycloidal components.

## Fusion API names

**[PB-API-LOOKUP] Look an API name up before you write it.** The `fusion:query-api` skill answers from a compiled
database of the Fusion Python API, and it resolves inherited members the way Python does. Ask `show <Class>.<member>`
for one call: it gives the declaring class, signature and docs. When `show` finds nothing, the name as written does not
exist on that class; `members <Class>` then lists what the class offers, which is how you find that `sketchCircles`
belongs to `SketchCurves`, not `Sketch`. `search <term>` matches names, `doc-search <term>` matches documentation, and
`tree <Class>` gives the base chain.
The reference leaves out some members that Fusion still runs and the current code calls: `Sketch.project` (the
reference lists `project2`), `SketchTexts.createInput2` ([PB-SKETCH-TEXT]) and
`FilletFeatureInput.addConstantRadiusEdgeSet` ([PB-FILLET-CHAMFER]). A name missing from the reference is therefore
not proof that Fusion lacks it, but a new call should use a name the reference lists.

**[PB-ADSK-MODULES] `adsk.core` or `adsk.fusion`: a name in the wrong module fails only at run time.** `adsk` cannot be
imported outside Fusion, so a parse accepts `adsk.fusion.SurfaceTypes` although `SurfaceTypes` lives in `adsk.core`;
it fails in Fusion with `AttributeError: module 'adsk.fusion' has no attribute 'SurfaceTypes'`. No test catches it:
`tests/test_addin_imports.py` stubs both modules with `MagicMock`. `show <Name>` gives the qualified name. Geometry and
value types (`Point3D`, `Vector3D`, `Matrix3D`, `ObjectCollection`, `ValueInput`, `SurfaceTypes`, `Curve3DTypes`) live
in `adsk.core`. Feature and topology types (`FeatureOperations`, `SurfaceProjectTypes`, `PatternComputeOptions`,
`PointContainment`, `Path`) live in `adsk.fusion`.

**[PB-API-SPELLING] Geometric-constraint method names are exact.** The collinear constraint is
`geometricConstraints.addCollinear(...)`, with two l's; `addColinear` raises `AttributeError`, and Fusion's message
suggests the right name. Copy the casing of `addCoincident`, `addPerpendicular`, `addParallel`, `addHorizontal`,
`addMidPoint` (capital P), `addOffsetDimension` and `addDiameterDimension` from `members GeometricConstraints` and
`members SketchDimensions`.

**[PB-SKETCHCURVES] Curve collections live under `sketch.sketchCurves`.** Add curves through
`sketch.sketchCurves.sketchCircles`, `.sketchLines`, `.sketchArcs` and `.sketchFittedSplines`. `sketch.sketchCircles`
raises `AttributeError: 'Sketch' object has no attribute 'sketchCircles'`. `sketch.sketchPoints`,
`sketch.sketchDimensions`, `sketch.geometricConstraints`, `sketch.sketchTexts` and `sketch.profiles` are members of
the sketch itself.

**[PB-FILLET-CHAMFER] Fillet and chamfer inputs take their edge sets in different places.**
  - Fillet: the reference documents `filletInput.edgeSetInputs.addConstantRadiusEdgeSet(edges, radius,
    isTangentChain)`. `decadfusion.decad_fillet` calls `filletInput.addConstantRadiusEdgeSet(...)` on the input itself,
    which the reference no longer lists but Fusion runs: the spur add-in loaded on 2026-10-23 used it. An earlier
    version of this entry said `edgeSetInputs` does not exist; the reference shows that it does.
  - Chamfer: `chamferInput = chamferFeatures.createInput2()`, then
    `chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distance, isFlipped)`, as `decad_chamfer` does.

**[PB-LOGGING] Log progress and let failures raise.** Log steps with `futil.log(...)`; `decadfusion.timed` and the
`logged` decorator time each operation under its role name ([PB-PROGRESS-FILE]). Do not catch an error inside a step to
carry on. `GearCommand.command_execute` catches it, shows it and calls `deleteComponent()`.

## Bodies and features

**[PB-EMPTY-RESULT] A geometry operation can yield nothing, so check the count where the collection is made.** A split
that misses, a face or edge search that finds nothing, a piece list after filtering: each can be empty, and `max([])`
or `coll[0]` then fails far from the cause. Raise right after the operation with the operation, the part and the
count. The runtime does this: an extrude, loft or revolve raises unless it made one body, `_edge_collection` raises on
a wrong or zero edge count, and `decad_split` raises unless the sheet made two pieces.

**[PB-SELF-DIAGNOSING] Make failures that depend on topology carry the measured quantities.** In a Fusion add-in the
exception text is the only telemetry. "expected 3 pieces, got 2" helps little; the candidate distances, pieces in and
out, and the entity selected point at the cause. `unconstrained_entities`, `_edge_collection` ("selected N edges, want
M") and `_pattern_copies` ("no pattern instance lies at θ rad") follow this. Prefer a rich error to returning
unchanged on a miss.

**[PB-PATTERN-BODIES] The bodies of a circular pattern feature, and joining them.** The reference says `Feature.bodies`
holds the bodies the feature modified or created. Older add-ins treated it as the seed plus the copies; the runtime
does not rely on whether the seed is listed and drops it by `entityToken` (`decad_pattern_copies`). `pattern.bodies` is
a `BRepBodies`, and `combineFeatures.createInput(targetBody, toolBodies)` takes an `ObjectCollection`, so copy the
bodies into `adsk.core.ObjectCollection.create()` first, as `_combine` does.

**[PB-CIRCULAR-PATTERN] Set every circular-pattern input.** `circularPatternFeatures.createInput(entities, axis)`, then
set `quantity`, `totalAngle = ValueInput.createByString('360 deg')` and `isSymmetric = False` before `add(input)`;
do not rely on Fusion's defaults. `patternComputeOption` (`adsk.fusion.PatternComputeOptions`) is also available; no
current gear sets it. `decad_pattern_copies` is the implementation. Its axis is a construction axis from a cylindrical
face on the pattern axis, or a construction line when no such face exists ([PB-FRAME-PLANE]).

**[PB-LAZY-JOIN] Join and copy once, not once per tooth.** Fusion's kernel joins any number of bodies in one combine,
and makes every copy of a body at equal angles in one circular pattern. The gallery joins and copies tooth by tooth,
because decad's kernel needs that, so the runtime records a `decad.Union` and a rotated `decad.Copy` and makes the
pattern and the single combine when a later step first reads the body. Thirty-one teeth cost two features, not about
ninety. A copy at an angle that is not part of an equal set is made on its own. `tests/test_bevel_batching.py` counts
the features.

**[PB-REVOLVE] Revolve.** `revolveFeatures.createInput(profile, axis, operation)`, then
`setAngleExtent(False, ValueInput.createByString('360 deg'))`, then `add(input)`. The axis is a sketch line or a
construction axis. The profile must not cross the axis: Fusion aborts with `RuntimeError … ASM_WIRE_X_AXIS … the
profile crosses the axis of revolution`. The gallery owns the geometry that keeps a profile on one side.
`decad_revolve` translates only a full revolution.

**[PB-LOFT] Loft, including from a point.** `loftFeatures.createInput(operation)`, then `loftSections.add(entity)` for
each section in order, then `add(input)`. A section may be a single point (a `SketchPoint` or `ConstructionPoint`) for
a pointed end. `decad_loft_from_point` lofts from the anchor `SketchPoint`, and `decad_loft_faces` lofts through planar
faces.

**[PB-THROUGH-CUT] Symmetric through-cut extent.** For an extrude cut that must pierce a body whatever its thickness,
use `extrudeInput.setSymmetricExtent(ValueInput.createByReal(largeDist), False)`. With `isFullLength = False`,
`largeDist` is the length on each side. The third argument, a taper angle, is optional and not needed. Restrict the cut
with `participantBodies = [theBody]`. No current gear cuts this way; cuts go through `combineFeatures`.

**[PB-SPLIT-BODY] Split a body.** `splitBodyFeatures.createInput(bodyToSplit, splittingTool, isSplittingToolExtended)`,
then `add(input)`. The reference accepts a solid or open body, a construction plane, a profile or a face as the tool.
`isSplittingToolExtended = True` extends the tool to cross the whole target. A split that does not intersect raises
`RuntimeError … SPLIT_TARGET_TOOL_NOT_INTERSECT`, and the message text may be localized. A surface built from a sketch
line in a sibling or parent component failed as a tool ([PB-NO-CROSS-SIBLING]). `decad_split` uses a sheet body
extruded in the same component.

**[PB-SPLIT-TOOL] A Split Body feature leaves its splitting tool in the design.** The gallery's `decad.Split` consumes
the sheet, so `decad_split` removes the sheet body after the split. A sheet left behind shows as a plate, one per cut,
across the gear.

**[PB-FACE-BY-MIDPOINT] Find a face by surface type, and break ties by trying the operation.** Iterate `body.faces` and
compare `face.geometry.surfaceType` with `adsk.core.SurfaceTypes` (`ConeSurfaceType`, `CylinderSurfaceType`,
`PlaneSurfaceType`). Measuring a point's distance with `surface.evaluator.getParameterAtPoint` and
`getPointAtParameter` measures to the infinite surface, so trimmed faces on one surface tie at distance 0, which is
common with near-coaxial cones. `getParameterAtPoint` returns no result near a singularity such as a cone's apex, so a
point on the surface can have no distance. When the face feeds an operation, order the candidates by distance, treat
an unevaluable distance as "try last", and keep the first face for which the operation succeeds. Identify a revolved
edge's face by the edge's midpoint, which is clear of the apex and differs between adjacent cones. Never break a tie by
the order of `body.faces`. The helpers `solids.find_cone_faces_by_midpoint` and `apply_conical_cut` implement this; no
current gear calls them.

**[PB-REMOVE-PIECES] Select and remove split pieces.** Identify pieces with `body.pointContainment(worldPoint)`, which
returns `adsk.fusion.PointContainment.PointInsidePointContainment`, `PointOnPointContainment`,
`PointOutsidePointContainment` or `UnknownPointContainment`, and rank them by `body.physicalProperties.volume`. Delete
an unwanted piece with `component.features.removeFeatures.add(body)`, which shows in the timeline, not `deleteMe()`.
`decad_remove` uses a Remove feature.

**[PB-MOVE-ROTATE] Move or turn a body.** `moveFeatures.createInput2(bodies)`, then `defineAsFreeMove(matrix)`, then
`add(input)`, with the matrix from `Matrix3D.setToRotation(angle, axisVector, originPoint)`. `decad_transform` does
this. An earlier note said `defineAsRotate` rejects a `SketchLine` axis; the reference says it accepts a linear edge,
construction axis or sketch line, and the case behind the note was not recorded.
  - A zero angle is refused. `setToRotation(0, axis, origin)` builds the identity, and Fusion refuses to move a body by
    it with `RuntimeError: 3 : invalid transform` (measured 2026-09-02 on the bevel pinion, whose mesh phase was 0 by
    default). `decad_transform` does not skip a zero angle, so a translated build that can turn by zero needs that
    check in the runtime. The current calls turn by π/teeth, the shaft angle or a spiral twist.

**[PB-SCALE-BASE-POINT] A Scale feature's base point is a sketch point, never a construction point at a bare point.**
The reference says `ConstructionPointInput.setByPoint` with a `Point3D` is valid only in direct modeling and fails in a
parametric design, where Fusion raises "3 : Environment is not supported". `decad_scaled_at` puts a sketch on a planar
face that passes through the anchor and adds the base point there with `modelToSketchSpace`. The previous bevel add-in
also made its Design occurrence active for its scale features (`docs/fusion/bevelgear-design-record.md`). `decad_scaled_at`
activates nothing, and the default spiral pair built in Fusion with it, so a Scale feature needs no active occurrence
when its base point is a sketch point.

## Construction geometry

**[PB-CONSTRUCTION-PLANES] Construction planes.** `constructionPlanes.createInput()`, then `setByOffset(plane,
ValueInput)`, `setByAngle(linearEntity, ValueInput, planarEntity)` (through a line, at an angle to a plane; `'90 deg'`
gives a perpendicular plane) or `setByDistanceOnPath(pathEntity, ValueInput)` (normal to a curve at a fraction 0 to 1
along it). Pass the `SketchLine`, edge or axis itself. Wrapping a sketch curve in `adsk.fusion.Path.create(curve, …)`
raised `RuntimeError … InternalValidationError : Utils::getObjectPath(sketchCurve, …)` when the curve's sketch was in
another component. The bevel's `Pair Plane` uses `setByAngle` on its `Tools` line, and `Runtime._plane_across` uses
`setByDistanceOnPath` on a construction line.

**[PB-CONSTRUCTION-AXES] Construction axes.** `constructionAxes.createInput()`, then `setByCircularFace(face)` (the axis
of a cylinder, cone or torus face) or `setByTwoPlanes(planeA, planeB)` (their intersection, which needs no face, unlike
`setByPerpendicularAtPoint`). `setByLine(InfiniteLine3D)` is valid only in direct modeling, per the reference. The
runtime's pattern axis uses `setByCircularFace` (`_axis_through`), and a construction line in a sketch when no face
lies on the axis.

**[PB-CONSTRUCTION-NEEDS-ACTIVE]** Replaced: "Environment is not supported" comes from construction inputs that are
valid only in direct modeling, not from an inactive component. `ConstructionPointInput.setByPoint` with a `Point3D` and
`ConstructionAxisInput.setByLine` are such inputs ([PB-SCALE-BASE-POINT], [PB-CONSTRUCTION-AXES]). Planes and axes
defined by entities build in a component that is not active: the spur gear's pattern axis and the bevel's `Pair Plane`
do. Where a point or line will do, a `SketchPoint` or sketch line is still the simplest choice.

**[PB-FRAME-PLANE] A plane across the base plane comes from a construction line.** The bevel pair's bore plane and
tooth plane are perpendicular to the base plane and pass through a point of it. A Fusion-only sketch on the base plane
holds a construction line from that point along the plane's normal, and the plane is normal to the line at its start,
so it follows the anchor. Fusion picks the axes of a sketch on such a plane, so the sketch maps the gallery's
coordinates to them by the dot products of the frame's axes and the sketch's. When that map is a reflection, it draws
each arc from the other end and extrudes the other way, and it anchors on the line's start point. A pattern about an
axis that no cylindrical face lies on uses the same kind of line as its axis.

**[PB-PAIR-PLANE] A gear that stands on the Target Plane is drawn in a plane across it, with its apex above the
point.** The bevel's driving gear stands on its flat back face on the Target Plane, with the Center Point at the centre
of that face. The pair's apex is above the point on the side the plane's normal points to, by R cos(gamma) plus the
driving base height, which is the blank's heel station. The pinion shaft is the driving shaft turned about the apex,
toward the greater X of the plane that holds both shafts. The gallery draws the pair with its apex at the origin, so
the add-in places it. A short line through the Center Point along the `Tools` sketch's X axis, fully constrained,
carries a construction plane at 90 degrees to the Target Plane, or at minus 90 when the pinion would lie toward the
lesser X. A `Pair Tools` sketch on that plane projects the Center Point and holds the apex, dimensioned horizontally and
vertically from it, or aligned with it where Fusion refuses a dimension of zero. The gallery's U is the Target Plane's
normal turned round, so the driving shaft runs from the apex down into the plane, and V completes a right-handed pair
about the sketch's normal. `Runtime.build_frame` takes them, and every sketch maps the gallery's axes to Fusion's by dot
products ([PB-FRAME-PLANE]). The user asked for this placement on 2026-10-25 (`docs/gallery-translation-goal.md`,
"Bevel").

**[PB-NO-GRID-SNAP] Decad's grid snap of a section about a rotated shaft is not applied in Fusion.** The gallery snaps
the points of every section about a rotated shaft to a grid whose step depends on the section's reach, so that decad's
revolve sees exact floats. In Fusion, the pinion faces that should lie on one cone (the toe dish of its blank, and the
faces its toe cone trims into every tooth) came out as 31 extra cone faces, one per tooth, and the teeth looked as if
they stood on the body with a crease at each end. The driving gear, built about an unrotated shaft and so not snapped,
merged them into one face (face counts 129 and 160, with the same volume to 0.009 percent). Fusion's kernel is exact,
so `decadfusion.shaft_snap` returns the section as it is. The rounding of the shaft direction to 26 bits stays
(`_direction_bits` in the bevel module), so every feature about one shaft uses the same value.

**[PB-READING-BOUND] Fusion reports a position as a plain number, with no error bound.** Decad reads a faceted body and
gives each reading a `Bound` that covers its error, and the gallery widens its comparisons by that bound.
`decadfusion.Position` and `Centroid` return a bound of 0, so such a comparison uses Fusion's value as it is.

## Components

**[PB-OCCURRENCE-TREE]** Create an occurrence with `parent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` and
set `occurrence.component.name`. `base.Generator.getOccurrence` makes the gear's component this way. Each gear builds in
that one component and makes its sub-components last ([PB-CONTAINER-BODIES]); the bevel no longer has the separate
`Design` component of the previous add-in.

**[PB-NEVER-ACTIVATE] Never call `occurrence.activate()`.** Build in a component by calling its own collections
(`comp.sketches.add(...)`, `comp.features.*`) without activating it. When a sub-occurrence was activated and a sketch
was then put on a plane the user selected in another component, Fusion resolved that plane in the activated
component's local frame and the build lay flat on world XY, whatever plane the user picked. `Runtime.activate()` in
the runtime only marks the Python object as the active runtime; it does not activate a Fusion occurrence.

**[PB-NO-CROSS-SIBLING] Fusion rejects sketch, `project` and `Path.create` references to a sibling component.** Entities
wrapped with `createForAssemblyContext` did not help. Run every feature in one component, then move finished bodies
with `body.moveToComponent(targetOccurrence)`, which keeps their world position and needs no activation
([PB-CONTAINER-BODIES]).

**[PB-TREE-CLEANUP]** Cleanup walks the tree: `solids.hide_construction_geometry` turns off `isLightBulbOn` on every
sketch, construction plane and construction axis in the component and its children, deduplicated by `entityToken`.
On an error, `deleteComponent()` deletes the gear's user parameters and its top occurrence.

**[PB-USE-SELECTED-PLANE] Use a selected plane or face directly; do not rebuild it inside an activated
sub-component.** A coplanar construction plane (`setByOffset(target, 0)`) created in an activated sub-occurrence, from
a face in another component, resolved in that sub-component's frame and lost the selected plane's orientation. No gear
activates anything ([PB-NEVER-ACTIVATE]). The spur, bevel and cycloidal generators turn a selected planar face into a
construction plane with `setByOffset(target, 0)` in the component they build in, and that keeps its orientation.

**[PB-CONTAINER-BODIES] A gallery container is a Fusion component made last.** The gallery nests containers and puts
sketches, planes and bodies in them. A feature on a body inside a component runs in that component's context, and its
inputs would have to be proxies. So the build runs in one component, `Container` only records which container a body
belongs to, and `Runtime.finish_containers` makes a component for each container that holds a body and moves the bodies
in. Sketches and planes stay where they are, so they keep following the anchor. A body that a boolean consumes leaves
its container. The cycloidal drive's sub-components follow the same rule (`_buildSubComponents`).

## Diagnosing a Fusion run

**[PB-STALE-MODULE] Fusion keeps imported modules across an add-in Stop and Run.** A deploy that changes
`decadfusion.py` and not the generated gear file can leave the old runtime running, and a traceback then names line
numbers and messages of code that is gone. The error box's Build line shows the runtime's hash and whether it matches
the file on disk, and `GearCommand.command_execute` reloads the runtime when the hash differs. The generator module is
not reloaded: the Build line compares the line numbers of a few of its methods (`generate` among them) with the
file, and each generated module logs `<gear> build <GEAR_BUILD_ID>` when Fusion imports it, which `gearport id <gear>`
matches. Read those lines before trusting a report.

**[PB-PROGRESS-FILE] A build that hangs leaves no trace in the Text Commands window.** The window does not repaint while
the add-in blocks Fusion's thread. `decadfusion.progress` appends a flushed line to `geargen-progress.log` in the
system temp directory for each timed step, so the last line names the step that never returned.
