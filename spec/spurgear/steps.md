# Spur Gear — compiled step list

This step list is proven by `proof/spurgear/geometry_test.go`, `proof/spurgear/sketches_test.go` and `proof/spurgear/solids_test.go`.

## Provenance

| Input | `git hash-object` |
|---|---|
| `spec/spurgear/instructions.md` | `f5ffe3451454bb3b187b1318e47b92281d9f0bb0` |
| `spec/spurgear/fusion.md` | `ea678245854cfec80055d67c46a8788772b0f9d4` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `29c38dc687f7d6b88f7c36ff3275482b2eff051f` |
| `spec/helicalgear/fusion.md` | `83fac920272341e3c4584f16031478a69b7472e7` |

Steps 1–5 are the module surface — classes, dialog, parameters, orchestration — which
Fusion's timeline never shows but the emitted module cannot exist without. Steps 6–20 are
the build itself, one timeline entry (or control-flow gate) each. Lengths in Fusion's
internal units are cm; the proof models the same geometry in mm, which scales lengths and
changes no count, ratio or angle.

## 1 `[PROSE]` Module surface and exported constants

`lib/geargen/spurgear.py` imports explicitly — no `import *`: `math`, `adsk.core, adsk.fusion`,
`from ...lib import fusion360utils as futil`, `from .misc import to_cm, get_design`,
`from .base import Generator, GenerationContext, get_value, get_boolean, get_selection`,
`from .utilities import get_normal, find_profile_by_curve_counts, find_circle_by_radius`.
Trim to what is used; spur needs no `solids` and no `spurproxy` import (bevel imports spur's
tooth generator, not the reverse).

Module-level constants are public API — dependents import them by name, and both the Python
identifiers and their string values are pinned:

- Input ids: `INPUT_ID_PARENT = 'parentComponent'`, `INPUT_ID_PLANE = 'plane'`,
  `INPUT_ID_ANCHOR_POINT = 'anchorPoint'`, `INPUT_ID_MODULE = 'module'`,
  `INPUT_ID_TOOTH_NUMBER = 'toothNumber'`, `INPUT_ID_PRESSURE_ANGLE = 'pressureAngle'`,
  `INPUT_ID_BORE_DIAMETER = 'boreDiameter'`, `INPUT_ID_THICKNESS = 'thickness'`,
  `INPUT_ID_CHAMFER_TOOTH = 'chamferTooth'`, `INPUT_ID_SKETCH_ONLY = 'sketchOnly'`.
- Parameter names: `PARAM_MODULE = 'Module'`, `PARAM_TOOTH_NUMBER = 'ToothNumber'`,
  `PARAM_PRESSURE_ANGLE = 'PressureAngle'`, `PARAM_BORE_DIAMETER = 'BoreDiameter'`,
  `PARAM_THICKNESS = 'Thickness'`, `PARAM_CHAMFER_TOOTH = 'ChamferTooth'`,
  `PARAM_SKETCH_ONLY = 'SketchOnly'`, `PARAM_PITCH_DIAMETER = 'PitchCircleDiameter'`,
  `PARAM_PITCH_RADIUS = 'PitchCircleRadius'`, `PARAM_BASE_DIAMETER = 'BaseCircleDiameter'`,
  `PARAM_BASE_RADIUS = 'BaseCircleRadius'`, `PARAM_ROOT_DIAMETER = 'RootCircleDiameter'`,
  `PARAM_ROOT_RADIUS = 'RootCircleRadius'`, `PARAM_TIP_DIAMETER = 'TipCircleDiameter'`,
  `PARAM_TIP_RADIUS = 'TipCircleRadius'`, `PARAM_INVOLUTE_STEPS = 'InvoluteSteps'`,
  `PARAM_TOOTH_SPACE_ANGLE = 'ToothSpaceAngleAtRoot'`,
  `PARAM_TOOTH_SPACE_ARC = 'ToothSpaceArcAtRoot'`,
  `PARAM_FILLET_CLEARANCE = 'FilletClearance'`, `PARAM_FILLET_RADIUS = 'FilletRadius'`.

The module defines exactly the four-class pattern, names public API:

1. `SpurGearCommandInputsConfigurator` — plain class, `@classmethod` `configure` (step 2).
2. `SpurGearGenerationContext(GenerationContext)` — data carrier; `__init__` declares, each
   `cast(None)`-initialised (`toothProfileIsEmbedded` starts `False`): `ctx.plane`,
   `ctx.anchorPoint`, `ctx.extrusionEndPlane`, `ctx.gearProfileSketch`, `ctx.toothBody`,
   `ctx.gearBody`, `ctx.centerAxis`, `ctx.extrusionExtent`, `ctx.toothProfileIsEmbedded`.
3. `SpurGearInvoluteToothDesignGenerator` — plain class, constructed
   `(sketch, parent, angle=0)` (step 5).
4. `SpurGearGenerator(Generator)` — the orchestrator (steps 3–4).

Dependents bind to this surface by name: `helicalgear.py` imports all four classes plus
`PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS` (herringbone subclasses helical's);
`bevelgear.py` constructs the tooth generator with a `VirtualSpurProxy` parent. Renaming any
of it is a breaking change.

**From:** `spec/spurgear/instructions.md` L13–33 L152–178 L253–268 L381–407,
`.claude/skills/generate-gear/PLAYBOOK.md` L17–40 L42–73

## 2 `[PROSE]` Command dialog inputs

`SpurGearCommandInputsConfigurator.configure(cls, cmd)` adds the inputs to
`cmd.commandInputs` in exactly this display order — never regrouped by input type. Target
Plane and Anchor Point are the first two so plane selection owns the dialog's initial focus
([PB-AUTOFOCUS-FIRST]), and Parent Component is last. `configure` is a name this module
defines, not one it calls: the shared command entry invokes it when the dialog opens, and
subclass configurators reach it through `super().configure(cmd)` from their own modules.
<!-- check-step-calls: ignore configure -->

1. Target Plane: `addSelectionInput(INPUT_ID_PLANE, ...)`, filters
   `addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)` and
   `addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)`,
   `setSelectionLimits(1, 1)`.
2. Anchor Point: `addSelectionInput(INPUT_ID_ANCHOR_POINT, ...)`, filters
   `ConstructionPoints` + `SketchPoints`, `setSelectionLimits(1, 1)`.
3. Module: `addValueInput(INPUT_ID_MODULE, ..., '', adsk.core.ValueInput.createByReal(1))`
   — unitless, default 1.
4. Tooth Number: `addValueInput` unitless, `createByReal(17)`.
5. Pressure Angle: `addValueInput(..., 'deg', ValueInput.createByReal(math.radians(20)))` —
   display unit `'deg'`, default in **radians**, per [PB-DIALOG-DEFAULT-UNITS]: a
   `createByReal` default is in internal units whatever the display unit says.
6. Bore Diameter: `addStringValueInput(INPUT_ID_BORE_DIAMETER, ..., '0 mm')` — a string
   input so it accepts expressions.
7. Thickness: `addValueInput(..., 'mm', ValueInput.createByReal(to_cm(10)))` — display
   `'mm'`, default in cm.
8. Apply chamfer to teeth: `addValueInput(..., 'mm', createByReal(0))`.
9. Generate sketches, but do not build body: `addBoolValueInput(INPUT_ID_SKETCH_ONLY, ...)`
   checkbox, default false.
10. Parent Component — **last**: `addSelectionInput(INPUT_ID_PARENT, ...)`, filters
    `Occurrences` + `RootComponents`, `setSelectionLimits(1, 1)`, pre-selecting
    `get_design().rootComponent`.

Filters are the enum attributes of `adsk.core.SelectionCommandInput`, per
[PB-SELECTION-FILTER-ENUM]. The display order is not the `processInputs` read order (step
3) — selections are read first there, but they are not placed last here.

This order is the subclass seam [SPUR-SUBCLASS-INPUT]: a subclass configurator appends its
own inputs after `super().configure(cmd)`, so its extras land after Parent Component.

The spec pins the pre-selection outcome but not its mechanism; `addSelection(...)` is the
API for adding to a selection input, and Fusion documents it as invalid during
commandCreated, which is when `configure` runs — see the compile report. The name is a
mention, not a requirement.
<!-- check-step-calls: ignore addSelection -->

**From:** `spec/spurgear/instructions.md` L35–62 L90–150 L171–178,
`.claude/skills/generate-gear/PLAYBOOK.md` L53–60 L128–143 L346–348 L492–497

## 3 `[PROSE]` Read the dialog and register parameters

`processInputs(inputs)` runs before any occurrence exists, and its read order dodges the
context shift: as soon as anything creates the child occurrence (directly or through the
first `addParameter(...)` / `parameterName(...)`), selection inputs holding entities in
another component can drop. So:

1. Read the three selections first, stashing entities on `self`:
   `get_selection(inputs, INPUT_ID_PARENT)` (resolve `Occurrence.component` vs `Component`
   into `self.parentComponent`), `get_selection(inputs, INPUT_ID_PLANE)` → `self.plane`,
   `get_selection(inputs, INPUT_ID_ANCHOR_POINT)` → `self.anchorPoint`.
2. Read each remaining input with the helper matching its declared type
   ([PB-INPUT-READ]): `get_value(inputs, INPUT_ID_MODULE, '')` and likewise for
   toothNumber (`''`), pressureAngle (`'rad'`), boreDiameter (`'mm'`), thickness (`'mm'`),
   chamferTooth (`'mm'`); the checkbox with `get_boolean(inputs, INPUT_ID_SKETCH_ONLY)` —
   never `get_value` on a bool input (`BoolValueCommandInput` has no `expression`).
   `get_value` returns a ready `ValueInput` and raises on a bad expression
   ([PB-GET-VALUE-CONTRACT]).
3. Register the input-sourced parameters via `addParameter(...)` with units: `Module`
   `''` (unitless — NOT `'mm'` — so `generateName` renders `M=1` and the derived mm
   expressions accept it), `ToothNumber` `''`, `PressureAngle` `'rad'`, `BoreDiameter`
   `'mm'`, `Thickness` `'mm'`, `ChamferTooth` `'mm'`, and `SketchOnly` as a real 1/0
   (the framework reads booleans back with `getParameterAsBoolean(PARAM_SKETCH_ONLY)`).
4. Call the hook `addExtraPrimaryParameters(inputs)` — a **no-op** on the spur base
   [SPUR-EXTRA-PARAMS], but it must exist: subclasses register their own primary
   parameters there, before any derived expression references them.
5. Register the derived parameters (`registerDerivedParameters`) as live expressions via
   `ValueInput.createByString(...)`, in dependency order, `mm` unless noted:
   - `PitchCircleDiameter = Module * ToothNumber`; `PitchCircleRadius = PitchCircleDiameter / 2`
   - `BaseCircleDiameter = PitchCircleDiameter * cos(PressureAngle)`; `BaseCircleRadius = BaseCircleDiameter / 2`
   - `RootCircleDiameter = PitchCircleDiameter - 2.5 * Module` (dedendum 1.25·Module);
     `RootCircleRadius = RootCircleDiameter / 2`
   - `TipCircleDiameter = PitchCircleDiameter + 2 * Module` (addendum 1.0·Module);
     `TipCircleRadius = TipCircleDiameter / 2`
   - `InvoluteSteps` = 15, unitless.
   - `ToothSpaceAngleAtRoot` — **pre-computed in Python** with
     `ValueInput.createByReal(...)`, value `pi / ToothNumber - 2 * (math.tan(pa) - pa)`
     where `pa` is the pressure angle in radians; Fusion's expression engine refuses to mix
     unitless `tan()` output with a radian value. Register it **unitless `''`, not
     `'rad'`** — the next parameter multiplies it by a length, and a `'rad'` factor makes
     that product `mm·rad`, which Fusion rejects with `RuntimeError: Invalid expression`.
   - `ToothSpaceArcAtRoot = RootCircleRadius * ToothSpaceAngleAtRoot`, `mm`, live.
   - `FilletClearance` = 0.9, unitless.
   - `FilletRadius = (ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>` where
     `<factor>` is the string returned by `filletHelixFactorExpression()` — `'1'` on the
     spur base; this splice is that hook's only consumption point.

All parameter names take the `parameterName(...)` prefix `f'{prefix}_{name}'` from
`base.Generator`. Dimensions and feature inputs snapshot these numerically at build time
([PB-NUMERIC-SNAPSHOT] / [SPUR-F-SNAPSHOT]): editing a `<prefix>_…` parameter later does
not move an existing gear.

**From:** `spec/spurgear/instructions.md` L35–88 L120–150 L324–331 L408–417,
`.claude/skills/generate-gear/PLAYBOOK.md` L99–127 L196–228 L220–228,
`spec/spurgear/fusion.md` L214–221

## 4 `[PROSE]` Generate orchestration and method contract

`generate` and `prefixBase` are names this module defines, not calls it makes: the shared
command entry invokes the generator's `generate(inputs)` on the module's behalf, and
`base.Generator.getOccurrence` reads `prefixBase()` when it builds the parameter prefix.
<!-- check-step-calls: ignore generate prefixBase -->

`generate(inputs)` runs, in order: `processInputs(inputs)`; name the component —
`component.name` from `generateName()`, which for spur returns
`'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression, thickness.expression)`
— the parameters' `.expression` strings, not `.value`, so units show through; normalize
`self.plane` (step 6); `ctx = self.newContext()` (a `SpurGearGenerationContext`);
`prepareTools(ctx)` (steps 7–8); `buildMainGearBody(ctx)`; `buildBore(ctx)` (steps 18–19);
`cleanup(ctx)` (step 20) — **unconditionally, as the very last action of `generate`, after
`buildBore`**, because `buildBore` re-projects `ctx.anchorPoint` from the Tools sketch and
projection fails once that sketch is hidden.

`buildMainGearBody(ctx)` runs `buildSketches(ctx)` (step 9); then, if SketchOnly, shows the
Gear Profile sketch and stops (step 10); else `buildTooth(ctx)` (steps 11–12),
`buildBody(ctx)` (steps 13–14), `patternTeeth(ctx)` (steps 15–16, which itself ends by
calling `createFillets(ctx)`, step 17).

These method boundaries are public API — helical/herringbone override at them and call
`super()` at specific points; merging or reordering them breaks the subclasses even if the
spur gear comes out identical. Specifically: `buildSketches` owns creating the Gear Profile
sketch and invoking the tooth generator; `buildTooth` owns `ctx.toothBody` and **must call
`self.chamferTooth(ctx)` as its last action** (so `buildMainGearBody` must not chamfer
separately); `chamferWantEdges()` returns `6` on the spur base and is read only by
`chamferTooth`; `filletHelixFactorExpression()` returns `'1'` and is consumed only in step
3's `FilletRadius` registration — `createFillets` reads the resulting parameter's numeric
`.value`, never the hook. `prefixBase()` returns `'SpurGear'`. `SpurGearGenerator.__init__`
pre-initialises `self._lastToothEmbedded = False`, `self.toolsSketch = None` and
`self.boreSketch = None`.

Use `futil.log(...)` for step progress; on failure the command entry's try/except calls
`deleteComponent()` — do not invent new silent failure paths ([PB-LOGGING]).

**From:** `spec/spurgear/instructions.md` L270–337,
`.claude/skills/generate-gear/PLAYBOOK.md` L75–101 L244–255 L257–280 L621–623,
`spec/spurgear/fusion.md` L193–198

## 5 `[PROSE]` Tooth generator reproduced surface

`SpurGearInvoluteToothDesignGenerator(sketch, parent, angle=0)`:

- The constructor stores `self.toothAngle = angle` (retained but **never used by
  `drawTooth`** — the live rotation always comes from `draw()`'s runtime argument; helical
  constructs with the default 0 and passes the helix angle to `draw`, so reading
  `self.toothAngle` would draw a flat tooth and kill the loft's twist). It also adds the
  movable local origin — the field is named exactly `self.anchorPoint`, a fresh
  `SketchPoint` at (0, 0, 0) via `sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`,
  never `sketch.originPoint` ([SPUR-F-LOCAL-ORIGIN]).
- `draw(anchorPoint, angle=0)` performs, in order: `drawCircles()`, `drawTooth(angle)`,
  the step-9 anchoring (projection + coincidence), and — very last, only when
  `angle != 0` — sets the confirming angular dimension's value to `angle`
  ([SPUR-F-ROTATE-CONFIRM]). The anchoring lives **inside `draw`**, not in
  `buildSketches`: helical calls `draw` directly on its twisted sketch and relies on that
  one call to anchor it.
- `drawBore(anchorPoint, diameter)` — step 18's circle; `calculateInvolutePoint(baseRadius, intersectionRadius)`
  — the flank math below; parameter accessors `getParameter(name)` and
  `getParameterValue(name)` must all exist under these names.
- `calculateInvolutePoint(baseRadius, intersectionRadius)` returns `None` when
  `intersectionRadius < baseRadius`, else the point at

  ```
  alpha = acos(baseRadius / intersectionRadius)
  t     = tan(alpha)          # the curve parameter is tan(alpha), NOT inv(alpha)
  x = baseRadius * (cos(t) + t * sin(t))
  y = baseRadius * (sin(t) - t * cos(t))
  ```

  Using `inv(alpha) = tan(alpha) - alpha` as the parameter is the classic mistake and
  mis-parameterises the flank.
- When a drawing step needs a circle from `drawCircles`, keep direct references or use
  `find_circle_by_radius(sketch, radius)` — never fall back to an arbitrary circle.
- Bevel borrowing constraint [PB-PRECOMPUTED-MODE]: inside `drawCircles`, `drawTooth` and
  `draw` (helpers included), read parameters ONLY from the key set `VirtualSpurProxy`
  serves — `Module`, `ToothNumber`, `PressureAngle`, the Pitch/Base/Root/Tip circle
  diameters and radii, `InvoluteSteps`. Any other key on those paths raises `KeyError` in
  the bevel build. `drawTooth` writes `self.parent._lastToothEmbedded` (step 9); bevel
  reads it back off the proxy — keep that write.

**From:** `spec/spurgear/instructions.md` L338–379 L393–407,
`.claude/skills/generate-gear/PLAYBOOK.md` L188–194 L762–773,
`spec/spurgear/fusion.md` L26–31

## 6 `[PROSE]` Normalize the target plane

If the stashed Target Plane is not already a `ConstructionPlane` (the user picked a planar
face), create a coplanar construction plane: `constructionPlanes.createInput()`, then
`setByOffset(selectedPlane, adsk.core.ValueInput.createByReal(0))` — the offset is a
`ValueInput`, never a bare number, which is a runtime `TypeError`
([PB-CONSTRUCTION-PLANES]) — then `constructionPlanes.add(planeInput)`. Replace
`self.plane` (and `ctx.plane`) with the result so profile detection never sees the picked
face's native profile. Keep `self.plane` readable by subclasses.

No harness models construction planes, so this step is prose; the proof records that
limit next to the Tools sketch it feeds (see `proof/spurgear/sketches_test.go`).

**From:** `spec/spurgear/instructions.md` L39 L421–423,
`.claude/skills/generate-gear/PLAYBOOK.md` L692–703

## 7 `[GO]` Tools sketch

Proof: stepToolsSketch.

`prepareTools(ctx)` creates a sketch named `Tools` on the target plane via
`createSketchObject('Tools', plane=self.plane)`, projects the user's Anchor Point into it
with `sketch.project(self.anchorPoint)`, and keeps the projected `SketchPoint` as
`ctx.anchorPoint`. This projection is the canonical handle of the anchor chain
([SPUR-F-ANCHOR-CHAIN]): later sketches re-project *this* point, so the whole gear tracks
the user's anchor. The sketch draws nothing else; leave it visible until cleanup (step 20)
— hiding it earlier breaks the bore's re-projection. Store the sketch on
`self.toolsSketch`.

**From:** `spec/spurgear/instructions.md` L41 L228–231 L425–427,
`spec/spurgear/fusion.md` L19–24

## 8 `[PROSE]` Extrusion end plane

Still in `prepareTools(ctx)`: create an offset construction plane named
`Extrusion End Plane` at distance `Thickness` from the target plane —
`constructionPlanes.createInput()`, `setByOffset(self.plane, thicknessValue)` with the
Thickness snapshot as a `ValueInput`, `constructionPlanes.add(planeInput)` — and store it
as `ctx.extrusionEndPlane`. Its only purpose is to be the to-entity target of the tooth
and body extrudes, so both end on the same face. Leave it visible while the extrudes run;
step 20 hides it with `isLightBulbOn = False` (`isVisible` does not hide a construction
plane).

The proof has no construction planes; the plane's one job is modelled as the extrude
distance, and the far-cap assertions of stepExtrudeTooth and stepExtrudeBody check both
bodies end at Thickness.

**From:** `spec/spurgear/instructions.md` L428–429,
`.claude/skills/generate-gear/PLAYBOOK.md` L576–588 L692–703

## 9 `[GO]` Gear profile sketch

Proof: stepGearProfileSketch.

`buildSketches(ctx)` creates the sketch named `Gear Profile` on the target plane
(`createSketchObject('Gear Profile', plane=self.plane)`, stored as
`ctx.gearProfileSketch`, made visible while consumed), instantiates
`SpurGearInvoluteToothDesignGenerator(sketch, self)` and calls
`draw(ctx.anchorPoint, angle=0)`; afterwards it copies
`ctx.toothProfileIsEmbedded = self._lastToothEmbedded`. Everything below happens inside
the tooth generator, and the whole scheme is proven to fully constrain — DOF 0, no
redundant or conflicting constraint, no discrete ambiguity, valid profiles — across the
regime in the proof's case table: coarse and fine sizes, the signed angle range including
both quarter turns, the low involute-step count, and the embedded shape by both routes.

**Circles** (`drawCircles`), all centred by passing the local-origin `SketchPoint`
directly — `sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)` — so all
four share the one point; never pass `.geometry` and re-coincident
([PB-SHARE-XOR-COINCIDENT], [SPUR-F-SHARED-ADJACENCY]):

1. Root Circle — solid — radius `RootCircleRadius`.
2. Tip Circle — construction — `TipCircleRadius`.
3. Base Circle — construction — `BaseCircleRadius`.
4. Pitch Circle — construction — `PitchCircleRadius`.

Each gets a driving diameter dimension
`sketchDimensions.addDiameterDimension(circle, textPoint)` — driving is the default; never
pass `isDriven=True` ([PB-DRIVING-DIM]) — with an off-centre text point
([PB-RADIAL-DIM]). Each is labeled with along-path text ([PB-SKETCH-TEXT]):
`sketchTexts.createInput2(text, size)` with the label
`'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` where
`size = TipCircleRadius - RootCircleRadius` (all internal cm `.value`s), then
`setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`,
then `sketchTexts.add(textInput)`.

**Involute tooth** (`drawTooth(angle)` — rotating by the `angle` argument, never
`self.toothAngle`):

1. Sample the flank endpoint-inclusively: with `steps = InvoluteSteps`, sample `i` (for
   `i = 0 … steps−1`) sits at radius
   `r = BaseCircleRadius + (TipCircleRadius − BaseCircleRadius) · i / (steps − 1)`, so the
   first sample is exactly the base radius and the last exactly the tip radius. Do NOT
   clamp the start to the root circle — the embedded case is detected from where the flank
   start lands, not by trimming. Each sample is `calculateInvolutePoint(baseRadius, r)`;
   drop `None` returns (inside the base circle).
2. Mirror the samples across +X (negate y) before rotating: the raw parametric involute's
   angular position grows with radius, which as a left flank makes a tooth wider at the
   tip; the proof asserts the mirrored flank's angular offset strictly decreases.
3. Centre the tooth: with `(px, py) = calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`,
   `rotate_angle = pi / (2 · ToothNumber) − atan2(−py, px)` — the analytic pitch crossing
   of the **mirrored** flank (`atan2(−py, px)`, not `atan2(py, px)`), computed rather than
   interpolated so the tooth lands right at any sample count.
4. Rotate the mirrored samples by `rotate_angle` (left flank), mirror across X for the
   right flank, then rotate **both** flanks — and the tooth-top point and rib midpoint
   seeds below — by the requested `angle`. The tooth is drawn already at its final
   rotation; do not leave it at +X for the angular dimension to swing into place — that
   picks the wrong solver branch and ruins the helical loft ([SPUR-F-ROTATE-CONFIRM]).
5. Draw each flank as a fitted spline through its point collection:
   `adsk.core.ObjectCollection.create()`, add each `Point3D`, then
   `sketchCurves.sketchFittedSplines.add(pointCollection)`.
6. Tooth-top arc [SPUR-F-TOOTHTOP-ARC]: materialize the tooth-top `SketchPoint` at
   `(TipCircleRadius · cos(angle), TipCircleRadius · sin(angle))` via
   `sketchPoints.add(...)`, constrain it onto the tip circle with
   `geometricConstraints.addCoincident(toothTopPoint, tipCircle)`; then
   `sketchCurves.sketchArcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)`
   — the shared origin as centre and the flank splines' end `SketchPoint`s passed
   directly. Add **no diameter dimension**: a free centre plus a diameter leaves the
   inward bulge available, and the shared centre also makes the last rib's perpendicular
   redundant below.
7. Spine and reference [SPUR-F-SPINE]: draw the spine
   `sketchCurves.sketchLines.addByTwoPoints(localOrigin, toothTopPoint)` (construction;
   both points shared — no extra coincident, no end-on-arc constraint). Build the +X
   reference for **every** angle, 0 included: a far endpoint `SketchPoint` seeded at
   `(TipCircleRadius, 0)`, pinned by two axis dimensions from the local origin —
   `sketchDimensions.addDistanceDimension(localOrigin, refEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
   at value `TipCircleRadius` and the `VerticalDimensionOrientation` one at `0` — not by
   coincidence onto the tip circle, whose extreme-x touch is numerically unstable. Draw
   the reference line `addByTwoPoints(localOrigin, refEnd)`, construction. Then
   `sketchDimensions.addAngularDimension(referenceLine, spineLine, textPoint)` in that
   argument order, text on the bisector of the intended angle so Fusion selects it, not
   its supplement ([PB-ANGULAR-DIM]). Do not use a plain `addHorizontal(spineLine)` for
   the angle-0 case — horizontal has no direction, and the tooth can settle 180° around;
   the signed angular dimension is what forbids the mirror, and one path serves every
   angle.
   <!-- check-step-calls: ignore addHorizontal -->
8. Ribs [SPUR-F-RIBS], one per fit-point index, first and last included, built in exactly
   this order per rib — any other order over-constrains:
   1. `addByTwoPoints(leftSpline.fitPoints[i], rightSpline.fitPoints[i])`, sharing the fit
      points; mark construction.
   2. An **axis** dimension across the rib — `addDistanceDimension(leftFit, rightFit, orientation, textPoint)`
      with `VerticalDimensionOrientation` while `|cos(angle)| >= |sin(angle)|`, else
      horizontal — created with the points already seeded on their sides, value the
      measured magnitude. Fusion dimension values are magnitudes with the direction
      captured from the seed ([PB-DIM-VALUE-SEMANTICS]); an aligned dimension would let
      the flanks swap and mirror the tooth.
   3. A fresh midpoint `SketchPoint` seeded **on the spine** at the foot of the left fit
      point: `t = fitX·cos(angle) + fitY·sin(angle)`, seed `(t·cos(angle), t·sin(angle))`.
   4. `geometricConstraints.addCoincident(midPoint, spineLine)` first,
   5. then `geometricConstraints.addMidPoint(midPoint, ribLine)`,
   6. then `geometricConstraints.addPerpendicular(spineLine, ribLine)` — **skipped for the
      last rib**, whose perpendicular the tooth-top arc already implies; keeping it throws
      `VCS_SKETCH_OVER_CONSTRAINTS`.

   Chain the midpoints down the spine with an axis dimension along it (horizontal while
   `|cos| >= |sin|`, else vertical), each from the previous rib's midpoint — and the first
   from the **local origin** — to this rib's midpoint; without the origin-to-first
   dimension the chain slides along the spine as a unit.
9. Close the tooth at the root [SPUR-F-FLANK-ROOT]. `embedded = firstRadius < RootCircleRadius`
   — strict `<`, raw values, no tolerance, `firstRadius` the distance from the local
   origin to the flank's first drawn fit point; exact equality counts as non-embedded and
   draws a zero-length stub. Set `self.parent._lastToothEmbedded = embedded` (the tooth
   generator has no ctx). Non-embedded: per side, a radial stub
   `addByTwoPoints(rootEndPoint, flankStartFitPoint)` sharing the spline's start point,
   its root end seeded at its exact computed position on the root circle and pinned by
   **exactly two** axis dimensions from the local origin —
   `addDistanceDimension(localOrigin, rootEnd, HorizontalDimensionOrientation, textPoint)`
   and the vertical one — values only the unsigned `abs(dx)` / `abs(dy)` magnitudes: **a
   negative `parameter.value` flips the point to the origin's other side** (found
   in-Fusion 2026-08-24; [PB-DIM-VALUE-SEMANTICS]). Never place the root end with
   root-end-on-circle plus origin-on-line instead: both are satisfied on the far side of
   the gear too, and the stub becomes a chord across it. The tooth loop then has 6 curves.
   Embedded: draw no stub; the loop has 4 curves.

**Anchoring** (inside `draw`, after `drawTooth`): re-project the Tools-sketch anchor —
`sketch.project(ctx.anchorPoint)` — then
`geometricConstraints.addCoincident(self.anchorPoint, projectedAnchor)` between the local
origin and the fresh projection. Everything above hangs off the local origin, so this one
coincidence slides the whole tooth onto the user's anchor. Then, only when `angle != 0`,
set the spine angular dimension's value to `angle` — `spineDimension.parameter.value = angle`
— as the very last action, after the entire constraint network exists.

The sketch closes exactly two regions, and their curve counts are the contract steps 11
and 13 match on: the tooth section (6 curves, or 4 embedded) and the disc inside the root
circle, split out of the solid root circle by the tooth's contact points. The proof counts
both on the sketch it actually draws.

**From:** `spec/spurgear/instructions.md` L180–251 L431–485,
`spec/spurgear/fusion.md` L26–43 L45–60 L69–114 L116–156 L158–198,
`.claude/skills/generate-gear/PLAYBOOK.md` L230–242 L441–447 L484–491 L543–553 L565–573 L599–609

## 10 `[PROSE]` Sketch-only short circuit

If `getParameterAsBoolean(PARAM_SKETCH_ONLY)` is true, `buildMainGearBody` makes the Gear
Profile sketch visible (`isVisible = True`) and returns — no tooth, body, pattern, fillet
or bore. `buildBore` and `cleanup` still run from `generate` (steps 18 and 20 carry their
own guards). Control flow, not a timeline entry; the visibility split it feeds is asserted
nowhere the harnesses reach — see the cleanup note in `proof/spurgear/sketches_test.go`.

**From:** `spec/spurgear/instructions.md` L60 L487–489 L297–304

## 11 `[GO]` Extrude the tooth

Proof: stepExtrudeTooth.

`buildTooth(ctx)` finds the tooth section by its curve counts —
`find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`
— never a hand-rolled loop search; the helper raises when nothing matches
([PB-PROFILE-MATCH]). Extrude it from the target plane to the Extrusion End Plane as a new
body: `extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)` set with
`setOneSideExtent(extentDefinition, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
then `extrudeFeatures.add(extrudeInput)`. Name the feature `Extrude tooth`; store the body
as `ctx.toothBody`. `buildTooth` ends by calling `chamferTooth(ctx)` (step 12).

The proof extrudes the chorded tooth section (substitutions recorded in
`proof/spurgear/geometry_test.go`: decad refuses both the involute fit spline's wall and a
circle-fragment boundary) and asserts the volume is section-area × Thickness, the far cap
sits at Thickness, and the front face carries what step 12 selects on: 6 edges (4
embedded), exactly one of them the root-radius arc.

**From:** `spec/spurgear/instructions.md` L218–226 L491–495,
`.claude/skills/generate-gear/PLAYBOOK.md` L145–158 L589–598

## 12 `[GO]` Chamfer the tooth

Proof: stepChamferTooth.

`chamferTooth(ctx)`: if the ChamferTooth snapshot is 0, return. Otherwise find the tooth's
front face with a single conjunction predicate over `ctx.toothBody.faces`: the first face
where **both** `face.edges.count == chamferWantEdges()` (spur: 6) **and** the face is
coplanar with the sketch plane — `sketchPlane.isCoPlanarTo(face.geometry)` with
`sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`. Raise if no face satisfies
both; never fall back to a partial match. (Known, accepted: an embedded profile's front
face has 4 edges while `chamferWantEdges()` stays 6, so chamfering an embedded spur tooth
raises — users disable chamfer there. Helical overrides only the count, to its own
flagged value — see `spec/helicalgear/fusion.md` [HELI-F-CHAMFER-COUNT]; do not
second-guess it here.)

Collect every front-face edge **except the root arc**, identified by radius, not size:
skip an edge whose `curveType` is `adsk.core.Curve3DTypes.Arc3DCurveType` and whose
`edge.geometry.radius` equals the `RootCircleRadius` parameter's `.value` within 0.001 cm
— chamfering the root arc would eat the neighbouring tooth. Apply with
`chamferFeatures.createInput2()`, then
`chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, chamferValue, False)` on the
input's edge-set collection (the chamfer-side shape of [PB-FILLET-CHAMFER]), then
`chamferFeatures.add(chamferInput)`, with `edges` an `ObjectCollection.create()` of the
kept edges.

The proof chamfers a complete front-cap loop (the root disc's), the only cap chamfer the
harness evaluates exactly, and asserts the equal-distance band removes exactly the ring
volume; the selection inputs it cannot reach — 6 edges, one root-radius arc — are
asserted by stepExtrudeTooth, and the reasons are recorded on the step function.

**From:** `spec/spurgear/instructions.md` L497–503,
`spec/helicalgear/fusion.md` L67–84,
`.claude/skills/generate-gear/PLAYBOOK.md` L498–504 L589–598

## 13 `[GO]` Extrude the body

Proof: stepExtrudeBody.

`buildBody(ctx)` finds the gear body profile — the solid disc inside the root circle,
whose boundary is exactly the 2 arcs the tooth's contact points split the root circle
into: `find_profile_by_curve_counts(sketch, arcs=2)`. It is not an annulus; the tip circle
is construction geometry and bounds nothing. Extrude from the target plane to the
Extrusion End Plane as a new body — `extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`,
`setOneSideExtent(extentDefinition, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`extrudeFeatures.add(extrudeInput)`. Name the feature `Extrude body`, the body
`Gear Body`, and store `ctx.gearBody`.

The proof asserts the disc body: volume π·root²·Thickness, exactly two planar caps and
one root-radius cylinder, near cap coplanar with the sketch plane, far cap at Thickness.

**From:** `spec/spurgear/instructions.md` L218–226 L505–514

## 14 `[GO]` Gear center axis and extrusion extent

Proof: stepGearCenterAxis.

While iterating the new body's faces (`extrude.bodies.item(0).faces`), classify each by
`face.geometry.surfaceType` and capture two references; raise if either is missing
([PB-EMPTY-RESULT] — never let a failed search surface three calls later):

- From any face with `adsk.core.SurfaceTypes.CylinderSurfaceType`: build the `Gear Center`
  construction axis — `constructionAxes.createInput()`,
  `setByCircularFace(cylindricalFace)`, `constructionAxes.add(axisInput)` — name it
  `Gear Center`, set `isLightBulbOn = False`, store `ctx.centerAxis`.
- Among faces with `PlaneSurfaceType`, `ctx.extrusionExtent` is the one parallel to but
  not coplanar with the sketch plane — with
  `sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`, pick the face where
  `sketchPlane.isParallelToPlane(face.geometry)` and not
  `sketchPlane.isCoPlanarTo(face.geometry)`; the near cap is coplanar and the
  cylindrical/side faces are not planar, so the far cap is the only survivor.

The proof derives the axis the way `setByCircularFace` does — from the cylindrical face —
and asserts it is parallel to the plane normal through the gear centre, and that the
parallel-not-coplanar predicate selects exactly one face, at Thickness.

**From:** `spec/spurgear/instructions.md` L505–514,
`.claude/skills/generate-gear/PLAYBOOK.md` L430 L704–707

## 15 `[GO]` Pattern the teeth

Proof: stepPatternTeeth.

`patternTeeth(ctx)` circular-patterns `ctx.toothBody` about `ctx.centerAxis`: put the
tooth body in an `ObjectCollection.create()`, then
`circularPatternFeatures.createInput(bodies, ctx.centerAxis)`, pin all three inputs
explicitly ([PB-CIRCULAR-PATTERN]) — `quantity` from
`ValueInput.createByReal(toothNumber)`, `patternInput.totalAngle = ValueInput.createByString('360 deg')`,
`patternInput.isSymmetric = False` — then `circularPatternFeatures.add(patternInput)`.

The proof places sampled slots of the pattern (the adjacent pair plus spread slots — the
harness's pairwise disjointness proof goes undecided when the many near-tangent tooth
prisms coexist, recorded on the step function) and asserts each copy is the seed rotated
by exactly k·360°/N about the axis with volume unchanged; the full count is carried by
step 16's tiling equality.

**From:** `spec/spurgear/instructions.md` L516–520,
`.claude/skills/generate-gear/PLAYBOOK.md` L610–620

## 16 `[GO]` Combine the teeth into the body

Proof: stepCombineTeeth.

Feed the pattern's bodies to a single Combine-Join: copy `pattern.bodies.item(i)` into a
fresh `ObjectCollection.create()` — the collection already includes the original tooth
body; do not re-add it ([PB-PATTERN-BODIES]), and `combineFeatures.createInput(targetBody, toolBodies)`
rejects a raw `BRepBodies` — with `ctx.gearBody` as the target, set
`combineInput.operation` to `adsk.fusion.FeatureOperations.JoinFeatureOperation`, then
`combineFeatures.add(combineInput)`. After the join, `patternTeeth` calls
`createFillets(ctx)` (step 17).

The proof builds the combine's one result — the whole-gear prism from the single tiled
outline (decad's boolean refuses the tooth-on-root-cylinder tangent contact the real join
crosses; recorded on the step function) — and asserts the exact tiling equality: whole
gear = root disc + N teeth, one watertight lump.

**From:** `spec/spurgear/instructions.md` L516–521,
`.claude/skills/generate-gear/PLAYBOOK.md` L610–614

## 17 `[GO]` Root fillets

Proof: stepRootFillets.

`createFillets(ctx)`: if the `FilletRadius` parameter's numeric `.value` is not positive,
skip. Otherwise round the corner where each root valley floor meets a tooth flank — the
structurally loaded corner, not the cosmetic front/back rims:

- Collect **every** cylindrical face whose radius equals `RootCircleRadius` (the
  pattern-and-combine usually splits the root cylinder into one patch per valley).
- On each such face keep only the **axial** straight edges: filter to
  `adsk.core.Curve3DTypes.Line3DCurveType`, take each line's direction from its geometry
  endpoints — `geometry.startPoint.vectorTo(geometry.endPoint)`, then `normalize()` — and
  keep it when parallel to the target-plane normal (`get_normal(self.plane)`) within
  `abs(abs(dot(direction, axisNormal)) - 1.0) < 0.01`, computed via
  `direction.dotProduct(axisNormal)`. Use exactly this tolerance — a tighter one drops
  slightly-off tessellated edges and leaves fillets missing. The circular end-cap rim
  edges fail the direction test and are dropped. Do not read directions via
  `edge.evaluator.getTangent(0)` — parameter 0 need not lie in the edge's range and
  Fusion raises `RuntimeError: invalid argument parameter`.
  <!-- check-step-calls: ignore getTangent -->
- If the collection is empty, return silently — an empty edge set must not reach
  `filletFeatures.add`.
- Apply: `filletFeatures.createInput()`, then
  `filletInput.addConstantRadiusEdgeSet(edges, radiusValue, False)` on the input
  **itself** — there is no `filletInput.edgeSetInputs`; that is the chamfer-side shape
  ([PB-FILLET-CHAMFER]) — with `isTangentChain=False` so Fusion cannot chain past the
  intended corner, then `filletFeatures.add(filletInput)`.

The proof fillets the whole-gear prism's concave axial edges at the derived FilletRadius
and asserts the selection finds exactly two corners per valley (2N cylinder blends at that
radius), drops the rims, and adds material.

**From:** `spec/spurgear/instructions.md` L86–88 L522–531,
`.claude/skills/generate-gear/PLAYBOOK.md` L430 L498–504

## 18 `[GO]` Bore profile sketch

Proof: stepBoreProfileSketch.

`buildBore(ctx)` runs unconditionally from `generate` and must itself early-return in two
cases: SketchOnly (in that mode `ctx.gearBody` and `ctx.extrusionExtent` were never set —
do not rely on the bore being 0), and BoreDiameter ≤ 0.

Otherwise create a sketch named `Bore Profile` on the target plane
(`createSketchObject('Bore Profile', plane=self.plane)`, stored on `self.boreSketch`) and
draw the circle by instantiating the tooth generator on it —
`SpurGearInvoluteToothDesignGenerator(boreSketch, self)` — and calling
`drawBore(ctx.anchorPoint, boreDiameter)` (diameter in cm), which re-projects the anchor
via `sketch.project(ctx.anchorPoint)`, draws the solid circle centred on the projection by
sharing it — `addByCenterRadius(projectedAnchor, radius)` — and gives it a driving
`addDiameterDimension(circle, textPoint)`. The constructor's stray local-origin
`SketchPoint` at (0,0,0) is faithful behaviour — keep it, and ground it on that same
projection: `addCoincident(toothGen.anchorPoint, projectedAnchor)`. Never ground it on
`boreSketch.originPoint` — that pins the point to the plane rather than the gear, and
[PB-CIRCLE-CENTER] records a solver failure from constraining to the sketch origin.
Ungrounded, the point keeps two free DOF and the sketch never fully constrains — which is
exactly what the proof's gate checks.

**From:** `spec/spurgear/instructions.md` L54 L533–537,
`spec/spurgear/fusion.md` L26–31,
`.claude/skills/generate-gear/PLAYBOOK.md` L441–447

## 19 `[GO]` Bore cut

Proof: stepBoreCut.

Extrude-cut the bore profile from the target plane to `ctx.extrusionExtent` — the far
end-cap face captured in step 14 — so the bore pierces the whole body regardless of
Thickness: `extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
`ToEntityExtentDefinition.create(ctx.extrusionExtent, False)`,
`setOneSideExtent(extentDefinition, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
restrict with `participantBodies = [ctx.gearBody]`, then `extrudeFeatures.add(extrudeInput)`.

The proof cuts a bore cylinder through the whole-gear prism (the tool extruded past both
caps, since the harness's boolean refuses flush face-on-face contact; recorded on the
step function) and asserts the removed volume is exactly the full-thickness bore
cylinder.

**From:** `spec/spurgear/instructions.md` L533–537,
`.claude/skills/generate-gear/PLAYBOOK.md` L646–649

## 20 `[PROSE]` Cleanup

`cleanup(ctx)` — the last action of `generate`, both modes, never guarded at the call
site ([SPUR-F-CLEANUP]):

- Always (both modes): turn off the light bulb on every construction plane/axis this run
  created — `ctx.extrusionEndPlane`, `ctx.centerAxis`, and the normalized plane if step 6
  made one — via `isLightBulbOn = False`. `isVisible` has no effect on construction
  geometry ([PB-HIDE-AFTER-USE]).
- Full build only: hide the Tools, Gear Profile and Bore Profile sketches with
  `isVisible = False`. Sketch-only mode leaves Tools and Gear Profile visible — that is
  the mode's whole point.
- Guard each entity individually: the `Gear Center` axis and Bore Profile sketch do not
  exist in sketch-only mode, and the bore sketch does not exist when no bore was cut.

Visibility state is beyond both harnesses; the limit is recorded in
`proof/spurgear/sketches_test.go`.

**From:** `spec/spurgear/instructions.md` L297–304 L487–489,
`spec/spurgear/fusion.md` L200–212,
`.claude/skills/generate-gear/PLAYBOOK.md` L576–588
