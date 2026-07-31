# Spur gear — step list

Compiled from the sources below. Each step is one entry in the Fusion timeline,
except `S1` (the dialog-reading and parameter-registration pass) and `S6` and
`S16` (control flow and cleanup), which produce no timeline feature but are
load-bearing and have nowhere else to live.

`[GO]` marks a step the proof at `.tmp/spurgear_test.go` exercises, and names the
function that does it. Everything three-dimensional is `[PROSE]`: the proof
engine models 2D sketches only, so an extrude, a chamfer, a pattern, a combine, a
fillet and a cut can be specified here but not executed. All three sketches in
this build are `[GO]`.

## Provenance

| Source | Blob hash |
|---|---|
| `spec/spurgear/instructions.md` | `591e9e05d2bd9386b651285aa7bfb045e61a7e61` |
| `spec/spurgear/fusion.md` | `5708cb98645e094a992e1cc5a79c9e24094ab0b8` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9297782f07cf4a68372a500eaa3b0e35a9d27091` |

## Defects found while compiling

These are marked inline at the step where they bite, with a ⚠️. Nothing below is
silently corrected.

1. `sketch.project(...)` — the name the spec uses for the projection chain — is
   not in the Fusion API index. `Sketch` carries `project2(entities, isLinked)`,
   `projectToSurface(...)` and `include(entity)`; there is no `project`.
2. `sketchTexts.createInput2(text, height)` is not in the index either. The
   index has `createInput3(expression: str, height: core.ValueInput)`, whose
   height is a `ValueInput` rather than the bare cm float the spec passes.
3. Step 11's fillet call is inverted relative to the index. The spec says to call
   `addConstantRadiusEdgeSet` on the `FilletFeatureInput` itself and that
   `filletInput.edgeSetInputs` does not exist; the index says the opposite.
4. Step 9 describes the body profile as "the annular loop bounded by exactly 2
   arcs (the root circle and the tip circle)". The tip circle is construction
   geometry and bounds no profile, so that loop does not exist.
5. Steps 9 and `[SPUR-F-FLANK-ROOT]` both attribute the embedded profile to "low
   tooth-count" combinations. It is a high-tooth-count case.
6. `addSelectionFilter`'s index signature takes a `str`, which contradicts
   `[PB-SELECTION-FILTER-ENUM]`.
7. `[SPUR-F-RIBS]` pins the signed-dimension axis only for `angle = 0` and
   otherwise says "whichever is better conditioned", which is not a rule a
   generator can follow.

---

## S1 `[PROSE]` Read the Dialog and Register Parameters

Not a timeline feature. It runs once, before every step below, and its ordering
constraint is what makes the rest of the build possible at all.

**Read the selections first.** Pull Parent Component, Target Plane and Anchor
Point out of `inputs` and stash them on `self` before anything touches the
design. The moment `parentComponent.occurrences.addNewComponent(...)` runs —
directly through `getOccurrence()`, or transitively through the first
`addParameter()` or `parameterName()` call — Fusion's active component context
shifts to the new occurrence, and a `SelectionCommandInput` holding an entity
that lives in a different component can drop its selection. Numeric and boolean
inputs are immune.

Read each input with the helper that matches how it was declared: `get_value`
for `addValueInput`/`addStringValueInput`, `get_boolean` for the checkbox,
`get_selection` for the three selections. Calling `get_value` on the checkbox
raises `AttributeError: 'BoolValueCommandInput' object has no attribute
'expression'`.

**Dialog display order is fixed** and is not the read order: Target Plane,
Anchor Point, Module, Tooth Number, Pressure Angle, Bore Diameter, Thickness,
Apply chamfer to teeth, Generate sketches but do not build body, and Parent
Component **last**. Input ids and registered parameter names are reproduced
surface: `plane`, `anchorPoint`, `module`, `toothNumber`, `pressureAngle`,
`boreDiameter`, `thickness`, `chamferTooth`, `sketchOnly`, `parentComponent`,
each exported as a module-level constant.

Selection filters and limits, per input: Target Plane takes
`ConstructionPlanes` + `PlanarFaces`; Anchor Point takes `ConstructionPoints` +
`SketchPoints`; Parent Component takes `Occurrences` + `RootComponents` and
pre-selects `get_design().rootComponent`. All three are
`setSelectionLimits(1, 1)`.

Dialog defaults go in Fusion's internal units whatever the display unit says:
Pressure Angle is `addValueInput(…, 'deg', ValueInput.createByReal(math.radians(20)))`
and Thickness is `addValueInput(…, 'mm', ValueInput.createByReal(to_cm(10)))`.
Bore Diameter is the one `addStringValueInput`, defaulting to `'0 mm'`.

Register the input-sourced parameters, then call the `addExtraPrimaryParameters`
hook (a no-op on the spur base, the seam where helical registers its Helix
Angle), then the derived ones. `Module` is registered **unitless**, not `'mm'`,
so `generateName` renders `M=1` and the dependent `mm` expressions read it as a
bare factor. `ToothSpaceAngleAtRoot` is pre-computed in Python as
`π / ToothNumber − 2 · (tan(PressureAngle) − PressureAngle)` and registered
unitless, because Fusion's expression engine will not subtract a radian value
from the unitless output of `tan()`, and because the next parameter multiplies it
by a length — registering it as `'rad'` makes that product `mm·rad` and Fusion
rejects it with `RuntimeError: Invalid expression`.

`FilletRadius` is the one place `filletHelixFactorExpression()` is consumed: its
return (`'1'` for spur) is spliced in as the last factor of the live expression
`(ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>`. `createFillets` never
reads the hook, only the resulting parameter's numeric `.value`.

Name the component `'Spur Gear (M={}, Tooth={}, Thickness={})'` from the three
parameters' `.expression` strings, not their `.value`.

⚠️ Defect 6: `[PB-SELECTION-FILTER-ENUM]` says a filter must be the enum constant
`adsk.core.SelectionCommandInput.ConstructionPlanes` and that a string raises
`TypeError`, but the API index gives `addSelectionFilter(filter: str) -> bool`.
Following the playbook, since it records observed Fusion behaviour.

Calls: `inputs.addSelectionInput(id, name, commandPrompt)`,
`selectionInput.addSelectionFilter(filter)`,
`selectionInput.setSelectionLimits(minimum, maximum)`,
`inputs.addValueInput(id, name, unitType, initialValue)`,
`inputs.addStringValueInput(id, name, initialValue)`,
`inputs.addBoolValueInput(id, name, isCheckBox, resourceFolder, initialValue)`,
`adsk.core.ValueInput.createByReal(realValue)`,
`adsk.core.ValueInput.createByString(stringValue)`,
`design.userParameters.add(name, value, units, comment)`,
`parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`.

**From:** `spec/spurgear/instructions.md` 36–178 (Variables, exact ids and
order), 372–381 (Generation Order), 234–300 (Method contract);
`spec/spurgear/fusion.md` 201–206 (`[SPUR-F-SNAPSHOT]`);
`.claude/skills/generate-gear/PLAYBOOK.md` 103–143 (`[PB-INPUT-READ]`,
`[PB-GET-VALUE-CONTRACT]`, `[PB-DIALOG-DEFAULT-UNITS]`, `[PB-SELECTION-DECL]`),
196–228 (`processInputs` pattern, `[PB-NUMERIC-SNAPSHOT]`), 474–479
(`[PB-SELECTION-FILTER-ENUM]`), 556–557 (`[PB-SELECTION-STASH]`).

## S2 `[PROSE]` Normalize the Target Plane

Only when the user picked something that is not already a `ConstructionPlane` —
a planar face, say. Build a coplanar construction plane at zero offset from the
selection and use that everywhere below, so profile detection is not confused by
the selected face's own profile. Keep a handle: S16 switches its light bulb off.

The offset argument is a `ValueInput`, not a number. `setByOffset(plane, 0)` is a
runtime `TypeError`.

Calls: `component.constructionPlanes.createInput()`,
`planeInput.setByOffset(planarEntity, adsk.core.ValueInput.createByReal(0))`,
`component.constructionPlanes.add(planeInput)`.

**From:** `spec/spurgear/instructions.md` 385–387 (step 1), 39 (Target Plane);
`.claude/skills/generate-gear/PLAYBOOK.md` 674–685 (`[PB-CONSTRUCTION-PLANES]`).

## S3 `[GO]` Tools Sketch — `stepToolsSketch`

One sketch named `Tools` on the target plane. It draws no geometry of its own.
Its whole purpose is to own one projection of the user's Anchor Point, kept as
`ctx.anchorPoint`; that projection is the canonical handle, and every later
sketch projects *it* in again rather than the user's original entity, so the
whole gear follows the anchor if it moves.

Leave the sketch visible. S15's bore re-projects from it, and projection has
failed on invisible sketches in this repo's history, so it is hidden only in
S16 — and only on the full-build path.

⚠️ Defect 1: the spec calls `sketch.project(...)`, which the API index does not
carry. The nearest existing member is
`sketch.project2(entities: list[core.Base], isLinked: bool) -> list[SketchEntity]`,
which takes a list and returns a list rather than a single entity. Named as the
spec has it, not corrected.

The proof models the projection as reference geometry — `CreateReferencePoint`,
this engine's externally-locked import. That is stronger than Fusion, where
`[PB-PROJECT-NOT-FIXED]` records that a projected point still carries free
degrees of freedom until something constrains it. The gap is covered in S5,
where the projection is exactly what the local origin is constrained to.

Calls: `component.sketches.add(planarEntity)`, `sketch.name`,
`sketch.project(anchorEntity)` ⚠️, `sketch.isVisible`.

**From:** `spec/spurgear/instructions.md` 389–391 (step 2), 41 (Anchor Point),
193–196 (Sketch Discipline), 222–223 (`ctx.anchorPoint`);
`spec/spurgear/fusion.md` 19–24 (`[SPUR-F-ANCHOR-CHAIN]`);
`.claude/skills/generate-gear/PLAYBOOK.md` 430–444 (`[PB-PROJECT-NOT-FIXED]`),
558–570 (`[PB-HIDE-AFTER-USE]`).

## S4 `[PROSE]` Extrusion End Plane

A construction plane offset from the target plane by `Thickness`, kept as
`ctx.extrusionEndPlane`. Its only job is to be the to-entity target for both the
tooth extrude (S7) and the body extrude (S9), so the two end on the same
well-defined face. Leave it visible while those run; S16 turns its light bulb
off. `isVisible = False` does not hide a construction plane.

The offset is a `ValueInput` here too.

Calls: `component.constructionPlanes.createInput()`,
`planeInput.setByOffset(targetPlane, adsk.core.ValueInput.createByReal(thickness))`,
`component.constructionPlanes.add(planeInput)`, `constructionPlane.name`,
`constructionPlane.isLightBulbOn`.

**From:** `spec/spurgear/instructions.md` 393 (step 2), 387 (step 1's
`ValueInput` note), 224 (`ctx.extrusionEndPlane`), 203–205 (Sketch Discipline);
`.claude/skills/generate-gear/PLAYBOOK.md` 558–570 (`[PB-HIDE-AFTER-USE]`),
674–685 (`[PB-CONSTRUCTION-PLANES]`).

## S5 `[GO]` Gear Profile Sketch — `stepGearProfileSketch`

One sketch named `Gear Profile` on the target plane, and one timeline entry
however much goes into it. The spec splits the contents across its steps 3, 4 and
5; they are all one sketch, and `draw()` runs them in one call, so they are one
step here. The order inside it is load-bearing throughout.

`buildSketches` owns creating this sketch and invoking the tooth generator, and
nothing else — helical overrides it, calls `super()`, and then draws a second
twisted profile sketch by constructing the same generator on it. The anchoring at
the end lives inside `draw()`, not in `buildSketches`, for the same reason: the
twisted sketch would otherwise be left unconstrained and the loft would float
off the anchor.

**The local origin.** The generator's constructor adds a fresh `SketchPoint` at
(0, 0, 0), the field `self.anchorPoint`. Not `sketch.originPoint`, which is
immutable and cannot be made coincident with something projected in from
elsewhere. Everything below is drawn relative to this point, which is what makes
the single anchoring constraint at the end move the whole drawing as a unit.

**The four circles** (`drawCircles`), in this order: Root Circle **solid**, then
Tip, Base and Pitch as **construction**. Each takes the local origin *directly*
as its centre — `addByCenterRadius(localOrigin, radius)`, never
`localOrigin.geometry` followed by a coincident, which stacks a redundant
self-coincident and kills the solver with `VCS_SKETCH_SOLVING_FAILED`. Each gets
a driving diameter dimension; never pass `isDriven=True`. Each is labelled with
along-path text reading `'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)`
from the radii's internal cm `.value`, with `size = TipCircleRadius −
RootCircleRadius` passed as the text height.

⚠️ Defect 2: the spec and `[PB-SKETCH-TEXT]` both call
`sketchTexts.createInput2(text, height)` with a bare cm float. The API index
carries only `createInput3(expression: str, height: core.ValueInput)`. Named as
the spec has it.

**The involute flanks** (`drawTooth`). Sample `InvoluteSteps` points,
endpoint-inclusive: sample `i` sits at radius `BaseCircleRadius + (TipCircleRadius
− BaseCircleRadius) · i / (steps − 1)`, so the first is exactly on the base circle
and the last exactly on the tip. Do not clamp the start to
`max(BaseCircleRadius, RootCircleRadius)`; the flank is sampled from the base
circle even when the base circle sits inside the root. Each sample is
`calculateInvolutePoint(BaseCircleRadius, r)`, whose curve parameter is
`tan(alpha)` and **not** `inv(alpha) = tan(alpha) − alpha`; substituting the
involute function is the common mistake and gives a mis-parameterised flank. Drop
any sample that returns `None`.

Then three transforms, in this order, all in Python point math:

1. **Mirror across +X** (negate y). The standard parametric involute spirals so
   its angular position grows with radius, which as a left flank gives a tooth
   wider at the tip than at the root.
2. **Rotate to centre the tooth on +X.** With
   `(px, py) = calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`, the
   mirrored pitch crossing sits at `atan2(−py, px)`, so
   `rotate_angle = π / (2 · ToothNumber) − atan2(−py, px)`. Computed
   analytically, not interpolated between samples, so few samples still place the
   tooth exactly. That gives the **left** flank; mirroring it across X gives the
   right.
3. **Apply the requested `angle`** to both flanks, the tooth-top point and the
   rib-midpoint seeds. `drawTooth` must rotate by the `angle` argument that flows
   in from `draw()` at call time, **not** by the constructor-stored
   `self.toothAngle`. Helical and herringbone construct the generator with the
   default `angle=0` and then call `draw(anchorPoint, angle=helixAngle)`; reading
   the stored field would draw a flat tooth and the loft would have no twist.

Draw each flank as a fitted spline through its point collection.

**The tooth-top arc.** Materialize a tooth-top point at
`(TipCircleRadius·cos(angle), TipCircleRadius·sin(angle))`, coincident to the tip
circle. Then `addByCenterStartEnd(localOrigin, rightFlankEnd, leftFlankEnd)`,
passing the local origin and both flank splines' end `SketchPoint`s directly so
the arc shares all three. Add **no** diameter dimension. A free centre plus a
diameter fixes the arc's size but not which way it curves: the same radius
through the same two ends can bulge inward through the tooth, and the sketch then
reaches DOF 0 with two valid answers.

**The spine and its angular pin.** A construction line
`addByTwoPoints(localOrigin, toothTopPoint)`, both existing points passed
directly. No separate start-coincident, and no constraint putting the spine's end
on the arc. Then, for **every** angle including 0: add a far endpoint at
`(TipCircleRadius, 0)` and pin it with two signed distance dimensions from the
local origin — horizontal at `TipCircleRadius`, vertical at `0` — rather than
`addCoincident(end, tipCircle)`, since a point on a circle has two answers and
pinning x at the tip radius touches the circle where the numbers go unstable.
Draw a construction line from the origin to it. Add an angular dimension
**from the reference to the spine, in that argument order**, with its text point
on the bisector `(R·cos(angle/2), R·sin(angle/2))` so Fusion picks `angle` and
not its supplement.

A plain `addHorizontal` on the spine will not do for `angle = 0`: horizontal
fixes the line's slope and says nothing about which way it points, so the tooth
can settle 180° around.

**The ribs.** One construction line per fit-point index, for **all** N indices —
the base-circle pair and the tip pair included, even though the tip ends are also
joined by the tooth-top arc. The fit points carry no other constraint, so an
omitted endpoint rib leaves that point free. Each rib is built in exactly this
order; a different order throws `VCS_SKETCH_OVER_CONSTRAINTS`:

1. `addByTwoPoints(leftSpline.fitPoints[i], rightSpline.fitPoints[i])`, both
   points passed directly; mark construction.
2. A **signed** dimension — vertical at the measured Δy for `angle = 0`. An
   aligned dimension gives only a length, which the mirrored tooth satisfies
   equally well.
3. A fresh midpoint `SketchPoint`, seeded **already on the spine**: with
   `t = fitX·cos(angle) + fitY·sin(angle)`, at `(t·cos(angle), t·sin(angle))`.
   Not at the rib's true 2-D midpoint, and not at `(fitX, 0)` for a rotated
   tooth.
4. `addCoincident(midpoint, spine)` — onto the spine first.
5. `addMidPoint(midpoint, rib)` — then the rib's midpoint.
6. `addPerpendicular(spine, rib)` — then perpendicular. **Skipped for the last
   rib**, whose perpendicular is what the tooth-top arc already says.

Then a **signed** dimension from each midpoint to the previous one along the
spine direction, with the chain starting at the **local origin** for the first
rib. Without that first link the whole chain slides along the spine as a unit and
the sketch never fully constrains.

⚠️ Defect 7: `[SPUR-F-RIBS]` pins the axis only for `angle = 0` (rib vertical,
chain horizontal) and otherwise says to use "whichever of the horizontal/vertical
pair is better conditioned for that angle", which does not say what to do. The
proof reads that as: the rib runs across the spine and the chain along it, so
they take opposite axes, chosen by whether `|cos(angle)| >= |sin(angle)|`. That
reduces to the spec's stated pair at `angle = 0`. The spec should state the rule.

**Closing at the root.** If the flank's first point lies outside the root circle,
draw a short radial line on each side as
`addByTwoPoints(rootEndGeometry, flankStartFitPoint)` — the flank spline's start
`SketchPoint` passed directly — and place the root end with **exactly two**
signed dimensions from the local origin, horizontal at its Δx and vertical at its
Δy. Not "root end on the root circle" plus "local origin on the line": those two
are satisfied by two points, because the line through the flank start and the
centre meets the root circle again on the far side, and the stub becomes a line
straight across the gear.

That gives a **6-curve** tooth loop (2 splines + 2 stubs + 2 arcs). If the flank
starts inside the root circle no stub is drawn and the loop has **4 curves** (2
splines + 2 arcs) — the profile is embedded. The test is strict `<` on raw
values: `embedded = firstRadius < RootCircleRadius`, so exact equality counts as
non-embedded and draws a zero-length stub. Keep the strict comparison. The
generator records the answer as `self.parent._lastToothEmbedded` (it cannot reach
`ctx`), `SpurGearGenerator.__init__` pre-initialises that to `False`, and
`buildSketches` copies it into `ctx.toothProfileIsEmbedded`.

⚠️ Defect 5: both the spec and `[SPUR-F-FLANK-ROOT]` say embedded happens at
"low tooth-count / pressure-angle combinations". It is the opposite: the base
circle falls below the root circle when `ToothNumber > 2.5/(1 − cos(PressureAngle))`
— 41.5 teeth at 20°, 26.7 at 25°. The proof's embedded cases are 45 teeth at 20°
and 30 teeth at 25°, and its near-degenerate case is 41 teeth at 20°, where the
stub is 14 micrometres long.

**Anchoring, and the confirmation.** Project the Tools anchor into this sketch
and add one coincidence between that fresh projection and the local origin. This
happens inside `draw()`, after `drawCircles()` and `drawTooth()`. Then, as the
**very last action** and only when `angle != 0`, set the spine's angular
dimension value to `angle`. The pre-rotation and the value-set are not
alternatives: the pre-rotation puts the geometry on the right solver branch, and
the dimension confirms and locks it rather than swinging the tooth into place
from +X.

Calls: `component.sketches.add(planarEntity)`, `sketch.name`,
`sketch.sketchPoints.add(adsk.core.Point3D.create(x, y, z))`,
`sketch.sketchCurves.sketchCircles.addByCenterRadius(centerPoint, radius)`,
`sketch.sketchDimensions.addDiameterDimension(entity, textPoint)`,
`sketch.sketchTexts.createInput2(text, height)` ⚠️,
`textInput.setAsAlongPath(path, isAbovePath, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, characterSpacing)`,
`sketch.sketchTexts.add(input)`,
`sketch.sketchCurves.sketchFittedSplines.add(fitPoints)`,
`adsk.core.ObjectCollection.create()`,
`sketch.sketchCurves.sketchArcs.addByCenterStartEnd(centerPoint, startPoint, endPoint)`,
`sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)`,
`sketchCurve.isConstruction`,
`sketch.geometricConstraints.addCoincident(point, entity)`,
`sketch.geometricConstraints.addMidPoint(point, midPointCurve)`,
`sketch.geometricConstraints.addPerpendicular(lineOne, lineTwo)`,
`sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`,
`adsk.fusion.DimensionOrientations.VerticalDimensionOrientation`,
`sketch.sketchDimensions.addAngularDimension(lineOne, lineTwo, textPoint)`,
`dimension.parameter.value`, `sketch.project(anchorEntity)` ⚠️,
`spline.fitPoints.item(index)`, `spline.startSketchPoint`,
`spline.endSketchPoint`, `find_circle_by_radius(sketch, radius)`.

**From:** `spec/spurgear/instructions.md` 395–404 (step 3), 406–442 (step 4),
444–448 (step 5), 180–216 (Sketch Discipline), 302–343 (tooth generator
reproduced surface), 272–279 (`buildSketches` / `buildTooth` boundaries),
229 (`ctx.toothProfileIsEmbedded`);
`spec/spurgear/fusion.md` 19–43 (`[SPUR-F-ANCHOR-CHAIN]`,
`[SPUR-F-LOCAL-ORIGIN]`, `[SPUR-F-SHARED-ADJACENCY]`), 47–60
(`[SPUR-F-ROTATE-CONFIRM]`), 69–87 (`[SPUR-F-TOOTHTOP-ARC]`), 89–112
(`[SPUR-F-SPINE]`), 114–147 (`[SPUR-F-RIBS]`), 149–183 (`[SPUR-F-FLANK-ROOT]`);
`.claude/skills/generate-gear/PLAYBOOK.md` 413–450 (`[PB-FULL-CONSTRAINT]`,
`[PB-CIRCLE-CENTER]`, `[PB-PROJECT-NOT-FIXED]`, `[PB-NO-OVERCONSTRAIN]`),
458–473 (`[PB-API-SPELLING]`, `[PB-SKETCHCURVES]`), 517–535 (`[PB-SEED-NEAR]`,
`[PB-SHARE-XOR-COINCIDENT]`, `[PB-DRIVING-DIM]`), 547–555 (`[PB-ANGULAR-DIM]`,
`[PB-RADIAL-DIM]`), 581–591 (`[PB-SKETCH-TEXT]`).

## S6 `[PROSE]` Sketch-Only Short-Circuit

No timeline feature. If `SketchOnly` is true, make the Gear Profile sketch
visible and return from `buildMainGearBody` — no tooth extrude, no chamfer, no
body, no pattern, no fillet. `buildBore` (S14–S15) and `cleanup` (S16) still run
unconditionally afterwards; both carry their own guards.

`SketchOnly` is persisted as a real-valued user parameter (1 or 0) because the
framework reads booleans back through `getParameterAsBoolean`, which reads
numbers.

Calls: `sketch.isVisible`, `generator.getParameterAsBoolean(name)`.

**From:** `spec/spurgear/instructions.md` 450–452 (step 6), 60 (the input),
147–150 (persistence), 252–259 (call graph);
`spec/spurgear/fusion.md` 187–197 (`[SPUR-F-CLEANUP]`).

## S7 `[PROSE]` Extrude the Tooth

`buildTooth` owns this, and **must call `self.chamferTooth(ctx)` as its last
action** — helical overrides it to loft and herringbone to loft-and-mirror, and
both still end that way, so `buildMainGearBody` must not chamfer separately.

Find the tooth cross-section with the framework helper rather than a hand-rolled
loop search: `find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if
ctx.toothProfileIsEmbedded else 2)`. It rejects loops whose counts do not match
and raises when nothing does. The two arcs are the tooth-top arc and the root arc
that Fusion derives by splitting the solid root circle where the stubs (or, in the
embedded case, the flanks) meet it — the tooth's root boundary is never drawn.

Extrude from the target plane to the Extrusion End Plane as a **New Body**, name
the feature `Extrude tooth`, and store the body as `ctx.toothBody`.

The proof checks that this sketch really offers that loop, counting distinct
boundary entities: 2 splines, 2 arcs (the tooth top plus the root circle standing
in for the arc Fusion splits from it), and 2 lines unless embedded. See
`requireExtrudableRegions` in `stepGearProfileSketch`.

Calls: `find_profile_by_curve_counts(sketch, nurbs, arcs, lines)`,
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(entity, isChained)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`component.features.extrudeFeatures.add(input)`, `feature.name`,
`extrude.bodies.item(0)`.

**From:** `spec/spurgear/instructions.md` 454–458 (step 7), 274–279 (method
contract), 434–442 (step 4's curve counts);
`.claude/skills/generate-gear/PLAYBOOK.md` 151–157
(`find_profile_by_curve_counts`), 571–580 (`[PB-PROFILE-MATCH]`).

## S8 `[PROSE]` Chamfer the Tooth

Only when Apply-Chamfer-To-Teeth > 0. Round every edge of the tooth's front face
*except* the arc shared with the root valley; chamfering that one would eat into
the neighbouring tooth.

Find the front face with a **single conjunction**: walk `ctx.toothBody.faces` and
take the first face where **both** `face.edges.count == chamferWantEdges()` (6 on
the spur base, an overridable hook) **and**
`sketchPlane.isCoPlanarTo(face.geometry)`, with
`sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`. Both conditions of
the same face — not an edge-count match with a coplanarity tiebreak. If no face
satisfies both, raise; do not fall back to a partial match.

Identify the root arc by **radius**, not by relative size: skip any edge that is
an `Arc3DCurveType` whose `edge.geometry.radius` equals `RootCircleRadius` within
0.001 cm. Everything else on the face gets chamfered.

Known and accepted: an embedded profile yields a 4-edge front face while
`chamferWantEdges()` stays 6, so chamfering an embedded spur tooth raises the
front-face-not-found error. Users disable chamfer for such gears.

⚠️ The spec writes the edge-set call as
`addEqualDistanceChamferEdgeSet(edges, <ChamferTooth value>, False)`. The index
signature is `(edges: core.ObjectCollection, distance: core.ValueInput,
isTangentChain: bool)`, so the distance must be a `ValueInput` and the third
argument is a tangent-chain flag, not the `isFlipped` that
`[PB-FILLET-CHAMFER]` calls it.

Calls: `ctx.toothBody.faces`, `face.edges`, `face.geometry`,
`sketchPlane.isCoPlanarTo(plane)`, `edge.geometry`, `arc.radius`,
`adsk.core.Curve3DTypes.Arc3DCurveType`, `adsk.core.ObjectCollection.create()`,
`component.features.chamferFeatures.createInput2()`,
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distance, isTangentChain)`,
`component.features.chamferFeatures.add(input)`.

**From:** `spec/spurgear/instructions.md` 460–466 (step 8), 58 (the input),
280–287 (`chamferWantEdges`);
`.claude/skills/generate-gear/PLAYBOOK.md` 480–486 (`[PB-FILLET-CHAMFER]`),
571–580 (`[PB-PROFILE-MATCH]`).

## S9 `[PROSE]` Extrude the Body

Find the body profile with `find_profile_by_curve_counts(sketch, arcs=2)` and
extrude it from the target plane to the Extrusion End Plane as a **New Body**.
Name the feature `Extrude body` and the body `Gear Body`, and store it as
`ctx.gearBody`.

While iterating `extrude.bodies.item(0).faces`, capture `ctx.extrusionExtent` —
the far end cap, used by the bore cut in S15. Among faces whose
`surfaceType` is `PlaneSurfaceType`, it is the one where
`sketchPlane.isParallelToPlane(face.geometry) and not
sketchPlane.isCoPlanarTo(face.geometry)`: the near cap is coplanar with the
sketch plane, and the cylindrical face is not planar at all. Use the
plane-geometry API, not a hand-rolled dot product. Raise if it is not found.

⚠️ Defect 4: the spec calls this profile "the annular loop bounded by exactly 2
arcs (the root circle and the tip circle)". The tip circle is construction
geometry (S5) and bounds no profile, so that loop does not exist. What is
actually there is the full root **disk**, whose boundary is the root circle split
in two by the tooth — which is why an `arcs=2` search still finds it. The
resulting body is a solid root-diameter cylinder, and the teeth are joined onto
it in S12, which is the right geometry; only the description is wrong. The proof
checks this region's identity but cannot check the count, because its engine
coalesces the split boundary back into one whole circle.

Calls: `find_profile_by_curve_counts(sketch, arcs)`,
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`component.features.extrudeFeatures.add(input)`, `feature.name`, `body.name`,
`body.faces`, `face.geometry`, `adsk.core.SurfaceTypes.PlaneSurfaceType`,
`plane.isParallelToPlane(plane)`, `plane.isCoPlanarTo(plane)`,
`sketch.referencePlane`.

**From:** `spec/spurgear/instructions.md` 468–477 (step 9), 226–228 (`ctx`
fields), 399–402 (which circles are construction);
`.claude/skills/generate-gear/PLAYBOOK.md` 151–157 (helper contract), 571–580
(`[PB-PROFILE-MATCH]`), 639–660 (`[PB-FACE-BY-MIDPOINT]`).

## S10 `[PROSE]` Gear Center Axis

A construction axis off any face of the new body whose `surfaceType` is
`CylinderSurfaceType`. Name it `Gear Center`, switch its light bulb off, and
store it as `ctx.centerAxis`. Raise if no cylindrical face is found. S11 patterns
around it, and S16 has nothing left to do for it.

The spec builds this inside step 9's face loop; it is a separate timeline entry,
so it is a separate step here.

Calls: `body.faces`, `face.geometry`,
`adsk.core.SurfaceTypes.CylinderSurfaceType`,
`component.constructionAxes.createInput()`,
`axisInput.setByCircularFace(circularFace)`,
`component.constructionAxes.add(axisInput)`, `constructionAxis.name`,
`constructionAxis.isLightBulbOn`.

**From:** `spec/spurgear/instructions.md` 472–477 (step 9), 227 (`ctx.centerAxis`);
`.claude/skills/generate-gear/PLAYBOOK.md` 686–689 (`[PB-CONSTRUCTION-AXES]`),
558–570 (`[PB-HIDE-AFTER-USE]`).

## S11 `[PROSE]` Pattern the Teeth

Circular-pattern `ctx.toothBody` around the `Gear Center` axis with quantity =
Tooth Number. Pin all three inputs explicitly rather than relying on defaults:
`quantity`, `totalAngle = ValueInput.createByString('360 deg')` (a full turn, set
as a string expression), and `isSymmetric = False`.

`patternTeeth` owns this and must call `self.createFillets(ctx)` after the
combine.

Calls: `adsk.core.ObjectCollection.create()`,
`component.features.circularPatternFeatures.createInput(inputEntities, axis)`,
`patternInput.quantity`, `patternInput.totalAngle`, `patternInput.isSymmetric`,
`adsk.core.ValueInput.createByString('360 deg')`,
`adsk.core.ValueInput.createByReal(toothNumber)`,
`component.features.circularPatternFeatures.add(input)`.

**From:** `spec/spurgear/instructions.md` 479–483 (step 10), 256 (call graph);
`.claude/skills/generate-gear/PLAYBOOK.md` 597–602 (`[PB-CIRCULAR-PATTERN]`).

## S12 `[PROSE]` Combine the Teeth into the Body

One Combine-Join with `Gear Body` as the target. Feed the pattern's `bodies`
collection as-is: it already includes the seed tooth alongside the copies, so do
not re-add it. Copy the members into a fresh `ObjectCollection` first —
`pattern.bodies` is a `BRepBodies`, and `combineFeatures.createInput` rejects it.

Calls: `pattern.bodies`, `bodies.item(i)`,
`adsk.core.ObjectCollection.create()`,
`component.features.combineFeatures.createInput(targetBody, toolBodies)`,
`combineInput.operation`,
`adsk.fusion.FeatureOperations.JoinFeatureOperation`,
`component.features.combineFeatures.add(input)`.

**From:** `spec/spurgear/instructions.md` 481–483 (step 10);
`.claude/skills/generate-gear/PLAYBOOK.md` 592–596 (`[PB-PATTERN-BODIES]`).

## S13 `[PROSE]` Root Fillets

Only when Fillet Radius > 0. Round the sharp inside corner where each valley
floor meets a tooth flank — the one that runs the full thickness parallel to the
gear axis, where bending stress concentrates. Not the front and back rim, which
is cosmetic and not wanted here.

Two things make the edge pick fiddly. After the pattern and combine, the root
cylinder is usually split into one patch per valley, so collect **every**
cylindrical face whose radius equals Root Circle Radius, not just the first. And
on each such face keep only the **axial straight** edges: filter to
`Line3DCurveType`, take each line's direction from its geometry endpoints via
`geometry.startPoint.vectorTo(geometry.endPoint)`, normalize, and keep it when
`abs(abs(dot(direction, axisNormal)) - 1.0) < 0.01`. Use exactly that tolerance;
a tighter test like `> 0.999` drops valid axial edges that tessellation has left
slightly off, leaving root fillets missing. Do **not** read the direction via
`edge.evaluator.getTangent(0)` — parameter 0 is not guaranteed to lie inside the
edge's range and Fusion raises `RuntimeError: invalid argument parameter`.

`isTangentChain` must be `False`: the collected edges are exactly the axial root
corners, and tangent-chaining would let Fusion pull in tangent-adjacent edges and
round more than intended.

If the edge collection is **empty**, silently skip — return without creating the
feature, no error. An empty edge set must not reach `filletFeatures.add`.

⚠️ Defect 3: the spec says to add the edge set on the input itself,
`filletInput.addConstantRadiusEdgeSet(edges, radius, isTangentChain)`, and that
`filletInput.edgeSetInputs` does not exist and raises `AttributeError`. The API
index says the reverse: `FilletFeatureInput` has an `edgeSetInputs ->
FilletEdgeSetInputs` property and **no** `addConstantRadiusEdgeSet`, which is
carried by `FilletEdgeSetInputs` and `FilletEdgeSets`. One of the two is stale
and this cannot be settled from the sources given. Named as the spec has it.

Calls: `ctx.gearBody.faces`, `face.geometry`,
`adsk.core.SurfaceTypes.CylinderSurfaceType`, `cylinder.radius`, `face.edges`,
`edge.geometry`, `adsk.core.Curve3DTypes.Line3DCurveType`,
`line.startPoint`, `adsk.core.Point3D.vectorTo(point)`,
`adsk.core.Vector3D.normalize()`, `adsk.core.Vector3D.dotProduct(vector)`,
`get_normal(entity)`, `adsk.core.ObjectCollection.create()`,
`component.features.filletFeatures.createInput()`,
`filletInput.addConstantRadiusEdgeSet(entities, radius, isTangentChain)` ⚠️,
`adsk.core.ValueInput.createByReal(filletRadius)`,
`component.features.filletFeatures.add(input)`.

**From:** `spec/spurgear/instructions.md` 485–494 (step 11), 86–88 (Fillet
Clearance and Radius), 280–287 (`filletHelixFactorExpression`);
`.claude/skills/generate-gear/PLAYBOOK.md` 480–486 (`[PB-FILLET-CHAMFER]`),
412 (`[PB-EMPTY-RESULT]`), 151–157 (`get_normal`).

## S14 `[GO]` Bore Profile Sketch — `stepBoreProfileSketch`

`buildBore` runs unconditionally from `generate()`, after `buildMainGearBody`, so
it must itself early-return in **two** cases: when `SketchOnly` is set, and when
Bore Diameter ≤ 0. The SketchOnly guard is the essential one — in that mode
`buildMainGearBody` short-circuits before S9, so `ctx.gearBody` and
`ctx.extrusionExtent` are never set and the cut would dereference `None`. Do not
rely on the bore diameter being 0 in sketch-only mode; the user may have set
both.

Otherwise create a sketch named `Bore Profile` on the target plane and draw the
bore circle by **instantiating the tooth generator on it** —
`SpurGearInvoluteToothDesignGenerator(boreSketch, self)` — and calling
`drawBore(ctx.anchorPoint, boreDiameter)`, which projects the anchor in, draws a
non-construction circle of that diameter centred on the projection with a driving
diameter dimension, and returns the circle.

That constructor always adds its local-origin `(0, 0, 0)` `SketchPoint`, so this
sketch carries one stray unused point. Keep it — the behaviour is faithful — and
**ground it** with `addCoincident(toothGen.anchorPoint, boreSketch.originPoint)`.
Every sketch has an `originPoint` fixed at the plane origin, so one constraint
grounds it. Without it the point is free in two directions and the sketch never
reaches `isFullyConstrained`.

This is the one place in the build where a point is grounded on the sketch's own
origin rather than on the projected anchor, and the proof does the same:
`s.AddConstraint(sketch.NewCoincident(stray, s.Origin()))`.

Calls: `component.sketches.add(planarEntity)`, `sketch.name`,
`sketch.project(ctx.anchorPoint)` ⚠️, `sketch.sketchPoints.add(point)`,
`sketch.sketchCurves.sketchCircles.addByCenterRadius(centerPoint, radius)`,
`sketch.sketchDimensions.addDiameterDimension(entity, textPoint)`,
`sketch.geometricConstraints.addCoincident(point, entity)`,
`sketch.originPoint`, `generator.getParameterAsBoolean(name)`.

**From:** `spec/spurgear/instructions.md` 496–500 (step 12), 54 (Bore Diameter),
210–212 (Sketch Discipline), 321–326 (`drawBore` surface), 258–268 (call graph);
`spec/spurgear/fusion.md` 26–31 (`[SPUR-F-LOCAL-ORIGIN]`);
`.claude/skills/generate-gear/PLAYBOOK.md` 413–418 (`[PB-FULL-CONSTRAINT]`),
430–444 (`[PB-PROJECT-NOT-FIXED]`).

## S15 `[PROSE]` Bore Cut

Extrude-cut the bore profile from the target plane to `ctx.extrusionExtent`, the
far end-cap face captured in S9, affecting only `ctx.gearBody`. Going to that
face rather than a fixed distance guarantees the bore pierces the gear whatever
Thickness is.

Calls: `sketch.profiles.item(0)`,
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`extrudeInput.participantBodies`,
`component.features.extrudeFeatures.add(input)`.

**From:** `spec/spurgear/instructions.md` 500 (step 12), 228
(`ctx.extrusionExtent`);
`.claude/skills/generate-gear/PLAYBOOK.md` 451–457 (`[PB-SINGLE-PROFILE]`).

## S16 `[PROSE]` Cleanup

The very last action of `generate()`, after `buildBore` — not inside
`buildMainGearBody`. Placement matters: `buildBore` re-projects
`ctx.anchorPoint` from the Tools sketch, and projection fails once that sketch is
hidden, so Tools must stay visible through the bore.

Call it **unconditionally** in both modes. The SketchOnly distinction lives
inside it, split by entity kind:

- **Construction planes and axes** — the Extrusion End Plane, the `Gear Center`
  axis, and the normalized target plane if S2 created one — are hidden with
  `isLightBulbOn = False` in **both** modes, so no stray plane floats.
- **Sketches** — Tools, Gear Profile, Bore Profile — are hidden with
  `isVisible = False` on the **full-build path only**. Sketch-only mode leaves
  them visible for inspection, which is the whole point of that mode.

Never cross the two properties: `isVisible = False` has no effect on a
construction plane or axis. Guard each entity individually, since the `Gear
Center` axis and the Bore Profile sketch do not exist in sketch-only mode.

Calls: `constructionPlane.isLightBulbOn`, `constructionAxis.isLightBulbOn`,
`sketch.isVisible`.

**From:** `spec/spurgear/instructions.md` 258–268 (call graph and placement),
203–205 (Sketch Discipline), 450–452 (step 6);
`spec/spurgear/fusion.md` 187–197 (`[SPUR-F-CLEANUP]`);
`.claude/skills/generate-gear/PLAYBOOK.md` 558–570 (`[PB-HIDE-AFTER-USE]`).

---

## What the proof covers

`.tmp/spurgear_test.go`, package `spurgear_test`. Three test functions, one per
`[GO]` step, each running its build over a table of parameter cases through
`proofkit.Run`, which gates on the engine's full verdict — DOF 0, no conflicting
or redundant constraints, valid profiles, a well-conditioned system, and a
constraint set that admits only one discrete configuration. Nothing is waived.

| Step | Function | Cases |
|---|---|---|
| S3 | `stepToolsSketch` | anchor at and away from the plane origin |
| S5 | `stepGearProfileSketch` | 13 sizes (see below) |
| S14 | `stepBoreProfileSketch` | two bore diameters, two anchors, and the zero-bore case, which is `Unmodelled` because no sketch exists |

The Gear Profile sweep is m1/t12, m1/t17, m2/t20 and m3/t15 — the required sizes,
all at 20° with 15 involute steps — plus: m1/t45 and m2/t30 at 25°, both
embedded; m1/t41, the near-degenerate size where the stub is 14 µm long; m1/t17
at +30°, m2/t20 at −25° and m5/t12 at +90°, where the spine is vertical and the
signed dimensions swap axis; m1/t45 at +120°, embedded and rotated together;
m1/t17 with the anchor at (37, −19); and m1.5/t24 at 14.5° with 8 involute steps.

Three constraint rules the spec states as failure modes were checked by breaking
them, and each failed the way the spec predicts:

- Omitting the endpoint rib leaves DOF 3, with both tip fit points reported free.
- Keeping the last rib's perpendicular gives DOF 0 with one redundant
  constraint.
- Replacing a signed rib dimension with an aligned one gives DOF 0 with **nine**
  discrete configurations, which is the mirrored tooth `[SPUR-F-RIBS]` warns
  about.
