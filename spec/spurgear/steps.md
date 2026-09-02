# Spur Gear — compiled step list

The proof for this step list is `proof/spurgear/sketches_test.go`, `proof/spurgear/solids_test.go`
and the generated `proof/spurgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/spurgear/instructions.md` | `560549a350c789b6598ee49b72d3cd7b4460e214` |
| `spec/spurgear/fusion.md` | `7cd4e5b0fa38dcd39cbd1b5bad1cf8489e2bc2ae` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `aa97523b214dbc98b679e8eea73c56c5115c4e54` |

## S1 `[PROSE]` Command dialog inputs

`SpurGearCommandInputsConfigurator` is a plain class with one `@classmethod` named `configure`,
taking `(cls, cmd)`. It adds the dialog inputs to `cmd.commandInputs` in exactly this order, which
is the display order and is **not** the `processInputs` read order:

1. Target Plane — `cmd.commandInputs.addSelectionInput('plane', 'Target Plane', ...)`, then
   `planeInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)` and the same
   for `PlanarFaces`, then `planeInput.setSelectionLimits(1, 1)`. The filter set and the limits are
   contract surface the spec declares per input (`[PB-SELECTION-DECL]`), and each filter is the
   named member of `adsk.core.SelectionCommandInput` rather than a hand-written literal
   (`[PB-SELECTION-FILTER-ENUM]`). Target Plane is added first because Fusion auto-focuses the
   first selection input in the dialog.
2. Anchor Point — `addSelectionInput` with id `anchorPoint`, filters `ConstructionPoints` and
   `SketchPoints`, limits `(1, 1)`.
3. Module — `cmd.commandInputs.addValueInput('module', 'Module', '', adsk.core.ValueInput.createByReal(1))`.
   The unit string is empty, not `'mm'`.
4. Tooth Number — `addValueInput` id `toothNumber`, unit `''`, default `createByReal(17)`.
5. Pressure Angle — `addValueInput` id `pressureAngle`, unit `'deg'`, default
   `adsk.core.ValueInput.createByReal(math.radians(20))`. The display unit is degrees and the
   default is in Fusion's internal radians, which is what a `createByReal` default always means
   whatever the unit string says (`[PB-DIALOG-DEFAULT-UNITS]`).
6. Bore Diameter — `cmd.commandInputs.addStringValueInput('boreDiameter', 'Bore Diameter', '0 mm')`,
   a string input so it accepts expressions.
7. Thickness — `addValueInput` id `thickness`, unit `'mm'`, default `createByReal(to_cm(10))`. The
   display unit is millimetres and the default is in internal centimetres.
8. Apply chamfer to teeth — `addValueInput` id `chamferTooth`, unit `'mm'`, default `createByReal(0)`.
9. The sketch-only checkbox — `cmd.commandInputs.addBoolValueInput('sketchOnly', label, True)`, with
   `True` making it a check box rather than a button. Its label is the literal string
   `Generate sketches, but do not build body`.
10. Parent Component — `addSelectionInput` id `parentComponent`, filters `Occurrences` and
    `RootComponents`, limits `(1, 1)`, pre-selecting `get_design().rootComponent`. It is **last**.

Every input id and every registered parameter name is exported as a module-level constant
(`[SPUR-EXPORTED-CONSTANTS]`) —
`INPUT_ID_PARENT`, `INPUT_ID_PLANE`, `INPUT_ID_ANCHOR_POINT`, `INPUT_ID_MODULE`,
`INPUT_ID_TOOTH_NUMBER`, `INPUT_ID_PRESSURE_ANGLE`, `INPUT_ID_BORE_DIAMETER`, `INPUT_ID_THICKNESS`,
`INPUT_ID_CHAMFER_TOOTH`, `INPUT_ID_SKETCH_ONLY`, and the `PARAM_…` roster — because
`helicalgear.py` and `herringbonegear.py` import them by name.

This step creates no timeline entry and no geometry, so there is nothing for a geometry proof to
build. Its content is a dialog, and the contract check on the input ids and the parameter names is
what holds it.

<!-- check-step-calls: ignore configure -->

`configure` is the method this class defines for the shared command wiring to call: the module
declares it and never calls it, so the directive above marks it as a mention. It is also the seam a
subclass gear extends, by subclassing the configurator and appending its own input after
`super().configure(cmd)` (`[SPUR-SUBCLASS-INPUT]`), which is why Parent Component sitting last here
still leaves a subclass's extra input below it.

**From:** `spec/spurgear/instructions.md` L20–22, L37–62, L90–150, L152–178;
`.claude/skills/generate-gear/PLAYBOOK.md` L53–60, L128–143, L346–348, L499–504

## S2 `[PROSE]` Read the inputs and register the parameters

`processInputs(inputs)` runs before anything touches the design, because creating the gear
occurrence shifts Fusion's active component and a selection input holding an entity from another
component can drop it (`[PB-SELECTION-STASH]`).

1. Read the three selection inputs first, with `get_selection(inputs, INPUT_ID_PARENT)` and the
   same for the plane and the anchor point, and stash the entities on `self.parentComponent`,
   `self.plane` and `self.anchorPoint`. Resolve an `Occurrence` selection to its `component`.
2. Read each input with the helper that matches the type it was declared with
   (`[PB-INPUT-READ]`): `get_value(inputs, id, units)` for the value and string inputs and
   `get_boolean(inputs, INPUT_ID_SKETCH_ONLY)` for the checkbox; a bool input has no `expression`,
   so `get_value` on it raises at generation time. `get_value` returns a `ValueInput` ready to hand
   straight to the parameter registration and raises rather than returning nothing on a bad
   expression, so no caller-side wrapping or ok-flag handling is needed
   (`[PB-GET-VALUE-CONTRACT]`). Register each with `self.addParameter(name, valueInput, units, comment)`:
   `Module` unitless, `ToothNumber` unitless, `PressureAngle` in `'rad'`, `BoreDiameter` in `'mm'`,
   `Thickness` in `'mm'`, `ChamferTooth` in `'mm'`, and `SketchOnly` as a real 1 or 0 so
   `self.getParameterAsBoolean(PARAM_SKETCH_ONLY)` can read it back.
3. Call the `addExtraPrimaryParameters(inputs)` hook, a no-op on the spur base, between the
   input-sourced parameters and the derived ones (`[SPUR-EXTRA-PARAMS]`). The call site has to be
   there for a subclass to hook.
4. `registerDerivedParameters` adds the rest as live expressions through
   `adsk.core.ValueInput.createByString(...)`: `PitchCircleDiameter = Module * ToothNumber`,
   `PitchCircleRadius = PitchCircleDiameter / 2`, `BaseCircleDiameter = PitchCircleDiameter * cos(PressureAngle)`,
   `BaseCircleRadius`, `RootCircleDiameter = PitchCircleDiameter - 2.5 * Module`, `RootCircleRadius`,
   `TipCircleDiameter = PitchCircleDiameter + 2 * Module`, `TipCircleRadius`, `InvoluteSteps = 15`,
   `ToothSpaceArcAtRoot = RootCircleRadius * ToothSpaceAngleAtRoot`,
   `FilletClearance = 0.9`, and
   `FilletRadius = (ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>` where `<factor>` is the
   string `self.filletHelixFactorExpression()` returns — `'1'` on the spur base.
   `ToothSpaceAngleAtRoot` is the one parameter computed in Python and registered with
   `adsk.core.ValueInput.createByReal(...)`, because Fusion's expression engine will not subtract a
   radian-valued `PressureAngle` from the unitless output of `tan`. Its value is
   `math.pi / toothNumber - 2 * (math.tan(pressureAngle) - pressureAngle)`, and it is registered
   **unitless**, not `'rad'`, so `ToothSpaceArcAtRoot` reads as a length.

`generate` then names the component with `self.generateName()`, which returns
`'Spur Gear (M={}, Tooth={}, Thickness={})'` formatted from the `.expression` strings of `Module`,
`ToothNumber` and `Thickness`.

Every dimension and feature input later in the build takes the *current numeric value* of its
parameter, never a live expression: editing a `SpurGear<N>_…` parameter does not change an existing
gear (`[PB-NUMERIC-SNAPSHOT]`, applied to this gear by `[SPUR-F-SNAPSHOT]`). The parameters stay in
the table for reference.

No timeline entry, no geometry: nothing here is a shape a geometry proof can build. The derived
formulas do reach the proof, but through the geometry they produce — the circle radii of S6, and
the Fillet Radius the S12 case table computes from this same tooth-space arc.

**From:** `spec/spurgear/instructions.md` L64–89, L147–150, L323–335, L407–416;
`.claude/skills/generate-gear/PLAYBOOK.md` L103–126, L196–228

## S3 `[PROSE]` Normalize the Target Plane

If `self.plane` is not already a `ConstructionPlane` — the user may have picked a planar face — build
a coplanar one and replace it: `planeInput = component.constructionPlanes.createInput()`, then
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))`, then
`component.constructionPlanes.add(planeInput)`. The offset argument is a `ValueInput`, never a bare
number (`[PB-CONSTRUCTION-PLANES]` gives the constructor's signature). Keep the result on both `self.plane` and, once the context exists, `ctx.plane`; subclasses
read `self.plane` directly.

This is a timeline entry, but only a construction plane coplanar with an entity the user chose.
The harnesses model a sketch on a plane and a solid built from it; neither carries a construction
plane whose position is derived from an external selection, so there is no geometry here for a
proof to check that is not already assumed by every step below it.

**From:** `spec/spurgear/instructions.md` L39, L420–422;
`.claude/skills/generate-gear/PLAYBOOK.md` L244–254, L711–722

## S4 `[GO]` Tools sketch

Create a sketch named `Tools` on the target plane with `self.createSketchObject('Tools', self.plane)`
and make it visible. Project the user's anchor point into it with `toolsSketch.project(self.anchorPoint)`
and keep the first returned entity as `ctx.anchorPoint`. The sketch draws no geometry of its own; it
exists to own this one reference, and every later sketch projects *this* point in again so the whole
gear tracks the anchor if the user moves it (`[SPUR-F-ANCHOR-CHAIN]`). A projection is brought in
associatively and is not itself fixed, so a sketch built on one reaches full constraint only when
the projection is tied to something the sketch already pins (`[PB-PROJECT-NOT-FIXED]`,
`[PB-FULL-CONSTRAINT]`).

Leave the sketch visible. It must stay visible through the bore, which re-projects from it, and
projection has failed on an invisible sketch in this repository's history (`[PB-HIDE-AFTER-USE]`:
hide a sketch only after everything that consumes it has run).

The proof function is `stepToolsSketch`. It models the projection with the sketch engine's
externally-locked reference point, which is what a Fusion projection is: a point whose position
comes from outside the sketch and which the solver never moves. It asserts the sketch carries that
one point and no curves, and the harness gates it on the engine's full verification verdict, so a
Tools sketch that did not reach zero free degrees of freedom would fail here.

<!-- proof-run: proofkit.Run(toolsCases, stepToolsSketch) -->

**From:** `spec/spurgear/instructions.md` L41, L258, L424–426, L228–230, L246–248;
`spec/spurgear/fusion.md` L19–24;
`.claude/skills/generate-gear/PLAYBOOK.md` L94–96, L455–469, L595–607

## S5 `[PROSE]` Extrusion End Plane

Create an offset construction plane named `Extrusion End Plane` at `Thickness` from the target
plane: `planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))`, added
through `component.constructionPlanes.add(planeInput)` (`[PB-CONSTRUCTION-PLANES]`). Keep it on
`ctx.extrusionEndPlane`.

Its only purpose is to be the `to-entity` target for the tooth and body extrudes, so both end on the
same well-defined face. Leave it visible while those extrudes run; the final cleanup switches its
light bulb off, which is the property that hides a construction plane — `isVisible` does not
(`[PB-HIDE-AFTER-USE]`).

There is no construction plane in either harness, and a plane at a distance from the sketch plane is
exactly the extent the two extrude steps below sweep through, so this step's content is proven where
it is consumed: S8 and S9 each assert that the body they build has one cap on the sketch plane and
one a full Thickness away.

**From:** `spec/spurgear/instructions.md` L259, L422, L428;
`.claude/skills/generate-gear/PLAYBOOK.md` L595–607, L711–722

## S6 `[GO]` Gear Profile sketch

One sketch, one timeline entry, and the whole involute construction inside it. Create it with
`self.createSketchObject('Gear Profile', self.plane)`, keep it on `ctx.gearProfileSketch`, and run
`SpurGearInvoluteToothDesignGenerator(gearProfileSketch, self).draw(ctx.anchorPoint, angle=0)`. The
generator's `draw` performs `drawCircles()`, then `drawTooth(angle)`, then the anchoring, then — only
when `angle != 0` — sets the confirming angular dimension's value (`[SPUR-F-ROTATE-CONFIRM]`). `buildSketches` afterwards copies
the embedded flag across with `ctx.toothProfileIsEmbedded = self._lastToothEmbedded`.

**The local origin** (`[SPUR-F-LOCAL-ORIGIN]`)**.** The generator's constructor adds a fresh
`SketchPoint` at (0, 0, 0) with
`gearProfileSketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))` and stores it as
`self.anchorPoint`. It is not `sketch.originPoint`, which is immutable and cannot be made coincident
with anything projected in; `[PB-CIRCLE-CENTER]` records the solver failure that comes of
constraining to it. Every piece of geometry below is drawn relative to this point.

**The four circles.** Draw each with
`gearProfileSketch.sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)` — the curve
collections live under `sketchCurves` (`[PB-SKETCHCURVES]`) — passing the local-origin `SketchPoint`
**object** so all four share it, which is `[PB-SHARE-XOR-COINCIDENT]` applied through
`[SPUR-F-SHARED-ADJACENCY]`: share the point or coincident a fresh one, never both. Draw the **Root Circle** solid at
`RootCircleRadius`, then the **Tip Circle**, **Base Circle** and **Pitch Circle** as construction
geometry by setting `circle.isConstruction = True`. Give each a driving diameter dimension with
`gearProfileSketch.sketchDimensions.addDiameterDimension(circle, textPoint)` — driving is the
default and the trailing driven flag is never passed (`[PB-DRIVING-DIM]`) — with the text point off
the centre, since a diameter dimension rejects a text point at the curve's centre
(`[PB-RADIAL-DIM]`). Label each circle along its own path (`[PB-SKETCH-TEXT]` fixes the three-call
shape):
`textInput = gearProfileSketch.sketchTexts.createInput2(text, size)`, then
`textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`,
then `gearProfileSketch.sketchTexts.add(textInput)`. The string is
`'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` with the radii read as internal `.value`
centimetres, and `size = TipCircleRadius - RootCircleRadius`, passed as the text height too.

**The involute flanks.** Sample `InvoluteSteps` points endpoint-inclusive, sample `i` at radius
`BaseCircleRadius + (TipCircleRadius - BaseCircleRadius) * i / (steps - 1)`, so the first sits
exactly on the base circle and the last exactly on the tip circle. Do not clamp the start to the
root circle. Each sample is `self.calculateInvolutePoint(baseRadius, r)`, whose parameter is
`math.tan(alpha)` for `alpha = math.acos(baseRadius / r)` — **not** `tan(alpha) - alpha` — giving
`x = baseRadius * (cos(t) + t * sin(t))`, `y = baseRadius * (sin(t) - t * cos(t))`; it returns `None`
below the base circle and those samples are dropped. Mirror the samples across +X by negating `y`,
because the standard parametric involute spirals the wrong way for a left flank. Rotate by
`math.pi / (2 * toothNumber) - math.atan2(-py, px)`, with `(px, py)` the analytic pitch-circle point,
so the tooth is symmetric about +X. Mirror that across X for the right flank. Then rotate **both**
collections by the requested `angle` here, in the Python point math, so the tooth is drawn already at
its final angular position. Draw each flank with
`gearProfileSketch.sketchCurves.sketchFittedSplines.add(pointCollection)` over an
`adsk.core.ObjectCollection.create()` of the sample points.

**The tooth-top arc** (`[SPUR-F-TOOTHTOP-ARC]`)**.** Add a tooth-top `SketchPoint` at
`(TipCircleRadius * cos(angle), TipCircleRadius * sin(angle))` and constrain it onto the tip circle
with `gearProfileSketch.geometricConstraints.addCoincident(toothTopPoint, tipCircle)`. Create the arc
with
`gearProfileSketch.sketchCurves.sketchArcs.addByCenterStartEnd(localOrigin, rightFlank.endSketchPoint, leftFlank.endSketchPoint)`,
passing the two flank splines' end `SketchPoint`s directly so the arc shares them. That call shares
the start and end but **copies** the centre — the one place `[PB-SHARE-XOR-COINCIDENT]` calls for a
coincident on a point that was passed in — so tie the copy back with
`gearProfileSketch.geometricConstraints.addCoincident(toothTopArc.centerSketchPoint, localOrigin)`.
Add no diameter dimension: the coincident centre and the two shared ends already determine the arc.

**The spine, the +X reference and the angular pin** (`[SPUR-F-SPINE]`)**.** Draw the spine as
`gearProfileSketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, toothTopPoint)`, sharing both
points, and mark it construction. Add a reference endpoint at `(TipCircleRadius, 0)` and pin it with
two axis dimensions from the local origin:
`gearProfileSketch.sketchDimensions.addDistanceDimension(localOrigin, referenceEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
set to `TipCircleRadius`, and the same with
`adsk.fusion.DimensionOrientations.VerticalDimensionOrientation` set to `0`. Both values are
non-negative magnitudes with the endpoint seeded on the +X side, because a linear dimension's value
is a magnitude and its direction is captured from the geometry at creation
(`[PB-DIM-VALUE-SEMANTICS]`). Draw the reference line
from the origin to that endpoint and mark it construction. Then add
`gearProfileSketch.sketchDimensions.addAngularDimension(referenceLine, spine, textPoint)` in that
argument order, placing the text on the bisector at `(R * cos(angle / 2), R * sin(angle / 2))` for a
small `R` so Fusion picks `angle` rather than its supplement — an angular dimension measures the
wedge its text point lies in (`[PB-ANGULAR-DIM]`). Build this reference and this dimension for
**every** angle, zero included. The value-set that confirms a non-zero rotation is the very last
action of `draw`, after the whole constraint network exists (`[SPUR-F-ROTATE-CONFIRM]`), and it
carries the sign that a mirrored scheme would lose.

**The ribs** (`[SPUR-F-RIBS]`)**.** One rib per fit-point index, for all `N` indices including the
base-circle pair and the tip pair. For each `i`, in this order:
`gearProfileSketch.sketchCurves.sketchLines.addByTwoPoints(leftSpline.fitPoints.item(i), rightSpline.fitPoints.item(i))`
marked construction; one **axis** dimension across the spine, vertical when
`abs(math.cos(angle)) >= abs(math.sin(angle))` and horizontal otherwise, created at the seeded
positions and left at the measured magnitude, so the direction that forbids a mirrored tooth comes
from the seed (`[PB-DIM-VALUE-SEMANTICS]`); a fresh midpoint `SketchPoint` seeded already **on the
spine**, at `(t * cos(angle), t * sin(angle))` for `t = fitX * cos(angle) + fitY * sin(angle)` — the
solver is seed-sensitive and a point that must end up on a line belongs near that line
(`[PB-SEED-NEAR]`);
`gearProfileSketch.geometricConstraints.addCoincident(midPoint, spine)`; then
`gearProfileSketch.geometricConstraints.addMidPoint(midPoint, rib)`; then
`gearProfileSketch.geometricConstraints.addPerpendicular(spine, rib)`, **skipped for the last rib**
because the tooth-top arc already holds its two ends at equal radius either side of the spine, and
adding a constraint an existing one already drives is what over-constrains a sketch
(`[PB-NO-OVERCONSTRAIN]`). Then
chain the midpoints: an axis dimension along the spine direction from the previous midpoint to this
one, horizontal when the rib took the vertical axis and vertical otherwise, with the first taken
from the local origin.

**The flank-to-root lines** (`[SPUR-F-FLANK-ROOT]`)**.** With `firstRadius` the distance from the local origin to the left
flank's first fit point, the profile is embedded when `firstRadius < RootCircleRadius`, compared raw
with no tolerance, so exact equality counts as **not** embedded and draws a zero-length stub. When it
is not embedded, draw a short radial line on each side with
`gearProfileSketch.sketchCurves.sketchLines.addByTwoPoints(rootEndPoint, flankSpline.startSketchPoint)`,
sharing the flank's start point, and place the root end with exactly two axis dimensions from the
local origin — `addDistanceDimension` with `HorizontalDimensionOrientation` and again with
`VerticalDimensionOrientation`. Seed the root end at its exact computed position **before** creating
the dimensions, and set each `dimension.parameter.value` to `abs(delta)`, never to the signed delta:
a negative value flips the point to the other side of the origin (`[PB-DIM-VALUE-SEMANTICS]`). Set
`self.parent._lastToothEmbedded` to the flag either way; `SpurGearGenerator.__init__` pre-initialises
it to `False`.

**The anchoring** (`[SPUR-F-ANCHOR-CHAIN]`)**.** Project the Tools-sketch anchor in again with
`gearProfileSketch.project(ctx.anchorPoint)` and add
`gearProfileSketch.geometricConstraints.addCoincident(toothGenerator.anchorPoint, projectedAnchor)`
between the projection and the local origin. This happens inside `draw`, not in `buildSketches`,
because helical and herringbone call `draw` directly on their own loft sketch and rely on that one
call to anchor it.

The finished sketch must be fully constrained, with no free degree of freedom and no redundant
constraint (`[PB-FULL-CONSTRAINT]`, `[PB-NO-OVERCONSTRAIN]`).

**What the sketch owes the two extrude steps** (`[PB-PROFILE-MATCH]`)**.** It closes exactly two
regions. The tooth section is
bounded by 2 NURBS, 2 arcs and 2 lines, or by 2 NURBS and 2 arcs when the profile is embedded. The
disc inside the root circle is bounded by exactly 2 arcs — the two pieces the tooth cuts the root
circle into — and it is a disc and not an annulus, because the tip circle is construction geometry
and bounds no profile.

The proof function is `stepGearProfileSketch`. Its case table sweeps the regime the spec names: two
sizes either side of the standard one, three and four involute samples as well as fifteen, the
requested angle at zero, both signs of a helix angle, both signs of a quarter turn where the rib and
chain dimensions swap axes, the bevel virtual tooth's half turn, both routes into the embedded shape,
and the strict-comparison boundary where the base circle lands exactly on the root circle. It asserts
the first and last flank samples sit on the base and tip circles, that the sketch closes two regions
and no more, that the disc is bounded by the root circle alone and has the root circle's area, that
the tooth loop carries the curve counts above, and that the tooth meets the root circle at exactly two
places — the two cuts that make the disc's two arcs. The harness then gates the sketch on the engine's
own verdict, which asks for zero free degrees of freedom, no redundant or conflicting constraint, valid
profiles, a system that is not near-singular, and no second discrete solution.

<!-- proof-run: proofkit.Run(profileCases, stepGearProfileSketch) -->

<!-- check-step-calls: ignore find_circle_by_radius addHorizontal -->

`find_circle_by_radius` is one of two ways the spec allows a later drawing step to reach a circle
`drawCircles` made — the other is keeping a direct reference — so it is an alternative, not a
requirement. `addHorizontal` is named only to forbid it: a horizontal constraint on the spine fixes
the line's direction but not which way it points, and the tooth can come out a half turn around.

**From:** `spec/spurgear/instructions.md` L228–248, L310–312, L337–378, L430–439, L441–478, L480–484;
`spec/spurgear/fusion.md` L19–43, L47–60, L69–104, L106–131, L133–173, L175–215;
`.claude/skills/generate-gear/PLAYBOOK.md` L230–242, L438–475, L483–498, L550–572, L584–592, L608–628

## S7 `[PROSE]` Sketch-only short-circuit

`buildMainGearBody(ctx)` calls `buildSketches(ctx)` and then, if
`self.getParameterAsBoolean(PARAM_SKETCH_ONLY)` is true, makes the Gear Profile sketch visible and
returns, skipping the tooth extrude, the body extrude, the pattern and the fillets. The end-of-build
cleanup still runs; its per-mode split is owned by `[SPUR-F-CLEANUP]` and written out in S16.

A branch that stops the build creates no geometry, so there is nothing to build and check. What the
branch protects is asserted where it would break: S14 covers the bore's own early return, which is
the guard that keeps the sketch-only path from dereferencing a Gear Body that was never made.

**From:** `spec/spurgear/instructions.md` L60, L286–296, L486–488

## S8 `[GO]` Extrude the tooth

Find the tooth cross-section with
`find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`
and do not re-implement the search; the helper raises when nothing matches rather than falling back to
a wrong profile. Profiles are found by the curve-type counts of their loop, never by index
(`[PB-PROFILE-MATCH]`).

Extrude it from the target plane to the Extrusion End Plane as a new body. Build the input with
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
then
`extrudeInput.setOneSideExtent(adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False), adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
then `component.features.extrudeFeatures.add(extrudeInput)`. Name the feature `Extrude tooth` and
store `extrude.bodies.item(0)` on `ctx.toothBody`.

The proof function is `stepExtrudeTooth` and its assertion is `assertExtrudeTooth`. The extent is
modelled as a sweep of Thickness along the profile normal, which is the same sweep the end plane
defines, and the assertion checks the body has exactly one cap on the sketch plane and one a full
Thickness away — the fact that naming the end plane buys. It also checks the body's volume is the
section's area swept through Thickness, so a tooth extruded on the wrong profile or in the wrong
direction fails. The proof file records that decad refuses to record a profile boundary drawn with
fitted splines, so the flanks are chorded through the same involute samples, and what that
substitution costs.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeTooth, assertExtrudeTooth) -->

**From:** `spec/spurgear/instructions.md` L265, L286–296, L490–494;
`.claude/skills/generate-gear/PLAYBOOK.md` L151–158, L608–617

## S9 `[GO]` Extrude the body

Find the disc inside the root circle with `find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)`
— exactly two arcs, the two pieces the tooth cut the root circle into (`[PB-PROFILE-MATCH]`). Extrude it from the target
plane to the Extrusion End Plane as a new body with the same
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)` and
`adsk.fusion.ExtentDirections.PositiveExtentDirection`. Name the feature `Extrude body`, name the
resulting body `Gear Body`, and store it on `ctx.gearBody`.

While iterating `extrude.bodies.item(0).faces`, classify each face by `face.geometry.surfaceType` and
capture two references:

- The **`Gear Center` construction axis**, from the face whose type is
  `adsk.core.SurfaceTypes.CylinderSurfaceType`. Build it with
  `component.constructionAxes.createInput()`, then `axisInput.setByCircularFace(cylindricalFace)`,
  then `component.constructionAxes.add(axisInput)`. Name it `Gear Center`, set
  `centerAxis.isLightBulbOn = False`, and store it on `ctx.centerAxis`.
- **`ctx.extrusionExtent`**, the far end-cap face, found by surface type and then disambiguated
  against the sketch plane rather than by enumeration order, from the faces whose type is
  `adsk.core.SurfaceTypes.PlaneSurfaceType`: with `sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`,
  it is the face where `sketchPlane.isParallelToPlane(face.geometry)` is true and
  `sketchPlane.isCoPlanarTo(face.geometry)` is false. Raise when either reference is not found.

The proof function is `stepExtrudeBody` and its assertion is `assertExtrudeBody`. The assertion
checks the volume is the root disc's, so an annulus or a tip-circle disc fails; that the body has
exactly one cylindrical face and that its radius is the root radius, which is the face the axis is
built from; and that of the two planar faces one lies on the sketch plane and one a full Thickness
away, which is the parallel-but-not-coplanar test written as a measurement.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeBody, assertExtrudeBody) -->

**From:** `spec/spurgear/instructions.md` L262–264, L496–505;
`.claude/skills/generate-gear/PLAYBOOK.md` L676–697, L723–726

## S10 `[GO]` Circular-pattern the tooth

Pattern `ctx.toothBody` about the `Gear Center` axis. Copy the tooth body into an
`adsk.core.ObjectCollection.create()`, then
`component.features.circularPatternFeatures.createInput(bodyCollection, ctx.centerAxis)`, then set
`patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)`,
`patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')` and
`patternInput.isSymmetric = False` — all three explicitly, never left to Fusion's defaults
(`[PB-CIRCULAR-PATTERN]`) — and add it with
`component.features.circularPatternFeatures.add(patternInput)`. The bodies the pattern returns are
the seed plus the copies (`[PB-PATTERN-BODIES]`), which is what S11 consumes.

The proof function is `stepPatternTeeth` and its assertion is `assertPatternTeeth`. The assertion
checks the pattern leaves Tooth Number bodies, the seed among them, each with the seed's volume, and
each seated at a full turn divided by Tooth Number from the one before it — which is what pinning
the total angle to a full turn and the symmetry to false buys. The proof file records that both of
the tooth's arcs are chorded away for this step alone, because every copy would otherwise share the
root cylinder and the tip cylinder with its neighbours and the evaluator cannot decide such a pair.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepPatternTeeth, assertPatternTeeth) -->

**From:** `spec/spurgear/instructions.md` L507–511;
`.claude/skills/generate-gear/PLAYBOOK.md` L629–639

## S11 `[GO]` Combine the patterned teeth into the Gear Body

One Combine-Join. Feed the pattern's own `bodies` collection to the combine as it stands — it already
holds the seed tooth alongside the copies, so the seed is not added twice — copying its items into a
fresh `adsk.core.ObjectCollection.create()`, since a `BRepBodies` is not an `ObjectCollection` and the
combine input rejects it. Both halves of that are `[PB-PATTERN-BODIES]`. Then
`component.features.combineFeatures.createInput(ctx.gearBody, toolBodies)`, set
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`, and
`component.features.combineFeatures.add(combineInput)`.

The proof function is `stepCombineTeeth` and its assertion is `assertCombineTeeth`. The assertion
checks the joined gear's volume is the disc's plus Tooth Number times the tooth's, each measured on
the section that produced it, and the harness's solid gate checks the result is one watertight,
manifold lump with no voids — which is what a join is for. The proof file records that decad's boolean
union joins the disc and the first tooth and then refuses the second, so the join's result is drawn as
one closed loop and extruded once rather than booleaned, and says what that leaves unproven.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCombineTeeth, assertCombineTeeth) -->

**From:** `spec/spurgear/instructions.md` L509–511;
`.claude/skills/generate-gear/PLAYBOOK.md` L629–633

## S12 `[GO]` Root fillets

If `FilletRadius` is greater than zero, round the corner where each valley floor meets a tooth flank.
Collect **every** cylindrical face whose radius equals `RootCircleRadius`, not just the first —
after the pattern and combine the root cylinder is usually one patch per valley. On each such face
keep only the **axial straight** edges: filter to `adsk.core.Curve3DTypes.Line3DCurveType` — the
member names end in `…CurveType` and are easy to get wrong (`[PB-PROFILE-MATCH]`) — take each
line's direction from `edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)`, `direction.normalize()`,
and keep it when `abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01`, with `axisNormal` from
`get_normal(self.plane)`. The circular edges that wrap the front and back end caps are rims, not
structural root corners, and are dropped.

Apply the fillet with `component.features.filletFeatures.createInput()`, then
`filletInput.addConstantRadiusEdgeSet(edgeCollection, adsk.core.ValueInput.createByReal(filletRadius), False)`
on the input itself — the fillet side of the asymmetry in `[PB-FILLET-CHAMFER]`, which has no
edge-set collection — then `component.features.filletFeatures.add(filletInput)`. The tangent-chain flag
is `False`, so Fusion does not pull in tangent-adjacent edges and round more than the root corner. If
the edge collection is empty, return without creating the feature; an empty edge set must not reach
`add`.

The proof function is `stepFilletRoots` and its assertion is `assertFilletRoots`. The step asserts the
selection finds exactly two axial root corners per valley — two per tooth, and no end-cap rim, which
is the whole point of the axial filter — and the assertion checks that a positive radius adds material
at those corners and no more than the corners can hold, and that a Fillet Radius of zero leaves the
volume where it was. The case table derives Fillet Radius the way the spec does, from the tooth-space
arc at the root, and carries one case at zero for the skip branch.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepFilletRoots, assertFilletRoots) -->

<!-- check-step-calls: ignore getTangent -->

`getTangent` is named only to forbid it: parameter 0 is not guaranteed to lie inside an edge's
parameter range, so reading a direction that way raises at generation time.

**From:** `spec/spurgear/instructions.md` L86–88, L513–522;
`.claude/skills/generate-gear/PLAYBOOK.md` L151–158, L505–511, L608–617

## S13 `[GO]` Bore Profile sketch

Only on the full-build path and only when `BoreDiameter` is greater than zero. Create a sketch named
`Bore Profile` on the target plane, instantiate the tooth generator on it —
`SpurGearInvoluteToothDesignGenerator(boreSketch, self)` — and call
`toothGenerator.drawBore(ctx.anchorPoint, boreDiameter)`, which projects the anchor into this sketch
with `boreSketch.project(ctx.anchorPoint)`, draws a solid circle of that diameter centred on the
projection with
`boreSketch.sketchCurves.sketchCircles.addByCenterRadius(projectedAnchor, boreDiameter / 2)`, gives it
a driving diameter dimension through
`boreSketch.sketchDimensions.addDiameterDimension(boreCircle, textPoint)` (`[PB-DRIVING-DIM]`,
`[PB-RADIAL-DIM]`), and returns the circle. The projection this sketch draws on is the next link in
the chain back to the user's anchor (`[SPUR-F-ANCHOR-CHAIN]`).

The tooth generator's constructor always adds its local origin at (0, 0, 0)
(`[SPUR-F-LOCAL-ORIGIN]`), so this sketch carries one stray unused sketch point. Ground it on the
same projection with
`boreSketch.geometricConstraints.addCoincident(toothGenerator.anchorPoint, projectedAnchor)` — not on
`boreSketch.originPoint`, which pins it to the plane rather than to the gear and has been observed to
fail the solver (`[PB-CIRCLE-CENTER]`). Without any grounding the point is free in two directions and
the sketch never reaches full constraint (`[PB-FULL-CONSTRAINT]`, `[PB-PROJECT-NOT-FIXED]`).

The proof function is `stepBoreProfileSketch`. It draws the projection, the dimensioned bore circle and
the stray local origin grounded on that projection, at three bore diameters and at an anchor away from
the plane origin, and the harness gates the sketch on the same full verdict as the Gear Profile
sketch — so the grounding that this step exists to add is what the gate measures.

<!-- proof-run: proofkit.Run(boreCases, stepBoreProfileSketch) -->

**From:** `spec/spurgear/instructions.md` L356–361, L524–528;
`spec/spurgear/fusion.md` L26–31;
`.claude/skills/generate-gear/PLAYBOOK.md` L448–454

## S14 `[GO]` Bore cut

`buildBore(ctx)` runs unconditionally from `generate`, so it returns early in two cases: when
**SketchOnly** is set — because `buildMainGearBody` short-circuited before `buildBody` and
`ctx.gearBody` and `ctx.extrusionExtent` are never set — and when **Bore Diameter is not positive**.
Both guards are needed; the user may set both.

Otherwise extrude-cut the bore profile from the target plane to `ctx.extrusionExtent`, the far end-cap
face, so the bore goes all the way through whatever the thickness:
`component.features.extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
then `extrudeInput.setOneSideExtent(adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False), adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
then `extrudeInput.participantBodies = [ctx.gearBody]` so only the gear is cut, then
`component.features.extrudeFeatures.add(extrudeInput)`. The extent is the to-entity far face rather
than a distance, because that is what guarantees the bore pierces the gear whatever the thickness.

The proof function is `stepBoreCut` and its assertion is `assertBoreCut`. The assertion checks the cut
removes exactly a cylinder of the stated diameter through the full thickness, and that a Bore Diameter
of zero leaves the body untouched. The proof file records that decad refuses the cut once the teeth are
on the body, so it is made on the Gear Body as the body extrude leaves it — the bore is coaxial and
lies wholly inside the root circle, so it removes the same material either way — and says what the
substitution leaves unproven.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepBoreCut, assertBoreCut) -->

**From:** `spec/spurgear/instructions.md` L264, L294, L524–528;
`.claude/skills/generate-gear/PLAYBOOK.md` L665–668

## S15 `[GO]` Chamfer the completed gear

`chamferTeeth(ctx)` runs from `generate` after `buildBore`, so it sees the patterned, joined, filleted
gear and an optional bore. It returns in SketchOnly mode and when `ChamferTooth` is zero.

Walk every planar face of `ctx.gearBody` parallel to the Gear Profile sketch plane and add each of its
edges once, de-duplicated by `edge.tempId`. That set includes the tooth flanks, the tooth tops and the
root-radius arcs. Exclude only an `adsk.core.Curve3DTypes.Circle3DCurveType` edge — that exact member spelling,
per `[PB-PROFILE-MATCH]` — whose radius is the positive Bore Diameter divided by two, within
`0.001` cm, so a bore never receives a chamfer. Raise when
no end-cap face or no chamfer edge remains; do not create a partial chamfer.

Apply the set with `component.features.chamferFeatures.createInput2()`, then
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edgeCollection, adsk.core.ValueInput.createByReal(chamferDistance), False)`
on the input's edge-set collection — the chamfer side of the asymmetry in `[PB-FILLET-CHAMFER]`,
not the fillet's — then `component.features.chamferFeatures.add(chamferInput)`.

The proof function is `stepChamferGear` and its assertion is `assertChamferGear`. The step reads the
edge roster off the real toothed gear and asserts both end caps carry the same number of edges and that
the number is one per curve of the gear's cross-section, which is the "every edge of every end-cap face"
rule stated as a count. The assertion checks an equal-distance chamfer of the stated distance removes
the ring wedge it should, and that a zero distance removes nothing. The proof file records that the
feature itself is run on the Gear Body rather than on the toothed gear, because the harness will not
vouch for the chamfered toothed gear's volume, and that the bore-exclusion rule therefore goes unproven
here.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepChamferGear, assertChamferGear) -->

**From:** `spec/spurgear/instructions.md` L313–316, L530–545;
`.claude/skills/generate-gear/PLAYBOOK.md` L505–511, L608–617

## S16 `[PROSE]` Cleanup

`cleanup(ctx)` is the very last action of `generate`, after `chamferTeeth`, and it is called
unconditionally in both modes. It must not move up into `buildMainGearBody`, because `buildBore`
re-projects `ctx.anchorPoint` out of the Tools sketch and projection fails once that sketch is hidden.

The recipe is `[SPUR-F-CLEANUP]`: two kinds of entity, hidden with two different properties, and
only one of them per mode.

- **Always, in both modes**, switch off the light bulb on every construction plane and axis this build
  created — `ctx.extrusionEndPlane.isLightBulbOn = False`, the `Gear Center` axis, and the normalized
  target plane if S3 created one — so no stray plane is left floating. A construction plane is not
  hidden by `isVisible`; `isLightBulbOn` is the property for construction geometry and `isVisible`
  the one for sketches, and the two are never crossed (`[PB-HIDE-AFTER-USE]`).
- **Only on the full-build path**, set `isVisible = False` on the Tools, Gear Profile and Bore Profile
  sketches. Sketch-only mode leaves them visible, which is the point of that mode.

Guard each entity individually: the `Gear Center` axis and the Bore Profile sketch do not exist in
sketch-only mode.

Hiding changes no geometry, so there is nothing here a geometry proof can measure. What it protects —
that the Tools sketch is still visible when the bore projects from it — is a property of the call order
rather than of a shape.

**From:** `spec/spurgear/instructions.md` L239–241, L296–306, L488;
`spec/spurgear/fusion.md` L219–229;
`.claude/skills/generate-gear/PLAYBOOK.md` L595–607
