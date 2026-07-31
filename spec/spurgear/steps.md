# Spur Gear — compiled step list

Compiled from these sources, at these blobs:

| Source | Blob hash |
|---|---|
| `spec/spurgear/instructions.md` | `18c98a202e636173b2d042d2fe82582ed4607c03` |
| `spec/spurgear/fusion.md` | `5708cb98645e094a992e1cc5a79c9e24094ab0b8` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9297782f07cf4a68372a500eaa3b0e35a9d27091` |

One step is one entry in the Fusion timeline. A `[GO]` step is exercised by the proof in
`.tmp/spurgear_test.go` and names the function that runs it; a `[PROSE]` step is not, because the
proof engine models 2D sketches only and everything else here is solid modelling, visibility, or
control flow.

Calls marked **⚠ NOT IN THE API INDEX** are written as the spec writes them, unchanged. The index at
`~/.cache/fusion360-gear-generator/fusion-api-index.jsonl` has no such member, or gives it a
different signature. They are flagged, not corrected.

---

## S1 `[PROSE]` Read the Dialog and Register Parameters

No timeline entry of its own, but it decides the whole build and it has a fixed order.

Read the three selection inputs **first** — `parentComponent`, `plane`, `anchorPoint` — and stash
the entities on `self` before anything creates the occurrence. Creating the occurrence shifts
Fusion's active component context, and a selection holding an entity in another component can drop
at that moment. Numeric and boolean inputs are immune.

Then register the parameters, under the `SpurGear<N>_` prefix:

- From the inputs: `Module` (unitless `''`, **not** `'mm'`), `ToothNumber` (`''`), `PressureAngle`
  (`'rad'`), `BoreDiameter`, `Thickness`, `ChamferTooth` (`'mm'`), `SketchOnly` (real 1/0, because
  the framework reads booleans back as numbers through `getParameterAsBoolean`).
- Then the `addExtraPrimaryParameters(inputs)` hook, a no-op on the spur base and the seam where a
  subclass registers its own primary parameters.
- Then the derived ones, as live expression strings: `PitchCircleDiameter = Module * ToothNumber`,
  `PitchCircleRadius`, `BaseCircleDiameter = PitchCircleDiameter * cos(PressureAngle)`,
  `BaseCircleRadius`, `RootCircleDiameter = PitchCircleDiameter - 2.5 * Module`, `RootCircleRadius`,
  `TipCircleDiameter = PitchCircleDiameter + 2 * Module`, `TipCircleRadius`, `InvoluteSteps` (15),
  `FilletClearance` (0.9), `ToothSpaceArcAtRoot = RootCircleRadius * ToothSpaceAngleAtRoot`, and
  `FilletRadius = (ToothSpaceArcAtRoot / 2) * FilletClearance * <filletHelixFactorExpression()>`,
  which is `'1'` on the spur base.
- `ToothSpaceAngleAtRoot` is the one exception: Fusion's expression engine refuses to subtract a
  radian-valued `PressureAngle` from the unitless output of `tan()`, so compute
  `π / ToothNumber − 2 · (tan(PressureAngle) − PressureAngle)` in Python and register it with
  `ValueInput.createByReal(...)`, **unitless** (`''`), not `'rad'`. Registering it as `'rad'` makes
  the product in `ToothSpaceArcAtRoot` a `mm·rad` and Fusion rejects it.

Dialog display order is fixed and is not the read order: Target Plane, Anchor Point, Module, Tooth
Number, Pressure Angle, Bore Diameter, Thickness, chamfer, sketch-only, Parent Component **last**.

Calls: `commandInputs.addSelectionInput(id, name, commandPrompt)`,
`selectionInput.addSelectionFilter(filter)`, `selectionInput.setSelectionLimits(1, 1)`,
`commandInputs.addValueInput(id, name, unitType, ValueInput.createByReal(default))`,
`commandInputs.addStringValueInput(id, name, '0 mm')`,
`commandInputs.addBoolValueInput(id, name, True, '', False)`,
`adsk.core.ValueInput.createByReal(math.radians(20))`,
`adsk.core.ValueInput.createByString(expr)`, `design.userParameters.add(name, ValueInput, units, comment)`,
`parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`.

Defaults go in Fusion internal units regardless of the display unit string: Pressure Angle is
`addValueInput(…, 'deg', ValueInput.createByReal(math.radians(20)))` and Thickness is
`addValueInput(…, 'mm', ValueInput.createByReal(to_cm(10)))`.

The component is named from the parameters' `.expression` strings, not their values:
`'Spur Gear (M={}, Tooth={}, Thickness={})'`.

**From:** `instructions.md` 36–178 (Variables, exact ids, subclass seam), 229–296 (method contract),
367–377 (Generation Order); `PLAYBOOK.md` 100–136 (`[PB-INPUT-READ]`, `[PB-GET-VALUE-CONTRACT]`,
`[PB-DIALOG-DEFAULT-UNITS]`), 137–146 (`[PB-SELECTION-DECL]`), 196–228 (`processInputs` pattern),
474–479 (`[PB-SELECTION-FILTER-ENUM]`).

⚠ `addSelectionFilter` — `[PB-SELECTION-FILTER-ENUM]` says the argument is an int enum constant and
that a string raises `TypeError`. The index gives `addSelectionFilter(filter: str) -> bool`, and
every `SelectionCommandInput` filter member is itself a string (`ConstructionPlanes =
'ConstructionPlanes'`). The prescribed call still works; the stated reason for it does not.

---

## S2 `[PROSE]` Normalize the Target Plane

Runs only when the user's Target Plane selection is not already a `ConstructionPlane` — a planar
face, typically. Build a coplanar construction plane at zero offset and use that everywhere
downstream, so profile detection never has to filter out the selected face's own profile. Keep the
handle: S16 switches its light bulb off.

The offset is a `ValueInput`, not a bare number. `setByOffset(plane, 0)` is a runtime `TypeError`.

Calls: `component.constructionPlanes.createInput()`,
`planeInput.setByOffset(selectedPlane, adsk.core.ValueInput.createByReal(0))`,
`component.constructionPlanes.add(planeInput)`.

Also set `self.plane` to the result. Subclasses read `self.plane` directly, and `ctx.plane` holds
the same object.

**From:** `instructions.md` 39, 217, 226–227, 380–382; `PLAYBOOK.md` 230–240 (`generate`
orchestration), 674–685 (`[PB-CONSTRUCTION-PLANES]`).

---

## S3 `[GO]` Tools Sketch

Proof: `stepToolsSketch`.

A sketch named `Tools` on the target plane. It draws no geometry. Its whole job is to own one
thing — the projection of the user's Anchor Point — and that projection is `ctx.anchorPoint`, the
canonical handle every later sketch re-projects from. Chaining projections this way is what makes
the finished gear follow the anchor if the user moves it later.

Leave the sketch visible. S15 re-projects from it, and projection has failed on invisible sketches
in this repo's history, which is why S16 runs after the bore rather than inside the body build.

Calls: `component.sketches.add(self.plane)`, `sketch.name = 'Tools'`,
`sketch.project(self.anchorPoint)` ⚠, `sketch.isVisible = True`.

⚠ **NOT IN THE API INDEX:** `Sketch.project`. The index carries
`Sketch.project2(entities: list[core.Base], isLinked: bool) -> list[SketchEntity]`,
`Sketch.projectCutEdges` and `Sketch.projectToSurface`, and no plain `project`. `[SPUR-F-ANCHOR-CHAIN]`
and steps 2, 5 and 12 all name `sketch.project(...)`.

In the proof the projection is `CreateReferencePoint`, the engine's model of geometry located from
outside the sketch: the solver never moves it, and the sketch is complete with that one point and
nothing holding it.

**From:** `instructions.md` 193–197, 217–218, 384–386; `fusion.md` 19–31 (`[SPUR-F-ANCHOR-CHAIN]`,
`[SPUR-F-LOCAL-ORIGIN]`); `PLAYBOOK.md` 558–570 (`[PB-HIDE-AFTER-USE]`), 430–444
(`[PB-PROJECT-NOT-FIXED]`).

---

## S4 `[PROSE]` Extrusion End Plane

An offset construction plane named `Extrusion End Plane`, at `Thickness` from the target plane. It
exists so the tooth extrude and the body extrude both end on the same well-defined face. Keep it on
`ctx.extrusionEndPlane`; it must stay visible while those extrudes run and is hidden in S16 with
`isLightBulbOn = False`, never `isVisible = False`, which has no effect on construction geometry.

The offset is again a `ValueInput`, and it is a numeric snapshot of `Thickness` taken now, not a
live link.

Calls: `component.constructionPlanes.createInput()`,
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))`,
`component.constructionPlanes.add(planeInput)`, `plane.name = 'Extrusion End Plane'`.

**From:** `instructions.md` 219, 382, 387–388; `fusion.md` 199–206 (`[SPUR-F-SNAPSHOT]`);
`PLAYBOOK.md` 220–228 (`[PB-NUMERIC-SNAPSHOT]`), 674–685 (`[PB-CONSTRUCTION-PLANES]`).

---

## S5 `[GO]` Gear Profile Sketch

Proof: `stepGearProfileSketch`.

One sketch named `Gear Profile` on the target plane, and one timeline entry, so one step. Inside it
`SpurGearInvoluteToothDesignGenerator(sketch, self).draw(ctx.anchorPoint, angle)` runs three parts
in this order: the circles, the tooth, then the anchoring. The order is contractual — helical and
herringbone call `draw()` directly on their own twisted sketch and rely on that single call to
anchor it.

The constructor adds the **local origin** first: a fresh `SketchPoint` at (0, 0, 0), stored as
`self.anchorPoint`. Not `sketch.originPoint`, which is immutable and cannot be made coincident with
anything projected in. Everything below is drawn relative to it.

Store the sketch on `ctx.gearProfileSketch`, and copy `ctx.toothProfileIsEmbedded =
self._lastToothEmbedded` after `draw()` returns. `SpurGearGenerator.__init__` pre-initialises
`self._lastToothEmbedded = False` alongside `self.toolsSketch = None` and `self.boreSketch = None`.

Calls: `component.sketches.add(self.plane)`, `sketch.name = 'Gear Profile'`,
`sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`.

### Part 1 — `drawCircles`: the four gear circles

In this order: **Root Circle** at `RootCircleRadius`, solid; **Tip Circle** at `TipCircleRadius`,
construction; **Base Circle** at `BaseCircleRadius`, construction; **Pitch Circle** at
`PitchCircleRadius`, construction. Each gets a driving diameter dimension.

Pass the local-origin `SketchPoint` **directly** as the centre of all four, so they share it. Do
not pass `localOrigin.geometry` and then add a centre coincident: sharing and re-coincidenting the
same point is the single most common over-constraint failure in this repo, and piling redundant
coincidents onto this shared origin is what makes the solver fail outright.

Each circle also carries an along-path label reading
`'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)`, with `size = TipCircleRadius −
RootCircleRadius` used both inside the string and as the text height. The label is annotation; it
carries no constraint and the proof does not draw it.

Calls: `sketch.sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)`,
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` — the text point must be
off-centre, since at the centre there is no radial direction to place it —
`sketch.sketchTexts.createInput2(text, height)` ⚠,
`textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`,
`sketch.sketchTexts.add(textInput)`.

⚠ **NOT IN THE API INDEX:** `SketchTexts.createInput2`. The index carries
`SketchTexts.createInput3(expression: str, height: core.ValueInput) -> SketchTextInput` and no
`createInput2`. Note also that `createInput3` wants the height as a `core.ValueInput`, where both
`[PB-SKETCH-TEXT]` and step 3 pass a float in cm.

### Part 2 — `drawTooth(angle)`: one involute tooth, drawn at its final angle

`drawTooth` rotates by the `angle` argument that arrives from `draw()` at call time, **never** by
the constructor-stored `self.toothAngle`. Helical and herringbone construct the generator with the
default `angle=0` and then pass the helix angle to `draw()`; reading the stored field would draw a
flat tooth and leave the loft with no twist.

1. Sample `InvoluteSteps` points along the flank, endpoint-inclusive: sample `i` sits at radius
   `BaseCircleRadius + (TipCircleRadius − BaseCircleRadius) · i / (steps − 1)`, so the first is
   exactly on the base circle and the last exactly on the tip circle. Do not clamp the start to
   `max(base, root)` — the embedded case is detected in part 9 from where the flank start lands, not
   by trimming the sampling. Each sample is `calculateInvolutePoint(BaseCircleRadius, r)`:
   `alpha = acos(baseRadius / intersectionRadius)`, `t = tan(alpha)`,
   `x = baseRadius · (cos t + t · sin t)`, `y = baseRadius · (sin t − t · cos t)`. The parameter is
   `tan(alpha)`, not `inv(alpha) = tan(alpha) − alpha`. Drop samples that return `None`.
2. Mirror the samples across +X (negate y). The standard parametric involute spirals the wrong way
   for a left flank — its angular position grows with radius — and used directly it gives a tooth
   wider at the tip than at the root.
3. Compute the rotation analytically, not by interpolating the samples: with
   `(px, py) = calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`,
   `rotate_angle = π / (2 · ToothNumber) − atan2(−py, px)`. The `−py` is the part-2 mirror applied
   to the analytic point.
4. Rotate the mirrored samples by `rotate_angle` to get the **left** flank; mirror that across X for
   the **right**. Then rotate both flanks, the tooth-top point and the rib-midpoint seeds by the
   requested `angle`, in the same Python point math. Draw the tooth at its final angular position.
   Do not draw it flat and let the angular dimension swing it into place: Fusion then picks a branch
   that can be ~180° off, which is how the helical loft ends up passing through the gear centre.
5. Draw each flank as a fitted spline through its point collection.
6. Tooth-top arc. Add a tooth-top `SketchPoint` at
   `(TipCircleRadius · cos(angle), TipCircleRadius · sin(angle))`, coincident to the tip circle. Then
   create the arc by passing the local origin and the two flanks' **end** `SketchPoint`s directly,
   so all three are shared and no coincidence is needed. Add **no** diameter dimension: a free centre
   plus a diameter fixes the arc's size but not which way it bulges, and an arc of the same radius
   through the same two ends can curve back through the tooth. Sharing the centre removes the
   choice.
7. Spine and the +X reference. The spine is a construction line from the local origin to the
   tooth-top point, both passed directly. No separate start-coincident to the origin, and no
   constraint putting the spine's end on the arc. Then, for **every** angle including 0: add a far
   endpoint at `(TipCircleRadius, 0)` pinned by two signed dimensions from the origin — horizontal
   at `TipCircleRadius`, vertical at `0` — draw a construction line from origin to that endpoint,
   and add an angular dimension **from the reference to the spine, in that argument order**, with
   its text point on the bisector `(R·cos(angle/2), R·sin(angle/2))` so Fusion picks the angle and
   not its supplement. Do not use `addHorizontal` on the spine for the `angle = 0` case: horizontal
   fixes the line's direction but not which way it points, and the tooth can settle 180° around.
8. Ribs, one per fit-point index for all N indices, **endpoints included**. Each fit point carries
   no other constraint, so an omitted rib leaves it free. Per rib, in exactly this order:
   (a) create the construction line by passing the two fit `SketchPoint`s directly;
   (b) dimension it with a **signed** dimension — vertical at the measured Δy for `angle = 0`,
   otherwise whichever of horizontal/vertical is better conditioned for that angle — never an
   aligned one, which gives only a length and is satisfied just as well by the mirrored tooth;
   (c) add a midpoint `SketchPoint` seeded **on the spine**, at the foot of the left fit point:
   `t = fitX·cos(angle) + fitY·sin(angle)`, seed `(t·cos(angle), t·sin(angle))` — not the rib's true
   2-D midpoint, and not `(fitX, 0)` for a rotated tooth;
   (d) coincident that point to the spine; (e) make it the rib's midpoint; (f) make the rib
   perpendicular to the spine — **skipped on the last rib**, where the tooth-top arc's shared centre
   already holds both flank ends at equal radius either side of the spine, so the perpendicular adds
   nothing and Fusion rejects it as over-constraining.
   Then chain the midpoints: a signed dimension along the spine direction from the previous
   midpoint, starting with the **local origin** as the previous one. Without that first link the
   whole chain slides along the spine as a unit and the sketch never closes.
9. Close the tooth at the root. If the flank's first point lies **outside** the root circle, draw a
   short radial line on each side from the root circle up to the flank start, passing the flank
   spline's start `SketchPoint` directly, and place the root end with **exactly two** signed
   dimensions from the local origin — horizontal at its Δx, vertical at its Δy — and nothing else.
   "Root end on the root circle" plus "origin on the line" would be satisfied by the far side of the
   gear as well, and the stub becomes a line straight across the centre. The tooth loop is then
   **6 curves**: 2 splines, 2 flank-to-root lines, 2 arcs. If the flank starts **inside** the root
   circle, draw no line; the loop is **4 curves**, 2 splines and 2 arcs, and the profile is
   embedded. The test is strict `<` on raw values, so exact equality counts as **not** embedded and
   draws a zero-length stub. Record the answer on `self.parent._lastToothEmbedded`.

Calls: `sketch.sketchCurves.sketchFittedSplines.add(pointCollection)` with the collection built by
`adsk.core.ObjectCollection.create()`, `sketch.sketchPoints.add(adsk.core.Point3D.create(x, y, 0))`,
`sketch.geometricConstraints.addCoincident(toothTopPoint, tipCircle)`,
`sketch.sketchCurves.sketchArcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)`,
`sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, toothTopPoint)`,
`line.isConstruction = True`,
`sketch.sketchDimensions.addDistanceDimension(localOrigin, xEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`,
`sketch.sketchDimensions.addDistanceDimension(localOrigin, xEnd, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, textPoint)`,
`sketch.sketchDimensions.addAngularDimension(referenceLine, spine, textPoint)`,
`sketch.geometricConstraints.addCoincident(midPoint, spine)`,
`sketch.geometricConstraints.addMidPoint(midPoint, rib)`,
`sketch.geometricConstraints.addPerpendicular(spine, rib)`,
`find_circle_by_radius(sketch, radius)` from `.utilities` when a circle handle is needed rather than
kept from part 1. Every dimension is driving; never pass `isDriven=True`.

### Part 3 — the anchoring

Project the Tools-sketch anchor into this sketch, then add one coincidence between that fresh
projection and the local origin — not `sketch.originPoint`. Every piece of geometry above hangs off
the local origin, so this one constraint drags the whole tooth onto the user's anchor as a unit.

Then, as the very last action of `draw()` and only when `angle != 0`, set the spine's angular
dimension value to `angle`. Drawing the rotation and confirming it are two distinct, both-required
actions: the pre-rotation puts the geometry on the right solver branch, and the dimension locks it
there.

Calls: `sketch.project(ctx.anchorPoint)` ⚠ (see S3),
`sketch.geometricConstraints.addCoincident(projectedPoint, localOrigin)`,
`spineAngularDimension.parameter.value = angle`.

**From:** `instructions.md` 180–210 (Sketch Discipline), 212–227 (context fields), 265–295 (method
contract), 297–338 (tooth-generator surface and the involute math), 390–443 (steps 3, 4 and 5);
`fusion.md` 17–43 (`[SPUR-F-ANCHOR-CHAIN]`, `[SPUR-F-LOCAL-ORIGIN]`, `[SPUR-F-SHARED-ADJACENCY]`),
45–60 (`[SPUR-F-ROTATE-CONFIRM]`), 69–88 (`[SPUR-F-TOOTHTOP-ARC]`), 90–112 (`[SPUR-F-SPINE]`),
114–147 (`[SPUR-F-RIBS]`), 149–183 (`[SPUR-F-FLANK-ROOT]`); `PLAYBOOK.md` 336–403
(`[PB-SKETCH-FIRST]` and the Fusion→engine constraint mapping), 413–450 (`[PB-FULL-CONSTRAINT]`,
`[PB-REFLINE-DIRECTION]`, `[PB-NO-OVERCONSTRAIN]`), 458–473 (`[PB-API-SPELLING]`,
`[PB-SKETCHCURVES]`), 517–555 (`[PB-SEED-NEAR]`, `[PB-SHARE-XOR-COINCIDENT]`, `[PB-DRIVING-DIM]`,
`[PB-ANGULAR-DIM]`, `[PB-RADIAL-DIM]`), 581–591 (`[PB-SKETCH-TEXT]`).

---

## S6 `[PROSE]` Sketch-Only Short-Circuit

Not a timeline entry, a branch. If `SketchOnly` is true, make the Gear Profile sketch visible and
stop: no tooth extrude, no body extrude, no pattern, no fillet, no chamfer, no bore. S16 still runs,
and leaves the sketches visible in this mode — inspecting the involute construction is the point of
it.

Calls: `self.getParameterAsBoolean(PARAM_SKETCH_ONLY)`, `ctx.gearProfileSketch.isVisible = True`.

**From:** `instructions.md` 60, 147–150, 246–253, 445–447; `fusion.md` 185–197 (`[SPUR-F-CLEANUP]`).

---

## S7 `[PROSE]` Extrude the Tooth

Find the tooth cross-section by curve counts, not by index, and use the framework helper rather than
a hand-rolled loop search — it raises when nothing matches instead of falling back to a wrong
profile. The counts come straight from S5's ninth part.

Extrude from the target plane to the Extrusion End Plane as a **New Body**, name the feature
`Extrude tooth`, and store the body on `ctx.toothBody`. `buildTooth` must call `self.chamferTooth(ctx)`
as its last action, so S8 is triggered from inside this step, not separately from `buildMainGearBody`.

Calls: `find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`
from `.utilities`, `component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`component.features.extrudeFeatures.add(extrudeInput)`, `extrude.name = 'Extrude tooth'`.

**From:** `instructions.md` 223–224, 265–274, 449–453; `PLAYBOOK.md` 151–156 (helper contract),
571–580 (`[PB-PROFILE-MATCH]`).

---

## S8 `[PROSE]` Chamfer the Tooth Front Face

Runs only when `ChamferTooth > 0`, and is called from inside S7.

Find the front face with a **single conjunction predicate** over `ctx.toothBody.faces`: the first
face for which `face.edges.count == chamferWantEdges()` (6 on the spur base) **and**
`sketchPlane.isCoPlanarTo(face.geometry)`, with `sketchPlane =
ctx.gearProfileSketch.referencePlane.geometry`. Both of the same face. Not an edge-count match with
a coplanarity tiebreak, and no partial-match fallback: if no face satisfies both, raise.

Then chamfer every edge of that face **except** the root arc, identified by radius rather than by
relative size: skip any edge whose `geometry.curveType` is `Arc3DCurveType` and whose
`geometry.radius` matches `RootCircleRadius` within `0.001` cm. Chamfering the root arc would eat
into the neighbouring tooth. The flanks, the tooth-top arc and both flank-to-root lines are
chamfered.

Known, accepted limitation: an embedded profile gives a 4-edge front face while `chamferWantEdges()`
stays 6, so chamfering an embedded spur tooth raises front-face-not-found. Users turn the chamfer off
for such gears. Helical and herringbone override only that count.

Calls: `component.features.chamferFeatures.createInput2()`,
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, ValueInput.createByReal(chamfer), False)` ⚠,
`component.features.chamferFeatures.add(chamferInput)`, `adsk.core.ObjectCollection.create()`,
`adsk.core.Curve3DTypes.Arc3DCurveType`.

⚠ Signature mismatch: the index gives
`addEqualDistanceChamferEdgeSet(edges: core.ObjectCollection, distance: core.ValueInput, isTangentChain: bool)`.
`[PB-FILLET-CHAMFER]` calls the third argument `isFlipped`; it is `isTangentChain`. Step 8 also
writes the second argument as `<ChamferTooth value>`, a number, where a `core.ValueInput` is
required.

**From:** `instructions.md` 58, 275–282, 455–461; `PLAYBOOK.md` 480–486 (`[PB-FILLET-CHAMFER]`),
571–580 (`[PB-PROFILE-MATCH]` for the curve-type constant names).

---

## S9 `[PROSE]` Extrude the Gear Body

Find the body profile as the loop bounded by exactly 2 arcs, extrude it from the target plane to the
Extrusion End Plane as a **New Body**, name the feature `Extrude body` and the body `Gear Body`, and
store it on `ctx.gearBody`.

While iterating `extrude.bodies.item(0).faces`, capture `ctx.extrusionExtent`: among the faces whose
`geometry.surfaceType` is `PlaneSurfaceType`, the one parallel to but not coplanar with the gear's
sketch plane. Use the plane-geometry API, not a hand-rolled dot product. The near cap is coplanar, so
`isCoPlanarTo` rules it out. Raise if it is not found.

Calls: `find_profile_by_curve_counts(sketch, arcs=2)` from `.utilities`,
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`component.features.extrudeFeatures.add(extrudeInput)`, `body.name = 'Gear Body'`,
`adsk.core.SurfaceTypes.PlaneSurfaceType`, `sketchPlane.isParallelToPlane(face.geometry)`,
`sketchPlane.isCoPlanarTo(face.geometry)`.

⚠ The spec calls this "the annular loop bounded by **exactly 2 arcs** (the root circle and the tip
circle)". The tip circle is construction (S5 part 1), so it is excluded from profile detection and
there is no annulus. The loop the search actually matches is the **root disk**, bounded by the two
arcs the root circle is split into — by the flank-to-root stub feet in the ordinary case, and by the
flanks' own crossings in the embedded one. The count `arcs=2` holds in both branches; the
parenthetical naming the circles does not. The proof confirms it from the other side: the root
circle carries a second detected region beside the tooth in every non-degenerate case.

**From:** `instructions.md` 220–223, 463–472; `PLAYBOOK.md` 151–156, 639–660 (`[PB-FACE-BY-MIDPOINT]`
for the surface-type test), 571–580 (`[PB-PROFILE-MATCH]`).

---

## S10 `[PROSE]` Gear Center Construction Axis

Off any face of the new body whose `geometry.surfaceType` is `CylinderSurfaceType`. Name it
`Gear Center`, set `isLightBulbOn = False`, store on `ctx.centerAxis`. Raise if no cylindrical face
is found. S11 patterns around it.

Calls: `component.constructionAxes.createInput()`,
`axisInput.setByCircularFace(cylindricalFace)`, `component.constructionAxes.add(axisInput)`,
`axis.name = 'Gear Center'`, `axis.isLightBulbOn = False`,
`adsk.core.SurfaceTypes.CylinderSurfaceType`.

**From:** `instructions.md` 222, 467–471; `PLAYBOOK.md` 686–689 (`[PB-CONSTRUCTION-AXES]`), 558–570
(`[PB-HIDE-AFTER-USE]`).

---

## S11 `[PROSE]` Pattern the Teeth

Circular-pattern `ctx.toothBody` about the `Gear Center` axis, quantity `ToothNumber`. Pin all three
inputs explicitly rather than trusting defaults: quantity, `totalAngle` as the string expression
`'360 deg'`, and `isSymmetric = False`.

Calls: `adsk.core.ObjectCollection.create()`,
`component.features.circularPatternFeatures.createInput(bodyCollection, ctx.centerAxis)`,
`patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)`,
`patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')`,
`patternInput.isSymmetric = False`,
`component.features.circularPatternFeatures.add(patternInput)`.

**From:** `instructions.md` 474–478; `PLAYBOOK.md` 597–602 (`[PB-CIRCULAR-PATTERN]`).

---

## S12 `[PROSE]` Combine the Teeth into the Gear Body

One Combine-Join. Feed the pattern's `bodies` collection as-is — it already includes the original
tooth body, so do not re-add the seed — but copy it into a fresh `ObjectCollection` first, because
`pattern.bodies` is a `BRepBodies` and `combineFeatures.createInput` rejects that type.
`patternTeeth` calls `self.createFillets(ctx)` after this.

Calls: `adsk.core.ObjectCollection.create()`, `pattern.bodies.item(i)`,
`component.features.combineFeatures.createInput(ctx.gearBody, toolBodies)`,
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`,
`component.features.combineFeatures.add(combineInput)`.

**From:** `instructions.md` 250–251, 476–478; `PLAYBOOK.md` 592–596 (`[PB-PATTERN-BODIES]`).

---

## S13 `[PROSE]` Root Fillets

Runs when `FilletRadius > 0`. `createFillets` reads only the registered `FilletRadius` parameter's
numeric `.value`; `filletHelixFactorExpression()` was already consumed back in S1, spliced into that
parameter's expression, and is **not** read here.

The target is the sharp inside corner where the root valley floor meets each tooth flank, running
the full thickness parallel to the gear's main axis — the structurally important one, not the
front/back rim.

Collect **every** cylindrical face whose radius matches `RootCircleRadius`: after the pattern and
combine, the root cylinder is usually split into one patch per valley rather than one continuous
surface. On each, filter to `Line3DCurveType` edges, take each line's direction from its **geometry
endpoints**, normalize, and keep it when `abs(abs(dot(direction, axisNormal)) - 1.0) < 0.01`. Use
exactly that tolerance; a tighter test drops valid axial edges that are slightly off from
tessellation and leaves root fillets missing. Do not read the direction via
`edge.evaluator.getTangent(0)` — parameter `0` is not guaranteed to lie inside the edge's parameter
range and Fusion raises.

`isTangentChain` must be `False`: the collected edges are exactly the axial root corners, and
tangent-chaining would let Fusion pull in adjacent edges and round more than intended.

If the edge collection ends up empty, return silently without creating the feature. An empty edge
set must not reach `filletFeatures.add`.

Calls: `component.features.filletFeatures.createInput()`,
`filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), False)` ⚠,
`component.features.filletFeatures.add(filletInput)`, `adsk.core.ObjectCollection.create()`,
`adsk.core.Curve3DTypes.Line3DCurveType`, `adsk.core.SurfaceTypes.CylinderSurfaceType`,
`get_normal(self.plane)` from `.utilities`.

⚠ **NOT IN THE API INDEX:** `FilletFeatureInput.addConstantRadiusEdgeSet`. The index gives
`FilletFeatureInput` exactly three members — `isRollingBallCorner`, `targetBaseFeature` and
`edgeSetInputs() -> FilletEdgeSetInputs` — and puts `addConstantRadiusEdgeSet(entities:
core.ObjectCollection, radius: core.ValueInput, isTangentChain: bool)` on `FilletEdgeSetInputs` and
on `FilletEdgeSets`. Step 11 and `[PB-FILLET-CHAMFER]` both insist on the opposite: add the edge set
on the input itself, and specifically **do not** route it through `filletInput.edgeSetInputs`,
"that is the chamfer-side shape ... and reaching for it on the fillet input fails". Per the index the
instruction is backwards, and `filletInput.edgeSetInputs.addConstantRadiusEdgeSet(...)` is the call
that exists.

**From:** `instructions.md` 86–88, 275–282, 480–489; `PLAYBOOK.md` 480–486 (`[PB-FILLET-CHAMFER]`),
571–580 (`[PB-PROFILE-MATCH]`), 412 (`[PB-EMPTY-RESULT]`).

---

## S14 `[PROSE]` Bore Profile Sketch

Runs from `buildBore`, which is called unconditionally from `generate()` and must itself early-return
in **two** cases: when `SketchOnly` is set, and when `BoreDiameter <= 0`. The SketchOnly guard is not
optional — in that mode S9 never ran, so `ctx.gearBody` and `ctx.extrusionExtent` are `None` and the
cut would dereference them. Do not lean on the bore diameter being 0 in sketch-only mode; the user
may have set both.

Create a sketch named `Bore Profile` on the target plane and draw the circle by instantiating the
tooth generator on it and calling `drawBore(ctx.anchorPoint, boreDiameter)`, which projects the
anchor in, draws a non-construction circle of that diameter centred on the projection, gives it a
driving diameter dimension, and returns it.

Accepted side effect: the tooth generator's constructor always adds its local-origin `(0, 0, 0)`
`SketchPoint`, so this sketch carries one stray unused point. The spec calls that faithful behaviour
and forbids suppressing it.

Calls: `component.sketches.add(self.plane)`, `sketch.name = 'Bore Profile'`,
`sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`, `sketch.project(ctx.anchorPoint)` ⚠
(see S3), `sketch.sketchCurves.sketchCircles.addByCenterRadius(projectedPoint, diameter / 2)`,
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`.

This step is `[PROSE]` and not `[GO]` even though it is a sketch, because as specified it cannot
pass the proof's gate. The stray origin point carries two free degrees of freedom and nothing
constrains it, and `[PB-PROJECT-NOT-FIXED]` says a projection is not fixed either, so the sketch is
under-constrained on two counts — against `[PB-FULL-CONSTRAINT]`, which the Sketch Discipline section
applies to "every sketch created below". Either the spec should constrain the bore circle's centre
and drop the stray point, or it should state the exemption; it does neither.

**From:** `instructions.md` 180–184 (Sketch Discipline preamble), 316–321 (`drawBore` surface),
491–495; `fusion.md` 19–31 (`[SPUR-F-ANCHOR-CHAIN]`, `[SPUR-F-LOCAL-ORIGIN]`); `PLAYBOOK.md`
413–444 (`[PB-FULL-CONSTRAINT]`, `[PB-PROJECT-NOT-FIXED]`).

---

## S15 `[PROSE]` Bore Cut

Extrude-cut the bore profile from the target plane to `ctx.extrusionExtent`, the far end-cap face
captured in S9, affecting only `ctx.gearBody`. Cutting to that face is what guarantees the bore goes
all the way through whatever the Thickness is.

Calls: `component.features.extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`extrudeInput.participantBodies = [ctx.gearBody]`,
`component.features.extrudeFeatures.add(extrudeInput)`.

**From:** `instructions.md` 54, 220–223, 491–495.

---

## S16 `[PROSE]` Cleanup

The very last action of `generate()`, after S15 — not inside `buildMainGearBody` — and called
**unconditionally** in both modes. The mode split lives inside it, never on the call. Placement
matters: S14 re-projects `ctx.anchorPoint` out of the Tools sketch, and projection fails once that
sketch is hidden.

Split by entity kind:

- Construction planes and axes — the Extrusion End Plane, the `Gear Center` axis, and the normalized
  target plane if S2 created one — get `isLightBulbOn = False`, **always, in both modes**, so no
  stray plane is left floating in sketch-only mode.
- The Tools, Gear Profile and Bore Profile sketches get `isVisible = False`, **only on the
  full-build path**. Sketch-only mode leaves them visible; inspecting them is the point of it.

Never cross the two properties: `isVisible = False` has no effect on construction geometry. Guard
each entity individually — the `Gear Center` axis and the Bore Profile sketch do not exist in
sketch-only mode.

Calls: `constructionPlane.isLightBulbOn = False`, `constructionAxis.isLightBulbOn = False`,
`sketch.isVisible = False`.

**From:** `instructions.md` 202–205, 252–263, 445–447; `fusion.md` 185–197 (`[SPUR-F-CLEANUP]`);
`PLAYBOOK.md` 558–570 (`[PB-HIDE-AFTER-USE]`).
