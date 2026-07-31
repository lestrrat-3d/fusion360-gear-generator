# Spur Gear — compiled step list

Compiled from the sources below. Nothing else was consulted: no existing implementation, no
earlier proof.

| Source | Blob hash |
|---|---|
| `spec/spurgear/instructions.md` | `c7f80cac009d9876c0e535ddf387d240b90294a9` |
| `spec/spurgear/fusion.md` | `25c648723c97cd52ffce75d656bbc507860d2c2d` |

Supporting documents these two cite by name, read in full and cited per step by line range:
`.claude/skills/generate-gear/PLAYBOOK.md` (the `[PB-…]` anchors).

## How to read this

One step is one entry in the Fusion timeline. A whole sketch is one step however much geometry
goes into it, and so is each construction plane, construction axis, extrude, chamfer, pattern,
combine, fillet and cut. Two steps below (S6, S16) create no timeline entry at all; they are
mode gates the build cannot be described without, and they say so.

`[GO]` marks a step the proof in `.tmp/spurgear_test.go` exercises, and names the function that
does it. Everything else is `[PROSE]`. The proof engine models 2D sketches only, so every 3D
step is `[PROSE]` by construction; one sketch step is `[PROSE]` for a reason of its own, given
at S14.

Every `adsk.*` name below was looked up in `~/.cache/fusion360-gear-generator/fusion-api-index.jsonl`
before it was written, per `[PB-API-LOOKUP]`. Three calls the sources name are not in that index.
They are written here as the sources name them and flagged `⚠ NOT IN API INDEX` where they occur,
rather than quietly swapped for something that is.

---

## S1 `[PROSE]` Read the Dialog and Create the Gear Component

The command dialog is built by `SpurGearCommandInputsConfigurator.configure(cmd)`, a
`@classmethod`, which adds its inputs in exactly this display order: Target Plane (`plane`),
Anchor Point (`anchorPoint`), Module (`module`), Tooth Number (`toothNumber`), Pressure Angle
(`pressureAngle`), Bore Diameter (`boreDiameter`), Thickness (`thickness`), Apply chamfer to
teeth (`chamferTooth`), Generate sketches but do not build body (`sketchOnly`), Parent Component
(`parentComponent`) last. Do not reorder by input type. This display order is independent of the
`processInputs` read order below; a configurator that puts the selections last because they are
read first has the rule backwards.

Bore Diameter is a string value input (default `'0 mm'`) so it accepts expressions. The rest are
numeric value inputs whose defaults are given in Fusion internal units regardless of the display
unit string: Pressure Angle is `'deg'` with `createByReal(math.radians(20))`, Thickness is `'mm'`
with `createByReal(to_cm(10))`, Module `createByReal(1)`, Tooth Number `createByReal(17)`,
chamfer `createByReal(0)`. Each of the three selection inputs takes `setSelectionLimits(1, 1)`
and exactly its declared filters: Target Plane `ConstructionPlanes` + `PlanarFaces`; Anchor Point
`ConstructionPoints` + `SketchPoints`; Parent Component `Occurrences` + `RootComponents`,
pre-selecting `get_design().rootComponent`.

`generate(inputs)` then runs `processInputs(inputs)`. Order is load-bearing: creating the
occurrence shifts Fusion's active component context, and a `SelectionCommandInput` holding an
entity that lives in another component can drop its selection when that happens. So read all
three selections first and stash them on `self`, before anything touches the design. Only then
register parameters — `addParameter` reaches `getOccurrence()` transitively, which is what
creates the occurrence and the timeline's New Component entry.

Register the input-sourced parameters (`Module` unitless `''`, `ToothNumber` `''`,
`PressureAngle` `'rad'`, `BoreDiameter` `'mm'`, `Thickness` `'mm'`, `ChamferTooth` `'mm'`,
`SketchOnly` as a real-valued 1/0), then call the `addExtraPrimaryParameters(inputs)` hook — a
no-op on the spur base, present so subclasses have a call site — then the derived parameters as
live expression strings: `PitchCircleDiameter = Module * ToothNumber`, `PitchCircleRadius`,
`BaseCircleDiameter = PitchCircleDiameter * cos(PressureAngle)`, `BaseCircleRadius`,
`RootCircleDiameter = PitchCircleDiameter - 2.5 * Module`, `RootCircleRadius`,
`TipCircleDiameter = PitchCircleDiameter + 2 * Module`, `TipCircleRadius`, `InvoluteSteps` 15,
`ToothSpaceAngleAtRoot`, `ToothSpaceArcAtRoot = RootCircleRadius * ToothSpaceAngleAtRoot`,
`FilletClearance` 0.9, `FilletRadius = (ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>`
where `<factor>` is `filletHelixFactorExpression()` — `'1'` on the spur base. `Module` must be
registered unitless, not `'mm'`, or `generateName` renders a unit suffix and the derived `mm`
expressions read a length where they want a factor. `ToothSpaceAngleAtRoot` is the one parameter
Fusion's expression engine cannot evaluate — it subtracts a unitless `tan()` result from a radian
value — so compute `π / ToothNumber − 2 · (tan(PressureAngle) − PressureAngle)` in Python and
register it with `createByReal`, unitless, not `'rad'`: the next parameter multiplies it by a
length and Fusion rejects a `mm·rad` product.

Name the component `'Spur Gear (M={}, Tooth={}, Thickness={})'` from the three parameters'
`.expression` strings, not their `.value`.

**Fusion API calls:** `cmd.commandInputs.addSelectionInput(id, name, commandPrompt)` ·
`selectionInput.addSelectionFilter(filter)` · `selectionInput.setSelectionLimits(1, 1)` ·
`cmd.commandInputs.addValueInput(id, name, unitType, initialValue)` ·
`cmd.commandInputs.addStringValueInput(id, name, '0 mm')` ·
`cmd.commandInputs.addBoolValueInput(id, name, True)` ·
`adsk.core.ValueInput.createByReal(realValue)` · `adsk.core.ValueInput.createByString(stringValue)` ·
`adsk.core.SelectionCommandInput.ConstructionPlanes` (and `PlanarFaces`, `ConstructionPoints`,
`SketchPoints`, `Occurrences`, `RootComponents`) ·
`parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` ·
`occurrence.component` · `component.name = generateName()` ·
`design.userParameters.add(name, value, units, comment)` · `design.rootComponent`

**From:** `spec/spurgear/instructions.md` 9–11, 14–33, 36–88, 90–178, 212–227, 229–295, 340–376;
`.claude/skills/generate-gear/PLAYBOOK.md` 42–73, 75–98, 103–118, 128–143, 196–228, 230–241,
474–479, 556–557

---

## S2 `[PROSE]` Normalize the Target Plane

If the user's Target Plane selection is already a `ConstructionPlane`, use it as is and create
nothing. Otherwise — a planar face, say — build a coplanar construction plane at offset zero and
use that for every sketch and plane below, so profile detection downstream never has to filter
out the selected face's own profile.

The offset argument is a `ValueInput`, not a bare number; `setByOffset(plane, 0)` is a runtime
`TypeError`. Hold the result on both `self.plane` and `ctx.plane` — subclasses read `self.plane`
directly. If this step created a plane, its light bulb is switched off in S16.

**Fusion API calls:** `component.constructionPlanes.createInput()` ·
`planeInput.setByOffset(planarEntity, adsk.core.ValueInput.createByReal(0))` ·
`component.constructionPlanes.add(planeInput)` · `constructionPlane.isLightBulbOn = False`

**From:** `spec/spurgear/instructions.md` 39, 229–254, 380–382;
`.claude/skills/generate-gear/PLAYBOOK.md` 230–241, 671–682

---

## S3 `[GO]` Tools Sketch

Create a sketch named `Tools` on the target plane and project the user's Anchor Point into it.
Keep the projection as `ctx.anchorPoint`. That one reference is the entire content of the sketch —
it draws no geometry of its own.

This projection is the canonical handle. Every later sketch that has to follow the anchor projects
*this* point in again rather than the user's original entity, so the whole gear hangs off one
chain of projections back to the anchor and moves with it. A sketch cannot reference another
sketch's points directly, which is why the chain exists at all.

Leave the sketch visible. It stays visible through S15, because S14 projects from it again and
projection has failed on invisible sketches in this repo's history. S16 hides it, and only on the
full-build path.

**Fusion API calls:** `component.sketches.add(planarEntity)` · `sketch.name = 'Tools'` ·
`sketch.project(entity)` ⚠ NOT IN API INDEX · `sketch.isVisible = False`

**From:** `spec/spurgear/instructions.md` 41, 180–210, 212–227, 384–386;
`spec/spurgear/fusion.md` 19–24; `.claude/skills/generate-gear/PLAYBOOK.md` 558–570

**Proof:** `stepToolsSketch`

---

## S4 `[PROSE]` Extrusion End Plane

Create a construction plane named `Extrusion End Plane`, offset from the target plane by
`Thickness`. Its only job is to be the to-entity target for the tooth extrude (S7) and the body
extrude (S9), so both end on the same well-defined face.

The offset is `ValueInput.createByReal(thickness)` — again a `ValueInput`, not a number. Keep the
handle as `ctx.extrusionEndPlane`. Leave it visible while those two extrudes run; S16 switches its
light bulb off. `isVisible = False` does not hide a construction plane.

**Fusion API calls:** `component.constructionPlanes.createInput()` ·
`planeInput.setByOffset(planarEntity, adsk.core.ValueInput.createByReal(thickness))` ·
`component.constructionPlanes.add(planeInput)` · `constructionPlane.name = 'Extrusion End Plane'` ·
`constructionPlane.isLightBulbOn = False`

**From:** `spec/spurgear/instructions.md` 212–227, 384–388;
`.claude/skills/generate-gear/PLAYBOOK.md` 558–570, 671–682

---

## S5 `[GO]` Gear Profile Sketch

Create a sketch named `Gear Profile` on the target plane and hand it to
`SpurGearInvoluteToothDesignGenerator(sketch, self)`. Its `draw(ctx.anchorPoint, angle=0)` does
everything below, in this order: `drawCircles()`, `drawTooth(angle)`, the anchoring, and — only
when `angle != 0` — the confirming angular dimension's value, set as the very last action after
the whole constraint network exists. All of it lands in one timeline entry, because it is one
sketch.

The generator's constructor adds the sketch's **local origin**: a fresh `SketchPoint` at
(0, 0, 0), held on the field `self.anchorPoint`. Not `sketch.originPoint`, which is immutable and
cannot be made coincident with anything brought in from elsewhere. Every piece of geometry below
is placed relative to this point, and the anchoring at the end slides the lot onto the user's
anchor as a unit.

**The four circles.** In order: Root Circle at `RootCircleRadius` **solid**, then Tip Circle,
Base Circle and Pitch Circle, all three **construction**, at their radii. Pass the local origin
`SketchPoint` **directly** as the centre of each so all four share it — not `localOrigin.geometry`
followed by a coincident, which double-binds the point and kills the solver. Give each a driving
diameter dimension; never pass `isDriven=True`. Each circle also carries along-path sketch text
reading `'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` from the radii's internal `.value`
in cm, where `size = TipCircleRadius − RootCircleRadius`, and that same `size` is the text height.

**The flank samples.** Sample `InvoluteSteps` points along the involute, endpoint-inclusive:
sample `i` sits at `r = BaseCircleRadius + (TipCircleRadius − BaseCircleRadius) · i / (steps − 1)`,
so the first is exactly on the base circle and the last exactly on the tip circle. Do not clamp
the start to `max(base, root)` — the flank is sampled from the base circle even when the base
circle sits inside the root circle. Each sample is `calculateInvolutePoint(BaseCircleRadius, r)`:
`alpha = acos(baseRadius / intersectionRadius)`, `t = tan(alpha)`, `x = baseRadius·(cos t + t·sin t)`,
`y = baseRadius·(sin t − t·cos t)`, returning `None` below the base circle. The curve parameter is
`tan(alpha)`, not `inv(alpha) = tan(alpha) − alpha`. Drop the `None` samples.

**Mirror, then rotate.** Negate every sample's y first. The standard parametric involute spirals
the wrong way for a left flank — its angular position grows with radius, which gives a tooth wider
at the tip than at the root. Then rotate by
`rotate_angle = π / (2 · ToothNumber) − atan2(−py, px)`, where `(px, py) =
calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`. The `−py` is the mirror applied to
the analytic point; `atan2(py, px)` is the wrong sign. Computing the pitch crossing analytically
rather than interpolating between samples puts the tooth at the right angle however few samples
were taken. Rotating gives the **left** flank; mirroring that across X gives the **right** flank.
Finally rotate both collections by `draw()`'s runtime `angle` argument — and by that argument, never
by the constructor-stored `self.toothAngle`, which helical and herringbone leave at 0 while passing
a non-zero `angle` to `draw()`. For the spur base `angle` is 0 and this is a no-op.

**The flanks.** Two `SketchFittedSpline`s through the two point collections.

**The tooth-top arc.** Materialize a tooth-top `SketchPoint` at
`(TipCircleRadius·cos(angle), TipCircleRadius·sin(angle))`, constrained coincident to the tip
circle. Then create the arc with the local origin as its centre and the two flank splines' end
`SketchPoint`s as its start and end, passed **directly** so the arc shares all three. Add **no**
diameter dimension. A free centre plus a diameter sizes the arc but not which way it bulges: the
same radius through the same two ends also curves inward, back through the tooth, and the sketch
reaches DOF 0 with both answers open. Sharing the centre removes the choice — and makes the last
rib's perpendicular redundant, which is why the ribs omit it.

**The spine and the +X reference.** Draw the spine as a construction line from the local origin to
the tooth-top point, sharing both existing points. No separate start-coincident to the origin, and
no constraint putting its end on the arc. Then, for **every** angle including 0, build the +X
reference: a far endpoint at `(TipCircleRadius, 0)` pinned by two signed dimensions from the local
origin — horizontal at `TipCircleRadius`, vertical at `0` — a construction line from the origin to
it, and an angular dimension from the reference to the spine **in that argument order**, its text
placed on the bisector `(R·cos(angle/2), R·sin(angle/2))` so Fusion selects the angle and not its
supplement. Pin the endpoint with the two signed dimensions rather than a coincident to the tip
circle: a point on a circle has two answers, and this one sits at the circle's extreme in x where
the numbers go unstable. Do not reach for a plain horizontal on the spine when `angle = 0` —
horizontal fixes the line's direction but not which way it points, and the tooth comes out 180°
around the gear.

**The ribs.** One construction line per fit-point index, for **all N indices including the first
(base-circle) and the last (tip) pair**. The fit points carry no other constraint, so a missing
rib leaves that pair free. Each rib is built in exactly this order, and a different order
over-constrains the sketch:

1. `addByTwoPoints(leftSpline.fitPoints[i], rightSpline.fitPoints[i])`, passing the two fit-point
   objects directly so the rib shares them; mark it construction.
2. Dimension it with a **signed** dimension — vertical at the measured Δy for `angle = 0`,
   otherwise whichever of horizontal/vertical is better conditioned. An aligned dimension gives
   only a length, which the left and right flanks satisfy equally well swapped over, and the tooth
   comes out mirrored.
3. Add a fresh midpoint `SketchPoint` seeded **already on the spine**, at the foot of the left fit
   point: with `t = fitX·cos(angle) + fitY·sin(angle)`, the seed is `(t·cos(angle), t·sin(angle))`.
   Not the rib's true 2-D midpoint, and not `(fitX, 0)` for a rotated tooth.
4. `addCoincident(midpoint, spine)` — onto the spine first.
5. `addMidPoint(midpoint, rib)` — then the rib's midpoint.
6. `addPerpendicular(spine, rib)` — then perpendicular. **Skip this on the last rib**, where the
   tooth-top arc already holds the two ends at equal radius either side of the spine.

Then chain the midpoints: a **signed** dimension along the spine direction (horizontal for
`angle = 0`) from each rib's midpoint to the previous one, with the chain starting at the **local
origin** for the first rib. Without that origin-to-first dimension the whole chain slides along the
spine as a unit and the sketch never fully constrains. Signed, so the chain runs outward; an
aligned dimension is equally happy running the other way.

**The root.** If the flank's first point lies outside the root circle, draw a short **radial**
flank-to-root line on each side: `addByTwoPoints(rootEndGeometry, flankStartFitPoint)`, sharing the
spline's start point as the far end, and place the root end with **exactly two** signed dimensions
from the local origin — horizontal at its Δx, vertical at its Δy, and nothing else. Do not place it
with "root end on the root circle" plus "origin on the line" instead: the line through the flank
start and the centre carries on and meets the root circle again on the far side, so both answers
satisfy those two and the stub can come out as a line straight across the gear. This is the common
case and gives a tooth loop of 6 curves. If the flank starts **inside** the root circle, draw no
stub; the loop has 4 curves and the profile is embedded. The test is strict `<` on raw values —
`embedded = firstRadius < RootCircleRadius` — so exact equality counts as not embedded and draws a
zero-length stub. Keep it strict. `drawTooth` records the answer as
`self.parent._lastToothEmbedded` (the tooth generator cannot reach `ctx`);
`SpurGearGenerator.__init__` pre-initialises that attribute to `False`, and `buildSketches` copies
it to `ctx.toothProfileIsEmbedded`.

**The anchoring.** Project `ctx.anchorPoint` from the Tools sketch into this sketch, then add a
coincident constraint between that fresh projection and the local origin — not
`sketch.originPoint`. This happens inside `draw()` itself, not in `buildSketches` after `draw()`
returns, because helical and herringbone call `draw()` directly on their twisted loft sketch and
rely on that one call to anchor it.

Store the sketch as `ctx.gearProfileSketch`.

**Fusion API calls:** `component.sketches.add(planarEntity)` · `sketch.name = 'Gear Profile'` ·
`sketch.sketchPoints.add(adsk.core.Point3D.create(x, y, z))` ·
`sketch.sketchCurves.sketchCircles.addByCenterRadius(centerPoint, radius)` ·
`sketchCircle.isConstruction = True` ·
`sketch.sketchDimensions.addDiameterDimension(entity, textPoint)` ·
`sketch.sketchTexts.createInput2(text, height)` ⚠ NOT IN API INDEX ·
`textInput.setAsAlongPath(path, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)` ·
`sketch.sketchTexts.add(input)` · `adsk.core.ObjectCollection.create()` ·
`objectCollection.add(item)` ·
`sketch.sketchCurves.sketchFittedSplines.add(fitPoints)` · `sketchFittedSpline.fitPoints` ·
`sketch.sketchCurves.sketchArcs.addByCenterStartEnd(centerPoint, startPoint, endPoint)` ·
`sketch.sketchCurves.sketchLines.addByTwoPoints(startPoint, endPoint)` ·
`sketchLine.isConstruction = True` ·
`sketch.geometricConstraints.addCoincident(point, entity)` ·
`sketch.geometricConstraints.addMidPoint(point, midPointCurve)` ·
`sketch.geometricConstraints.addPerpendicular(lineOne, lineTwo)` ·
`sketch.sketchDimensions.addDistanceDimension(pointOne, pointTwo, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
(and `VerticalDimensionOrientation`) ·
`sketch.sketchDimensions.addAngularDimension(lineOne, lineTwo, textPoint)` ·
`dimension.parameter.value = angle` · `sketch.project(entity)` ⚠ NOT IN API INDEX ·
`find_circle_by_radius(sketch, radius)` (framework helper, `.utilities`)

**From:** `spec/spurgear/instructions.md` 180–210, 212–227, 265–295, 297–338, 390–443;
`spec/spurgear/fusion.md` 19–43, 47–58, 60–181;
`.claude/skills/generate-gear/PLAYBOOK.md` 151–158, 413–450, 458–473, 525–535, 547–555, 581–591

**Proof:** `stepGearProfileSketch`

---

## S6 `[PROSE]` Sketch-Only Short-Circuit

No timeline entry. If `SketchOnly` is true, make the Gear Profile sketch visible and stop:
no tooth extrude, no chamfer, no body, no pattern, no fillet, no bore. S7 to S15 are skipped.
S16 still runs — unconditionally, in both modes — and leaves the sketches visible on this path.

**Fusion API calls:** `sketch.isVisible = True` · `generator.getParameterAsBoolean('SketchOnly')`
(framework, `base.Generator`)

**From:** `spec/spurgear/instructions.md` 60, 147–150, 229–263, 445–447;
`spec/spurgear/fusion.md` 185–195

---

## S7 `[PROSE]` Extrude the Tooth

Find the single tooth cross-section in the Gear Profile sketch with the framework helper —
`find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`.
Do not re-implement the loop search; the helper rejects loops whose counts do not match and raises
when nothing does. The two arcs are the tooth top and the root arc between the flank feet — the
latter exists because the root circle was drawn solid and whole, and profile detection splits it
where the tooth's boundary meets it.

Extrude that profile from the target plane to the Extrusion End Plane as a **New Body**, name the
feature `Extrude tooth`, and store the resulting body as `ctx.toothBody`. Then, as the last action
of `buildTooth`, call `self.chamferTooth(ctx)` (S8) — the chamfer is triggered from inside
`buildTooth`, so `buildMainGearBody` must not chamfer separately.

**Fusion API calls:** `component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)` ·
`adsk.fusion.ToEntityExtentDefinition.create(entity, False)` ·
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)` ·
`component.features.extrudeFeatures.add(input)` · `feature.name = 'Extrude tooth'` ·
`feature.bodies.item(0)` · `find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=2)`
(framework helper, `.utilities`)

**From:** `spec/spurgear/instructions.md` 212–227, 265–282, 429–437, 449–453;
`spec/spurgear/fusion.md` 147–181; `.claude/skills/generate-gear/PLAYBOOK.md` 151–158, 571–580

---

## S8 `[PROSE]` Chamfer the Tooth

Only when `ChamferTooth > 0`. Find the tooth's front face with a **single conjunction predicate**:
walk `ctx.toothBody.faces` and take the first face for which **both** `face.edges.count ==
chamferWantEdges()` (6 on the spur base) **and** the sketch plane is coplanar with the face's
geometry. Both of the same face — this is not an edge-count match with a coplanarity tiebreak. If
no face satisfies both, raise; do not fall back to a partial match. An embedded profile yields a
4-edge front face while `chamferWantEdges()` stays 6, so chamfering an embedded spur tooth raises
that error; the accepted answer is that users turn the chamfer off for such gears.

Add every edge of that face to the chamfer set **except** the root arc, identified by radius, not
by relative size: skip an edge whose curve type is `Arc3DCurveType` and whose `edge.geometry.radius`
equals `RootCircleRadius` within `0.001` cm. That match is exact — the root arc is the only edge on
the root circle — and chamfering it would eat into the neighbouring tooth. The flanks, the tooth-top
arc and the two flank-to-root lines all get chamfered.

Helical and herringbone override `chamferWantEdges()` and nothing else here.

**Fusion API calls:** `ctx.toothBody.faces` · `face.edges.count` ·
`ctx.gearProfileSketch.referencePlane.geometry` · `sketchPlane.isCoPlanarTo(face.geometry)` ·
`edge.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType` · `edge.geometry.radius` ·
`adsk.core.ObjectCollection.create()` · `objectCollection.add(item)` ·
`component.features.chamferFeatures.createInput2()` ·
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(distance), False)` ·
`component.features.chamferFeatures.add(input)`

**From:** `spec/spurgear/instructions.md` 58, 265–282, 455–461;
`.claude/skills/generate-gear/PLAYBOOK.md` 480–486, 571–580

---

## S9 `[PROSE]` Extrude the Body

Find the gear body profile with `find_profile_by_curve_counts(sketch, arcs=2)` and extrude it from
the target plane to the Extrusion End Plane as a **New Body**. Name the feature `Extrude body` and
the body `Gear Body`, and store it as `ctx.gearBody`.

The two arcs are the two fragments the root circle is split into where the tooth's boundary meets
it, so the profile is the **disc** inside the root circle and the body is a solid cylinder that
the teeth are joined onto. The spec calls this loop annular and bounded by the root and tip
circles; it is not, and cannot be — the tip circle is construction geometry and bounds no profile.
The curve counts the helper is given are right, the description of them is not. See the closing
defect list.

While iterating the new body's faces, capture `ctx.extrusionExtent`: among faces whose
`surfaceType` is `PlaneSurfaceType`, the one parallel to but **not** coplanar with the gear's
sketch plane. Test it with the plane-geometry API, not a hand-rolled dot product — the near cap is
coplanar, so `isCoPlanarTo` rules it out, and the cylindrical face is not planar at all. Raise if
it is not found.

**Fusion API calls:** `find_profile_by_curve_counts(sketch, arcs=2)` (framework helper,
`.utilities`) ·
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)` ·
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)` ·
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)` ·
`component.features.extrudeFeatures.add(input)` · `feature.name = 'Extrude body'` ·
`extrude.bodies.item(0)` · `body.name = 'Gear Body'` · `body.faces` ·
`face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType` ·
`sketchPlane.isParallelToPlane(face.geometry)` · `sketchPlane.isCoPlanarTo(face.geometry)`

**From:** `spec/spurgear/instructions.md` 212–227, 463–472;
`.claude/skills/generate-gear/PLAYBOOK.md` 151–158, 571–580

---

## S10 `[PROSE]` Gear Center Construction Axis

Still while iterating the body's faces from S9, take any face whose `surfaceType` is
`CylinderSurfaceType` and build a construction axis from it. Name it `Gear Center`, switch its
light bulb off immediately, and store it as `ctx.centerAxis`. Raise if no cylindrical face is
found. This is its own timeline entry, separate from the extrude that produced the face.

**Fusion API calls:** `face.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType` ·
`component.constructionAxes.createInput()` · `axisInput.setByCircularFace(circularFace)` ·
`component.constructionAxes.add(axisInput)` · `constructionAxis.name = 'Gear Center'` ·
`constructionAxis.isLightBulbOn = False`

**From:** `spec/spurgear/instructions.md` 212–227, 463–472;
`.claude/skills/generate-gear/PLAYBOOK.md` 683–686

---

## S11 `[PROSE]` Pattern the Teeth

Circular-pattern `ctx.toothBody` around the `Gear Center` axis, quantity = `ToothNumber`. Pin all
three inputs explicitly rather than trusting defaults: `quantity`, `totalAngle` as the **string**
expression `'360 deg'`, and `isSymmetric = False`.

**Fusion API calls:** `adsk.core.ObjectCollection.create()` · `objectCollection.add(ctx.toothBody)` ·
`component.features.circularPatternFeatures.createInput(inputEntities, ctx.centerAxis)` ·
`patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)` ·
`patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')` ·
`patternInput.isSymmetric = False` ·
`component.features.circularPatternFeatures.add(patternInput)`

**From:** `spec/spurgear/instructions.md` 474–478;
`.claude/skills/generate-gear/PLAYBOOK.md` 594–599

---

## S12 `[PROSE]` Combine the Teeth into the Body

One Combine-Join of the patterned tooth bodies into `Gear Body`. Feed the pattern feature's
`bodies` collection to the combine **as-is**: it already contains the seed tooth alongside the
copies, so re-adding `ctx.toothBody` duplicates it. Then, as the last action of `patternTeeth`,
call `self.createFillets(ctx)` (S13).

**Fusion API calls:** `patternFeature.bodies` ·
`component.features.combineFeatures.createInput(ctx.gearBody, toolBodies)` ·
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation` ·
`component.features.combineFeatures.add(combineInput)`

**From:** `spec/spurgear/instructions.md` 249–251, 474–478;
`.claude/skills/generate-gear/PLAYBOOK.md` 592–593

---

## S13 `[PROSE]` Root Fillets

Only when `FilletRadius > 0`, read as the registered parameter's numeric `.value` —
`filletHelixFactorExpression()` is not consulted here; it was spliced into that parameter's
expression back in S1.

Round the inside corner where each valley floor meets a tooth flank: the sharp corner running the
full thickness of the gear, parallel to its main axis, where bending stress concentrates. Two
things make the edge pick fiddly. First, after the pattern and combine the root cylinder is usually
split into one patch per valley, so collect **every** cylindrical face whose radius equals
`RootCircleRadius`, not just the first. Second, on each such face keep only the **axial straight**
edges: filter to `Line3DCurveType`, take each line's direction from its **geometry endpoints**,
normalize, and keep it when `abs(abs(dot(direction, axisNormal)) - 1.0) < 0.01`. Use exactly that
tolerance — a tighter test drops valid edges that tessellation left slightly off, and the root
fillets go missing. The circular edges wrapping the front and back end caps are rims, not
structural corners; drop them. Do not read the direction from `edge.evaluator.getTangent(0)`:
parameter 0 is not guaranteed to lie inside the edge's parameter range and Fusion raises.

`isTangentChain` must be `False`. The collected edges are already exactly the root corners, and
tangent-chaining would pull in their neighbours. If the edge collection comes out **empty**, return
silently without creating the feature — an empty edge set must not reach `filletFeatures.add`.

**Fusion API calls:** `body.faces` ·
`face.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType` ·
`face.geometry.radius` · `face.edges` ·
`edge.geometry.curveType == adsk.core.Curve3DTypes.Line3DCurveType` ·
`edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)` · `vector.normalize()` ·
`vector.dotProduct(axisNormal)` · `get_normal(entity)` (framework helper, `.utilities`) ·
`adsk.core.ObjectCollection.create()` · `objectCollection.add(item)` ·
`component.features.filletFeatures.createInput()` ·
`filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(radius), False)`
⚠ NOT IN API INDEX — the index puts this method on `filletInput.edgeSetInputs`, which the sources
explicitly forbid; written here as the sources name it ·
`component.features.filletFeatures.add(filletInput)`

**From:** `spec/spurgear/instructions.md` 86–88, 265–282, 480–489;
`.claude/skills/generate-gear/PLAYBOOK.md` 412, 480–486

---

## S14 `[PROSE]` Bore Profile Sketch

`buildBore` runs unconditionally from `generate()`, after `buildMainGearBody`, so it must
early-return in **two** cases: when `SketchOnly` is set, and when `BoreDiameter <= 0`. The
SketchOnly guard is the essential one — on that path `ctx.gearBody` and `ctx.extrusionExtent` were
never set, and the cut would dereference `None`. Do not lean on the bore diameter being zero in
sketch-only mode; the user may have set both.

Otherwise create a sketch named `Bore Profile` on the target plane and draw the bore circle by
instantiating the tooth generator on that sketch and calling
`drawBore(ctx.anchorPoint, boreDiameter)`. It projects the anchor in, draws a solid (non-construction)
circle of that diameter centred on the projection, gives it a driving diameter dimension, and
returns the circle.

The tooth generator's constructor always adds its local-origin `SketchPoint` at (0, 0, 0), so this
sketch carries one stray, unused point. The spec calls that faithful behaviour and says not to
suppress it.

**Why this step is `[PROSE]` and not `[GO]`.** That stray point is exactly the kind of thing the
proof engine gates on: nothing constrains it, so this sketch has two free degrees of freedom and
can never reach DOF 0. Proving it would mean drawing a sketch the generator does not draw. The
step is left unproved and the contradiction is reported instead — see the defect list.

**Fusion API calls:** `component.sketches.add(planarEntity)` · `sketch.name = 'Bore Profile'` ·
`sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))` · `sketch.project(entity)`
⚠ NOT IN API INDEX ·
`sketch.sketchCurves.sketchCircles.addByCenterRadius(centerPoint, radius)` ·
`sketch.sketchDimensions.addDiameterDimension(entity, textPoint)`

**From:** `spec/spurgear/instructions.md` 54, 297–338, 491–495; `spec/spurgear/fusion.md` 19–31;
`.claude/skills/generate-gear/PLAYBOOK.md` 413–450, 551–555

---

## S15 `[PROSE]` Cut the Bore

Extrude-cut the Bore Profile from the target plane to `ctx.extrusionExtent` — the far end-cap face
captured in S9 — affecting only `ctx.gearBody`. Going to that face rather than a distance
guarantees the bore pierces the gear whatever the Thickness is.

**Fusion API calls:**
`component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)` ·
`adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)` ·
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)` ·
`extrudeInput.participantBodies = [ctx.gearBody]` ·
`component.features.extrudeFeatures.add(input)`

**From:** `spec/spurgear/instructions.md` 491–495

---

## S16 `[PROSE]` Cleanup

No timeline entry — visibility only. `cleanup(ctx)` is the **very last** action of `generate()`,
after `buildBore`, and it is called **unconditionally** in both modes. Placement matters: S14
re-projects from the Tools sketch, and projection fails once that sketch is hidden. Do not move the
call into `buildMainGearBody`, and do not guard the call — the mode split lives inside.

Construction planes and axes: switch the light bulb off on every one this build created — the
Extrusion End Plane, the `Gear Center` axis, and the normalized target plane if S2 made one. This
half runs in **both** modes, so no stray plane is left floating in sketch-only mode.

Sketches: set `isVisible = False` on Tools, Gear Profile and Bore Profile. This half runs **only**
on the full-build path; sketch-only mode leaves them visible, which is the entire point of that
mode. Guard each entity individually — the `Gear Center` axis and the Bore Profile sketch do not
exist in sketch-only mode. Never cross the two properties: `isVisible` does not hide construction
geometry, and `isLightBulbOn` is not how a sketch is hidden.

**Fusion API calls:** `constructionPlane.isLightBulbOn = False` ·
`constructionAxis.isLightBulbOn = False` · `sketch.isVisible = False`

**From:** `spec/spurgear/instructions.md` 201–205, 229–263, 386–388, 445–447;
`spec/spurgear/fusion.md` 185–195; `.claude/skills/generate-gear/PLAYBOOK.md` 558–570

---

## Defects found while compiling

Recorded here because the compilation had to make a call on each; none of them is smoothed over
above.

1. **`sketch.project(...)` is not in the API index** (S3, S5, S14; `[SPUR-F-ANCHOR-CHAIN]`). The
   index has `Sketch.project2(entities: list[core.Base], isLinked: bool) -> list[SketchEntity]`
   and `Sketch.include(entity: core.Base) -> core.ObjectCollection`, and no `project`. Written as
   the spec names it.
2. **`sketchTexts.createInput2(text, height)` is not in the API index** (S5; `[PB-SKETCH-TEXT]`).
   `SketchTexts` carries `createInput3(expression: str, height: core.ValueInput)` — note the
   height is a `ValueInput`, not the raw cm float the playbook passes. Written as the playbook
   names it.
3. **`filletInput.addConstantRadiusEdgeSet(...)` is not on `FilletFeatureInput`** (S13;
   `[PB-FILLET-CHAMFER]`). The index puts `addConstantRadiusEdgeSet(entities, radius,
   isTangentChain)` on `FilletEdgeSetInputs`, reached through `filletInput.edgeSetInputs` — the
   exact route the playbook says fails. The rule is inverted relative to the index. Written as the
   playbook names it.
4. **S9's profile is described wrongly.** The spec calls it "the annular loop bounded by exactly 2
   arcs (the root circle and the tip circle)". The tip circle is construction geometry (S5) and
   bounds no profile, and the proof's profile detection confirms what the two arcs really are: the
   two fragments the solid root circle is split into. The loop is a disc, and the body is a solid
   cylinder — which is what `ctx.gearBody`'s own description ("the cylindrical body the teeth are
   joined into") says. `find_profile_by_curve_counts(sketch, arcs=2)` is still the right call.
5. **The Bore Profile sketch cannot be fully constrained** (S14). `[PB-FULL-CONSTRAINT]` requires
   every sketch to reach zero free degrees of freedom; the spec simultaneously requires the tooth
   generator's constructor to leave an unconstrained stray point in this sketch and calls that
   faithful. Both cannot hold.
6. **Nothing says where the anchor projection lands in the sketch's own frame** (S3, S5). Every
   coordinate S5 hands Fusion is absolute in the sketch's local frame — the tip point at
   `(TipCircleRadius·cos angle, …)`, the reference endpoint at `(TipCircleRadius, 0)`, every rib
   midpoint seed — while every dimension is relative to the local origin. The two agree only when
   the projected anchor sits at plane-local (0, 0). When it does not, the dimensions are still
   right and the seeds are all displaced by the anchor offset, which is what `[PB-SEED-NEAR]`
   warns costs a converged solve. The spec never states the assumption or handles the offset. The
   proof puts the projection at (0, 0), so it cannot see this either.
7. **The near-degenerate transition is not observably fragile.** `[SPUR-F-FLANK-ROOT]` keeps the
   embedded test at a strict `<` on the grounds that exact equality draws a zero-length stub and
   calls that "the ill-conditioned region the bench proof flags". At module 1 and 41 teeth the stub
   is 0.014 mm long and the sketch passes the full gate — DOF 0, no redundancy, conditioning
   1.3e-3, one configuration. That case is in the proof's sweep. The strict comparison is still
   worth keeping for its own reason, but the flagged fragility does not reproduce.
8. **`[PB-SELECTION-FILTER-ENUM]`'s failure mode is not real** (S1). It says passing
   `'ConstructionPlanes'` as a string raises a `TypeError` on the argument. The index has
   `addSelectionFilter(filter: str) -> bool`, and `SelectionCommandInput.ConstructionPlanes` is an
   enum member whose value is the string `'ConstructionPlanes'`. The two forms are the same value.
   Using the constant is still better practice; the stated consequence of not doing so is wrong.
