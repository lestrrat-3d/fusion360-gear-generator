# Spur Gear — compiled step list

One step is one entry in the Fusion timeline. A whole sketch is one step however much
geometry goes into it, and so is each construction plane, construction axis, extrude,
chamfer, pattern, combine and fillet. Everything a step needs is inside that step.

`[GO]` marks a step the proof in `.tmp/spurgear_test.go` exercises. The proof engine
models 2D sketches only, so the three sketches are `[GO]` and every 3D step is `[PROSE]`.

Three calls the spec names are absent from the Fusion API database and are marked inline
where they appear; they are written as the spec names them, not corrected.

## Provenance

| Source | Blob hash |
|---|---|
| `spec/spurgear/instructions.md` | `6ce986c609c1d086e5158592f439e6d1c62309e3` |
| `spec/spurgear/fusion.md` | `3e8b0b338e80a1199eb7eb95f9f004a6f0bb747d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `802a53d6e2be4743227920a9d58c3dbe132553dc` |

---

## S1 `[PROSE]` Read the dialog, create the gear component, register the parameters

Before any geometry: read the dialog and build the parameter table. The occurrence
creation this triggers is the build's first timeline entry.

**Read order is load-bearing and is not the dialog display order.** As soon as anything
creates the occurrence — `Occurrences.addNewComponent(transform)` directly, or
`parameterName(name)` / `addParameter(name, value, units, comment)` transitively — Fusion's
active component shifts, and a `SelectionCommandInput` holding an entity from another
component can drop its selection. So:

1. Pull all three selections out first and stash them on `self`: Parent Component
   (`parentComponent`), Target Plane (`plane`), Anchor Point (`anchorPoint`), via
   `get_selection(inputs, id)`. Resolve the parent's `Occurrence.component` vs `Component`
   into `self.parentComponent`; raise on a wrong count or type.
2. Only then touch the design.

The dialog itself is added by `SpurGearCommandInputsConfigurator.configure(cls, cmd)` in
exactly this display order, which must not be reordered by input type:
Target Plane, Anchor Point, Module, Tooth Number, Pressure Angle, Bore Diameter,
Thickness, Apply chamfer to teeth, Generate sketches but do not build body, Parent
Component **last**.

Selection inputs: `CommandInputs.addSelectionInput(id, name, commandPrompt)` then
`SelectionCommandInput.addSelectionFilter(filter)` and
`SelectionCommandInput.setSelectionLimits(1, 1)` on each —
`adsk.core.SelectionCommandInput.ConstructionPlanes` + `PlanarFaces` for the plane,
`ConstructionPoints` + `SketchPoints` for the anchor, `Occurrences` + `RootComponents` for
the parent (pre-selecting `get_design().rootComponent`).

Value inputs: `CommandInputs.addValueInput(id, name, unitType, initialValue)` with the
default in **internal** units regardless of the display unit —
`adsk.core.ValueInput.createByReal(math.radians(20))` for Pressure Angle at display unit
`'deg'`, `adsk.core.ValueInput.createByReal(to_cm(10))` for Thickness at `'mm'`,
`createByReal(1)` / `createByReal(17)` / `createByReal(0)` for Module, Tooth Number and
chamfer. Bore Diameter is a string input so it accepts expressions:
`CommandInputs.addStringValueInput(id, name, '0 mm')`. SketchOnly is
`CommandInputs.addBoolValueInput(id, name, True, '', False)` and is read with
`get_boolean(inputs, id)`, never `get_value` — a `BoolValueCommandInput` has no
`.expression`.

Register the input-sourced parameters, then call the `addExtraPrimaryParameters(inputs)`
hook (a no-op on the spur base, the seam a subclass registers its own primaries through),
then the derived ones as live expression strings via
`adsk.core.ValueInput.createByString(expression)` on `UserParameters.add(name, value, units, comment)`.
`Module` is registered **unitless** (`''`), not `'mm'`, so `generateName()` renders `M=1`
and the `mm`-registered derived expressions read the bare factor.
`ToothSpaceAngleAtRoot` is pre-computed in Python (`ValueInput.createByReal`) because
Fusion's expression engine will not subtract a radian-valued `PressureAngle` from the
unitless output of `tan()`, and it is registered **unitless** so that
`ToothSpaceArcAtRoot = RootCircleRadius * ToothSpaceAngleAtRoot` reads as a length rather
than `mm·rad`. `FilletRadius` splices in `filletHelixFactorExpression()` (spur base `'1'`)
as its last factor. `SketchOnly` is stored as a real `1`/`0`.

Name the component with `generateName()` —
`'Spur Gear (M={}, Tooth={}, Thickness={})'` over the parameters' `.expression` strings,
not their `.value`.

**From:** `spec/spurgear/instructions.md` L9–11, L13–33, L36–88, L90–178, L217–232,
L234–300, L345–370, L372–381; `.claude/skills/generate-gear/PLAYBOOK.md` L18–40, L42–73,
L75–98, L100–146, L196–240, L242–266, L474–479.

## S2 `[PROSE]` Normalize the Target Plane

If the user's Target Plane selection is already a `ConstructionPlane`, use it unchanged and
this step adds no timeline entry. Otherwise (a planar face, say) build a coplanar
construction plane at zero offset and use that everywhere downstream, so profile detection
is never confused by the selected face's own profile.

`ConstructionPlanes.createInput(occurrenceForCreation)` →
`ConstructionPlaneInput.setByOffset(planarEntity, offset)` with
`offset = adsk.core.ValueInput.createByReal(0)` →
`ConstructionPlanes.add(input)`.

The offset argument is a `ValueInput`, never a bare number: `setByOffset(plane, 0)` is a
runtime `TypeError`. Keep a handle — if this step created a plane, S16 switches its light
bulb off. The active plane is held both as `self.plane` and as `ctx.plane`; subclasses read
`self.plane` directly, so keep both.

**From:** `spec/spurgear/instructions.md` L39, L217–232, L385–387;
`.claude/skills/generate-gear/PLAYBOOK.md` L230–240, L674–685.

## S3 `[GO]` Tools sketch — `stepToolsSketch`

Create a sketch named `Tools` on the target plane with `Sketches.add(planarEntity, occurrenceForCreation)`
(the framework's `createSketchObject(name, plane)` does the add, the naming and the initial
`Sketch.isVisible = False`; make it visible before anything projects from it).

Project the user's Anchor Point into it and keep the result as `ctx.anchorPoint`:
`Sketch.project(entity)` — *absent from the Fusion API database (tracked); the database has
`Sketch.project2(entities, isLinked)`*. This one projected point is the sketch's entire
content — it draws no geometry of its own — and it is the canonical handle: the Gear Profile
and Bore Profile sketches each project **this** in again, so the whole gear tracks the
user's anchor if it moves.

Leave `Sketch.isVisible = True` until the build is finished. Projection has failed on
invisible sketches in this repo's history, and S14 still projects from this sketch, which is
why S16 runs after the bore and not before.

**What the proof checks.** `stepToolsSketch` models the projection as a reference point —
geometry whose coordinates the solver may not move, which is what Fusion's associative
projection of an external anchor is — and shows the sketch closes with nothing free. Note
the gap this papers over: `[PB-PROJECT-NOT-FIXED]` says Fusion's own projected point still
carries free DOF, and the spec's answer is that every consumer is made coincident to it
(S5, S14), not that the projection is fixed.

**From:** `spec/spurgear/instructions.md` L41, L180–215 (bullets 1 and 2), L217–232,
L389–391 (first paragraph), L445–449; `spec/spurgear/fusion.md` L19–24, L26–31;
`.claude/skills/generate-gear/PLAYBOOK.md` L86–98, L430–444, L558–570.

## S4 `[PROSE]` Extrusion End Plane

Create an offset construction plane named `Extrusion End Plane` at distance `Thickness`
from the target plane, and store it as `ctx.extrusionEndPlane`.

`ConstructionPlanes.createInput(occurrenceForCreation)` →
`ConstructionPlaneInput.setByOffset(planarEntity, offset)` with
`offset = adsk.core.ValueInput.createByReal(thickness)` →
`ConstructionPlanes.add(input)`.

Its only purpose is to be the to-entity target for the tooth extrude (S7) and the body
extrude (S9), so both end on the same well-defined face. It stays visible while those
extrudes run and is hidden at the very end with
`ConstructionPlane.isLightBulbOn = False` — `isVisible = False` does not hide a
construction plane.

**From:** `spec/spurgear/instructions.md` L217–232, L385–387, L392–393;
`spec/spurgear/fusion.md` L190–200; `.claude/skills/generate-gear/PLAYBOOK.md` L558–570,
L674–685.

## S5 `[GO]` Gear Profile sketch — `stepGearProfileSketch`

One sketch, one timeline entry, and the whole of the spec's steps 3, 4 and 5: the four gear
circles, the involute tooth, and the anchoring that slides the drawing onto the user's
anchor. `buildSketches(ctx)` creates the sketch named `Gear Profile` on the target plane
and runs `SpurGearInvoluteToothDesignGenerator(sketch, self).draw(ctx.anchorPoint, angle=0)`;
`draw` performs `drawCircles()`, `drawTooth(angle)`, the anchoring, and — only when
`angle != 0` — the confirming angular value-set, in that order. The anchoring lives inside
`draw()`, not in `buildSketches`, because helical and herringbone call `draw` directly on
their own twisted sketch and rely on that one call to anchor it.

Everything is drawn relative to a **movable local origin**: a fresh `SketchPoint` at
(0, 0, 0) added by the generator's constructor and held as `self.anchorPoint`, never
`Sketch.originPoint`, which is immutable and cannot be made coincident with something
brought in from elsewhere.

### Circles

`SketchPoints.add(point)` for the local origin, then four circles with
`SketchCircles.addByCenterRadius(centerPoint, radius)` passing the local-origin
`SketchPoint` **directly** as the centre, so all four share it and no centre coincidence is
added. Root circle **solid**; tip, base and pitch circles construction
(`SketchCurve.isConstruction = True`). Each gets a driving diameter dimension —
`SketchDimensions.addDiameterDimension(entity, textPoint, isDriving)` with `textPoint` off
the centre (a text point at the centre is rejected) and `isDriving` left at its default
`True`; never pass `isDriven=True`.

Each circle also carries an along-path label: `SketchTexts.createInput2(text, height)` —
*absent from the Fusion API database (tracked); the database has
`SketchTexts.createInput3(expression, height)`* — then
`SketchTextInput.setAsAlongPath(path, isAbovePath, horizontalAlignment, characterSpacing)`
with `adsk.core.HorizontalAlignments.CenterHorizontalAlignment` and `0`, then
`SketchTexts.add(input)`. The string is
`'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` over the radii's internal `.value`
in cm, with `size = TipCircleRadius − RootCircleRadius`, and that same `size` is the text
height.

The tip circle being construction is what makes the body profile in S9 a disc rather than
an annulus: construction geometry bounds no profile.

### Involute tooth

Sample the flank endpoint-inclusively: for `i = 0 … steps−1`,
`r = BaseCircleRadius + (TipCircleRadius − BaseCircleRadius) · i / (steps − 1)`, so the first
sample is exactly on the base circle and the last exactly on the tip circle. Do not clamp the
start to `max(base, root)`. Each sample is `calculateInvolutePoint(BaseCircleRadius, r)`
— `alpha = acos(rb/r)`, `t = tan(alpha)` (**not** `inv(alpha) = tan(alpha) − alpha`),
`x = rb·(cos t + t·sin t)`, `y = rb·(sin t − t·cos t)` — and a `None` return is dropped.

Mirror the samples across +X (negate y) so the spiral narrows from base to tip, then rotate
by `rotate_angle = π/(2·ToothNumber) − atan2(−py, px)` where `(px, py) =
calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`. That is the left flank; mirror
it across X for the right. Then rotate **both** flanks, the tooth-top point and the rib
midpoint seeds by the runtime `angle` argument — `drawTooth` must use the `angle` that
flows in from `draw()`, never the constructor-stored `self.toothAngle`, or the helical loft
comes out with no twist.

Draw the two flanks with `SketchFittedSplines.add(fitPoints)` over an
`adsk.core.ObjectCollection.create()` of the points.

**Tooth-top arc.** Add a tooth-top `SketchPoint` at
`(TipCircleRadius·cos(angle), TipCircleRadius·sin(angle))` and constrain it to the tip circle
with `GeometricConstraints.addCoincident(point, entity)`. Then
`SketchArcs.addByCenterStartEnd(centerPoint, startPoint, endPoint, normal)` passing the local
origin and the two flanks' `SketchFittedSpline.fitPoints` end `SketchPoint`s directly, right
flank first. **No diameter dimension.** The shared centre is what says the arc bulges
outward; a free centre plus a diameter reaches DOF 0 with an inward-bulging answer available
too.

**Spine and angular pin.** `SketchLines.addByTwoPoints(startPoint, endPoint)` from the local
origin to the tooth-top point, both passed as existing `SketchPoint`s, marked
`SketchCurve.isConstruction = True`; add no start-coincident and do not constrain its end onto
the arc. Build the +X reference line for **every** angle including 0: a far endpoint at
`(TipCircleRadius, 0)` pinned with two signed dimensions from the local origin —
`SketchDimensions.addDistanceDimension(pointOne, pointTwo, orientation, textPoint, isDriving)`
with `adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation` at `TipCircleRadius`
and `VerticalDimensionOrientation` at `0` — then the construction line from the origin to it,
then `SketchDimensions.addAngularDimension(lineOne, lineTwo, textPoint, isDriving)` with the
**reference first and the spine second**, its text point on the bisector
`(R·cos(angle/2), R·sin(angle/2))` so Fusion picks the angle and not its supplement. A plain
`GeometricConstraints.addHorizontal(line)` on the spine is not a substitute at angle 0:
horizontal fixes the direction but not which way it points, and the tooth comes out 180°
around.

**Ribs**, one per fit-point index including the first and last, each in exactly this order:

1. `SketchLines.addByTwoPoints(startPoint, endPoint)` over `leftSpline.fitPoints[i]` and
   `rightSpline.fitPoints[i]` passed directly; `SketchCurve.isConstruction = True`.
2. A **signed** `SketchDimensions.addDistanceDimension(...)` at the measured delta — vertical
   for the rib and horizontal for the midpoint chain when `|cos(angle)| >= |sin(angle)|`,
   swapped otherwise. An aligned dimension gives only a length, which the left and right
   flanks satisfy equally well swapped over, and the tooth comes out mirrored.
3. `SketchPoints.add(point)` for the midpoint, seeded **on the spine** at the foot of the left
   fit point: with `t = fitX·cos(angle) + fitY·sin(angle)`, seed `(t·cos(angle), t·sin(angle))`.
4. `GeometricConstraints.addCoincident(point, entity)` — midpoint onto the spine, first.
5. `GeometricConstraints.addMidPoint(point, midPointCurve)` — then midpoint of the rib.
6. `GeometricConstraints.addPerpendicular(lineOne, lineTwo)` — then rib perpendicular to the
   spine, **skipped on the last rib**, whose perpendicular the tooth-top arc already implies.

Then chain the midpoints with signed distance dimensions along the spine direction, starting
from the **local origin to the first rib's midpoint**. Without that first link the whole chain
slides along the spine as a unit.

**Close the tooth at the root.** With `firstRadius` the distance from the local origin to the
left flank's first fit point, `embedded = firstRadius < RootCircleRadius` — strict `<`, raw
values, no tolerance. When not embedded, draw a short radial line on each side with
`SketchLines.addByTwoPoints(startPoint, endPoint)` from a fresh root-end point to the flank
spline's start `SketchPoint` passed directly, then place the root end with **exactly two**
signed dimensions from the local origin (horizontal at its Δx, vertical at its Δy) and nothing
else. "Root end on the root circle" plus "origin on the line" is satisfied by the far
intersection too, and the stub becomes a line straight across the gear. Record the flag on the
parent — `self.parent._lastToothEmbedded` — which `buildSketches` copies to
`ctx.toothProfileIsEmbedded`; the tooth generator cannot reach `ctx`.

### Anchoring

`Sketch.project(entity)` — *absent from the Fusion API database (tracked)* — to re-project
`ctx.anchorPoint`, then `GeometricConstraints.addCoincident(point, entity)` between the local
origin and that projection. Because every piece of geometry above is relative to the local
origin, this one constraint drags the whole tooth profile onto the anchor.

Finally, when `angle != 0`, set the confirming angular dimension:
`SketchDimension.parameter` `.value = angle`, as the very last action after the whole
constraint network exists. The pre-rotation puts the geometry on the right solver branch; the
value-set locks it.

Gate the result with `Sketch.isFullyConstrained`.

**What the proof checks.** `stepGearProfileSketch` builds this scheme across a sweep of
modules, tooth counts, pressure angles, sample counts and draw angles, and gates on the
engine's full verdict — DOF 0, no redundant or conflicting constraints, valid profiles, a
well-conditioned system, and no second discrete configuration. It also asserts the two closed
regions S7 and S9 consume, so the root-circle split is proved rather than assumed. Two of the
spec's rules were confirmed by breaking them: replacing the signed rib dimension with an
aligned one yields 13 admissible configurations, and keeping the last rib's perpendicular
reports it redundant.

**From:** `spec/spurgear/instructions.md` L180–215, L217–232, L302–343, L395–404, L406–443,
L445–449; `spec/spurgear/fusion.md` L19–43, L47–60, L69–87, L89–112, L114–149, L151–186;
`.claude/skills/generate-gear/PLAYBOOK.md` L336–403, L413–450, L458–473, L525–535, L547–555,
L581–591.

## S6 `[PROSE]` Sketch-only short-circuit

Not a timeline entry — a branch. If the `SketchOnly` parameter is true (read back with
`getParameterAsBoolean(name)`), set `Sketch.isVisible = True` on the Gear Profile sketch and
stop: no tooth extrude, no body extrude, no pattern, no combine, no fillet, no chamfer, no
bore. `buildBore(ctx)` and `cleanup(ctx)` still run from `generate()`, and `buildBore` guards
itself (S15).

**From:** `spec/spurgear/instructions.md` L234–268, L451–453; `spec/spurgear/fusion.md`
L190–200.

## S7 `[PROSE]` Extrude the tooth

`buildTooth(ctx)` owns this step and **must call `self.chamferTooth(ctx)` as its last
action** — helical overrides it to loft and herringbone to loft-and-mirror, and both still
end that way, so `buildMainGearBody` must not chamfer separately.

Find the tooth cross-section with the framework helper, not a hand-rolled loop:
`find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`.
Two NURBS are the flanks, two arcs are the tooth top and the root arc between the stubs, and
the two lines are the flank-to-root stubs an embedded profile does not have.

`ExtrudeFeatures.createInput(profile, operation)` with
`adsk.fusion.FeatureOperations.NewBodyFeatureOperation` →
`ExtrudeFeatureInput.setOneSideExtent(extent, direction, taperAngle)` with
`extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)` and
`direction = adsk.fusion.ExtentDirections.PositiveExtentDirection` →
`ExtrudeFeatures.add(input)`. Name the feature `Extrude tooth` and store
`ExtrudeFeature.bodies.item(0)` as `ctx.toothBody`.

**From:** `spec/spurgear/instructions.md` L234–300, L455–459;
`.claude/skills/generate-gear/PLAYBOOK.md` L148–194, L571–580.

## S8 `[PROSE]` Chamfer the tooth (optional)

Only when `ChamferTooth > 0`. Find the tooth's front face with a **single conjunction**: walk
`BRepBody.faces` and take the first face for which **both** `BRepFace.edges` `.count ==
chamferWantEdges()` (spur base `6`) **and** `Plane.isCoPlanarTo(plane)` holds between
`ctx.gearProfileSketch.referencePlane.geometry` and `BRepFace.geometry`. Both conditions must
hold of the same face; if no face satisfies both, raise, and do not fall back to a partial
match. (Accepted limitation: an embedded profile gives a 4-edge front face while
`chamferWantEdges()` stays 6 for spur, so chamfering an embedded spur tooth raises.)

Walk that face's edges and add each to an `adsk.core.ObjectCollection.create()`, **skipping**
any edge whose `BRepEdge.geometry` `.curveType == adsk.core.Curve3DTypes.Arc3DCurveType` and
whose `Arc3D.radius` equals `RootCircleRadius` within `0.001` cm. That radius match is exact —
the root arc is the only edge on the root circle — and it is what stops the chamfer eating
into the neighbouring tooth. Everything else on the face is chamfered.

`ChamferFeatures.createInput2()` →
`ChamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distance, isTangentChain)` on the
input's `chamferEdgeSets` collection with the `ChamferTooth` value and `False` →
`ChamferFeatures.add(input)`. The edge set goes on `chamferEdgeSets` for a chamfer and on the
input itself for a fillet; the two are asymmetric.

**From:** `spec/spurgear/instructions.md` L234–300, L461–467;
`.claude/skills/generate-gear/PLAYBOOK.md` L480–486, L571–580.

## S9 `[PROSE]` Extrude the body

Find the gear body profile with `find_profile_by_curve_counts(sketch, arcs=2)` — the solid
disc inside the root circle, bounded by the two pieces the tooth cuts the root circle into.
It is not an annulus and the tip circle is not part of it: the tip circle is construction
geometry and construction geometry bounds no profile.

Same extrude shape as S7: `ExtrudeFeatures.createInput(profile, operation)` with
`adsk.fusion.FeatureOperations.NewBodyFeatureOperation`,
`ExtrudeFeatureInput.setOneSideExtent(adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False), adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`ExtrudeFeatures.add(input)`. Name the feature `Extrude body` and the body `Gear Body`; store
it as `ctx.gearBody`.

While iterating `ExtrudeFeature.bodies.item(0).faces`, classify each face by
`BRepFace.geometry` `.surfaceType` and capture `ctx.extrusionExtent`: among the
`adsk.core.SurfaceTypes.PlaneSurfaceType` faces, the one where
`Plane.isParallelToPlane(plane)` is true and `Plane.isCoPlanarTo(plane)` is false against
`ctx.gearProfileSketch.referencePlane.geometry`. That is the far end cap; the near cap is
coplanar and is ruled out. Raise if it is not found. Use the plane-geometry API rather than a
hand-rolled dot product.

**From:** `spec/spurgear/instructions.md` L217–232, L234–300, L469–478;
`.claude/skills/generate-gear/PLAYBOOK.md` L148–194, L571–580.

## S10 `[PROSE]` `Gear Center` construction axis

Its own timeline entry, built from the same face walk as S9. Take any face whose
`BRepFace.geometry` `.surfaceType` is `adsk.core.SurfaceTypes.CylinderSurfaceType` and build
the axis: `ConstructionAxes.createInput(occurrenceForCreation)` →
`ConstructionAxisInput.setByCircularFace(circularFace)` → `ConstructionAxes.add(input)`.
Name it `Gear Center`, set `ConstructionAxis.isLightBulbOn = False`, and store it on
`ctx.centerAxis`. Raise if no cylindrical face is found.

**From:** `spec/spurgear/instructions.md` L217–232, L469–478;
`.claude/skills/generate-gear/PLAYBOOK.md` L558–570, L686–689.

## S11 `[PROSE]` Circular-pattern the tooth

`CircularPatternFeatures.createInput(inputEntities, axis)` with an
`adsk.core.ObjectCollection.create()` holding `ctx.toothBody` and the `Gear Center` axis.
Pin all three inputs explicitly rather than trusting defaults:
`CircularPatternFeatureInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)`,
`CircularPatternFeatureInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')`
(a string expression), `CircularPatternFeatureInput.isSymmetric = False`. Then
`CircularPatternFeatures.add(input)`.

`patternTeeth(ctx)` owns this step and S12, and calls `self.createFillets(ctx)` afterwards.

**From:** `spec/spurgear/instructions.md` L234–300, L480–484;
`.claude/skills/generate-gear/PLAYBOOK.md` L592–602.

## S12 `[PROSE]` Combine the teeth into the body

One Combine-Join. `CircularPatternFeature.bodies` already contains the seed tooth plus the
copies, so feed it as-is and do not re-add the seed — but copy it into a fresh
`adsk.core.ObjectCollection.create()` first, item by item, because `pattern.bodies` is a
`BRepBodies` and `CombineFeatures.createInput(targetBody, toolBodies)` rejects it.

`CombineFeatures.createInput(targetBody, toolBodies)` with `ctx.gearBody` as the target →
`CombineFeatureInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation` →
`CombineFeatures.add(input)`.

**From:** `spec/spurgear/instructions.md` L480–484;
`.claude/skills/generate-gear/PLAYBOOK.md` L592–596.

## S13 `[PROSE]` Root fillets (optional)

Only when `FilletRadius > 0`. Round the corner where the root valley floor meets each tooth
flank — the sharp inside corner running the full thickness, parallel to the gear's axis,
where bending stress concentrates. Not the front or back rim.

Collect **every** cylindrical face whose radius equals `RootCircleRadius`, not just the first:
after the pattern and combine, the root cylinder is usually one patch per valley. On each such
face, filter to edges whose `BRepEdge.geometry` `.curveType` is
`adsk.core.Curve3DTypes.Line3DCurveType`, take each line's direction from its geometry
endpoints — `Line3D.startPoint` `.vectorTo(point)` `Line3D.endPoint`, then
`Vector3D.normalize()` — and keep it when
`abs(abs(Vector3D.dotProduct(vector)) - 1.0) < 0.01` against the target plane's normal. Use
exactly that tolerance; a tighter test drops valid axial edges and leaves root fillets
missing. Do **not** read the direction via `edge.evaluator.getTangent(0)` — parameter `0` is
not guaranteed to lie inside the edge's parameter range and Fusion raises
`RuntimeError: invalid argument parameter`.

`FilletFeatures.createInput()` →
`FilletFeatureInput.addConstantRadiusEdgeSet(edges, radius, isTangentChain)` — *absent from
the Fusion API database (tracked); the database has `FilletFeatureInput.edgeSetInputs`* — on
the input **itself**, with `isTangentChain = False` so Fusion cannot pull in tangent-adjacent
edges → `FilletFeatures.add(input)`.

If the edge collection ends up empty, return silently without creating the feature. An empty
edge set must not reach `FilletFeatures.add(input)`.

**From:** `spec/spurgear/instructions.md` L234–300, L486–495;
`.claude/skills/generate-gear/PLAYBOOK.md` L412, L480–486.

## S14 `[GO]` Bore Profile sketch — `stepBoreProfileSketch`

`buildBore(ctx)` runs unconditionally from `generate()`, so it must early-return in **two**
cases: when `SketchOnly` is set, and when `BoreDiameter <= 0`. The SketchOnly guard is
essential on its own — in that mode `ctx.gearBody` and `ctx.extrusionExtent` were never set,
and proceeding would dereference `None`. Do not rely on the bore diameter being 0 in
sketch-only mode; the user may have set both.

Otherwise create a sketch named `Bore Profile` on the target plane with
`Sketches.add(planarEntity, occurrenceForCreation)`, then draw the circle **through the tooth
generator**: `SpurGearInvoluteToothDesignGenerator(boreSketch, self)` and
`drawBore(ctx.anchorPoint, boreDiameter)`, which does `Sketch.project(entity)` — *absent from
the Fusion API database (tracked)* — on `ctx.anchorPoint`, then
`SketchCircles.addByCenterRadius(centerPoint, radius)` centred on that projection with a
driving `SketchDimensions.addDiameterDimension(entity, textPoint, isDriving)`. The circle is
not construction.

The generator's **constructor** always adds its local-origin `SketchPoint` at (0, 0, 0), so
this sketch carries one stray unused point. That is faithful behaviour — do not suppress it —
but it must be grounded, exactly as S5 grounds the Gear Profile's:
`GeometricConstraints.addCoincident(point, entity)` between `toothGen.anchorPoint` and the
same projected anchor `drawBore` already made. Do **not** ground it on
`Sketch.originPoint`: that pins it to the plane rather than to the gear, and constraining to
`originPoint` has been observed to throw `VCS_SKETCH_SOLVING_FAILED`. Ungrounded it is free in
two directions and the sketch never reaches `Sketch.isFullyConstrained`.

**What the proof checks.** `stepBoreProfileSketch` builds the sketch at two bore diameters
and gates it on the same full verdict as S5, so the stray point's grounding is proved rather
than argued. The `BoreDiameter <= 0` case is reported as not modelled rather than passed:
there is no sketch in that mode, so there is nothing to prove.

**From:** `spec/spurgear/instructions.md` L180–215 (bullet on the Bore Profile sketch),
L302–343, L497–501; `spec/spurgear/fusion.md` L19–31;
`.claude/skills/generate-gear/PLAYBOOK.md` L423–429, L430–444, L534–535, L551–555.

## S15 `[PROSE]` Extrude-cut the bore

Extrude-cut the Bore Profile from the target plane to `ctx.extrusionExtent`, affecting only
`ctx.gearBody`.

`ExtrudeFeatures.createInput(profile, operation)` with
`adsk.fusion.FeatureOperations.CutFeatureOperation` →
`ExtrudeFeatureInput.setOneSideExtent(adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False), adsk.fusion.ExtentDirections.PositiveExtentDirection)`
→ `ExtrudeFeatureInput.participantBodies = [ctx.gearBody]` → `ExtrudeFeatures.add(input)`.

Cutting to the far end-cap face rather than by a distance guarantees the bore goes all the way
through regardless of Thickness.

**From:** `spec/spurgear/instructions.md` L217–232, L497–501;
`.claude/skills/generate-gear/PLAYBOOK.md` L196–228.

## S16 `[PROSE]` Cleanup

No timeline entry — visibility toggles. `cleanup(ctx)` is the **very last** action of
`generate()`, after `buildBore`, and is called **unconditionally** in both modes; the
SketchOnly distinction lives inside it. Placement after the bore matters because S14
re-projects from the Tools sketch and projection fails once that sketch is hidden.

Split by entity kind and by mode, guarding each entity individually because not all of them
exist in every mode:

- **Always, in both modes** — `ConstructionPlane.isLightBulbOn = False` on the Extrusion End
  Plane and on the normalized target plane if S2 created one, and
  `ConstructionAxis.isLightBulbOn = False` on `Gear Center`. So no stray plane floats in
  sketch-only mode either.
- **Full build only** — `Sketch.isVisible = False` on the Tools, Gear Profile and Bore
  Profile sketches. Sketch-only mode leaves Tools and Gear Profile visible for inspection,
  which is the whole point of that mode.

Never cross the two properties: `isVisible` hides sketches, `isLightBulbOn` hides construction
planes and axes.

**From:** `spec/spurgear/instructions.md` L180–215, L234–268, L451–453;
`spec/spurgear/fusion.md` L190–200; `.claude/skills/generate-gear/PLAYBOOK.md` L558–570.
