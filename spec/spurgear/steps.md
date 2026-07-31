# Spur Gear — compiled step list

Compiled from the sources below. A step is one entry in the Fusion timeline; a whole sketch is one
step however much geometry goes into it. `[GO]` marks a step the proof in `.tmp/spurgear_test.go`
exercises; everything 3D is `[PROSE]`, because the proof engine models 2D sketches only.

| Source | Blob hash |
|---|---|
| `spec/spurgear/instructions.md` | `0c83fdadf6ae6e0c0477ac16832bb406c4f4c602` |
| `spec/spurgear/fusion.md` | `3e8b0b338e80a1199eb7eb95f9f004a6f0bb747d` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9297782f07cf4a68372a500eaa3b0e35a9d27091` |

Two steps below produce no timeline entry of their own — S1 (the dialog) and S7 (the sketch-only
short-circuit). They are kept as steps because they decide what the rest of the timeline contains.

Inline `⚠ SPEC` notes mark places where the spec names a call that does not exist or passes an
argument of the wrong type. They are marked, not corrected.

---

## S1 `[PROSE]` Command Dialog Inputs

`SpurGearCommandInputsConfigurator.configure(cls, cmd)` is a `@classmethod` that adds the dialog
inputs to `cmd.commandInputs`. **The add order is the display order and is fixed**: Target Plane,
Anchor Point, Module, Tooth Number, Pressure Angle, Bore Diameter, Thickness, Apply chamfer to
teeth, Generate sketches but do not build body, Parent Component **last**. Do not regroup by input
type. This order is independent of `processInputs`' read order (S2), which reads the selections
first for a different reason.

| Dialog input | input id | call |
|---|---|---|
| Target Plane | `plane` | `commandInputs.addSelectionInput('plane', …, …)` |
| Anchor Point | `anchorPoint` | `commandInputs.addSelectionInput('anchorPoint', …, …)` |
| Module | `module` | `commandInputs.addValueInput('module', …, '', adsk.core.ValueInput.createByReal(1))` |
| Tooth Number | `toothNumber` | `commandInputs.addValueInput('toothNumber', …, '', adsk.core.ValueInput.createByReal(17))` |
| Pressure Angle | `pressureAngle` | `commandInputs.addValueInput('pressureAngle', …, 'deg', adsk.core.ValueInput.createByReal(math.radians(20)))` |
| Bore Diameter | `boreDiameter` | `commandInputs.addStringValueInput('boreDiameter', …, '0 mm')` |
| Thickness | `thickness` | `commandInputs.addValueInput('thickness', …, 'mm', adsk.core.ValueInput.createByReal(to_cm(10)))` |
| Apply chamfer to teeth | `chamferTooth` | `commandInputs.addValueInput('chamferTooth', …, 'mm', adsk.core.ValueInput.createByReal(0))` |
| Generate sketches, but do not build body | `sketchOnly` | `commandInputs.addBoolValueInput('sketchOnly', …, True)` |
| Parent Component | `parentComponent` | `commandInputs.addSelectionInput('parentComponent', …, …)` |

`addValueInput` defaults are in Fusion internal units — cm for length, radians for angle —
regardless of the unit string, which controls display and expression parsing only.

Each selection input takes its filters and is limited to exactly one selection:

- Target Plane: `selectionInput.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)`,
  `selectionInput.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)`,
  `selectionInput.setSelectionLimits(1, 1)`.
- Anchor Point: `adsk.core.SelectionCommandInput.ConstructionPoints`,
  `adsk.core.SelectionCommandInput.SketchPoints`, `setSelectionLimits(1, 1)`.
- Parent Component: `adsk.core.SelectionCommandInput.Occurrences`,
  `adsk.core.SelectionCommandInput.RootComponents`, `setSelectionLimits(1, 1)`, and pre-select the
  root with `selectionInput.addSelection(get_design().rootComponent)`.

Filters are enum constants, never quoted strings. A subclass extends this by subclassing the
configurator and appending after `super().configure(cmd)`, so its extra inputs land below Parent
Component.

**From:** `spec/spurgear/instructions.md` 20-22, 39-62, 90-150, 171-178;
`.claude/skills/generate-gear/PLAYBOOK.md` 53-60, 128-143, 332-334, 474-479.

---

## S2 `[PROSE]` Create the Gear Component and Register Parameters

Read order is load-bearing. `parentComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`
— reached directly through `getOccurrence()` or transitively through the first `addParameter()` /
`parameterName()` call — shifts Fusion's active component context, and a `SelectionCommandInput`
holding an entity that lives in another component can drop its selection when that happens. So:

1. `get_selection(inputs, 'parentComponent')`, `get_selection(inputs, 'plane')`,
   `get_selection(inputs, 'anchorPoint')` first, stashed on `self` (`self.parentComponent`,
   `self.plane`, `self.anchorPoint`). Nothing that touches the design yet.
2. Then the occurrence, then `component.name = self.generateName()`. `generateName()` returns
   `'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression,
   thickness.expression)` — the parameters' `.expression` strings, not `.value`.
3. Register the input-sourced parameters. Each is read with the helper matching the `add*Input` it
   was declared with: `get_value(inputs, id, units)` for the value and string inputs,
   `get_boolean(inputs, 'sketchOnly')` for the checkbox — never `get_value` on a bool input, which
   raises `AttributeError` on the missing `expression`. Register with
   `design.userParameters.add(self.parameterName(name), valueInput, units, comment)` via
   `addParameter`. `SketchOnly` is stored as a real 1/0 and read back with `getParameterAsBoolean`.
4. Call the `addExtraPrimaryParameters(inputs)` hook — a no-op on the spur base, the seam where a
   subclass registers its own primary parameters.
5. Register the derived parameters as live expressions with
   `adsk.core.ValueInput.createByString(f'{self.parameterName(…)} * …')`:
   `PitchCircleDiameter = Module * ToothNumber`, `PitchCircleRadius`, `BaseCircleDiameter =
   PitchCircleDiameter * cos(PressureAngle)`, `BaseCircleRadius`, `RootCircleDiameter =
   PitchCircleDiameter - 2.5 * Module`, `RootCircleRadius`, `TipCircleDiameter =
   PitchCircleDiameter + 2 * Module`, `TipCircleRadius`, `InvoluteSteps` (15), `FilletClearance`
   (0.9), `ToothSpaceArcAtRoot = RootCircleRadius * ToothSpaceAngleAtRoot`, and
   `FilletRadius = (ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>` where `<factor>` is
   whatever `filletHelixFactorExpression()` returns (spur base: `'1'`). This hook is consumed here
   and only here; `createFillets` never reads it.

Two units are pinned with a reason and must not be changed. `Module` is registered **unitless**
(`''`), not `'mm'`, so `generateName` renders `M=1` and the `mm` expressions above read the
unitless factor. `ToothSpaceAngleAtRoot` is registered **unitless** even though it holds radians,
because the next parameter multiplies it by a length and Fusion rejects a `mm·rad` product with
`RuntimeError: Invalid expression`. `ToothSpaceAngleAtRoot` is also **pre-computed in Python** and
registered with `adsk.core.ValueInput.createByReal(math.pi / toothNumber - 2 * (math.tan(pressureAngle) - pressureAngle))`,
because Fusion's expression engine will not subtract a radian value from the unitless output of
`tan()`.

Every parameter name in "Exact input ids" and the derived list, and every input id, is a
module-level constant that dependent modules import by name.

**From:** `spec/spurgear/instructions.md` 36-88, 152-169, 234-259, 288-300, 372-381;
`.claude/skills/generate-gear/PLAYBOOK.md` 38-40, 75-98, 99-126, 196-228, 230-240, 556-557.

---

## S3 `[PROSE]` Normalize the Target Plane

If `self.plane` is not already an `adsk.fusion.ConstructionPlane` — the user may have picked a
planar face — build a coplanar construction plane and use that everywhere downstream, so
profile detection is never confused by the selected face's own profile.

```
planeInput = component.constructionPlanes.createInput()
planeInput.setByOffset(selectedPlane, adsk.core.ValueInput.createByReal(0))
component.constructionPlanes.add(planeInput)
```

The offset is a `ValueInput`, not a bare number: `setByOffset(plane, 0)` is a runtime `TypeError`.
Store the result on `self.plane` and on `ctx.plane`; subclasses read `self.plane` directly, so keep
both available.

**From:** `spec/spurgear/instructions.md` 39, 217-232, 385-387;
`.claude/skills/generate-gear/PLAYBOOK.md` 230-240, 674-685.

---

## S4 `[GO]` Tools Sketch — proof function `stepToolsSketch`

`self.createSketchObject('Tools', self.plane)`, i.e. `component.sketches.add(self.plane)` plus the
name. The sketch draws no geometry of its own. It exists to own one reference:

```
projected = sketch.project(self.anchorPoint)
ctx.anchorPoint = projected.item(0)
```

That projection is the canonical handle. A sketch cannot reference a `SketchPoint` owned by another
sketch, so every later sketch calls `sketch.project(ctx.anchorPoint)` again, forming a chain back to
the user's original anchor entity — which is what makes the whole gear move if the anchor moves.

Leave the sketch **visible** while later sketches are still projecting from it. It is hidden with
`sketch.isVisible = False` in S16, after the bore, and not before: `project` fails on a hidden
sketch, and S14 projects from it.

The proof models the projection as engine reference geometry — externally located, solver-locked —
which is what a Fusion projection is. The sketch is determinate with nothing else in it.

⚠ SPEC: `Sketch.project` is absent from the API index but is called successfully by the shipped
add-in; written as the spec names it.

**From:** `spec/spurgear/instructions.md` 41, 180-215, 222-224, 389-391, 445-449;
`spec/spurgear/fusion.md` 19-24; `.claude/skills/generate-gear/PLAYBOOK.md` 92-96, 430-444, 558-570.

---

## S5 `[PROSE]` Extrusion End Plane

An offset construction plane at distance `Thickness` from the target plane, kept on
`ctx.extrusionEndPlane`.

```
planeInput = component.constructionPlanes.createInput()
planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(thickness))
ctx.extrusionEndPlane = component.constructionPlanes.add(planeInput)
```

Its only purpose is to be the to-entity target for both the tooth extrude (S8) and the body extrude
(S10), so the two end on the same well-defined face. Leave it visible while those run; it is hidden
in S16 with `constructionPlane.isLightBulbOn = False` — `isVisible = False` has no effect on
construction geometry.

**From:** `spec/spurgear/instructions.md` 217-232, 385-387, 393;
`spec/spurgear/fusion.md` 190-200; `.claude/skills/generate-gear/PLAYBOOK.md` 558-570, 674-685.

---

## S6 `[GO]` Gear Profile Sketch — proof function `stepGearProfileSketch`

One sketch, one timeline entry, and the whole constraint-bearing part of the build.
`buildSketches(ctx)` creates it — `self.createSketchObject('Gear Profile', self.plane)` — then runs
`SpurGearInvoluteToothDesignGenerator(gearProfileSketch, self).draw(ctx.anchorPoint)`, and
afterwards copies `ctx.toothProfileIsEmbedded = self._lastToothEmbedded`. Helical overrides
`buildSketches`, calls `super()`, and then draws a second twisted profile sketch the same way with a
non-zero `angle`, so everything below has to work at any angle.

`draw(anchorPoint, angle=0)` performs, in order: `drawCircles()`, `drawTooth(angle)`, the anchoring,
then the confirming angular dimension. `drawTooth` must rotate by the `angle` argument that arrives
at call time, never by the constructor-stored `self.toothAngle`; helical constructs the generator
with the default `angle=0` and passes the helix angle to `draw`.

### Constructor: the movable local origin

`toothGen.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`. A fresh
`SketchPoint`, **not** `sketch.originPoint`, which is immutable and cannot be tied to something
brought in from elsewhere. All geometry below is drawn relative to this point, and the anchoring at
the end slides the lot onto the user's anchor as a unit.

### drawCircles

Four circles, each created by passing the local origin `SketchPoint` **directly** as the centre so
all four share that one point — never `localOrigin.geometry` followed by a centre coincident, which
double-binds the point and kills the solver.

1. **Root Circle**, radius `RootCircleRadius`, **solid**.
2. **Tip Circle**, radius `TipCircleRadius`, construction.
3. **Base Circle**, radius `BaseCircleRadius`, construction.
4. **Pitch Circle**, radius `PitchCircleRadius`, construction.

```
circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(localOrigin, radius)
circle.isConstruction = True                       # all but the root circle
sketch.sketchDimensions.addDiameterDimension(circle, textPoint)
```

Every diameter dimension is **driving** — never pass the trailing `isDriving=False`. The text point
must be off-centre (a point at radius `r` from the centre); a text point at the centre is rejected
with `RuntimeError: 3 : … invalid input arguments`.

Each circle is labelled with along-path text:

```
textInput = sketch.sketchTexts.createInput2(label, size)
textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)
sketch.sketchTexts.add(textInput)
```

with `label = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)`, `size = TipCircleRadius −
RootCircleRadius`, and every number the parameter's internal `.value` in cm. The same `size` is the
text height.

⚠ SPEC: `SketchTexts.createInput2` is absent from the API index but is called successfully by the
shipped add-in; written as the spec names it.

⚠ SPEC: the four `name` strings the label format consumes are never given anywhere in the spec,
though the format string itself is pinned as reproduced surface.

### drawTooth — the flanks

Sample the involute flank endpoint-inclusively: with `steps = InvoluteSteps`, sample `i` for
`i = 0 … steps−1` sits at `r = BaseCircleRadius + (TipCircleRadius − BaseCircleRadius)·i/(steps−1)`,
so the first sample is exactly on the base circle and the last exactly on the tip circle. Do **not**
clamp the start to `max(BaseCircleRadius, RootCircleRadius)`. Each sample is
`calculateInvolutePoint(BaseCircleRadius, r)`; drop any that returns `None`.

`calculateInvolutePoint(baseRadius, intersectionRadius)` returns `None` when `intersectionRadius <
baseRadius`, else:

```
alpha = math.acos(baseRadius / intersectionRadius)
t     = math.tan(alpha)          # tan(alpha), NOT inv(alpha) = tan(alpha) - alpha
x = baseRadius * (math.cos(t) + t * math.sin(t))
y = baseRadius * (math.sin(t) - t * math.cos(t))
```

Then, in this order:

1. **Mirror across +X** (negate y). The standard parametric involute spirals the wrong way for a
   left flank — its angular position grows with radius, giving a tooth wider at the tip than at the
   root.
2. **Rotate** so the tooth is symmetric about +X. With `(px, py) =
   calculateInvolutePoint(BaseCircleRadius, PitchCircleRadius)`, the mirrored pitch crossing sits at
   `math.atan2(-py, px)`, so `rotate_angle = math.pi / (2 * toothNumber) - math.atan2(-py, px)`.
   Compute it analytically like this, not by interpolating between samples, so the tooth lands right
   however few samples are taken. The `-py` is the mirror; `atan2(py, px)` is wrong.
3. **Mirror the rotated result across X** to get the right flank.
4. **Apply the requested `angle`** to the whole +X-centred tooth here, in the same Python point
   math — both flank collections, the tooth-top point, and the rib midpoint seeds. Draw the tooth at
   its final angular position; do not draw it flat and let the spine dimension swing it round.

Both flanks are `SketchFittedSpline`s:

```
points = adsk.core.ObjectCollection.create()
# … points.add(adsk.core.Point3D.create(x, y, 0)) per sample …
spline = sketch.sketchCurves.sketchFittedSplines.add(points)
```

### drawTooth — the tooth-top arc

The arc caps the tooth at the tip circle, so it is part of that circle and must bulge outward. Say
that by sharing the centre, and add nothing else.

1. `toothTopPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(TipCircleRadius * math.cos(angle), TipCircleRadius * math.sin(angle), 0))`,
   then `sketch.geometricConstraints.addCoincident(toothTopPoint, tipCircle)`.
2. `sketch.sketchCurves.sketchArcs.addByCenterStartEnd(localOrigin, rightFlank.endSketchPoint, leftFlank.endSketchPoint)`
   — the local origin and both flank end points passed directly, so the arc shares all three and
   needs no coincidences. The right-then-left argument order is what makes the sweep go the right
   way.
3. **No diameter dimension.** A free centre plus a diameter reaches DOF 0 with the arc free to bulge
   inward, back through the tooth, and the solver picks by where the centre was seeded.

Sharing the centre is also what makes the last rib's perpendicular redundant, below.

### drawTooth — the spine and its +X reference

```
spine = sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, toothTopPoint)
spine.isConstruction = True
```

Both existing `SketchPoint`s passed directly. No separate start-coincident to the origin, and no
constraint putting the spine's end on the arc.

Build the +X reference for **every** angle, including 0:

1. A far endpoint at `(TipCircleRadius, 0)`, pinned by two **signed** dimensions from the local
   origin — `sketch.sketchDimensions.addDistanceDimension(localOrigin, refEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
   at `TipCircleRadius` and the same with
   `adsk.fusion.DimensionOrientations.VerticalDimensionOrientation` at `0`. Not
   `addCoincident(refEnd, tipCircle)`: a point on a circle has two answers, and this one would sit
   at the circle's extreme where the numbers go unstable.
2. `refLine = sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, refEnd)`;
   `refLine.isConstruction = True`.
3. `spineAngularDimension = sketch.sketchDimensions.addAngularDimension(refLine, spine, textPoint)`
   — reference first, spine second — with the text point on the bisector of the intended angle,
   `(R·cos(angle/2), R·sin(angle/2))` for small `R`, so Fusion picks `angle` and not its supplement.

Do **not** use `addHorizontal` on the spine for the `angle = 0` case. Horizontal fixes the line's
direction but not which way it points, and the tooth comes out 180° around.

⚠ SPEC: the text-point radius `R` for the angular dimension is left as "small `R`" and never pinned;
at `angle = 0` the bisector lies on both lines, so the wedge the text point is meant to select is
degenerate.

### drawTooth — the ribs

A construction line between each matching pair of left/right flank fit points, **one per index for
all N indices**, endpoints included. The fit points carry no other constraint, so a missing endpoint
rib leaves that point free. Build each in exactly this order; a different order over-constrains the
sketch:

1. `rib = sketch.sketchCurves.sketchLines.addByTwoPoints(leftSpline.fitPoints.item(i), rightSpline.fitPoints.item(i))`,
   the two fit-point `SketchPoint`s passed directly; `rib.isConstruction = True`.
2. A **signed** dimension on the rib, never an aligned one — an aligned dimension gives only a
   length, which the two flanks satisfy equally well swapped over, and the tooth comes out mirrored.
   The rib takes the axis **across** the spine and the midpoint chain the one **along** it: rib
   vertical and chain horizontal when `abs(math.cos(angle)) >= abs(math.sin(angle))`, both swapped
   otherwise. At `angle = 0` that reduces to
   `addDistanceDimension(left, right, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, textPoint)`
   at the measured Δy. A tooth at 90° fails without the swap.
3. A fresh midpoint `SketchPoint`, seeded **already on the spine** at the foot of the left fit point:
   with `t = fitX*math.cos(angle) + fitY*math.sin(angle)`, the seed is
   `(t*math.cos(angle), t*math.sin(angle))`. Not the rib's true 2-D midpoint, and not `(fitX, 0)`
   for a rotated tooth.
4. `sketch.geometricConstraints.addCoincident(midpoint, spine)` — onto the spine first.
5. `sketch.geometricConstraints.addMidPoint(midpoint, rib)` — then the rib's midpoint.
6. `sketch.geometricConstraints.addPerpendicular(spine, rib)` — then perpendicular. **Skip for the
   last rib**: the tooth-top arc already holds the two tips at equal radius either side of the
   spine, so this one says nothing new and Fusion rejects it with `VCS_SKETCH_OVER_CONSTRAINTS`.

Then chain the midpoints with **signed** dimensions along the spine direction (horizontal at
`angle = 0`), each from the previous rib's midpoint — and **the first from the local origin**.
Without that origin-to-first dimension the whole chain slides along the spine as a unit and the
sketch never fully constrains.

### drawTooth — closing at the root

`embedded = firstRadius < RootCircleRadius`, where `firstRadius` is the distance from the local
origin to the left flank's first fit point. Strict `<`, raw values, no tolerance: exact equality
counts as **not** embedded and draws a zero-length stub, which is the ill-conditioned region. This
happens above `2.5 / (1 - cos(PressureAngle))` teeth — 41.5 at 20°, 78.5 at 14.5°, 26.7 at 25°.

Not embedded: a short radial line on each side, from the root circle up to the flank's first fit
point, which it shares.

```
stub = sketch.sketchCurves.sketchLines.addByTwoPoints(rootEndGeometry, flankSpline.startSketchPoint)
sketch.sketchDimensions.addDistanceDimension(localOrigin, stub.startSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)
sketch.sketchDimensions.addDistanceDimension(localOrigin, stub.startSketchPoint, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, textPoint)
```

Exactly those two signed dimensions and no others. "Root end on the root circle" plus "local origin
on the line" is satisfied by **two** points — the line through the flank start and the centre meets
the root circle again on the far side — and the stub becomes a line straight across the gear at
DOF 0. The signed offsets rule the far one out.

Tooth loop: **6 curves** when not embedded (2 splines + 2 stubs + 2 arcs), **4** when embedded (2
splines + 2 arcs). Record it with `self.parent._lastToothEmbedded = True/False`; the tooth generator
cannot reach `ctx`, and `SpurGearGenerator.__init__` must pre-initialise `self._lastToothEmbedded =
False` alongside `self.toolsSketch = None` and `self.boreSketch = None`.

### Anchoring, then the confirming rotation

```
projected = sketch.project(ctx.anchorPoint)
sketch.geometricConstraints.addCoincident(toothGen.anchorPoint, projected.item(0))
```

Against the tooth generator's own local origin, **not** `sketch.originPoint`. Because everything is
constrained relative to that point, this one constraint drags the whole tooth profile onto the
anchor. It happens inside `draw()`, not in `buildSketches` afterwards — helical and herringbone call
`draw()` directly on their loft sketch and rely on that single call to anchor it.

Then, as the very last action after the entire constraint network exists:

```
if angle != 0:
    spineAngularDimension.parameter.value = angle
```

Drawing rotated and confirming with the dimension are two required actions, not alternatives. The
pre-rotation puts the geometry on the correct solver branch; the value-set locks it. Drawing flat
and relying on the dimension alone lets Fusion pick the branch ~180° off, which sends a helical loft
through the gear centre.

Every adjacency in the loop is a **shared** `SketchPoint` — ribs on `fitPoints[i]`, the arc on the
flanks' `endSketchPoint`s, the stubs on the flanks' `startSketchPoint`s. Raw `Point3D`s at matching
coordinates create fresh points and the loop is then not recognised as a closed profile.

The sketch must end **fully constrained** — `sketch.isFullyConstrained` — with no redundant or
conflicting constraints. That is what the proof checks, across the size sweep, before any of this is
written as Fusion code.

**From:** `spec/spurgear/instructions.md` 180-215, 217-232, 234-259, 270-280, 302-343, 395-404,
406-443, 445-449; `spec/spurgear/fusion.md` 19-43, 47-60, 62-67, 69-87, 89-112, 114-149, 151-186;
`.claude/skills/generate-gear/PLAYBOOK.md` 336-403, 413-450, 458-473, 525-535, 547-555, 581-591.

---

## S7 `[PROSE]` Sketch-Only Short-Circuit

If `SketchOnly` is true, set `ctx.gearProfileSketch.isVisible = True` and stop — no tooth extrude,
no body extrude, no pattern, no fillet, no chamfer, no bore. Useful for inspecting the involute
construction.

This is a branch in `buildMainGearBody`, not a timeline entry. It is a step here because it decides
whether S8 through S15 exist at all. S16 still runs, unconditionally, in both modes.

**From:** `spec/spurgear/instructions.md` 234-259, 451-453; `spec/spurgear/fusion.md` 190-200.

---

## S8 `[PROSE]` Extrude the Tooth

`buildTooth(ctx)` owns this and **must call `self.chamferTooth(ctx)` as its last action**; helical
overrides it to loft instead, herringbone to loft and mirror, and both still end with that call.
`buildMainGearBody` must not chamfer separately.

Find the single tooth cross-section with the framework helper — never a hand-rolled loop search:

```
profile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2,
                                       lines=0 if ctx.toothProfileIsEmbedded else 2)
```

Extrude it from the target plane to the Extrusion End Plane as a **New Body**:

```
extrudeInput = component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)
extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
extrude = component.features.extrudeFeatures.add(extrudeInput)
extrude.name = 'Extrude tooth'
ctx.toothBody = extrude.bodies.item(0)
```

⚠ SPEC: the spec names the extent type, the direction and the New Body operation but never names
`extrudeFeatures.createInput` / `setOneSideExtent` / `extrudeFeatures.add`, and the playbook covers
only the features *beyond* extrude. The call shape above is inferred, not cited.

**From:** `spec/spurgear/instructions.md` 224-229, 270-280, 455-459;
`.claude/skills/generate-gear/PLAYBOOK.md` 151-157, 571-580.

---

## S9 `[PROSE]` Chamfer the Tooth (optional)

Only when `ChamferTooth > 0`. Chamfer every edge of the tooth's front face **except** the arc it
shares with the root valley — chamfering that one would eat into the neighbouring tooth.

Find the front face with a **single conjunction**: walk `ctx.toothBody.faces` and take the first
face where **both** `face.edges.count == self.chamferWantEdges()` (6 on the spur base) **and**
`sketchPlane.isCoPlanarTo(face.geometry)`, with `sketchPlane =
ctx.gearProfileSketch.referencePlane.geometry`. Both of the same face. If no face satisfies both,
raise; never fall back to a partial match.

Skip the root arc by radius, not by relative size: skip any edge whose
`edge.geometry.curveType == adsk.core.Curve3DTypes.Arc3DCurveType` and whose `edge.geometry.radius`
equals `RootCircleRadius` within `0.001` cm. That match is exact — the root arc is the only edge on
the root circle. Everything else on the face is chamfered.

```
chamferInput = component.features.chamferFeatures.createInput2()
chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(chamferValue), False)
component.features.chamferFeatures.add(chamferInput)
```

Known and accepted: an embedded profile yields a 4-edge front face while `chamferWantEdges()` stays
6, so chamfering an embedded spur tooth raises front-face-not-found. Users disable chamfer for such
gears. Helical and herringbone override only `chamferWantEdges()`.

⚠ SPEC: the spec writes the distance argument as `<ChamferTooth value>`, a number, but
`ChamferEdgeSets.addEqualDistanceChamferEdgeSet(edges: core.ObjectCollection, distance:
core.ValueInput, isTangentChain: bool)` requires a `ValueInput`. Marked, not corrected; the call
above wraps it because `[SPUR-F-SNAPSHOT]` says feature inputs carry the parameter's current numeric
value.

**From:** `spec/spurgear/instructions.md` 58, 202-215, 281-287, 461-467;
`spec/spurgear/fusion.md` 204-209; `.claude/skills/generate-gear/PLAYBOOK.md` 480-486, 571-580.

---

## S10 `[PROSE]` Extrude the Body

`buildBody(ctx)`. Find the solid disc inside the root circle — boundary **exactly 2 arcs**, the two
pieces the tooth cuts the root circle into:

```
profile = find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)
```

It is not an annulus and the tip circle is not part of it: the tip circle is construction geometry,
and construction geometry bounds no profile. Extrude as a New Body, same extent as S8, name the
feature `Extrude body` and the body `Gear Body`, and store it on `ctx.gearBody`.

While iterating `extrude.bodies.item(0).faces`, capture two references, classifying by
`face.geometry.surfaceType`:

- **`Gear Center` axis** — from any face whose type is `adsk.core.SurfaceTypes.CylinderSurfaceType`:

  ```
  axisInput = component.constructionAxes.createInput()
  axisInput.setByCircularFace(cylindricalFace)
  ctx.centerAxis = component.constructionAxes.add(axisInput)
  ctx.centerAxis.name = 'Gear Center'
  ctx.centerAxis.isLightBulbOn = False
  ```

- **`ctx.extrusionExtent`** — among faces whose type is `adsk.core.SurfaceTypes.PlaneSurfaceType`,
  the one where `sketchPlane.isParallelToPlane(face.geometry) and not
  sketchPlane.isCoPlanarTo(face.geometry)`, using the plane-geometry API rather than a hand-rolled
  dot product. The near cap is coplanar, so `isCoPlanarTo` rules it out.

Raise if either reference is not found.

⚠ SPEC: the spec attributes the 2-arc split to "the two pieces the tooth's flank-to-root lines cut
the root circle into", but an embedded tooth draws no flank-to-root lines and the flank splines cut
the circle instead. The helper call is unaffected; the explanation is wrong for that case.

**From:** `spec/spurgear/instructions.md` 217-232, 469-478;
`.claude/skills/generate-gear/PLAYBOOK.md` 151-157, 571-580, 686-689.

---

## S11 `[PROSE]` Pattern the Teeth

`patternTeeth(ctx)`, which must call `self.createFillets(ctx)` after the combine.

```
inputEntities = adsk.core.ObjectCollection.create()
inputEntities.add(ctx.toothBody)
patternInput = component.features.circularPatternFeatures.createInput(inputEntities, ctx.centerAxis)
patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)
patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')
patternInput.isSymmetric = False
pattern = component.features.circularPatternFeatures.add(patternInput)
```

All three inputs pinned explicitly — do not rely on Fusion's defaults matching.

**From:** `spec/spurgear/instructions.md` 234-259, 480-484;
`.claude/skills/generate-gear/PLAYBOOK.md` 592-602.

---

## S12 `[PROSE]` Combine the Teeth into the Body

One Combine-Join of the patterned tooth bodies into `Gear Body`. Feed the pattern's `bodies`
collection as-is: it already includes the seed tooth, so do not re-add it. It is a `BRepBodies`, and
`combineFeatures.createInput` rejects that, so copy it into an `ObjectCollection` first.

```
toolBodies = adsk.core.ObjectCollection.create()
# … for i in range(pattern.bodies.count): toolBodies.add(pattern.bodies.item(i)) …
combineInput = component.features.combineFeatures.createInput(ctx.gearBody, toolBodies)
combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
component.features.combineFeatures.add(combineInput)
```

⚠ SPEC: the spec says "Combine-Join" but never names `combineFeatures.createInput` /
`CombineFeatureInput.operation` / the `JoinFeatureOperation` member; the playbook covers only the
`ObjectCollection` half. The call shape above is inferred.

**From:** `spec/spurgear/instructions.md` 480-484;
`.claude/skills/generate-gear/PLAYBOOK.md` 592-596.

---

## S13 `[PROSE]` Root Fillets (optional)

`createFillets(ctx)`, called from `patternTeeth`. Only when `FilletRadius > 0` — read the numeric
`.value` of the registered `FilletRadius` parameter, which already carries the helix factor from S2.

Round the sharp inside corner where the root valley floor meets each tooth flank, running the full
thickness parallel to the gear axis. That is where bending stress concentrates; the front and back
rim rounding is not wanted.

- Collect **every** cylindrical face whose radius equals `RootCircleRadius`, not just the first: the
  pattern-and-combine usually splits the root cylinder into one patch per valley.
- On each, keep the **axial straight edges**. Filter to
  `adsk.core.Curve3DTypes.Line3DCurveType` first, take each line's direction from
  `edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)`, normalize, and keep it when
  `abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01`. Use exactly that tolerance; a tighter
  `> 0.999` drops valid axial edges that are slightly off from tessellation and leaves root fillets
  missing. Drop the circular end-cap rims.
- Do **not** read the direction via `edge.evaluator.getTangent(0)` — parameter `0` is not guaranteed
  to lie inside the edge's range and Fusion raises `RuntimeError: invalid argument parameter`.

```
filletInput = component.features.filletFeatures.createInput()
filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), False)
component.features.filletFeatures.add(filletInput)
```

The edge set goes on the input **itself**, not through a `filletInput.edgeSetInputs` collection —
that is the chamfer-side shape and reaching for it fails. `isTangentChain` must be `False`; the
collected edges are exactly the axial root corners, and tangent-chaining would pull in neighbours.

If the edge collection ends up **empty**, silently skip: return without creating the feature. An
empty edge set must not reach `filletFeatures.add`.

⚠ SPEC: `FilletFeatureInput.addConstantRadiusEdgeSet` is absent from the API index but is called
successfully by the shipped add-in; written as the spec names it.

⚠ SPEC: the spec writes the radius argument as `<Fillet Radius value>`, a number, but every indexed
`addConstantRadiusEdgeSet` signature takes `radius: core.ValueInput`. Marked, not corrected.

⚠ SPEC: `axisNormal` is described only as "the target plane's normal"; the framework helper
`get_normal` is listed among the module's imports but no step ever names its call site.

**From:** `spec/spurgear/instructions.md` 86-88, 281-287, 345-350, 486-495;
`spec/spurgear/fusion.md` 204-209; `.claude/skills/generate-gear/PLAYBOOK.md` 151-157, 480-486.

---

## S14 `[GO]` Bore Profile Sketch — proof function `stepBoreProfileSketch`

`buildBore(ctx)` runs unconditionally from `generate()`, after `buildMainGearBody`, so it must
early-return in **two** cases: when `SketchOnly` is set, and when `BoreDiameter <= 0`. The
SketchOnly guard is essential — in that mode `buildMainGearBody` short-circuits before `buildBody`,
so `ctx.gearBody` and `ctx.extrusionExtent` were never set and the cut would dereference `None`. Do
not lean on the bore being 0 in sketch-only mode; the user may have set both.

Otherwise create a `Bore Profile` sketch on the target plane and draw the circle by instantiating
the tooth generator on it — `SpurGearInvoluteToothDesignGenerator(boreSketch, self)` — and calling
`drawBore(ctx.anchorPoint, boreDiameter)`, which projects the anchor in, draws a non-construction
circle of that diameter centred on the projection with a driving diameter dimension, and returns it.

```
projected = boreSketch.project(ctx.anchorPoint)
circle = boreSketch.sketchCurves.sketchCircles.addByCenterRadius(projected.item(0), boreDiameter / 2)
boreSketch.sketchDimensions.addDiameterDimension(circle, textPoint)
boreSketch.geometricConstraints.addCoincident(toothGen.anchorPoint, boreSketch.originPoint)
```

The constructor always adds its local-origin `(0, 0, 0)` `SketchPoint`, so this sketch carries one
stray unused point — faithful behaviour, do not suppress it. **Ground it** on the sketch's own
`originPoint`, which Fusion fixes at the plane origin. One constraint, and the sketch is fully
constrained like every other sketch here; without it the point is free in two directions and
`isFullyConstrained` never comes true.

The proof models the projection as reference geometry and the grounding as a coincidence to the
sketch origin, which is what this is.

⚠ SPEC: `[PB-CIRCLE-CENTER]` says flatly not to `addCoincident(..., sketch.originPoint)` — observed
to throw `VCS_SKETCH_SOLVING_FAILED` — and `[SPUR-F-LOCAL-ORIGIN]` calls `originPoint` immutable and
un-coincidentable, while this step requires exactly that constraint. Left as the spec states it.

**From:** `spec/spurgear/instructions.md` 210-212, 234-268, 321-326, 497-501;
`spec/spurgear/fusion.md` 19-31; `.claude/skills/generate-gear/PLAYBOOK.md` 423-429, 430-444.

---

## S15 `[PROSE]` Bore Cut

Extrude-cut the bore profile from the target plane to `ctx.extrusionExtent` — the far end-cap face
captured in S10 — affecting only `ctx.gearBody`. Going to the far face rather than a distance
guarantees the bore pierces the gear whatever `Thickness` is.

```
extrudeInput = component.features.extrudeFeatures.createInput(boreProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)
extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)
extrudeInput.participantBodies = [ctx.gearBody]
component.features.extrudeFeatures.add(extrudeInput)
```

⚠ SPEC: as in S8, the extrude call shape is inferred; the spec names only the extent target and the
restriction to `ctx.gearBody`.

**From:** `spec/spurgear/instructions.md` 217-232, 497-501.

---

## S16 `[PROSE]` Cleanup

The very last action of `generate()`, after `buildBore` — not inside `buildMainGearBody`. Called
**unconditionally** in both modes; the mode split lives inside it. Placement matters: `buildBore`
re-projects `ctx.anchorPoint` from the Tools sketch, and `project` fails once that sketch is hidden.

- **Always, in both modes:** `entity.isLightBulbOn = False` on every construction plane and axis it
  created — `ctx.extrusionEndPlane`, the `Gear Center` axis, and the normalized target plane if S3
  created one. Even in sketch-only mode, so no stray plane floats.
- **Full build only:** `sketch.isVisible = False` on the Tools, Gear Profile and Bore Profile
  sketches, so only the finished body shows. Sketch-only mode leaves Tools and Gear Profile visible
  for inspection — the whole point of that mode.

Guard each entity individually: the `Gear Center` axis and the Bore Profile sketch do not exist in
sketch-only mode. `isVisible` hides sketches and `isLightBulbOn` hides construction planes and axes;
never cross them.

⚠ SPEC: nothing in the spec says when the Gear Profile sketch is made **visible** on the full-build
path. `base.createSketchObject` returns it hidden, S7 makes it visible only in sketch-only mode, yet
S8 and S10 extract profiles from it and this step hides it again — which implies it was visible.
`[PB-HIDE-AFTER-USE]` records that profile extraction has failed on invisible sketches in this
repo's history, so the omission is load-bearing.

**From:** `spec/spurgear/instructions.md` 202-215, 234-268, 451-453;
`spec/spurgear/fusion.md` 190-200; `.claude/skills/generate-gear/PLAYBOOK.md` 558-570.

---

## Step-to-proof map

| Step | Proof function |
|---|---|
| S4 Tools Sketch | `stepToolsSketch` |
| S6 Gear Profile Sketch | `stepGearProfileSketch` |
| S14 Bore Profile Sketch | `stepBoreProfileSketch` |

No other proof function exists, and no other step claims one.
