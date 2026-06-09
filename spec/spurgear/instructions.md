# Spur Gear Creation Instructions

## Component Setup

A spur gear is a single cylindrical body with straight teeth cut along the axis. Unlike the bevel generator there is no pairing — one invocation of the command produces exactly one gear. The new gear is added as a child occurrence of the user-selected Parent Component.

## Variables

User inputs are listed below in the order they appear in the command dialog. Derived (calculated) parameters are registered as Fusion user parameters under the `SpurGear<N>_` prefix so they are visible and editable after generation; they are listed after the inputs they depend on.

Target Plane: user-specified plane. The gear's front face is sketched on this plane; the gear body is extruded away from it by Thickness. Any selection that isn't already a `ConstructionPlane` (for example a planar face) is converted to a coplanar construction plane at generation time so sketch-profile detection is not confused by the selected face itself.

Anchor Point: user-specified point. The gear center is aligned with this point. May be a `ConstructionPoint` or a `SketchPoint`. The anchor point is projected into the Tools sketch on the target plane; the Gear Profile sketch then constrains its own local origin (0,0,0) to that projected point, so changing the anchor point downstream moves the gear.

Module: user-supplied number. Specifies the module of the gear.

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
accepts expressions; the rest are numeric `addValueInput`s. `SketchOnly` is persisted as a
real-valued user parameter (1 = true, 0 = false), since the framework only reads numeric
parameters as booleans (`getParameterAsBoolean`). The derived parameters in the list above
(Pitch/Base/Root/Tip circles, InvoluteSteps, ToothSpace…, Fillet…) keep exactly those names.

## Sketch Discipline

A few rules apply across every sketch created below. They're not obvious from the step list, and the whole construction falls apart without them.

**Sketches are isolated.** A sketch can't reference a `SketchPoint` or curve owned by another sketch. When the Gear Profile or Bore Profile sketches need the user's anchor, they call `sketch.project(...)` to pull it in locally. The Tools-sketch projection is the canonical handle; later sketches project *that* in again, forming a chain of projections all tied back to the user's original anchor entity so the whole gear tracks the anchor if it moves later.

**Every adjacency in the tooth profile loop is a shared `SketchPoint`**, not two free points that happen to have the same coordinates. Ribs pass through the flank splines' `fitPoints[i]`. The tooth-top arc passes through the flanks' `endSketchPoint`s. Flank-to-root lines end at the flanks' `startSketchPoint`s. Handing Fusion raw `Point3D`s at matching coordinates creates fresh sketch points, and then the tooth loop won't be recognised as a closed profile when the extrude step searches for it.

**Each sketch that needs to follow the anchor keeps its own local origin.** That means a fresh `SketchPoint` added at (0, 0, 0) — not `sketch.originPoint`, which is immutable and can't be coincident-constrained to anything brought in from elsewhere. The tooth generator draws all its geometry relative to this local origin, then at the end constrains it coincident with the projected anchor; Fusion then slides the whole sketch onto the user's anchor.

**Anchor to an existing sketch point by *sharing* it, never by re-coinciding a fresh point.** When a construction line or circle should start at / be centered on a point that already exists (the local origin, or the tooth-top point), pass that `SketchPoint` **object directly** into the creation call — `sketchCircles.addByCenterRadius(localOrigin, r)`, `sketchLines.addByTwoPoints(localOrigin, toothTopPoint)`, `addByThreePoints(flankEnd, …, flankEnd)` — so the new entity reuses the same point. Do **not** create the entity from the point's `.geometry` (a raw `Point3D`, which makes a *new* coincident point) and then `addCoincident(newPoint, original)` to tie them back. Each such redundant coincident piles another constraint onto the shared origin; with the four circle centers, the spine start and the rib chain all anchored to the local origin, accumulating these makes Fusion's solver fail outright (`VCS_SKETCH_SOLVING_FAILED`) or over-constrain. Sharing the point adds zero constraints and always solves. (This is the same rule as the profile-adjacency one above, applied to construction geometry and the origin.)

**The helical rotation is drawn AND confirmed — two distinct, both-required actions.** When a non-zero `angle` is passed, the tooth geometry is drawn **already rotated by `angle`** in the Python point math (step 4 — every flank point, the tooth-top point, and the rib-midpoint seeds are at their `angle`-rotated positions). Then, **as the very last action after the entire constraint network exists**, the spine-to-horizontal angular dimension's value is set to `angle` (step 7). These are NOT alternatives — do both: the pre-rotation puts the geometry on the correct solver branch, and the final dimension value-set *confirms and locks* that rotation rather than swinging the tooth into place from +X. (An earlier approach drew the tooth flat and relied solely on the dimension to swing it; that lets Fusion pick the wrong ~180°-off branch and ruins the helical loft — see step 4. The fix was to pre-rotate *and* keep the confirming dimension.) Concretely, the dimension value-set is mandatory even though the geometry already sits at `angle`: `if angle != 0: spineAngularDimension.parameter.value = angle`. For `angle = 0` (spur) there is no angular dimension and nothing to set.

**Hide a sketch only after you're done with it.** Setting `isVisible = False` on a sketch effectively takes it offline — projections, profile extraction, and further edits stop working on an invisible sketch. So the pattern is: create the sketch visible, draw and project and constrain everything, run any feature that consumes profiles from it, *then* toggle `isVisible = False` to keep the browser tidy. The same applies to any intermediate construction plane that needs to remain consumable by later features.

**Construction planes and axes hide with `isLightBulbOn = False`, not `isVisible`.** A `ConstructionPlane`/`ConstructionAxis` is *not* hidden by `isVisible = False` (that has no visible effect on it — a Fusion gotcha; the stock generator left a commented-out `extrusionEndPlane.isVisible = False` "I can't make this work", which is exactly why the Extrusion End Plane stayed visible). Use `entity.isLightBulbOn = False`. As with sketches, only hide a construction plane *after* the features that consume it (here, the tooth and body extrudes that target the Extrusion End Plane) have run. At the very end of the build the cleanup must turn off the light bulb on **every** construction plane and axis it created — the Extrusion End Plane, the `Gear Center` axis, and the normalized target plane if one was created in step 1 — and set `isVisible = False` on the Tools and Gear Profile (and Bore Profile) sketches, so only the finished gear body is left showing. **Split the cleanup by entity kind:** the construction-plane/axis hiding (`isLightBulbOn = False`) **always runs, in both modes** — including Generate-Sketches-Only, so no stray plane is left floating. The **sketch** hiding (`isVisible = False` on Tools/Gear Profile/Bore Profile) runs **only on the full-build path**; in Generate-Sketches-Only mode the sketches are deliberately left visible (see step 6). Guard each entity individually (only hide it if it was actually created — e.g. the `Gear Center` axis and Bore Profile sketch don't exist in sketch-only mode).

**Dimensions are driving by default; don't pass `isDriven=True`.** `Sketch*Dimensions.addDiameterDimension(curve, textPoint)` creates a *driving* dimension — one that pins the curve's size to the stated value. Passing `True` as the third argument inverts it to a *driven* (measured) dimension that merely reports the current size, letting the curve float. All diameter dimensions in this file (the four gear circles, the tooth-top arc, the bore circle) must be driving, so just omit the third argument.

**Sketch dimensions and feature inputs are numeric snapshots — to apply parameter changes, regenerate.** Every dimension and every feature input (extrude offset, chamfer distance, fillet radius, pattern count, bore-circle diameter) is set with the *current numeric value* of its source parameter at generation time, not with a live expression that would track the source.

We tried multiple times to wire live links — `dim.parameter.expression = self.parameterName(<PARAM>)` for sketch dims, `ValueInput.createByString(self.parameterName(<PARAM>))` for feature inputs — and the changes did not propagate reliably in Fusion. Sketch dims especially seem to need the sketch to be reopened/edited for downstream parameters to drive geometry; feature inputs sometimes capture the expression but sometimes don't. Rather than ship a half-working illusion, the rule here is: editing any user parameter under `<prefix>_…` does *not* change the existing gear. Re-run the Spur Gear dialog with the new values to regenerate.

This matches the original implementation. The user parameters under `<prefix>_…` remain visible in the parameter table for reference (so you can read off what the gear was generated with), but they aren't editable in a way that changes the design.

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
lives *inside* `cleanup`: it always hides the construction planes/axes (`isLightBulbOn = False`),
and hides the sketches (`isVisible = False`) only when NOT SketchOnly. So sketch-only mode leaves
the sketches visible but still tidies away the floating construction planes. Placement after
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
- **`chamferTooth` / `createFillets`** read `chamferWantEdges()` and the fillet helix factor
  (`filletHelixFactorExpression()`); these hooks must exist on the generator.
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
  **step-5 anchoring as its own final action** (see step 5). **`drawTooth` MUST rotate by the
  `angle` argument that flows in from `draw()` at call time — NOT by the constructor-stored
  `self.toothAngle`.** This is load-bearing for subclasses: helical/herringbone construct the
  generator with the default `angle=0` (so `self.toothAngle == 0`) and then call
  `draw(ctx.anchorPoint, angle=helixAngle)`. If `drawTooth` used `self.toothAngle` it would draw a
  flat tooth and the helical loft would have no twist.
- Methods `drawCircles`, `drawTooth`, `drawBore`, and `calculateInvolutePoint(baseRadius,
  intersectionRadius)` must all exist. The tooth generator also exposes parameter accessors
  `getParameter(name)` and `getParameterValue(name)` (these names are part of the reproduced
  surface).
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

1. Sample a sequence of points along the involute flank, starting on the base circle and walking outward toward the tip circle in equal radial steps (Involute Steps samples in total). The first sample radius is **exactly `Base Circle Radius`** — do **not** clamp the start to `max(Base Circle Radius, Root Circle Radius)`; the flank is sampled from the base circle even when the base circle sits inside the root circle (the embedded case is detected later in step 9 from where the flank *start* lands, not by trimming the sampling). Each sample is `calculateInvolutePoint(Base Circle Radius, r)` for that step's radius `r` (the exact math is pinned in the tooth-generator surface section above). Drop any sample that returns `None` (i.e. whose radius is below the base circle) — those sit inside the base circle and have no valid involute.
2. **Watch the spiral direction.** A correctly-formed involute tooth narrows from base to tip, so the left flank's angular distance above +X must *decrease* as the radius grows. The standard parametric involute (`rb*(cos t + t sin t, sin t - t cos t)`) spirals the opposite way — its angular position *increases* with radius — and using it as a left flank directly produces a tooth that splays outward (wider at the tip than at the root). Before rotating, mirror the samples across the +X axis (negate y) so the spiral matches a left flank's shape. The rotation in the next step lifts the mirrored curve from −Y back up into +Y where the left flank belongs.
3. Decide how far to rotate the (mirrored) sequence so the tooth ends up symmetric about +X. Measure where the mirrored involute crosses the pitch circle, then rotate by exactly the amount that lands that pitch-circle crossing at angle `+π / (2 · ToothNumber)` above +X. (The angular width of a single tooth at the pitch circle is `π / Tooth Number`, so half that — `π / (2 · ToothNumber)` — is where the left flank's pitch crossing must end up.) Compute the pitch-circle crossing angle **analytically** — evaluate `calculateInvolutePoint(Base Circle Radius, Pitch Circle Radius)` and take its polar angle — rather than interpolating between the sampled flank points; the analytic value places the tooth at exactly the right angle regardless of how few involute samples are taken.
4. Rotate the (mirrored) sampled points by `rotate_angle`. This produces the **left** flank. Mirror that result across the X axis to produce the **right** flank. You now have a tooth symmetric about +X.
   **Then apply the requested `angle`.** The generator's `draw(anchorPoint, angle=0)` takes an `angle` (0 for spur; the helix angle for helical; 180° for the bevel virtual tooth) — the seed tooth must end up rotated by exactly that. Do this by rotating the **whole** +X-centered tooth by `angle` right here, in the same Python point math: rotate both flank point collections by `angle` (and, below, place the tooth-top point and seed the rib midpoints at the rotated positions too). Draw the tooth directly at its final angular position. Do **not** instead leave the tooth at +X and rely on the spine's angular dimension to swing it into place after the fact — that makes the `angle = 0` and `angle != 0` cases settle on ~180°-different baselines (Fusion's solver picks the wrong branch when the dimension jumps from ≈0 to `angle`), which is invisible for a spur gear (its teeth are circular-patterned, so the seed's absolute angle doesn't matter) but ruins a helical loft (bottom profile at 0°, top ~180° away → the loft passes through the gear centre). Because both the bottom (`angle = 0`) and top (`angle = helixAngle`) profiles now share the same `rotate_angle` baseline and differ by exactly `angle`, the loft twists by exactly the helix angle regardless of the absolute baseline. For `angle = 0` this whole step is a no-op (rotating by 0).
5. Draw the two flanks as `SketchFittedSpline`s through the point collections.
6. Draw the **tooth-top arc**. This step is constraint-sensitive: add the *minimal* set below and **nothing more** — the extra constraints people reach for here (pinning the arc's centre, or separately coincident-constraining the arc endpoints) over-determine the sketch and it blows up later, when the last rib's perpendicular is added, with `VCS_SKETCH_OVER_CONSTRAINTS`.
   1. Materialize a **tooth-top point**: a `SketchPoint` at the tip, **rotated by `angle`** to match the rotated flanks — `(Tip Circle Radius · cos(angle), Tip Circle Radius · sin(angle))` — constrained **coincident to the tip circle** — i.e. the *point* lies on the tip circle. (This is the only coincidence here. Do **not** constrain the arc's `centerSketchPoint` to anything — no concentric/centre-on-origin constraint.)
   2. Create the arc with `sketchArcs.addByThreePoints(rightFlankEndPoint, toothTopPoint.geometry, leftFlankEndPoint)` — pass the two flank splines' **end `SketchPoint`s directly** as the first and third arguments (so the arc shares those endpoints; no separate coincidence constraints), and the tooth-top point's *geometry* as the middle through-point.
   3. Add a single **driving diameter dimension** on the arc equal to the tip circle diameter. The two shared flank ends plus this diameter dimension fully determine the arc; that is the complete constraint set for it.
7. Draw the **spine**: a construction line created as `addByTwoPoints(localOrigin, toothTopPoint)` — pass **both** existing `SketchPoint`s directly (the local-origin point as the start, the step-6 tooth-top point as the far end), so the spine *shares* them. Do **not** create it from `.geometry` coordinates, do **not** add a separate start-coincident to the origin (sharing already ties it; an extra coincident here is what makes the solver fail), and do **not** constrain the spine's end onto the arc (the tooth-top point already lies on the tip circle). Because the tooth (and so the tooth-top point and spine) is already drawn at its final `angle` (step 4), the spine starts out pointing in the `angle` direction. Pin its absolute rotation:
   - For `angle = 0` (spur), the spine's only additional constraint is **horizontal**.
   - Otherwise add a horizontal reference construction line from the origin **that always points +X** — give it a far endpoint at `(Tip Circle Radius, 0)` (a fixed positive x; do *not* derive it from the tooth-top point's x, which goes negative once `angle > 90°` and would flip the reference). **Then pin that far endpoint so the reference line is fully determined — `addCoincident(horizontal.endSketchPoint, tipCircle)`.** The endpoint at `(Tip Circle Radius, 0)` already lies on the tip circle, and with the Horizontal direction plus the shared origin start, coincidence-to-the-tip-circle fixes it exactly at the +X intersection. **This pin is required:** `addHorizontal` fixes only the line's *direction*, so without it the reference line's far end is a **free degree of freedom** (its length floats), and that single loose end leaves the whole tooth sketch under-constrained (`isFullyConstrained` False) — even though the tooth geometry is correct (the angular dimension only needs the line's direction, not its length). (This loose end appears only on the `angle != 0` path — helical / herringbone / the bevel virtual tooth; the spur `angle == 0` path has no separate reference line and is unaffected.) Add an angular dimension measured **from the spine to that horizontal reference, in that argument order** (`addAngularDimension(spine, horizontal, …)`); place its text on the **bisector of the intended angle** (a point at `(R·cos(angle/2), R·sin(angle/2))` for some small R) so Fusion selects `angle` rather than its supplement. Then set its value to `angle` as the very last action, after the whole tooth is constrained. Since the spine is *already* drawn at `angle`, this dimension **confirms** the position rather than swinging the tooth into it — that is what avoids the wrong-branch ~180° flip.
8. Draw a **rib** construction line between each pair of matching left/right flank points — passing through the splines' `fitPoints[i]` so the ribs share endpoints with the splines. Each rib needs a materialized **midpoint sketch point** on the spine, because a rib's midpoint isn't a referenceable entity you can dimension to unless you create a point for it. Build each rib like this, **in this exact order** — the order matters, a different order over-constrains the sketch (Fusion raises `VCS_SKETCH_OVER_CONSTRAINTS`):

   1. Add the rib line with `addByTwoPoints(leftSpline.fitPoints[i], rightSpline.fitPoints[i])` — pass the two fit-point `SketchPoint`s **directly** so the rib shares them as its endpoints; mark it construction. (Do not create the rib from raw coordinates and then add separate coincidence constraints to the fit points — share the points directly, which is fewer constraints.)
   2. Dimension the rib's **aligned length** to its current measured value.
   3. Add a fresh `SketchPoint` for the midpoint, created **already on the spine**. The spine is the line at `angle` through the local origin (it is the x-axis, `y = 0`, only when `angle = 0`), so seed the midpoint at the **foot of the left fit point on that line**: with `t = fitX·cos(angle) + fitY·sin(angle)`, the seed is `(t·cos(angle), t·sin(angle))`. (For `angle = 0` this reduces to `(fitX, 0)`.) Do **not** create it at the rib's true 2-D midpoint `((xL+xR)/2, (yL+yR)/2)`, and do **not** seed it at `(fitX, 0)` for a rotated tooth — starting it off the spine and then constraining it on is what triggers the over-constraint.
   4. `addCoincident(midpoint, spine)` — pin the point onto the spine **first**.
   5. `addMidPoint(midpoint, rib)` — then make it the rib's midpoint.
   6. `addPerpendicular(spine, rib)` — then make the rib perpendicular to the spine.

   Then dimension the aligned distance from each rib's midpoint to the previous rib's midpoint — **and for the first rib, dimension the distance from the local origin to its midpoint** (i.e. start the chain with `previous = local origin`). Without that origin-to-first dim the whole rib chain has one residual degree of freedom (it can slide along the spine as a unit), and the sketch never fully constrains. With it, the ribs lock the flanks down without pinning any individual point to an absolute coordinate — the flank still rebuilds cleanly when `Module` or `Tooth Number` changes.

   (Per rib this is exactly determined: the rib's four endpoint DOF are removed by the perpendicular, the midpoint-on-spine, the length dimension, and the chain distance. Adding any further constraint — or applying coincident/midpoint in the wrong order, or seeding the midpoint off the spine — over-constrains it.)
9. If the flank's first point (the one on the base circle) lies **outside** the root circle, draw a short **radial** line from the root circle up to that start point on each side, so the tooth closes at the root. Build each line as `addByTwoPoints(rootEndGeometry, flankStartFitPoint)` — pass the flank spline's **start `SketchPoint` directly** as the far endpoint so the line shares it (exactly like the rib lines in step 8; do **not** then add a separate coincident constraint for that shared endpoint — that self-coincidence is redundant and over-constrains the sketch). Then add **exactly these two** constraints, and no others:
   - (a) the line's **root-end** point coincident to the **root circle**;
   - (b) the **local origin** coincident to the line itself (point-on-line, treating the line as infinite) — this pins the line to a radial direction.

   Both are required and together they exactly constrain the line: the root end starts with 2 DOF, (a) drops it to 1 (on the circle), and (b) drops it to 0 (the line must pass through the origin, so the root end lands where the origin→flank-start ray meets the root circle). The flank-start endpoint is already fully located by the rib chain, so (b) constrains the *root end's* direction, not the flank point — there is no conflict. Omitting (b) leaves the root end free to slide around the circle (under-constrained, and the revolve/tooth comes out skewed); adding a *third* constraint (e.g. re-coinciding the shared flank endpoint) over-constrains it. This is the common case; the resulting tooth loop has 6 curves (2 splines + 2 flank-to-root lines + 2 arcs — the tooth top and the root arc). If instead the flank starts **inside** the root circle (happens when the tooth count / pressure angle combination drops the base circle below the root), no flank-to-root line is drawn and the loop has 4 curves (2 splines + 2 arcs); the profile is "embedded". Record which shape was drawn so the extrude step knows which edge count to look for. **Mechanism (the tooth generator has no `ctx`):** the tooth generator sets the boolean on its **parent generator** — `self.parent._lastToothEmbedded = True/False` — during `drawTooth`. `SpurGearGenerator.__init__` MUST pre-initialise `self._lastToothEmbedded = False` (alongside `self.toolsSketch = None` and `self.boreSketch = None`, the other fields cleanup guards on). Then `buildSketches`, which does hold `ctx`, copies it across: `ctx.toothProfileIsEmbedded = self._lastToothEmbedded`. Do not try to set `ctx.toothProfileIsEmbedded` directly from inside the tooth generator — it cannot reach `ctx`.

### 5: Anchor the Sketch

This is the step that slides the whole drawing onto the user's anchor. Project the Tools-sketch anchor into the Gear Profile sketch (this re-projection is what chains the two sketches together), then add a coincidence constraint between that freshly-projected point and the Gear Profile's local origin — the sketch point the tooth generator added at (0, 0, 0) in step 4 (the field `self.anchorPoint`), *not* `sketch.originPoint`. Because every piece of geometry above is constrained relative to the local origin, constraining it here drags the entire tooth profile onto the anchor as a unit.

**This anchoring is the final action of the tooth generator's `draw()` method itself** — `draw()` does `drawCircles()`, then `drawTooth()`, then this projection-and-coincidence — *not* a separate step the generator performs after `draw()` returns. This matters because helical/herringbone build their twisted loft profile by calling `SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=…)` directly and rely on that single call to anchor the sketch. If the anchoring were moved up into `buildSketches`, the twisted sketch would be left unconstrained and the loft would float off the anchor.

### 6: Sketch-Only Short-Circuit

If the Generate-Sketches-Only input is `true`, make the Gear Profile sketch visible and stop — skip the remaining body operations (tooth/body/pattern/fillet/bore). The end-of-build cleanup still runs in this mode, but does only **half** its job: it **hides the construction planes/axes** (the Extrusion End Plane, and the normalized Target Plane if one was created) so no stray plane is left floating, while **leaving the sketches visible** (Tools and Gear Profile stay shown for inspection — the whole point of this mode is to see the involute construction). So the SketchOnly end state is: gear sketches visible, construction planes/axes hidden, no body. (See the cleanup rule in Sketch Discipline: the plane/axis hiding always runs; only the sketch hiding is full-build-only.)

### 7: Extrude the Tooth

Find the single tooth cross-section profile in the Gear Profile sketch. The profile has 2 NURBS (the two flanks), 2 arcs (the tooth top and the root arc between them), and — unless `toothProfileIsEmbedded` — 2 short line segments (the flank-to-root lines). Reject loops whose curve counts don't match.

Extrude this profile from the target plane to the Extrusion End Plane (`ToEntityExtentDefinition`, PositiveExtentDirection) as a **New Body**. Name the feature `Extrude tooth`. Store the resulting body as `ctx.toothBody`.

### 8: Chamfer Tooth (optional)

If Apply-Chamfer-To-Teeth > 0, round off every edge of the tooth's front face *except* the arc shared with the root valley. The target face is identified by its edge count matching the loop we drew in step 4 — 6 for a spur tooth, different for subclasses. Skipping the root arc is the critical part: chamfering it would eat into the neighbouring tooth.

**Identify the root arc by radius, not by relative size.** Walk the front face's edges and add each to the chamfer edge set, *except* skip any edge that is an `Arc3DCurveType` whose `edge.geometry.radius` equals `Root Circle Radius` (compare against the registered parameter's `.value`, tolerance `0.001` cm). That radius match is exact — the root arc is the only edge lying on the root circle — so it is more robust than "the smallest-radius arc." (This is the same radius-matching technique step 11 uses to find the root cylinders.) Everything else on the face — the two flanks, the tooth-top arc, and the two flank-to-root lines — gets chamfered. Apply with `chamferFeatures.createInput2()` → `chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, <ChamferTooth value>, False)`.

Helical and herringbone subclasses override the expected edge count (a lofted embedded tooth has 4 edges, not 6), and may additionally filter the face by content — e.g. "must contain two NURBS flanks" — because on a tilted tooth more than one face can match the raw edge count and only one of them is the actual front.

### 9: Extrude the Body

Find the gear body profile — the annular loop bounded by **exactly 2 arcs** (the root circle and the tip circle): a `profileLoop` whose `profileCurves.count == 2` and every curve's `geometry.curveType` is `Arc3DCurveType`. Extrude it from the target plane to the Extrusion End Plane (`ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`, `PositiveExtentDirection`) as a **New Body**. Name the feature `Extrude body` and the resulting body `Gear Body`.

While iterating the new body's faces (`extrude.bodies.item(0).faces`), capture two references needed later. Classify each face by `face.geometry.surfaceType`:

- **`Gear Center` construction axis** — from any face whose `surfaceType` is `CylinderSurfaceType`. Build it with `constructionAxes.createInput()` → `axisInput.setByCircularFace(cylindrical_face)` → `constructionAxes.add(axisInput)`. Name it `Gear Center`; set `isLightBulbOn = False`. Store on `ctx.centerAxis`.
- **`ctx.extrusionExtent`** (the far end-cap face, used later by the bore cut) — among faces whose `surfaceType` is `PlaneSurfaceType`, the one that is parallel to but **not** coplanar with the gear's sketch plane. Test it with the plane-geometry API rather than a hand-rolled dot-product: let `sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`, and pick the face where `sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry)`. (The near cap is coplanar with the sketch plane, so `isCoPlanarTo` rules it out; the cylindrical and side faces aren't planar.) Raise if either reference isn't found.

Finally store `ctx.gearBody` (the `Gear Body` body).

### 10: Pattern Teeth

Circular-pattern `ctx.toothBody` around the `Gear Center` axis, quantity = Tooth Number. Combine the patterned tooth bodies into `Gear Body` via a single Combine-Join.

Important Fusion-API detail: `CircularPatternFeature.bodies` already includes *all* N tooth bodies — the original input body at position 0 plus the N-1 new copies. Feed the entire collection to the combine tools as-is; do **not** add `ctx.toothBody` again or the original will be double-counted.

### 11: Fillets

If Fillet Radius > 0, round the corner where the root valley floor meets each tooth flank — the sharp inside corner that runs the full thickness of the gear, parallel to its main axis. This is where bending stress concentrates at the tooth root, so it's the structurally important fillet (not the front/back rim, which is a cosmetic rounding the user doesn't want here). Two things make picking the right edges fiddly:

- After the pattern-and-combine step, the root cylinder is usually split into one patch per valley rather than a single continuous surface. Collect *every* cylindrical face whose radius equals Root Circle Radius, not just the first one found.
- On each such face, keep the **axial straight edges** — the ones whose direction is parallel to the target plane's normal (i.e. parallel to the gear's main axis). Those are the two valley-floor-to-tooth-flank corners on each valley patch. Drop the *circular* edges that wrap around the circumference at the front and back end caps; those are end-cap rims, not the structural root corners. Filter first to `Line3DCurveType` edges, then take each line's direction from its **geometry endpoints** (`geometry.startPoint.vectorTo(geometry.endPoint)`), normalize, and keep it if it is parallel to the axis within tolerance: `abs(abs(dot(direction, axisNormal)) - 1.0) < 0.01`. (Use exactly this tolerance — a tighter test like `> 0.999` can drop valid axial edges that are slightly off due to tessellation, leaving root fillets missing.)

Apply the fillet with `filletFeatures.createInput()` → `edgeSetInputs.addConstantRadiusEdgeSet(edges, <Fillet Radius value>, isTangentChain=False)`. **`isTangentChain` must be `False`** — the collected edges are exactly the axial root corners; tangent-chaining (`True`) would let Fusion pull in tangent-adjacent edges and round more than the intended root corner. Do **not** read the direction via `edge.evaluator.getTangent(0)` — parameter `0` is not guaranteed to lie inside the edge's parameter range and Fusion raises `RuntimeError: invalid argument parameter`.

### 12: Bore (optional)

`buildBore` runs unconditionally from `generate()` (after `buildMainGearBody`), so it MUST itself early-return in two cases: when **SketchOnly** is set, and when **Bore Diameter ≤ 0**. The SketchOnly guard is essential — in sketch-only mode `buildMainGearBody` short-circuits before `buildBody`, so `ctx.gearBody` and `ctx.extrusionExtent` are never set; proceeding into the cut would dereference `None`. (Do not rely on the bore diameter being 0 in sketch-only mode — the user may have set both.)

Otherwise (full build, Bore Diameter > 0), create a separate `Bore Profile` sketch on the target plane, draw a construction-less circle of that diameter at the projected Anchor Point, then extrude-cut that profile from the target plane to `ctx.extrusionExtent` (the far end-cap face), affecting only `ctx.gearBody`. The `ToEntityExtentDefinition` to the far face guarantees the bore goes all the way through regardless of Thickness.
