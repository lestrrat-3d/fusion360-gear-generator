# Bevel Gear Creation Instructions

## Component Setup

Bevel gears are inherently created in pairs. As such, we shall have a single component containing three components

The first two components are for the driving gear and pinion gear. Because the designs are shared between the two components, the third one will be the design component.

## Variables

User inputs are listed below in the order they appear in the command dialog. Target Plane comes first so it receives keyboard/pick focus when the dialog opens (Fusion auto-focuses the first `SelectionCommandInput` and does not respect a later `hasFocus = True` override); Center Point follows so the user flows naturally from plane to point; Parent Component comes third since it is already pre-selected to the root component. Calculated values are listed after the inputs they depend on.

Target Plane: user-specified plane. This is where the bottom of the driving gear will sit flush against.

Center Point: user-specified point. This is where the driving bevel gear will be centered on. Does not need to be co-planar with target plane.

Parent Component: user-specified component. Defaults to the root component (pre-selected).

Module: user-supplied number. Specifies the module of gears.

Shaft Angle: User-supplied angle in degrees between 30° and 150°. Default 90° (perpendicular shafts — the classic bevel pair). The input is entered as a Fusion expression with a `deg` unit (e.g., `60 deg`); when read back via `UnitsManager.evaluateExpression(..., 'deg')` Fusion returns the value in its internal angle unit (radians), so validate-and-use code must convert back to degrees (`math.degrees(...)`) before comparing against the 30–150 range.

Driving Gear Teeth Number: user-specified number of teeth on the driving gear. Default is 31.

Pinion Gear Teeth Number: user-specified number of teeth on the pinion gear. Default is 31.

Driving Gear Pitch Diameter: calculated number. Module * Driving Gear Teeth Number.

Pinion Gear Pitch Diameter: calculated number. Module * Pinion Gear Teeth Number.

Driving Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified).

Pinion Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified).

Enable Bore: user-specified boolean, default `true`. Applies to both gears. When unchecked, no bore is cut on either gear and the per-gear bore diameter inputs below are ignored.

Driving Gear Bore Diameter: user-specified positive number, default 0mm. Only consulted when Enable Bore is checked. A value of 0 means "auto-calculate" — use `Driving Gear Pitch Diameter / 4` as the bore diameter.

Pinion Gear Bore Diameter: user-specified positive number, default 0mm. Only consulted when Enable Bore is checked. A value of 0 means "auto-calculate" — use `Pinion Gear Pitch Diameter / 4` as the bore diameter.

Face Width: User-specified positive number. If unspecified, default to (Cone Distance / 6). In **every** case (default or user-specified) the Face Width is bounded by the Maximum Face Width (defined below):
- If unspecified, use `min(Cone Distance / 6, Maximum Face Width)`.
- If the user specifies a value greater than the Maximum Face Width, this is an error: reject it with a message stating the maximum, rather than proceeding (the gear-body revolve in "Create the Pinion Gear" would otherwise fail — see the Maximum Face Width rationale).

Cone Distance: calculated number. `sqrt((Module * Driving Gear Teeth Number)**2 + (Module * Pinion Gear Teeth Number)**2)`.

Maximum Face Width: a geometric upper bound that cannot be evaluated until the Gear Profiles sketch points A, B, C, D, H, J exist (see §2 — apply the bound there). It is `0.95 *` the smaller of:
- the perpendicular distance from point A to the line through C and H (the Pinion Gear Dedendum line, i.e. Apex2->C extended), and
- the perpendicular distance from point B to the line through D and J (the Driving Gear Dedendum line, i.e. Apex2->D extended).

Rationale (do not drop this when regenerating): the toe line M->N is C->H offset *toward the Apex* by Face Width, with N pinned to line A->Apex2; its mirror O->P is D->J offset toward the Apex by Face Width, with P pinned to line B->Apex2. When the offset reaches the perpendicular distance from A to line C->H, point N lands exactly on A; any larger value drives N **past** A, across the gear's own shaft axis (Apex->A). The frustum profile (hexagon A, G, H, C, M, N, built here in §2 and revolved later in "Create the Pinion Gear") is revolved about that shaft axis, so a profile that has crossed the axis self-intersects the axis of revolution and Fusion aborts the revolve (`RuntimeError ... ASM_WIRE_X_AXIS ... the profile crosses the axis of revolution`). The pinion side is normally the binding one (its smaller pitch radius gives the smaller distance), but compute both and take the minimum so the bound holds for any Shaft Angle. The `0.95` factor keeps N clearly off A, since a near-coincident N≈A degenerates the toe edge even before it strictly crosses. At Shaft Angle 90° this limit equals `Pinion Gear Pitch Diameter**2 / (2 * Cone Distance)`, so the naive `Cone Distance / 6` default exceeds it — and the gear fails to generate — for any gear ratio above roughly √2 (e.g. Driving 31 / Pinion 17).

### Exact input ids and parameter-name strings

These literal strings are part of the reproduced surface. Use them verbatim. The dialog **display
order** (the order `configure()` adds inputs) is fixed as the rows below — Target Plane first so it
wins Fusion's auto-focus, then Center Point, then the pre-selected Parent Component, then the
numeric/bool fields. Module-level constants name the input ids (`INPUT_ID_PLANE = 'targetPlane'`, …).

| # | Dialog input | input id | input type | unit | default | selection filters |
|---|---|---|---|---|---|---|
| 1 | Target Plane | `targetPlane` | `addSelectionInput` | — | — | `ConstructionPlanes`, `PlanarFaces`; limit 1 |
| 2 | Center Point | `centerPoint` | `addSelectionInput` | — | — | `ConstructionPoints`, `SketchPoints`; limit 1 |
| 3 | Parent Component | `parentComponent` | `addSelectionInput` | — | root component pre-selected | `Occurrences`, `RootComponents`; limit 1 |
| 4 | Module | `module` | `addValueInput` | `''` | `createByReal(1)` | — |
| 5 | Shaft Angle | `shaftAngle` | `addValueInput` | `deg` | `createByString('90 deg')` | — |
| 6 | Driving Gear Teeth | `drivingTeeth` | `addValueInput` | `''` | `createByReal(31)` | — |
| 7 | Pinion Gear Teeth | `pinionTeeth` | `addValueInput` | `''` | `createByReal(31)` | — |
| 8 | Driving Gear Base Height | `drivingBaseHeight` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 9 | Pinion Gear Base Height | `pinionBaseHeight` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 10 | Enable Bore | `boreEnable` | `addBoolValueInput` (checkbox) | — | `True` | — |
| 11 | Driving Gear Bore Diameter | `drivingBore` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 12 | Pinion Gear Bore Diameter | `pinionBore` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 13 | Face Width | `faceWidth` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |

Selection filters are passed as `adsk.core.SelectionCommandInput.<Member>` enum constants (not
strings) and each selection sets `setSelectionLimits(1)`; the Parent selection pre-selects
`get_design().rootComponent` (see PLAYBOOK). The numeric `mm`/`deg` defaults are passed in internal
units (`to_cm(...)` for lengths; `createByString('90 deg')` for the angle so the expression engine
parses it). Shaft Angle is read back with `unitsManager.evaluateExpression(expr, 'deg')`, which
returns **radians**, so convert with `math.degrees(...)` before range-checking 30–150°.

**No live Fusion user parameters.** Unlike the spur family, bevel registers **no** user parameters
under a prefix. Every value (pitch diameters, cone distance, base heights, bore diameters, virtual
tooth counts, face width) is **precomputed in Python in internal cm** and written into geometry
numerically — sketch dimensions via `dimension.parameter.value = <number>` and feature inputs via
`ValueInput.createByReal(<number>)`. There are therefore no `PARAM_*` name strings to reproduce;
the only module-level constants are the 13 `INPUT_ID_*` strings. (See PLAYBOOK, "all-Python-
precomputed mode".)

**Reading the raw numbers.** Read each numeric/angle input by evaluating its expression directly:
`design.unitsManager.evaluateExpression(input.expression, units)` — `''`/`'mm'`/`'deg'` as
appropriate. `evaluateExpression` always returns the value in Fusion **internal units** regardless
of the unit string (cm for lengths, radians for angles). Do **not** read values via
`ValueInput.realValue` off a `get_value` result — evaluating the expression honors typed
expressions and user-parameter references in the field.

**Units — critical (Module is unitless mm; everything else is already internal):**
- The `'mm'` inputs (Driving/Pinion Base Height, Driving/Pinion Bore Diameter, Face Width) and the
  `'deg'` input (Shaft Angle) come back from `evaluateExpression(expr, 'mm'|'deg')` **already in
  internal units** (cm / radians). Use them as-is; do **not** `to_cm` them again.
- **`Module` is read with unit `''`, so it comes back as a raw number that means *millimetres*** (a
  module of `1` is 1 mm). Therefore **every length derived from Module must be `to_cm`-converted
  before it touches geometry**: Pitch Diameter = `to_cm(Module * teeth)`, Cone Distance = `to_cm(...)`,
  dedendum = `to_cm(1.25 * Module)`, the module-length construction extensions (E, F, G, H, I, J),
  and the default Face Width (`Cone Distance / 6`). The `_VirtualSpurProxy` likewise receives Module
  in mm and applies the standard spur formulas, then `to_cm`'s the resulting circle radii/diameters
  it serves (they must be in cm, matching what the spur tooth generator expects). Mixing a raw-mm
  Module-derived length with an already-cm `'mm'` input (e.g. comparing Face Width against the
  Module-derived Maximum Face Width) without this conversion makes the gear come out ~10× off and
  the Face-Width bound meaningless. The boolean Enable-Bore input is read with
`get_boolean` (or `input.value`). Read all inputs up front in a single `_readInputs` pass that
validates ranges (teeth ≥ 3, shaft angle 30–150°, non-negative heights/bores/width) and returns the
primary values, stashing the rest on `self`.

## Architecture

Bevel uses a **standalone generator** — it does **not** subclass `base.Generator` and uses **no
`GenerationContext`**. Three plain classes (names are the reproduced surface; the entry point binds
the first and third by name):

1. **`BevelGearCommandInputsConfigurator`** — `@classmethod def configure(cls, cmd)` that adds the
   13 dialog inputs above in display order.
2. **`BevelGearGenerator`** — `__init__(self, design)` (stores `self.design`, `self.bevelOccurrence
   = None`); `generate(inputs)`; `deleteComponent()`. Creates the occurrence tree directly with
   `parent.occurrences.addNewComponent(...)` — it does **not** use `getOccurrence` / `addParameter`
   / `parameterName` / `createSketchObject` from `base.Generator`, and registers no user parameters.
3. **`_VirtualSpurProxy`** (with a tiny `_Val` value-wrapper) — a fake spur `parent` so the borrowed
   spur tooth generator can run without registering Fusion user parameters. See Dependencies.

`base.py` is star-imported but its `Generator`/`ParamNamePrefix`/`ComponentCleaner` machinery is
unused.

## Generation Context — none

Bevel carries **no `GenerationContext` object.** State is threaded three ways: (a) `_readInputs`
returns a 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
shaftAngle_deg)` and stashes the rest as instance attributes (`self._drivingBaseHeight_cm`,
`self._pinionBaseHeight_cm`, `self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`,
`self._faceWidth_cm`); (b) the ~30 geometric anchors (Apex, Apex2, points A–P, K, L, the two shaft
axes, the cone-cutter faces, the two tooth sketches, the per-gear bodies) are passed as locals /
return values between `_buildGearProfiles` and the per-gear helpers; (c) `self.bevelOccurrence`
holds the top occurrence for cleanup. A regenerated bevel must not introduce a `ctx` object.

## Method contract — call graph

No gear subclasses bevel, so there are **no override boundaries to preserve** (unlike spur). The
only hard external bindings are `commands/bevelgear/entry.py` → `BevelGearCommandInputsConfigurator.
configure(args.command)` and `BevelGearGenerator(design).generate(inputs)` + `deleteComponent()` on
failure. The internal decomposition below is the intended structure; private helper names may vary,
but keep the step boundaries (they map to the Instructions sections):

```
generate(inputs)
  → _readInputs(inputs)                      # read+validate all inputs; returns 7-tuple, stashes the rest on self
  → resolve pitch diameters & bores (Python, cm)
  → build component tree                     # Bevel Gear → Design (Pinion/Driving components made in _createGearBody)
  → _buildAnchorSketch(design, plane, center)        # §1 → anchorLine
  → _buildGearProfiles(...)                  # §2 + §3 + per-gear body creation; internally:
        → _buildVirtualSpurProfile(...)  ×2  # §3 pinion then driving: tooth plane + spur tooth + axis
        → _createGearBody(...)           ×2  # revolve → loft → 2 conical cuts → pattern → combine → bore → (driving) mesh-rotate → moveToComponent
  → _hideConstructionGeometry(bevelComponent)        # Cleanup
deleteComponent()                            # error rollback (entry point calls on exception)
```

Helpers used by the above (names may vary; behaviour is pinned by the Instructions): `_readInputs`,
`_pointWorldGeometry` (world Point3D for a SketchPoint via `.worldGeometry` / ConstructionPoint via
`.geometry`), `_findSpurToothProfile` (match the tooth cross-section loop by curve-**type** mix, not count alone, or an unrelated loop can match: **2 NURBS flanks + 2 arcs (tip/root) + 2 lines** for a non-embedded tooth, or **2 NURBS + 2 arcs + 0 lines** when embedded),
`_applyConicalCut` (sequential split by cone faces; keep the largest non-apex piece),
`_findConeFaceForCutLine` (the `ConeSurfaceType` face whose surface contains both world endpoints of
a cut line), `_surfaceDistance`, `_cutBore`. Pinion is built first (no mesh offset); driving second
(mesh offset `180° / drivingTeeth`).

## Sketch Discipline (bevel-specific)

Bevel's sketch work differs from the spur family — note these deltas (the general Fusion gotchas in
PLAYBOOK still apply):

- **Constraint-network construction — but never double-bind a point.** §2 builds the A–P / K / L /
  Apex / Apex2 lattice with `coincident` / `perpendicular` / `colinear` / `angular` / dimensional
  constraints. When a line must *start at* an already-existing point (Apex, A, B, C, …), follow the
  PLAYBOOK rule **"share OR coincident, never both"**: the §2 prose "beginning of this line should
  use coincidence constraint with <point>" means **create the line from raw `Point3D` coordinates
  and add exactly one `addCoincident(line.startSketchPoint, <point>)`** — do **NOT** also pass the
  existing `SketchPoint` object into `addByTwoPoints` (that shares the point, and the extra
  coincident then over-constrains → `VCS_SKETCH_SOLVING_FAILED`). Concretely: `lineB1 =
  addByTwoPoints(apex_local_Point3D, bEnd_Point3D); addCoincident(lineB1.startSketchPoint, apex)` —
  not `addByTwoPoints(apex_point_object, …)` followed by a coincident.
- **The Apex is a free line-endpoint, not a fixed point.** Build it as the **end** of a vertical
  construction line from the projected center: `centerToApex = addByTwoPoints(projectedCenter.
  geometry, apex_local_Point3D); addCoincident(centerToApex.startSketchPoint, projectedCenter);
  addVertical(centerToApex); apex = centerToApex.endSketchPoint`. Compute `apex_local` as
  `sketch.modelToSketchSpace(apex_world)` — a `Point3D`→`Point3D` method call, **not** a matrix you
  fetch and `transformBy` (see PLAYBOOK) — where `apex_world` is the projected-center world point
  offset along `get_normal(targetPlane)` by Driving Gear Pitch Diameter; derive `y_up_sign` by
  transforming `centerWorld + normal` the same way and comparing its sketch-space Y to the projected
  center's. Seed `apex_local` at that computed world position (for solver conditioning) but **do not
  fix or dimension it** — the apex Y stays free and is pinned later through the chained constraints
  ("Constrain Point I with center point"). Do not create the Apex as a standalone
  `sketchPoints.add(...)` point.
- **The dimensional constraints in §2 are load-bearing, and the "do NOT add a dimensional
  constraint" notes equally so:** the along-shaft lengths (Apex→A, Apex→B) and the module-length
  extensions are fixed by the closing/colinear constraints, so adding a length dimension there
  over-constrains the sketch.
- **Apex is placed numerically, not constrained.** Its position is computed in world coordinates
  (target-plane normal, `get_normal` + `_pointWorldGeometry` + `modelToSketchSpace`) and set by
  coordinate, deliberately *without* a constraint, so the figure sits above the anchor line for any
  Shaft Angle. This world→sketch round-trip is correct **only because no occurrence is activated and
  every component transform is identity** (see the no-`activate()` rule below) — `modelToSketchSpace`
  then maps the world apex into the sketch faithfully. If anything were activated, this same
  round-trip would resolve against the wrong frame.
- **NEVER call `occurrence.activate()` — this is the bug that silently collapses the whole gear
  onto world XY.** All sketches/features run in the Design component, created via that component's
  collections (`designComponent.sketches.add(...)`, `designComponent.features.*`) **without
  activating it** — exactly as the spur generator builds in its own non-activated component. The
  reason: the Anchor Sketch is created on the user's *external* (root-owned) target plane; if the
  Design occurrence is **activated**, Fusion resolves that external plane in the activated
  component's **local frame (identity → world-XY-aligned)**, so the anchor line and the
  `setByAngle(anchorLine, '90 deg', targetPlane)` Gear Profiles plane come out XY-aligned regardless
  of the real plane tilt, and the entire gear builds flat on XY. Leaving every occurrence
  non-activated lets `sketches.add(targetPlane)` carry the true orientation (spur proves this works
  for any plane). `moveToComponent` at the end still works without activation (it preserves world
  position), and all features run in the single Design component, so no cross-sibling reference is
  ever needed during construction.
- **Cleanup hides by entity kind.** Recursively walk the Bevel Gear component tree (dedupe by
  `entityToken`) and set `isLightBulbOn = False` on every sketch, construction plane, and
  construction axis (construction planes/axes are **not** hidden by `isVisible`). There is no
  sketch-only mode and no per-mode guard — bevel always builds solids.

## Generation Order

`generate()` reads **all** inputs first (`_readInputs`), then creates occurrences. Because bevel
registers no user parameters, nothing creates an occurrence until after every selection is already
read, so the selection-context-shift hazard the spur spec warns about does not bite here — but keep
the order (read inputs → build tree → build geometry) so it stays that way. The geometry steps run
in the order of the Instructions below (§1 → §2 → §3 → Pinion → Driving → Cleanup), pinion before
driving.

## Dependencies

Bevel **borrows the spur tooth generator** — `from .spurgear import
SpurGearInvoluteToothDesignGenerator`. It is used only inside `_buildVirtualSpurProfile`, once per
gear:

```python
proxy  = _VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))   # the 180° tooth rotation IS the draw() angle
```

The borrowed generator's surface (from `spurgear.md`): constructor `(sketch, parent, angle=0)`;
`draw(anchorPoint, angle=0)` runs `drawCircles()` → `drawTooth(angle)` → anchor-projection; it reads
parameters via `parent.getParameter(name).value`. So bevel must supply a `parent` exposing
`getParameter(name)` → an object with a `.value`. That is **`_VirtualSpurProxy`**: it precomputes,
in internal cm, exactly the keys the spur drawer reads and returns each wrapped in `_Val`:
`Module`, `ToothNumber`, `PressureAngle` (radians; bevel hardcodes 20° — pressure angle is not a
bevel dialog input), `PitchCircleDiameter`, `PitchCircleRadius`, `BaseCircleDiameter`,
`BaseCircleRadius`, `RootCircleDiameter`, `RootCircleRadius`, `TipCircleDiameter`, `TipCircleRadius`,
`InvoluteSteps` (15). Standard spur formulas: pitch = teeth·module, base = pitch·cos α,
root = pitch − 2.5·module, tip = pitch + 2·module.

The **180° rotation is delivered through the `draw()` angle argument** (the spur generator rotates
the whole tooth by `angle`, per `spurgear.md`), *not* a post-hoc Move/sketch rotation — this relies
on spur's radial flank-to-root pinning so the connecting lines rotate with the tooth. (The
construction axis through point K / L, built normal to the tooth plane via `setByTwoPlanes`, is
still created as described in §3.)

## Instructions

### Create the Parent Component

Create the Bevel Gear component as a child of the Parent Component.

## Creating the Design Component

The Design Component shall contain the necessary sketches and construction planes / axis / etc that will be used by the components creating the actual gears later.

Create the Design component as a child of the Bevel Gear component.

### 1: Anchor Sketch

Start the Anchor Sketch **directly on the user-selected target plane** — `designComponent.sketches.add(targetPlane)` — whether the selection is a `ConstructionPlane` or a `PlanarFace`. **Do NOT "normalize" the target plane first** (e.g. `setByOffset(targetPlane, 0)` into a new construction plane), and in particular do not build such a plane inside the Design sub-component: a construction plane offset from a face that lives in another component resolves in the Design component's own (identity → world XY) frame and **loses the target plane's orientation, so the whole gear builds flat on world XY regardless of the plane the user picked**. `sketches.add` accepts a planar face directly and preserves its world orientation — use it as-is. Mark the center point by projecting the user-specified center point onto the sketch.

Create a line that intersects with the projected center point by applying the intersection constraint. Use the mid-point constraint to make the center point divide the line in half. This line shall only be used as a reference, and therefore its dimension does not really matter. Pick a number, say 10mm, and apply it to the line. This line shall be known as the Anchor Line.

### 2: Gear Profiles

Using ConstructionPlaneInput.setByAngle, create a plane that includes the Anchor Line, and set it at 90 degrees (as by default it would lie flush to the plane of the anchor line, but we want it perpendicular). **Pass the original `targetPlane` as `setByAngle`'s reference plane** — not the anchor sketch's own `referencePlane` and not any re-derived/offset plane — so the Gear Profiles plane inherits the true target-plane orientation (this is the other place the target-plane orientation reaches the bodies; substituting a different plane here also collapses the gear onto XY). Create a sketch on this plane.

In the sketch, project the center point from the Anchor Sketch.

From the projected center point, draw a construction line that runs along the direction of the target plane's normal (away from the target plane, on the side the normal points to). Apply Horizontal/Vertical constraint to it so it is perpendicular to the anchor line. The end of this line should be well above the Anchor line; Use x,y coordinates where x is the same as the center point, but y is shifted upwards the same amount as Driving Gear Pitch Diameter (however, do NOT use constraints). This end point shall be called the Apex.

Create a construction line from the apex representing the Driving Gear Shaft Axis. Apply a vertical constraint so it runs parallel to the previous (target-plane-normal) line, pointing downward from the apex toward the anchor line. Beginning of this line should use coincidence constraint with apex. The end of this line shall be called point B. Do **not** dimension the line's length — it is determined by the closing constraint at Apex 2 below.

Create a construction line from the apex representing the Pinion Gear Shaft Axis. Apply an angular dimension between this line and the Driving Gear Shaft Axis equal to Shaft Angle (this is the traditional "angle between the two shaft axes" — Shaft Angle = 90° gives the classic perpendicular bevel pair). **Place the angular dimension's text point inside the wedge between the two shaft axes** so Fusion measures Σ and not its supplement 180−Σ (see PLAYBOOK). The pinion shaft is drawn on the side away from the anchor sketch's leading direction, typically the +X half-plane of the Gear Profiles sketch. Beginning of this line should use coincidence constraint with apex. The end of this line shall be called point A. Do **not** dimension the line's length — it is determined by the closing constraint at Apex 2 below.

From A, create a construction line perpendicular to the Pinion Gear Shaft Axis, drawn toward the side where Apex 2 will lie (between the two shaft axes, in the direction of the anchor line). Apply a perpendicular constraint against the Pinion Gear Shaft Axis. Apply a dimensional constraint with length = Pinion Gear Pitch Diameter / 2 (this equals the pinion's pitch radius at the heel, which is the perpendicular distance from Apex 2 to the Pinion Gear Shaft Axis for any Shaft Angle). Beginning of this line should use coincidence constraint with A.

From B, create a construction line perpendicular to the Driving Gear Shaft Axis, drawn toward the side where Apex 2 will lie. Apply a perpendicular constraint against the Driving Gear Shaft Axis. Apply a dimensional constraint with length = Driving Gear Pitch Diameter / 2 (the driving pitch radius at the heel, perpendicular distance from Apex 2 to the Driving Gear Shaft Axis for any Shaft Angle). Beginning of this line should use coincidence constraint with B.

Constrain the end points of the two perpendicular lines from the previous two paragraphs with a coincident constraint. Let this point be called Apex 2. (At Shaft Angle = 90° the four points Apex, A, Apex 2, B form a rectangle. For other shaft angles the figure is a non-rectangular parallelogram-like quadrilateral; the lengths of Apex→A and Apex→B adjust so the perpendicular drops of length PPD/2 and DPD/2 coincide at Apex 2.)

Note that this quadrilateral deliberately lies well above the anchor line. The Apex's upward offset from the anchor line (= Driving Gear Pitch Diameter) is chosen to keep the whole figure above the anchor line for Shaft Angle in the supported range 30°–150°.

**Seed the along-shaft lengths (Apex→A, Apex→B) with the closed-form cone geometry** so the solver converges on the right branch for any Shaft Angle Σ (these are seed coordinates only — the lengths stay undimensioned, fixed by the Apex2 closing constraint): `tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`, `γ_g = Σ − γ_p`, cone distance `R = (PPD/2) / sin γ_p`; seed `|Apex→A| = R · cos γ_p` and `|Apex→B| = R · cos γ_g`. (γ_p, γ_g are also reused in §3 for the virtual tooth radii.) Seeding A/B merely by a pitch diameter is wrong for Σ≠90° and can send the solver to the wrong branch.

Draw a construction line from Apex to Apex 2. This line shall be called the Pitch Line. Each end of the Pitch Line should be constrained to the respective points using coincidence constraint.

From the Apex 2, create a construction line in either side whose length is constrained Module * 1.25, and are perpendicular to the Pitch Line. Constrain them against the Pitch Line with the Perpendicular constraint. Let the line drawn towards the anchor line be Driving Gear Dedendum, whose end point shall be point D. The one drawn away from the anchor line be Pinion Gear Dedendum; whose end point shall be point C.

Draw two construction lines, from the Apex to the point D and point C, respectively. Apply coincidence constraints on beginning and end of these lines. These lines shall be the Root Axis for driving and pinion gear, respectively.

From point A, create construction line colinear with the line from Apex to point A, extending for length equals to module (but do NOT add dimensional constraint). The line should receive a colinear constraint, and the end of Apex->A and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point E.

Draw a construction line from point C to point E. constrain each end to respective points from pre-existing lines. The lines A->E and C->E should be constrained with perpendicular constraint.

From point B, create a construction line colinear with Apex->B, extending for length equals to module (but do NOT add dimensional constraint). The line should receive a colinear constraint, and the end of Apex->B and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point F.

Draw a construction line from point D to point F. constrain each end to respective points from pre-existing lines. The lines B->F and D->F should be constrained with perpendicular constraint.

Draw a construction line from point E colinear to line A->E, with length equal to module (but do NOT add dimensional constraint). Constrain point E and the beginning of this line. Let the end be known as point G.

From point C, draw a line with length equal to module (but do NOT add dimensional constraint). Constrain point C and the beginning of this line. Let the end of the new line be point H. Line C->H should be colinear with line Apex2->C.

Connect point G and H with a line. Constrain end points of line accordingly with coincidence constraints. Constrain line E->G and H->G with a perpendicular constraint.


Draw a construction line from point F colinear to line B->F, with length equal to module (but do NOT add dimensional constraint). Constrain point F and the beginning of this line. Let the end be known as point I.

From point D, draw a line with length equal to module (but do NOT add dimensional constraint). Constrain point D and the beginning of this line. Let the end of the new line be point J. Line D->J should be colinear with line Apex2->D.

Connect point I and J with a line. Constrain end points of line accordingly with coincidence constraints. Constrain line F->I and J->I with a perpendicular constraint.

Create an **offset dimension between the B->Apex2 perpendicular drop line and J->I**. **"B->Apex2" here is the perpendicular construction line of length DPD/2 you drew earlier "From B, perpendicular to the Driving Gear Shaft Axis" — NOT the Apex->B shaft axis.** This matters: `addOffsetDimension` requires its two lines be parallel, and J->I is parallel to that perpendicular drop (both run in the dedendum/perpendicular direction), whereas the Apex->B shaft axis is not parallel to J->I — passing the shaft axis makes the offset dimension unsolvable (`VCS_SKETCH_SOLVING_FAILED`). Set the value equal to Driving Gear Base Height _if_ specified (non-0); otherwise `module * Driving Gear Teeth Number / 8`.

Create an **offset dimension between the A->Apex2 perpendicular drop line and G->H** — again "A->Apex2" is the perpendicular construction line of length PPD/2 (drawn "From A, perpendicular to the Pinion Gear Shaft Axis"), **not** the Apex->A shaft axis; G->H is parallel to it, the shaft axis is not. The value should be equal to Pinion Gear Base Height _if_ specified (non-0); otherwise `Driving Gear Base Height * (Pinion Gear Teeth Number / Driving Gear Teeth Number)`.

Draw a line from A to G. Constrain endpoints appropriately.

Constrain Point I with center point.


Draw a construction line away from Apex, starting from point G, extending along Apex->A, and call its end point K. Then **pin K with two point-on-line coincident constraints** — `addCoincident(K, line Apex->A)` and `addCoincident(K, the Pinion Dedendum line Apex2->C extended)` — rather than `addCollinear` on the connecting lines. By the time K is added, G and C are already fixed, so an `addCollinear` here over-constrains the sketch and Fusion errors; the two point-on-line coincidents locate K exactly (intersection of the two lines) without over-constraining. Draw a construction line from point C to K for reference.

At this point all of A, B, C, D, H, J exist, so resolve the **Maximum Face Width** (see the Parameters section) and apply it before using Face Width below: cap the auto default to it, and reject a user value that exceeds it. Skipping this makes the M->N line below push point N across the shaft axis for gear ratios above ~√2, which fails the gear-body revolve in "Create the Pinion Gear".

Create line M->N. **Seed it near its solved position** (the solver is seed-sensitive — see PLAYBOOK): seed M at roughly the **midpoint of Apex->C**, and seed N by sliding from that M-seed **along the C->H direction far enough to roughly reach line A->Apex2** (e.g. by the distance from the M-seed to A). Do NOT seed M/N just `Face Width` away from C/H — that starts N near H, far from its constraint target (line A->Apex2), and the solve fails to converge. Then apply **exactly these constraints** — all four are required, and the two coincident pins are what make the Maximum-Face-Width guarantee real (N reaches A exactly at the cap):
- `addCoincident(M, Pinion Root Axis)` — M lies on the Apex->C root axis;
- `addCoincident(N, line A->Apex2)` — N lies on the A->Apex2 line (this is the pin; do not merely place N numerically);
- `addParallel(M->N, C->H)` — the toe line is parallel to C->H;
- `addOffsetDimension(C->H, M->N, textPoint).parameter.value = Face Width` — the parallel offset between C->H and M->N equals Face Width.

(Because N is pinned to A->Apex2, the Maximum Face Width above is exactly the value at which N reaches A; the capped Face Width keeps N between Apex2 and A.)

Let the beginning of this new line be point M, the end be point N. Draw a line from M to C. Draw a line from N to A.

Draw a construction line away from Apex, starting from point I, extending along Apex->B, and call its end point L. **Pin L the same way as K** — `addCoincident(L, line Apex->B)` and `addCoincident(L, the Driving Dedendum line Apex2->D extended)`; do not use `addCollinear`. Draw a construction line from point D to L for reference.

Create line O->P, the mirror of M->N on the driving side. Seed it the same way (O near the midpoint of Apex->D, P slid along D->J toward line B->Apex2), then apply the same four constraints:
- `addCoincident(O, Driving Root Axis)` — O on the Apex->D root axis;
- `addCoincident(P, line B->Apex2)` — P on the B->Apex2 line;
- `addParallel(O->P, D->J)`;
- `addOffsetDimension(D->J, O->P, textPoint).parameter.value = Face Width`.

Let the beginning of this new line be point O, the end be point P. Draw a line from O to D. Draw a line from P to B. Draw line from B to I.

### 3: Gear Tooth Profiles

**API note for this whole section:** pass the relevant **sketch line directly** to `setByAngle` and
`setByDistanceOnPath` (e.g. `setByAngle(lineCtoK, '90 deg', gearProfilesPlane)`,
`setByDistanceOnPath(lineCtoK, 1.0)`). Do **NOT** wrap the line in `adsk.fusion.Path.create(...)`
first — `Path.create` on a sketch curve fails here with `InternalValidationError :
Utils::getObjectPath(sketchCurve, …, nullptr, contextPath)` (the cross-component path-resolution
trap, the same one the gear-body section warns about). The plane/axis builders take the line as-is.

Compute the pinion's virtual (back-cone / Tredgold) tooth number from the closed form, **not** by measuring Apex2->K (K is still used as the tooth's center point, below): virtual pitch radius = `(Pinion Gear Pitch Diameter / 2) / cos(γ_p)`, where `γ_p` is the pinion pitch-cone half-angle from §2 (`tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`). Virtual tooth number = `floor(2 · virtualPitchRadius / Module)`, as an int. (Module here is the raw mm value; the radius is a length — keep the unit handling consistent, see the Units note.)

Create a new plane that includes line C->K. Use setByAngle to make this plane perpendicular to the Gear Profiles sketch plane.

Using the new plane and point K as the center point, create a spur gear tooth profile with module and the virtual tooth number obtained from the previous step. Draw it **already rotated 180°** by passing `angle=math.radians(180)` to the spur tooth generator's `draw(anchorPoint, angle=…)` (see Dependencies) — the generator rotates the whole tooth by that angle; do not draw it flat and rotate the sketch afterward.

Create a construction axis through point K, normal to the plane the tooth profile was drawn on, via `setByTwoPlanes` (`setByPerpendicularAtPoint` would need a `BRepFace`). The two planes are: the **Gear Profiles plane** and a **helper plane built `setByDistanceOnPath(C->K line, 1.0)`** (perpendicular to C->K at its far end, K); their intersection is the line through K normal to the tooth plane.

Compute the driving gear's virtual tooth number the same way (L is still the tooth center): virtual pitch radius = `(Driving Gear Pitch Diameter / 2) / cos(γ_g)`, where `γ_g = Σ − γ_p` is the driving pitch-cone half-angle from §2. Virtual tooth number = `floor(2 · virtualPitchRadius / Module)`, as an int.

Create a new plane that includes line D->L. Use setByAngle to make this plane perpendicular to the Gear Profiles sketch plane.

Using the new plane and point L as the center point, create a spur gear tooth profile with module and the virtual tooth number obtained from the previous step. Draw it **already rotated 180°** by passing `angle=math.radians(180)` to the spur tooth generator's `draw(anchorPoint, angle=…)` (see Dependencies), exactly as for the pinion.

Create a construction axis through point L, normal to the tooth plane, via `setByTwoPlanes`: the **Gear Profiles plane** intersected with a **helper plane `setByDistanceOnPath(D->L line, 1.0)`**.

## Create the Pinion Gear

Create a new component as a child of the **Bevel Gear** component (the same component that owns Design — *not* the user's Parent Component; this intentionally overrides the looser "child of Parent Component" phrasing so the pair nests cleanly inside Bevel Gear). The resulting bodies for this gear end up in this component. (Implementation note: Fusion's API rejects cross-sibling sketch and project calls even when the target is activated or the entities are wrapped in `createForAssemblyContext` proxies, so the actual feature operations run in the Design component and the finished bodies are `moveToComponent`'d here at the end. The visible end state is identical.)

Open a **fresh sketch on the axial (Gear Profiles) plane** named e.g. `Pinion Profile`, project the six points A, G, H, C, M, N into it, and draw the closed hexagon A -> G -> H -> C -> M -> N -> A there as six `SketchLine`s — **one profile sketch per gear**, so `sketch.profiles` holds exactly this one loop (do not draw both gears' hexagons in the shared Gear Profiles sketch — that would leave two identically-shaped loops to disambiguate).

**The shaft axis used by every body operation below is this profile sketch's first edge (the A->G line you just drew), NOT the §2 `Apex->A` construction line.** A→G is colinear with the shaft axis but lives in the *same* sketch as the profile, which is what Fusion's revolve/pattern/path accept; reusing the §2 construction line (a different sketch) fails or misbuilds. Use that A->G edge as the axis for: the revolve, the circular pattern, the bore-plane `setByDistanceOnPath`, and the meshing-rotation axis.

Revolve the profile around that A->G edge; let the result be Pinion Gear Body. Because M->N is one edge of the revolved profile, the body already carries the conical face produced by sweeping M->N around the axis — that face is reused as the cutting tool below.

Create a loft using the Apex point and the Pinion Gear Tooth profile. Let this be the Pinion Gear Tooth Body.

Cut the Pinion Gear Tooth Body twice, in order:

1. First, cut using the conical face on Pinion Gear Body that was generated from the M->N edge. (Implementation note: Fusion's `Path.create` on a sketch line whose owner sketch is in a sibling/parent component fails with `getObjectPath(sketchCurve, ..., nullptr, contextPath)`, so building a fresh revolved surface from the M->N sketch line does not work. Instead, locate the existing M->N face on Pinion Gear Body by walking its faces, keeping only `ConeSurfaceType` faces whose underlying surface contains both M and N in world coordinates within tolerance `1e-2` cm, and — **if more than one qualifies — choosing the best match (smallest total endpoint distance), not the first found**; use that face as the splitting tool with `isSplittingToolExtended=True`.)
2. Then, cut the resulting bodies using the conical face on Pinion Gear Body that was generated from the C->H edge. Locate it the same way: among the cone faces of Pinion Gear Body, pick the **best-matching** one whose underlying surface contains both C and H in world coordinates (min total distance), and use it as the splitting tool with `isSplittingToolExtended=True`.

Apply the cuts **sequentially over the current set of pieces** (after cut 1, feed each resulting piece into cut 2). **The second cone may not intersect every piece** — wrap each split in a `try/except RuntimeError` and, when the message contains `SPLIT_TARGET_TOOL_NOT_INTERSECT` (or its localized form `交差`), keep that piece intact and continue rather than aborting. After both cuts you must have **exactly three pieces**; it is an error otherwise.

**Make every cut failure self-diagnosing** (these errors are how the build is debugged when run headless, so they must carry the measured numbers, not a bare count):
- **Cone-face lookup never silently returns "not found."** When no `ConeSurfaceType` face has both cut-line endpoints within tolerance, the finder must `raise` an error naming the cut (#1/#2, pinion/driving) and listing, for **every** cone face on the body, its two endpoint distances `(dA, dB)` and the tolerance — so a too-tight tolerance vs. a genuinely-absent face is distinguishable. Do **not** return the bodies unchanged on a miss (that turns a found-no-face into a misleading downstream "wrong piece count").
- **Log each cut's outcome** with `futil.log(..., force_console=True)`: how many bodies went in, which face was selected (with its `(dA,dB)`), and how many pieces came out (or "tool did not intersect, kept intact").
- **The piece-count error must report the per-cut history**, e.g. `"<gear>: expected 3 pieces, got N (cut#1: faces found=…, selected dist=…, produced=…; cut#2: …)"`, so the message alone reveals which cut under- or over-split and whether it was a tolerance/face-selection problem.

Remove the piece that contains the Apex. Identify it by `BRepBody.pointContainment(apexWorld)` returning `PointInside` or `PointOn`; remove it via `RemoveFeatures.add()` (so the deletion appears in the timeline).

Of the two remaining pieces, remove the smaller one (compare by `BRepBody.physicalProperties.volume`); remove it via `RemoveFeatures.add()`.

Circular-pattern the remaining tooth piece around the **A->G edge** (the shaft axis — the same in-sketch profile edge used for the revolve, not the §2 `Apex->A` construction line). The number of copies equals the Pinion Gear Teeth Number. (Although the pitch diameter shrinks from heel toward apex, the *angular* spacing around the shaft axis stays constant at `360° / N` for the entire face width — the radial taper is already produced by the loft from Apex to the heel-end tooth profile, so the pattern just rotates that single tapered tooth into N evenly spaced copies.)

Join all patterned tooth pieces with Pinion Gear Body in a single Combine-Join (Pinion Gear Body as the target, the patterned tooth bodies as the tools).

If Enable Bore is checked, cut a cylindrical bore through Pinion Gear Body along the shaft axis. The bore diameter is the Pinion Gear Bore Diameter if specified (non-zero); otherwise use `Pinion Gear Pitch Diameter / 4`. Build the bore plane `setByDistanceOnPath(A->G edge, 0.0)` (normal to the shaft at its start; pass the in-sketch A->G edge, not the §2 construction line). Sketch the bore circle **centered at the sketch origin (0,0,0)** — the plane is rooted at the shaft start, so the origin is on the axis; no projection or diameter dimension is needed. Extrude-cut it with `setSymmetricExtent(ValueInput.createByReal(max(1000.0, bore_diameter_cm * 100)), False)` so it pierces the whole body either way, restricting `participantBodies` to `[Pinion Gear Body]`. Skip this step entirely if Enable Bore is unchecked.



## Create the Driving Gear

Create a new component as a child of the **Bevel Gear** component (not the user's Parent Component — same override as the Pinion Gear section). The resulting bodies end up in this component. (Same Fusion API caveat as the Pinion Gear section above.)

Open a **fresh sketch on the axial plane** named e.g. `Driving Profile`, project B, I, J, D, O, P into it, and draw the closed hexagon B -> I -> J -> D -> O -> P -> B there (one profile per gear, as for the pinion). As for the pinion, **the shaft axis for all body operations is this sketch's first edge (the B->I line), not the §2 `Apex->B` construction line** — used for the revolve, pattern, bore-plane path, and mesh axis. Revolve it around that B->I edge; let the result be Driving Gear Body. As with the pinion, the O->P edge revolves into a conical face on the body that is reused as the cutting tool below.

Create a loft using the Apex point and the Driving Gear Tooth profile. Let this be the Driving Gear Tooth Body.

Cut the Driving Gear Tooth Body twice, in order:

1. First, cut using the conical face on Driving Gear Body generated from the O->P edge: among the `ConeSurfaceType` faces of Driving Gear Body, pick the **best-matching** one whose surface contains both O and P in world coordinates within `1e-2` cm (smallest total endpoint distance, not the first found); splitting tool `isSplittingToolExtended=True`.
2. Then, cut the resulting bodies using the cone face generated from the D->J edge (the analogue of C->H on the pinion), located the same best-match way (contains D and J).

Apply the two cuts sequentially over the current pieces, catching `SPLIT_TARGET_TOOL_NOT_INTERSECT` / `交差` and keeping the un-split piece intact (exactly as for the pinion). After both cuts you must have exactly three pieces; error otherwise.

Remove the piece that contains the Apex (using `pointContainment` + `RemoveFeatures.add()` as described for the pinion).

Of the two remaining pieces, remove the smaller one (compare by `BRepBody.physicalProperties.volume`); remove it via `RemoveFeatures.add()`.

Circular-pattern the remaining tooth piece around the **B->I edge** (the shaft axis — the in-sketch profile edge used for the revolve, not the §2 `Apex->B` construction line). The number of copies equals the Driving Gear Teeth Number, for the same reason as the pinion: the angular spacing around the shaft axis is constant at `360° / N`, regardless of the radial taper from heel toward apex.

Join all patterned tooth pieces with Driving Gear Body in a single Combine-Join (Driving Gear Body as the target, the patterned tooth bodies as the tools).

If Enable Bore is checked, cut a cylindrical bore through Driving Gear Body along the shaft axis, exactly as for the pinion (bore plane `setByDistanceOnPath(B->I edge, 0.0)`, circle centered at sketch origin, `setSymmetricExtent(…, False)` large extent, `participantBodies = [Driving Gear Body]`). The bore diameter is the Driving Gear Bore Diameter if specified (non-zero); otherwise use `Driving Gear Pitch Diameter / 4`. Skip if Enable Bore is unchecked.

**Meshing rotation (driving gear only) — do this here, in the Design component, before the body is moved out.** Rotate Driving Gear Body about the Driving Gear Shaft Axis (Apex->B) by `180° / Driving Gear Teeth Number` (half a tooth pitch) so a driving valley sits where the pinion tooth crosses the axial plane, giving the interlocked meshing look. Use a `MoveFeatures` free-move with a rotation matrix: `Matrix3D.setToRotation(math.radians(180/drivingTeeth), axisVector, originPoint)` where the axis vector and origin come from the **world** endpoints of the B->I profile edge (`startPoint.worldGeometry` → `endPoint.worldGeometry`), the same shaft axis used for the revolve/pattern. Use `defineAsFreeMove(matrix)` — `defineAsRotate` rejects a `SketchLine` axis, and adding a construction axis in the moved-out gear component fails with "Environment is not supported", which is why this runs in Design before `moveToComponent`.

## Cleanup

Recursively walk the Bevel Gear component tree (dedupe by `entityToken`) and hide every sketch, construction plane, and construction axis with `isLightBulbOn = False` (construction planes/axes are **not** hidden by `isVisible`; sketches use `isVisible = False` or the same light-bulb). Leave only the two finished gear bodies visible.

(The driving gear's half-tooth-pitch **meshing rotation** is performed earlier, at the end of "Create the Driving Gear", in the Design component before the body is moved out — see that section. It is not a cleanup step, and the rationale is: both gears are patterned from a starting tooth in the axial plane, so without the offset a driving tooth and a pinion tooth would both sit at the axial-plane crossing and visually collide; rotating the driving gear by half its own tooth pitch interlocks them.)