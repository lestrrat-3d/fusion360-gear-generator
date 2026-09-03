# Bevel Gear — compiled step list

The proof for this step list is `proof/bevelgear/geometry_test.go`, `proof/bevelgear/sketches_test.go`, `proof/bevelgear/tooth_test.go`, `proof/bevelgear/solids_test.go`, `proof/bevelgear/frustum_test.go`, `proof/bevelgear/bodies_test.go`, `proof/bevelgear/segments_test.go`, `proof/bevelgear/spiral_test.go` and the generated `proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `0d2d197ed8d4f921a520066e18075e5463cabf5c` |
| `spec/bevelgear/fusion.md` | `f4c93cac37dc49e16bfc1672a7e5fc601a4ff925` |
| `spec/bevelgear/spiral-tooth-trace.md` | `c9ec08561ced7975aa0ed9ad6a330186259c0d08` |
| `spec/spurgear/instructions.md` | `560549a350c789b6598ee49b72d3cd7b4460e214` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `aa97523b214dbc98b679e8eea73c56c5115c4e54` |

## 1 `[PROSE]` Add the seventeen dialog inputs

Add a `BevelGearCommandInputsConfigurator` class with `@classmethod def configure(cls, cmd)`, which
adds the seventeen inputs to `cmd.commandInputs` in this display order. The order is contract
surface: Target Plane is first so Fusion's auto-focus lands on it (`[PB-AUTOFOCUS-FIRST]`), Center
Point follows so the user flows from plane to point, and Parent Component is third because it is
already pre-selected.

| # | label | id | call | unit | default |
|---|---|---|---|---|---|
| 1 | Target Plane | `targetPlane` | `addSelectionInput` | — | tooltip `Plane the bottom of the driving gear sits flush against` |
| 2 | Center Point | `centerPoint` | `addSelectionInput` | — | tooltip `Point the driving bevel gear is centered on` |
| 3 | Parent Component | `parentComponent` | `addSelectionInput` | — | tooltip `Component the gear pair is created under`; pre-selects the root component |
| 4 | Module | `module` | `addValueInput` | `''` | `createByReal(1)` |
| 5 | Shaft Angle | `shaftAngle` | `addValueInput` | `deg` | `createByString('90 deg')` |
| 6 | Driving Gear Teeth | `drivingTeeth` | `addValueInput` | `''` | `createByReal(31)` |
| 7 | Pinion Gear Teeth | `pinionTeeth` | `addValueInput` | `''` | `createByReal(31)` |
| 8 | Driving Gear Base Height | `drivingBaseHeight` | `addValueInput` | `mm` | `createByReal(to_cm(0))` |
| 9 | Pinion Gear Base Height | `pinionBaseHeight` | `addValueInput` | `mm` | `createByReal(to_cm(0))` |
| 10 | Enable Bore | `boreEnable` | `addBoolValueInput` | — | `True` |
| 11 | Driving Gear Bore Diameter | `drivingBore` | `addValueInput` | `mm` | `createByReal(to_cm(0))` |
| 12 | Pinion Gear Bore Diameter | `pinionBore` | `addValueInput` | `mm` | `createByReal(to_cm(0))` |
| 13 | Face Width | `faceWidth` | `addValueInput` | `mm` | `createByReal(to_cm(0))` |
| 14 | Tooth Spacing | `toothSpacing` | `addValueInput` | `mm` | `createByReal(to_cm(0))` |
| 15 | Mean Spiral Angle | `spiralAngle` | `addValueInput` | `deg` | `createByString('35 deg')` |
| 16 | Hand of Spiral | `spiralHand` | `addDropDownCommandInput` | — | `DropDownStyles.TextListDropDownStyle`, items `Right` selected and `Left` |
| 17 | Cutter Radius | `cutterRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` |

The three selection inputs take their filter enums through `addSelectionFilter` and their limits
through `setSelectionLimits(1, 1)` (`[PB-SELECTION-FILTER-ENUM]`, `[PB-SELECTION-DECL]`): plane —
`ConstructionPlanes`, `PlanarFaces`; point — `ConstructionPoints`, `SketchPoints`; parent —
`Occurrences`, `RootComponents`. Every numeric default is passed in internal units
(`[PB-DIALOG-DEFAULT-UNITS]`). Seventeen module-level `INPUT_ID_*` constants hold the id strings in
row order, plus `_HAND_RIGHT = 'Right'` and `_HAND_LEFT = 'Left'`. Bevel registers **no** Fusion user
parameters at all — every value is precomputed in Python and written into geometry numerically
(`[PB-PRECOMPUTED-MODE]`), so there are no `PARAM_*` names.

**Conditional visibility.** Hand of Spiral and Cutter Radius are shown only when the Mean Spiral
Angle is above zero. Add `@classmethod _updateSpiralInputVisibility(cls, inputs)`, which reads the
spiral input's `.expression` through `evaluateExpression` in `'rad'` and sets `isVisible` on the two
spiral-only inputs to that value being positive; it returns early when any of the three inputs is
missing, and wraps the evaluation in `try`/`except`, leaving both shown on failure. `configure` calls
it as its last step, and `@classmethod def handle_input_changed(cls, args)` calls it again on every
input change. Both `configure` and `handle_input_changed` are bound by name from
`commands/bevelgear/entry.py`, so they are methods this module defines for the framework to call
rather than calls it makes.

<!-- check-step-calls: ignore configure handle_input_changed -->
<!-- check-compile: ignore handle_input_changed -->

**From:** `spec/bevelgear/instructions.md` L131–172, L246–256; `.claude/skills/generate-gear/PLAYBOOK.md` L124–146, L336–343

## 2 `[PROSE]` Read and validate every input, and resolve the derived values

`BevelGearGenerator.__init__(self, design)` stores `self.design` and `self.bevelOccurrence = None`.
`generate(inputs)` calls `_readInputs(inputs)` first, before anything creates an occurrence, and
`_readInputs` returns the 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth,
pinionTeeth, shaftAngle_deg)` and stashes the rest on `self`.

Read each numeric and angle input with `evaluateExpression` on the input's expression
(`[PB-EVAL-EXPRESSION]`): every one comes back in Fusion internal units — centimetres for length,
radians for angle — whatever unit string is passed, so a `deg` field needs `math.degrees` before any
degree range check. The boolean is read with `get_boolean` and the dropdown through its
`selectedItem` name, defaulting to `Right`. Both tooth counts are coerced with `int(round(...))`.

**Units.** `Module` is read with `''` and comes back as a raw number that means millimetres, so every
length derived from it must be converted to centimetres before it touches geometry: the two pitch
diameters, the Cone Distance, the dedendum, the module-length extensions, and the default Face Width.
The `mm` and `deg` inputs are already internal and must not be converted again. Mixing the two makes
the gear come out about ten times off and the Face Width bound meaningless.

Validate in this order, because each check is what makes the next one well posed.

1. Module above zero; both tooth counts at least three; the heights, bores, width and tooth spacing
   non-negative; the Mean Spiral Angle in `[0, 60)` degrees; the Cutter Radius non-negative.
2. Shaft Angle at least 30 degrees and below the **Maximum Shaft Angle**, which is
   `min(degrees(acos(-smaller / larger)), 150)` over the two pitch diameters, with the `acos` half
   exclusive and the 150 half inclusive. Name the computed limit in the rejection message. The
   `acos` is a hard singularity: a pitch cone angle reaching 90 degrees turns that gear's cone
   inside out, so `R * cos(gamma)` passes through zero and changes sign.
3. Compute the two pitch cone angles from the closed form —
   `tan(gamma_p) = sin(Sigma) * PPD / (DPD + PPD * cos(Sigma))`, `gamma_g = Sigma - gamma_p`,
   `R = (PPD / 2) / sin(gamma_p)` — and stash them as `self._gamma_p`, `self._gamma_g`.
4. Check each gear's tooth count against its own **Minimum Teeth** floor, `5.27 * cos(gamma)` for
   that gear's own gamma, on top of the blanket three. Name the computed floor.
5. Resolve each gear's base height against its own two bounds:
   `Minimum Base Height = 1.05 * 1.25 * Module * sin(gamma)` and
   `Maximum Base Height = 0.95 * (r - 1.25 * Module * cos(gamma)) * tan(gamma)`. Raise a fallback
   below the minimum, cap one above the maximum, and reject a user value outside either end naming
   the bound it broke. The driving fallback is `Module * drivingTeeth / 8`; the pinion fallback is
   the **resolved** driving height times the tooth ratio, then held to the pinion's own bounds.
   Running the Minimum Teeth check first is what makes this window non-empty.

**Two bounds this step cannot resolve yet.** The Maximum Face Width needs the solved §2 geometry, so
Face Width is resolved in step 6; the bore diameters resolve to `Pitch Diameter / 4` when the input
is zero, and are consulted only when Enable Bore is checked.

No harness reaches this step, and no substitute was attempted, because validation builds nothing: it
reads numbers and either raises or returns them. There is no geometry to substitute for. What every
bound resolved here is FOR is checked in step 6, on the figure those numbers build: `assertHeel`
measures the heel corner's distance from the shaft axis on the solved lattice and compares it with
`r - baseHeight / tan(gamma)`, so the Maximum Base Height is checked against the crossing at
`r * tan(gamma)` it exists to stay under, and `stepGearProfiles` re-checks the Shaft Angle, the
Minimum Teeth floors and both base-height windows on every case it builds.

**From:** `spec/bevelgear/instructions.md` L27–129, L212–244, L273–295, L390–400;
`.claude/skills/generate-gear/PLAYBOOK.md` L106–118, L791–797

## 3 `[PROSE]` Build the occurrence tree

Create the occurrences directly with `parent.occurrences.addNewComponent(...)`, passing
`adsk.core.Matrix3D.create()`, and name each through `occurrence.component.name`
(`[PB-OCCURRENCE-TREE]`). The tree is the Bevel Gear component under the user's Parent Component,
and a Design component under it. The two per-gear components are created later, one at the start of
each gear's body chain. Bevel does not subclass `base.Generator`, so none of `getOccurrence`,
`addParameter` or `createSketchObject` is used.

**Never call `activate` on any occurrence** (`[PB-NEVER-ACTIVATE]`, `[BEVEL-F-NEVER-ACTIVATE]`). The
Anchor Sketch is created on the user's external, root-owned target plane, and an activated occurrence
resolves that external plane in its own local frame, collapsing the build onto world XY whatever
plane the user picked. Every feature runs in the single Design component, so no cross-sibling
reference is ever needed (`[PB-NO-CROSS-SIBLING]`). The one exception is the spiral crown's scale
feature, in step 19.

`deleteComponent()` calls `deleteMe()` on the top occurrence, and the shared entry point calls it on
any exception.

No harness reaches this step, and no substitute was attempted: `decad.Document` owns a flat list of
bodies with no owner and no assembly context, and `sketch.World` owns planes and sketches the same
way, so there is no tree to build and no body whose geometry would differ if one were.

Three of the names above are mentions rather than calls this module makes. `generate` and
`deleteComponent` are the two methods the entry point binds by name, so the module defines them; and
`deleteMe` is reached only from inside `deleteComponent`, which the entry point enters on an
exception, so it is not reachable from `generate`.

<!-- check-step-calls: ignore activate generate deleteComponent deleteMe -->

**From:** `spec/bevelgear/instructions.md` L19–23, L246–272, L434–442;
`spec/bevelgear/fusion.md` L145–150; `.claude/skills/generate-gear/PLAYBOOK.md` L735–752

## 4 `[GO]` Anchor Sketch

Create the sketch with `sketches.add` **directly on the user-selected target plane**, whether the
selection is a `ConstructionPlane` or a `PlanarFace`, and name it `Anchor`. Do not re-derive or
offset it: a coplanar construction plane built inside a sub-component resolves in that component's
own frame and silently loses the selected plane's orientation (`[PB-USE-SELECTED-PLANE]`).

Mark the centre by projecting the user's centre point in with `sketch.project(centerPoint)`.

Draw the Anchor Line with `sketchCurves.sketchLines.addByTwoPoints`, seeding its two endpoints at
exactly plus and minus 0.5 cm from the projected centre along sketch-local X, so the seeded length is
10 mm. Then constrain it:

- `geometricConstraints.addCoincident(projectedCenter, anchorLine)` — the centre lies on the line;
- `geometricConstraints.addMidPoint(projectedCenter, anchorLine)` — and bisects it. Add both, not
  the midpoint alone;
- `sketchDimensions.addDistanceDimension(...)` on the two endpoints, with **no** assignment to
  `.parameter.value`: the dimension simply locks the seeded 10 mm, and the value is arbitrary
  because nothing downstream reads this length;
- `geometricConstraints.addHorizontal(anchorLine)` — sketch-local, per `[PB-REFLINE-DIRECTION]`.
  A world-axis lock would mis-orient the line on a tilted target plane.

Stash the projected-centre `SketchPoint` on `self`, because step 6 re-projects **that** point rather
than the raw user selection. End the step by gating `sketch.isFullyConstrained` and raising, naming
the sketch (`[PB-FULL-CONSTRAINT]`, `[BEVEL-F-FULL-CONSTRAINT]`).

The proof draws this sketch as `stepAnchorSketch` and checks the solved length, the bisection and the
line's direction. Its length dimension is written there as a signed horizontal distance, because the
engine's target is signed while Fusion's aligned dimension is a magnitude whose direction comes from
the seed (`[PB-DIM-VALUE-SEMANTICS]`); the sign is what refuses the end-for-end flip that midpoint
plus magnitude plus horizontal would otherwise leave as a second answer.

<!-- proof-run: proofkit.Run(anchorCases, stepAnchorSketch) -->

**From:** `spec/bevelgear/instructions.md` L444–448; `spec/bevelgear/fusion.md` L21–26;
`.claude/skills/generate-gear/PLAYBOOK.md` L470–479, L753–764

## 5 `[PROSE]` Gear Profiles Plane

Build the plane with `constructionPlanes.createInput()` then
`setByAngle(anchorLine, ValueInput.createByString('90 deg'), targetPlane)` and
`constructionPlanes.add(...)`, and name it `Gear Profiles Plane`. Pass the `SketchLine` directly;
never wrap it in `Path.create` (`[PB-CONSTRUCTION-PLANES]`).

**Build it off the original `targetPlane`**, not a re-derived copy (`[PB-USE-SELECTED-PLANE]`). This
is the second and last place the target plane's orientation reaches the bodies, and substituting a
different plane here collapses the gear onto world XY exactly as re-deriving the Anchor Sketch's
plane does.

A construction plane on its own closes no sketch and bounds no body, so there is nothing for a gate
to hold. The substitute that was considered and rejected is building a sketch on it and gating that:
the sketch would be step 6's, which is already gated, and the plane's own contribution — its
orientation relative to the target plane — is not a property of any sketch drawn on it, because the
sketch engine places a sketch in whatever frame it is given. What the plane is for is proved one step
later all the same: the §2 lattice is built in this plane's own 2-D frame, and `stepGearProfiles`
reaches DOF 0 there with the apex placed by an in-plane perpendicular to the projected anchor line —
which is only the target-plane normal because this plane is perpendicular to the target plane and
contains the anchor line.

<!-- check-step-calls: ignore Path.create -->

**From:** `spec/bevelgear/instructions.md` L450–452; `spec/bevelgear/fusion.md` L115–126;
`.claude/skills/generate-gear/PLAYBOOK.md` L716–727

## 6 `[GO]` Gear Profiles sketch — the §2 lattice

Create the sketch on the Gear Profiles Plane and name it `Gear Profiles`. **Every line in it is a
construction line** — `isConstruction = True` — the lattice lines, the two toe lines and the short
reference lines alike; the solid features later consume the per-gear Profile sketches, never a §2
curve. **Every length dimension in it is an aligned distance dimension**: this figure has no
axis-aligned line in it, so `addDistanceDimension` is called with
`adsk.fusion.DimensionOrientations.AlignedDimensionOrientation` and a horizontal or vertical
orientation would dimension a projection instead of a length.

Three construction rules govern the whole sketch and are as binding as the geometry.

- **`[BEVEL-F-COINCIDENT-STYLE]`** — build **every** §2 line, lattice and short reference line
  alike, from raw `adsk.core.Point3D.create` coordinates, then `addCoincident` each endpoint to the
  point it meets. Never pass an existing `SketchPoint` into `addByTwoPoints` to share it. Sharing
  without a coincident leaves the sketch under-constrained; sharing *and* coinciding makes the solve
  fail outright.
- **`[BEVEL-F-LINE-ONCE]`** — each named segment is drawn once and the helper that draws it returns
  the line object, which later steps reuse. A second line over the same segment carries its own
  constraints and over-determines the coupled net.
- **`[BEVEL-F-DRIVEN-DIMS]`** — the along-shaft lengths Apex→A and Apex→B and the four
  module-length extensions carry **no** dimension. They are driven by the closing and collinear
  constraints, and dimensioning them is the over-constraint `[PB-NO-OVERCONSTRAIN]` warns about.

**Projection.** Project the Anchor Sketch's stashed centre `SketchPoint` and the Anchor Line with
`sketch.project(entity)`. Project the anchor-sketch point, not the raw user selection: both are
coincident, but the anchor-sketch point keeps the chain inside the Design component. Write the call
as `project`. The compiled API reference declares `project2(entities, isLinked)` and no `project`, so
every gate in this repo reports the call as unverified; that report is expected, this repo's settled
position is to keep the call and keep reporting it, and `project2` is not a drop-in replacement since
it takes and returns lists.

<!-- check-step-calls: ignore project2 -->

**The figure, in the sketch's own 2-D frame** (`[BEVEL-F-APEX-LOCAL]` — never compute a §2 position
from a world round trip; the single permitted world use is reading `targetPlane.geometry.normal` as a
*direction* to pick the grow side, per `[BEVEL-F-GROW-SIDE]`). With `c` the projected centre and `d`
the projected anchor line's unit direction, `perp = (-d.y, d.x)` signed toward the target normal:

1. **Centre to Apex.** A construction line from `c`, `addCoincident` on its start, and
   `addPerpendicular` against the projected anchor line. Seed its free end at
   `c + perp * (R * cos(gamma_g) + resolved Driving Gear Base Height)` — that is where the net closes
   it, and the old `Driving Gear Pitch Diameter` seed sat 11.6 mm away on the default pair
   (`[PB-SEED-NEAR]`). Add no length constraint. Its end is the **Apex**.
2. **Driving Gear Shaft Axis.** From the Apex toward the anchor line, seeded at
   `apex - perp * (R * cos(gamma_g))`, with `addCoincident` on its start and
   `addParallel(drivingShaftAxis, centerToApex)`. Do **not** use `addVertical`: it forces the
   sketch's world vertical, which is wrong on any tilted target plane. Its end is **B**, undimensioned.
3. **Pinion Gear Shaft Axis.** From the Apex, its direction the driving direction rotated about the
   Apex by the Shaft Angle. Form **both** candidate ends and keep the one with the greater X in this
   sketch; do not rotate one fixed sense and flip only on a negative X, because when both candidates
   have positive X that shortcut keeps the wrong one and mirrors the whole gear. Add
   `addCoincident` on its start and `sketchDimensions.addAngularDimension(pinionAxis, drivingAxis,
   textPoint)` set to the Shaft Angle, with the text point inside the wedge — on the interior
   bisector at `apex + normalize(pinionDir + drivingDir) * (PPD / 4)` — so it measures the angle and
   not its supplement (`[PB-ANGULAR-DIM]`). Its end is **A**, undimensioned.
4. **The two drops to Apex 2.** From A, a line with `addPerpendicular` against the pinion axis and an
   aligned dimension of `PPD / 2`; from B, the same against the driving axis with `DPD / 2`. Both
   must point into the interior wedge *between* the two shafts: pick each sense by the sign of its
   dot product with the direction toward the other shaft's point, never against a "toward the anchor
   line" reference. The driving side is the trap — the driving shaft is itself parallel to the grow
   direction, so that dot product is about zero and silently selects an arbitrary side, after which
   the coincidence closing the two drops flips the entire frame to the mirror solution. Close them
   with `addCoincident` on the two far ends; that point is **Apex 2**.
5. **Pitch Line** Apex→Apex 2, coincident at both ends.
6. **The two dedendum lines** from Apex 2, each `addPerpendicular` to the Pitch Line with an aligned
   dimension of `Module * 1.25`. The one drawn toward the anchor line is the Driving Gear Dedendum,
   ending at **D**; the one away from it is the Pinion Gear Dedendum, ending at **C**.
7. **The two Root Axes** Apex→C and Apex→D, coincident at both ends.
8. **The module-length extensions.** From A a line with `addCollinear` against the pinion axis and
   `addCoincident` joining it to A, ending at **E**; then C→E with `addPerpendicular` against A→E.
   From E, collinear again, ending at **G**. Mirror on the driving side for **F** and **I**.
9. **The two heel edges.** From C a line of module length with `addCollinear` against the Pinion
   Dedendum, ending at **H**; then G→H, and `addPerpendicular` between E→G and H→G. The same on the
   driving side for **J** and I→J. That perpendicular is required in Fusion because
   `addOffsetDimension` needs its second entity already parallel to the first, and this is what
   supplies the parallelism.
10. **The two base-height offsets.** `sketchDimensions.addOffsetDimension(dropB, lineIJ, textPoint)`
    set to the resolved Driving Gear Base Height, and the same between the A→Apex 2 drop and G→H set
    to the resolved Pinion Gear Base Height. Both lines are already parallel by construction, so add
    **no** `addParallel` (`[PB-OFFSET-DIM]`). The value is the one after that gear's own Maximum and
    Minimum Base Height have been applied, because this offset is what drives the heel edge toward
    the shaft axis.
11. **Close the figure.** Draw A→G and B→I, then `addCoincident` **Point I with the projected centre
    point**. That single coincidence is what pins the whole figure's height above the anchor line;
    without it the net reaches every angle and length and still slides along the perpendicular.
12. **The tooth centres.** From G a line ending at **K**, pinned with two point-on-line coincidents —
    `addCoincident(K, pinionAxis)` and `addCoincident(K, pinionDedendum)` — rather than
    `addCollinear`, which over-constrains here because G and C are already fixed. Draw C→K for
    reference. The same on the driving side for **L** and D→L.
13. **The Tooth Spacing offsets.** When Tooth Spacing is zero, build nothing: K′ is K and the
    existing C→K line is the reference line, because a zero-length dimensioned line is degenerate and
    one segment gets one line. When it is positive, draw one line from K with its far end seeded past
    K along the dedendum direction, pinned the same way K is, with an aligned length dimension equal
    to the Tooth Spacing; its far end is **K′**. Then draw C→K′. The same for **L′** and D→L′. Build
    both here, before this sketch's gate, so the gate covers them.
14. **The Maximum Face Width.** A, B, C, D, H and J are now solved, so read their `.geometry` —
    never the seed coordinates (`[PB-SOLVED-GEOMETRY]`) — and resolve
    `0.95 * min(distance(A, line C-H), distance(B, line D-J))`. Cap the automatic default to it and
    reject a user value above it.
15. **The two toe edges.** Seed M near the midpoint of Apex→C and N by sliding from that seed along
    the C→H direction far enough to reach the A→Apex 2 drop; do not seed them a Face Width from C and
    H, which starts N near H and the solve fails to converge (`[PB-SEED-NEAR]`). Then apply exactly
    four constraints: `addCoincident(M, pinionRootAxis)`; `addCoincident(N, dropA)` — the
    perpendicular **drop**, never the Apex→A shaft axis, because pinning N to the axis puts it on the
    axis of revolution and the later conical split fails for asymmetric tooth counts;
    `addParallel(lineMN, lineCH)`; and `addOffsetDimension(lineCH, lineMN, textPoint)` set to the
    resolved Face Width, its text point in the gap on the apex side. Mirror all of it for O→P on the
    driving side. Then draw M→C, N→A, O→D, P→B and B→I.

End the step by gating `sketch.isFullyConstrained` and raising, naming the sketch.

The proof builds this whole lattice as `stepGearProfiles` and asserts the solved positions of all
twenty named points against the closed form, the two solved cone angles to nine decimals, the two
driven along-shaft lengths, Point I landing on the projected centre, the tooth centres at the
back-cone distance `R / cos(gamma)`, the Maximum Face Width measured on the solved figure against
`R * sin(gamma)^2` per side, and that no toe or heel corner has reached its own shaft axis
(`[PB-REVOLVE]`). Its case table sweeps the Shaft Angle from the documented 30 degree floor to 142
degrees, both ratio directions, the Minimum Teeth floor at four teeth, both ends of each base
height's range, the Face Width at its maximum, and both sides of the Tooth Spacing branch.

Three of the spec's constraints are written differently in the proof and the proof says why at each
site: an unsigned `addPerpendicular` plus a length admits both sides, so each side choice the spec
makes with a seed is written there as a signed angle of the same arity; the engine's collinear is two
point-on-line rows and one of them is implied by the coincident already on that end; and the engine's
offset holds both endpoints of its target, so it carries the parallelism the offset dimension needs.
That last one reaches **two** pairs of constraints, not one. The spec's paragraph spells the reason
out for the E→G / F→I perpendiculars; the two toe lines' `addParallel` constraints fall to exactly
the same argument, and measured, a toe line carrying both its parallel and its offset comes back with
one redundant row that the engine names as the offset. Both pairs are therefore left out of the
proof, and the spec's paragraph should name both.

**Two configurations in that table are refused by the sketch engine as near-singular**, and each is
recorded rather than waived. The 30 degree Shaft Angle floor comes back at conditioning 2.94e-05 on
the default pair, reproducing to three figures one of the three lattices the spec measured, and this
net first clears at 35 degrees. Base heights at their **minima** come back at 2.68e-05, which the
spec does not record: at the minimum the heel edge C→H is only `0.0625 * Module` long, 62 microns at
module 1, and its direction is what carries the point-on-line pin that locates H.

<!-- proof-run: proofkit.Run(latticeCases, stepGearProfiles) -->

<!-- check-step-calls: ignore addVertical -->

**From:** `spec/bevelgear/instructions.md` L88–129, L450–546; `spec/bevelgear/fusion.md` L76–142;
`.claude/skills/generate-gear/PLAYBOOK.md` L480–520, L556–566, L604–613

## 7 `[PROSE]` `{gearLabel} Plane` — the tooth plane

Once per gear, pinion first. Create a construction plane that includes this gear's tooth-centre
reference line — C→K′ for the pinion, D→L′ for the driving gear — and is perpendicular to the Gear
Profiles sketch plane, through the framework's `plane_by_angle`, which wraps
`setByAngle` at 90 degrees. Name it `{gearLabel} Plane`. Pass the sketch line straight in; never wrap
it in `Path.create` (`[PB-CONSTRUCTION-PLANES]`).

A bare construction plane bounds no body, so no gate holds one, and the substitute — gating a sketch
drawn on it — would gate step 8's sketch rather than this plane. What this plane is for is asserted
in step 13's proof instead, and there it is asserted about the plane itself:
`assertToothPlacement` checks that it sits at exactly the Pitch Cone Distance R from the apex along
its own normal, so it is the back-cone plane through Apex 2, and that the tooth centre on it lies on
the shaft axis at cone distance `R / cos(gamma)`.

<!-- check-step-calls: ignore Path.create -->

**From:** `spec/bevelgear/instructions.md` L548–556;
`.claude/skills/generate-gear/PLAYBOOK.md` L716–727

## 8 `[GO]` `{gearLabel} Tooth` sketch — the virtual spur tooth

Compute this gear's virtual, back-cone tooth number from the closed form and **never** by measuring
the lattice: `virtualPitchRadius_mm = (pitchDiameter_cm * 10 / 2) / cos(gamma)` and
`virtualTeeth = floor(2 * virtualPitchRadius_mm / Module)` as an int. The times-ten is the
centimetre-to-millimetre conversion, and skipping it makes the count about ten times off.

Create a sketch on `{gearLabel} Plane` named `{gearLabel} Tooth`, and draw the tooth by borrowing the
spur generator:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))
```

The anchor point is this gear's tooth-centre point, K′ or L′. The 180 degree rotation is delivered
**through `draw`'s angle argument** — the generator rotates the whole tooth in its own point math —
never as a post-hoc move. Import `VirtualSpurProxy` from the framework; do not define a local copy.

After `draw` returns, read `proxy._lastToothEmbedded` back and thread it onward: it is the
deterministic selector for the tooth loop's line count in step 12, and skipping it grabs an unrelated
loop whose loft dies with `LOFT_NO_TOOLBODY`.

Do **not** raise on this sketch's `isFullyConstrained`; at most log it. The two tooth sketches are
the one exemption from the full-constraint gate, and `[BEVEL-F-FULL-CONSTRAINT]` is explicit that the
exemption covers the four along-path circle labels (`[PB-SKETCH-TEXT]`) and nothing else — the
geometry itself must read fully constrained with the labels deleted.

The proof draws exactly that: the same tooth with no labels, gated normally, which is the measurement
the exemption asks for before it is renewed. `stepToothSketch` also checks the virtual tooth count
against the closed form, the tooth sitting at the 180 degrees it was given with its top on the tip
circle, and that **exactly one** loop in the sketch carries the curve count the profile search selects
on — the spec's warning about an impostor loop with two NURBS, two arcs and the other line count.

<!-- proof-run: proofkit.Run(toothCases, stepToothSketch) -->

<!-- check-step-calls: ignore isFullyConstrained -->

**From:** `spec/bevelgear/instructions.md` L316–334, L402–430, L548–560;
`spec/bevelgear/fusion.md` L27–45; `.claude/skills/generate-gear/PLAYBOOK.md` L188–192, L640–652

## 9 `[PROSE]` `{gearLabel} Tooth Axis`

Create a construction axis through the tooth-centre point, normal to the plane the tooth was drawn
on, with `constructionAxes.createInput()` then `setByTwoPlanes(gearProfilesPlane, helperPlane)` and
`constructionAxes.add(...)`, named `{gearLabel} Tooth Axis`. The helper plane is built
`setByDistanceOnPath(<tooth-centre reference line>, 1.0)` — perpendicular to that line at its far
end, the tooth centre. Use `setByTwoPlanes`; `setByPerpendicularAtPoint` would need a `BRepFace` this
build does not have (`[PB-CONSTRUCTION-AXES]`).

Creating this axis inside the never-activated Design component is proven to work, so it does not hit
`[PB-CONSTRUCTION-NEEDS-ACTIVE]`; keep it.

A bare construction axis bounds nothing, so no gate holds one. The substitute considered was to
revolve or pattern about it and gate that body, which is what steps 12 and 22 do — but they turn
about the profile sketch's own first edge, which the spec is explicit is the axis every body
operation uses, not this one. This axis is created for the user to see and for nothing downstream to
consume, so there is no body whose geometry would differ if it were absent.

<!-- check-step-calls: ignore setByPerpendicularAtPoint -->

**From:** `spec/bevelgear/instructions.md` L562–563;
`.claude/skills/generate-gear/PLAYBOOK.md` L728–731

## 10 `[PROSE]` Create the `{gearLabel} Gear` component

Create a new component as a child of the **Bevel Gear** component — the same component that owns
Design, not the user's Parent Component — named `Pinion Gear` or `Driving Gear`. The finished bodies
for this gear end up here, moved in at the end of its chain; every feature until then runs in Design,
because Fusion rejects cross-sibling sketch and project calls even through assembly-context proxies
(`[PB-NO-CROSS-SIBLING]`).

No harness reaches this step, and no substitute was attempted, for the reason step 3 gives: neither
harness has a component to create, and no geometry changes when one is.

**From:** `spec/bevelgear/instructions.md` L659–667;
`.claude/skills/generate-gear/PLAYBOOK.md` L747–752

## 11 `[GO]` `{gearLabel} Profile` sketch — the frustum hexagon

Open a **fresh** sketch on the axial Gear Profiles plane, named `Pinion Profile` or
`Driving Profile`. One profile sketch per gear, so `sketch.profiles` holds exactly this one hexagon
loop; drawing both gears' hexagons in the shared §2 sketch would leave two identically shaped loops
to disambiguate.

Build it on fixed vertices by the `[PB-PROJECT-NOT-FIXED]` recreate-share-fix recipe, and the order
is the constraint:

1. recreate the six §2 vertices as new points at their exact world-mapped positions —
   `sketchPoints.add(sketch.modelToSketchSpace(src.worldGeometry))`, valid because §2 is fully
   constrained by now (`[PB-SPACE-METHODS]`);
2. draw the closed hexagon as six `addByTwoPoints` lines **sharing** those points, in the draw order
   A→G→H→C→M→N→A for the pinion and B→I→J→D→O→P→B for the driving gear;
3. only then set `isFixed` on the lines' endpoints. Fixing a bare point before it is consumed as a
   line endpoint does not leave the sketch fully constrained.

The hexagon's **first edge is the gear's shaft axis** for the revolve, the pattern, the bore plane
and the meshing rotation, so it must be fixed well enough to carry a trustworthy `worldGeometry`
(`[PB-WORLDGEO-CONSTRAINED]`); a free edge resolves against a default frame and silently moves the
body onto world XY. The §2 `Apex->A` / `Apex->B` construction line is **not** usable as that axis: it
is collinear but lives in a different sketch, which the revolve and pattern reject.

Gate `isFullyConstrained` and raise, naming the sketch.

The proof draws this hexagon as `stepProfileSketch` and checks that it closes exactly one loop of six
lines, that its area is the §2 hexagon's, that the first edge lies on the shaft axis and every other
corner is strictly off it. It also reads off the section itself the two
quantities the steps below it consume — the volume a full turn of it sweeps, and the half-angles of
the two conical faces the trim's face search matches on — so a hexagon drawn on the wrong corners is
caught here as well as there.

<!-- proof-run: proofkit.Run(profileCases, stepProfileSketch) -->

<!-- check-step-calls: ignore isFixed -->

**From:** `spec/bevelgear/instructions.md` L659–673;
`.claude/skills/generate-gear/PLAYBOOK.md` L493–504, L523–527, L529–535

## 12 `[GO]` Revolve the hexagon into the Gear Body

This sketch holds exactly one closed loop, so take `sketch.profiles.item(0)` directly rather than
filtering (`[PB-SINGLE-PROFILE]`), and revolve it about the hexagon's first edge:
`revolveFeatures.createInput(profile, axis, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
then `setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))`, then
`revolveFeatures.add(input)`. The result is the Gear Body, the frustum.

The profile must not cross the axis of revolution, or Fusion aborts with `ASM_WIRE_X_AXIS`
(`[PB-REVOLVE]`); that is what the Maximum Face Width and the two base-height bounds are for, and
step 6's proof asserts it on the solved figure.

Because the toe edge is one edge of the revolved profile, the body already carries the conical face
that edge sweeps, and likewise the heel edge's cone. Those two faces are the cutting tools step 21
reuses.

SUBSTITUTE IN THE PROOF. decad returns a revolved body's area and volume with a bound of 200 per
cent of the reading — measured with a plain rectangle swept into a solid cylinder, where 785.398 mm³
comes back carrying a bound of 1570.796 — so the document verifies Suspect and no gate accepts it.
A solid of revolution is the union of the bands its profile edges sweep, though, and a band between
two coaxial sections is a loft, which this evaluator does record exactly. So `stepRevolveGearBody`
builds the Gear Body as the three bands its three non-axial hexagon edges sweep, laid apart along the
shaft axis. Each band's volume is checked against the exact volume its own two sections bound, each
band's cone half-angle against the edge that swept it, and the three signed contributions are summed
and compared with the hexagon's own Pappus volume — the identity a wrongly ordered or wrongly signed
decomposition cannot satisfy. What it costs: the sections are chorded to 48-sided polygons, because a
circular one needs 257 chord cells against the 250 its share of the loft's 500-station cap allows, so
every volume is the chorded solid's and the circular value is quoted beside it; and the union that
makes the three bands one body is not exercised, because decad refuses a boolean on a loft operand.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepRevolveGearBody, assertRevolveGearBody) -->

**From:** `spec/bevelgear/instructions.md` L675–677;
`.claude/skills/generate-gear/PLAYBOOK.md` L654–661, L521–528

## 13 `[GO]` Loft the Apex to the tooth profile

Select the tooth cross-section with
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)`, where
`wantLines = 0 if embedded else 2` and `embedded` is the flag read back off the proxy in step 8. Do
**not** accept "0 or 2": for a given gear only one of those is the real tooth, an unrelated loop
between the drawCircles circles can carry the same two NURBS and two arcs with the other line count,
and lofting that impostor fails with `ASM_RBI_INTERNAL` / `LOFT_NO_TOOLBODY`.

Loft the **§2 Apex sketch point** — `centerToApex.endSketchPoint`, the degenerate point section — to
that profile: `loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`, then
`loftSections.add(...)` for the apex point and then the profile, then `loftFeatures.add(input)`
(`[PB-LOFT]`). Use the sketch point directly; do not create a construction point for it, because the
Design component is never active (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). The result is the Tooth Body.

The proof builds this as `stepLoftTooth`. decad's loft takes two profiles and has no point section,
so the apex end is replaced by the same tooth section scaled about the apex on a parallel plane a
tenth of R out from it. A uniform scale about the apex is exactly the section the point loft passes
through there, so the ruled walls are the same walls; what the truncation costs is the sliver between
that plane and the apex, and `assertLoftTooth` measures the body against the closed-form frustum
volume `area * R * (1 - s^3) / 3` that truncation predicts rather than against the full cone. The
tooth section is chorded there for the reason `proof/bevelgear/bodies_test.go` gives at the top: one
fitted spline withdraws exact trims from every region of its sketch, so the drawn tooth cannot be
lofted at all.

`assertToothPlacement` carries the three placement facts the surrounding steps rest on: the tooth
plane at R from the apex, the tooth centre on the shaft axis at `R / cos(gamma)`, and the dedendum
corner C at the virtual root radius from that centre — which is what makes the tooth drawn at 180
degrees sit with its root on C and its tip a module beyond, pointing away from the shaft axis.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepLoftTooth, assertLoftTooth) -->

**From:** `spec/bevelgear/instructions.md` L352–366, L679–681;
`.claude/skills/generate-gear/PLAYBOOK.md` L662–666

## 14 `[PROSE]` `{gear} Cone Element` sketch and `{gear} Trace Plane` (ψ > 0 only)

Everything from here to step 20 runs only when the Mean Spiral Angle is above zero. The tooth-body
hook's first line is the gate `if self._spiralAngle_rad <= 0: return cut_conical_ends(...)`, so a
straight bevel skips straight to step 20 and is byte-for-byte the prior behaviour.

Build the frame first (§3a step A), from the geometry this gear already has: `axisDir` from the two
world endpoints of the profile sketch's first edge; `coneVec = normalize(heelConeWorld - apex)`, the
root cone element Apex→C or Apex→D; `v = axisDir x coneVec`, the circumferential direction; and
the cone distance `distAlong` of a point, `(p - apex) . coneVec`. The four points the caller hands in are exactly: `toeMid` the
midpoint of the **toe** edge M→N or O→P, `heelMid` the midpoint of the **heel** edge C→H or D→J,
`toeConeWorld` the toe edge's inner endpoint M or O, and `heelConeWorld` the dedendum corner C or D —
never H or J, which sit one module off the root cone element. Before building `coneVec`, check the
two midpoints and swap both pairs if the heel is nearer the apex: a negative span silently inverts
the whole spiral frame, flipping the cutter arc, the slice direction and the per-segment twist with
no error. Then `R_toe`, `R_heel`, `R_mean` and `span = R_heel - R_toe`.

Draw a cone-element construction line Apex→(Apex + R_heel·coneVec) in a sketch on the axial plane
named `{gear} Cone Element`, then build the tangent plane as that axial plane rotated 90 degrees
about it, through `plane_by_angle`, named `{gear} Trace Plane`. Both sketches are transient, are
hidden in cleanup, and are **exempt** from the full-constraint gate.

The world points are passed straight into the sketch calls and consumed as sketch-space input, with
no `modelToSketchSpace` conversion. That is deliberate and it is safe only because nothing downstream
consumes either sketch: the twist is computed analytically in step 17, and the Trace Plane carries
only the inspection-only trace sketch. If a later revision ever makes a feature consume either, both
sketches need the conversion on every point.

No gate holds a construction plane, and the cone-element sketch is one construction line whose only
consumer is that plane. Two substitutes were considered. Gating the cone-element sketch through
`proofkit` fails on its own terms: the line has two free endpoints seeded from world coordinates and
is deliberately left that way, so it cannot reach DOF 0 without dimensions the build does not add,
and adding them would be gating a different sketch. Gating the Trace Plane through a body built on it
fails because the only thing built on it is step 15's inspection-only sketch, which no feature
consumes. The frame the two of them set up is what `spiralOf` builds in
`proof/bevelgear/spiral_test.go`, including the swap guard, and every quantity derived from it is
asserted in steps 15 and 18.

**From:** `spec/bevelgear/instructions.md` L565–595, L296–316;
`spec/bevelgear/spiral-tooth-trace.md` L30–88; `spec/bevelgear/fusion.md` L46–54

## 15 `[GO]` `{gear} 2D Tooth Trace` sketch (ψ > 0 only)

The cutter radius `r_c` is the Cutter Radius input when non-zero and `R_mean` otherwise. The hand
sign is `+1` for `Right` and `-1` for `Left`, then **negated for the pinion**, because a meshing pair
is cut with opposite hands. In the tangent-plane frame — apex at the origin, x along `coneVec` so a
point's x is its cone distance, y along `v` — the cutter circle's centre is

```
Cx = R_mean - r_c * sin(psi)
Cy = handSign * r_c * cos(psi)
```

The hand sign goes on the **cos** term. Opposite hands are mirror images across the cone element,
which flips `Cy`; putting the sign on `Cx` mirrors about `x = R_mean` instead, which is a different
curve and gives the two gears unequal twist.

The arc's ends are taken a hair past the face so the kept arc reaches cleanly through the end trims:
`circle_intersect_nearest` against apex circles of radius `R_toe - 0.06 * span` and
`R_heel + 0.06 * span`, keeping the intersection nearest the mean point `(R_mean, 0)`.

Add a sketch on the Trace Plane named `{gear} 2D Tooth Trace` and draw, mapping 2-D coordinates to
world with `combine_point`:

- the **cutter circle**, `sketchCurves.sketchCircles.addByCenterRadius` at the centre with radius
  `r_c`, `isConstruction`, its `centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`) and a
  `sketchDimensions.addDiameterDimension` of `2 * r_c`;
- the **trace arc**, `sketchCurves.sketchArcs.addByThreePoints` through the toe end, the mean point
  and the heel end, with `addCoincident` tying its centre to the cutter circle's and a
  `sketchDimensions.addRadialDimension` of `r_c`, so it is the genuine cutter circle rather than a
  look-alike spline. Both dimension text points sit off-centre, on or near their curve
  (`[PB-RADIAL-DIM]`).

This sketch is **deliberately left with free DOF** — the arc's ends are pinned by the three-point
construction, not by endpoint dimensions — and is exempt from the full-constraint gate. Do not gate
it.

The proof draws it as `stepToothTrace` and, because the harness has no such exemption, pins the two
ends by the signed coordinates `circle_intersect_nearest` computes. What that substitution costs is
the constraint form of the circle intersection, which carries the two-branch ambiguity the helper
resolves by picking the branch the mean point sits on, and a scheme whose answer depends on that
choice is one the harness refuses. So the step proves the arithmetic: that the computed ends lie on
both circles, that the arc through them is the cutter circle, that the trace meets the cone element
at exactly the Mean Spiral Angle at the mean point, that flipping the hand mirrors the centre across
the element and moves nothing else, and that at psi = 0 the centre would sit due north of the mean
point. Those are `spiral-tooth-trace.md`'s invariants 1 through 7, one for one. The engine's arc
carries its own equal-radius row, so the centre is held on the perpendicular bisector of the two ends
and one signed coordinate closes it; fixing the centre outright the way `[PB-CIRCLE-CENTER]` has
Fusion do it would make that implicit row redundant.

<!-- proof-run: proofkit.Run(spiralCases, stepToothTrace) -->

<!-- check-step-calls: ignore isFixed -->

**From:** `spec/bevelgear/instructions.md` L577–595;
`spec/bevelgear/spiral-tooth-trace.md` L108–184, L218–238;
`.claude/skills/generate-gear/PLAYBOOK.md` L484–492, L610–614

## 16 `[GO]` Slice the straight tooth into cross-section slabs (ψ > 0 only)

Split the uncut apex-to-heel Tooth Body into slabs with planes perpendicular to the cone element,
through `slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)`. The
base is the parent transverse tooth plane from step 7. The first cut plane is that plane offset
toward the apex by `span / 6`, and the offsets are `sign * (k + 1) * span / 6` for k = 0 through 7 —
a fixed scheme of about eight planes, not user-configurable. Choose `sign` per gear so it moves
apex-ward, by testing `(apex - planeOrigin) . normal`, because the parent plane's normal points
opposite ways for the two gears.

**The slice must actually split the tooth.** If the body is still in one piece after the loop, retry
the whole cut once with the opposite sign; if it is still one piece, `raise` a self-diagnosing error
naming the gear, the final piece count, the span and the sign tried (`[PB-EMPTY-RESULT]`,
`[PB-SELF-DIAGNOSING]`). Returning an unsliced result leaves the next step's segment list empty and
the crown then dies with `max() iterable argument is empty` far from the cause.

SUBSTITUTE IN THE PROOF. decad has no split-by-plane, so `stepSliceSegments` lofts each slab
directly between the two sections its cut planes would have produced, and lays the slabs apart along
the shaft axis, because the read-only interference pass cannot tessellate a loft payload and so
cannot judge two slabs whose boxes meet. It asserts the piece count the scheme produces — eight
cross-section slabs and the apex scrap — each slab's volume against the exact volume its own two
sections bound, and the tiling, from the sections rather than from the positions: consecutive slabs
share a section, so one's heel section is the next one's toe section. What it costs: the cut planes
are drawn parallel to the tooth plane, which is perpendicular to the pitch line, where the build cuts
perpendicular to the cone element — the two differ by the dedendum angle `atan(1.25 * Module / R)`,
under two degrees on every case in the table — and the retry-with-the-opposite-sign guard is not
reached, because nothing here can produce the single piece that triggers it.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepSliceSegments, assertSliceSegments) -->

**From:** `spec/bevelgear/instructions.md` L597–601;
`.claude/skills/generate-gear/PLAYBOOK.md` L468–469, L692–700

## 17 `[GO]` Order the segments and drop the apex scrap (ψ > 0 only)

Sort the segments by the `distAlong` of their centroid, read from
`body.physicalProperties.centerOfMass`. The first, apex-most piece is the long scrap below the toe.
Re-slice the list first and delete afterwards — `segments = segments[1:]` before
`removeFeatures.add(scrap)` (`[PB-REMOVE-PIECES]`). After the drop, `segments` must be non-empty; if
it is empty the slice failed in step 16, so `raise` a clear error rather than proceeding into the
twist and the crown, which both assume at least one segment.

SUBSTITUTE IN THE PROOF. `stepDropScrap` builds the same slab set as step 16, orders it, and returns
everything but the apex-most piece, which is the drop. It asserts that the piece dropped is the one
running from the tooth's own apex end, that it is the LONG one — more than a slab's span, since it
reaches from the apex to the first cut while every kept slab is exactly one step — and that eight
slabs remain. What it costs is the removal feature itself: the drop is a re-slice of the list here,
so nothing exercises `removeFeatures.add` or the timeline entry it leaves.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepDropScrap, assertDropScrap) -->

**From:** `spec/bevelgear/instructions.md` L603–605;
`.claude/skills/generate-gear/PLAYBOOK.md` L468–469, L701–707

## 18 `[GO]` Twist the segments about the shaft axis (ψ > 0 only)

Rotate each segment about the shaft axis through the apex, centred on `R_mean` so the mid-face
section stays unrotated — that section then meshes exactly like the straight tooth, and the pinion's
zero mesh nudge depends on it. The total toe-to-heel twist comes from the conjugate crown-gear
generation law, computed analytically with no projection and no curve sampling:

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`phi_crown` is the angle the cutter arc's two ends subtend **at the apex** in the flat crown frame,
and `gamma` is this gear's **pitch** cone angle from §2 — `self._gamma_p` or `self._gamma_g`. Do not
use `acos(coneVec . axisDir)`: that is the root cone angle, smaller by the dedendum angle, and it
inflates the twist by about 1.6 times on a 17-tooth pinion. The two members of a pair legitimately
get different twists — same cutter, same spiral angle, different gamma, so different `1 / sin(gamma)`
— which is why equal-teeth pairs meshed under methods that got the factor wrong while ratio pairs did
not.

Each segment's share is linear in the cone distance of its **heel face**:

```
ang = -handSign * total * (R_mean - R_heelFace(seg)) / span
```

A slab's heel face is the face whose centroid has the **greatest** `distAlong`, searched across
**all** of the slab's faces with no surface-type filter; the toe face is the least. Do not restrict
the search to planar faces — a sliced slab is bounded by a mix of the two planar cuts and ruled side
faces, and a type filter picks the wrong one, after which the step 20 loft fails with
`ASM_NOT_ALL_SECTIONS_MEET` / `LOFT_NO_TOOLBODY`. Key the twist on that face and not on the
centroid: the loft samples the heel face, so that face is what must land at the right azimuth, and
centroid-keying leaves the mid-face section rotated by half a segment.

Apply each rotation as a free move: `moveFeatures.createInput2(bodyCollection)` then
`defineAsFreeMove(matrix)` then `moveFeatures.add(input)`, with the matrix built by
`adsk.core.Matrix3D.setToRotation(ang, axisDir, apex)` (`[PB-MOVE-ROTATE]`). Use `defineAsFreeMove`
with a matrix, not `defineAsRotate`, which rejects a sketch-line axis.

The proof applies this law in `stepTwistTooth`, and it stands in for the whole slice-twist-crown-loft
chain because decad has neither a multi-section loft nor a scale: instead of slicing the tooth into
slabs and lofting through their heel faces, it lofts one band of the tooth directly between its toe
and heel sections, each already turned by the share this step gives it. The two ends therefore carry
the same total twist, in the same sense, as the sliced build's outermost slabs.
`assertTwistTooth` checks that the two end angles are symmetric about zero, that their difference is
`handSign * total`, that the mid-face section does not move, and — read off the finished solid rather
than off the numbers that built it — that the band's centroid lies on the heel side of the mid face
and inside its own angular span, and that the opposite hand mirrors it. The step also asserts that
the root cone angle is strictly below the pitch cone angle and that keying the twist on it would come
out larger, which is the error this paragraph forbids.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepTwistTooth, assertTwistTooth) -->

<!-- check-step-calls: ignore defineAsRotate -->

**From:** `spec/bevelgear/instructions.md` L607–637;
`spec/bevelgear/spiral-tooth-trace.md` L186–216;
`.claude/skills/generate-gear/PLAYBOOK.md` L708–713

## 19 `[GO]` Lengthwise crown (ψ > 0 only)

Scale every segment **except the outermost** down by a monotonic factor, full at the heel and growing
toward the toe. For each segment compute its heel-distance fraction `u = (R_heel - R_heelFace) /
span`, with `R_heelFace` found by step 18's all-faces-by-centroid rule but **recomputed after the
twist has moved the slabs**; `u` runs 0 at the held-full heel to 1 at the toe. The outermost segment
is the one with the greatest post-twist heel-face `distAlong`. Then

```
factor = 1 - _CROWN_PER_RAD * (abs(total) / 2) * u
```

with `_CROWN_PER_RAD` a tunable class constant, default `0.5`. A factor at or below zero is a
`raise`, naming the gear, the segment's `u` and the factor — never scale by a non-positive factor.

Do **not** key the relief on the twist magnitude. That is symmetric about the mid face and maximal at
both ends, so with the heel slab held full the slab just inside it becomes the most relieved one and
dips below both its neighbours, reversing the heel-to-toe taper — observed at factor 0.932 against
0.972 on the next slab inward.

Three mechanics. The scale base must be a **sketch point** or a BRep vertex, and `scaleFeatures`
through `scaleFeatures.createInput(entities, point, ValueInput)` is the one exception to
never-activate: call `designOccurrence.activate()` before the crown scales and restore afterwards, in
a `finally`, with `design.activateRootComponent()`. A `Component` has no `.activate()` — writing
`design.rootComponent.activate()` raises `AttributeError`; only an `Occurrence` has it, and the root
comes back through `Design.activateRootComponent()`. Skip the outermost segment, whose heel face is
the loft's heel end and must stay full so the heel cone trims it flush. And anchor the scale on the
heel face's **root edge**, not its centroid: a uniform scale keeps every line through its base point
invariant, so a centroid base pulls the tooth's root upward by half the relief and the Combine-Join
leaves a visible gap. Take the two vertices of `heelFace.vertices` with the smallest perpendicular
distance to the shaft axis — the root corners, since the tip corners are farthest — and put the base
sketch point at their midpoint, mapped in with `modelToSketchSpace`.

SUBSTITUTE IN THE PROOF. decad has no scale feature and `r3.Transform` is rigid, so a uniform scale
about a point cannot be performed; but it can be drawn, because a uniform scale about a point carries
a plane to a parallel plane and a section to a scaled copy of itself. `stepCrownSegments` therefore
draws each slab's two sections already scaled about the anchor. It asserts the factor's shape — every
factor positive and at most one, falling monotonically from the held-full outermost slab toward the
toe, so the relief grows that way and the heel-to-toe taper is never reversed — and then reads the anchor back off the drawn solid: the two
root corners land where a uniform scale about that anchor puts them, they are a symmetric pair
equidistant from the shaft axis, the anchor is the midpoint of the two, and the slab's centroid sits
FARTHER from the axis than the anchor, which is the difference between anchoring on the root edge and
anchoring on the face centroid. What it costs is the feature: `scaleFeatures` and the
activate-and-restore dance it needs go unexercised.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepCrownSegments, assertCrownSegments) -->

<!-- check-step-calls: ignore activate -->

**From:** `spec/bevelgear/instructions.md` L639–645;
`.claude/skills/generate-gear/PLAYBOOK.md` L732–742

## 20 `[GO]` Loft the crowned segments into `{gear} Spiral Tooth` (ψ > 0 only)

Re-sort the segments by their heel-face cone distance **here**, after the twist and the crown — never
reuse the pre-twist slice order. The twist rotates each slab about the shaft axis, and for high-twist
unequal-ratio pairs that rotation reorders adjacent slabs; lofting in the stale order assembles the
cross-sections out of sequence and the crowned tooth comes out distorted, which is the single thing
that makes a ratio pair like 31/17 fail while 31/31 looks fine.

Loft a new body through, in that order: first the **toe-most** segment's apex-side face, so the loft
runs past the toe cone and the toe trim bites, then the heel-facing face of every segment in order,
the last reaching past the heel cone. Name the body `{gear} Spiral Tooth`, then remove the segment
scaffolding — the loft has captured their faces.

SUBSTITUTE IN THE PROOF. decad's `Loft` takes exactly two sections, so a nine-section loft cannot be
expressed as one call; it can be expressed as the chain of bands between consecutive sections, which
carries the same sections in the same order. `stepLoftSpiralTooth` builds that chain, laid apart, and
asserts what the step is actually about: the sections arrive in strictly increasing post-twist cone
distance, computed after the twist has turned each slab about the shaft axis — which moves it,
because the cone element is inclined to that axis. It also reports, per case, whether the post-twist
order and the stale pre-twist order differ, rather than assuming they do. What it costs is the single
body: the chain is not fused, so the loft's own tool-body formation goes unexercised, and the
toe-most segment's apex-side face leading the section list is a claim about that formation rather
than about the order.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepLoftSpiralTooth, assertLoftSpiralTooth) -->

**From:** `spec/bevelgear/instructions.md` L647–651;
`.claude/skills/generate-gear/PLAYBOOK.md` L662–666

## 21 `[GO]` Conical cuts — the toe-then-heel flush trim

Trim the tooth to a flush band by returning
`cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)`. For
a straight bevel this is the whole tooth-body step; for a spiral it runs on the curved tooth after
step 20. Do not re-implement the cut machinery.

**Two distinct bodies are involved.** The cutting tools are `ConeSurfaceType` faces of the **Gear
Body**, the revolved-hexagon frustum — the lofted tooth has no cone faces, so searching it finds none
— and the target being split is the **Tooth Body**. The caller's obligations are the four arguments:
`toeMid` the toe edge's world midpoint, `heelMid` the heel edge's, `apexWorld` the §2 Apex sketch
point's world geometry, and `gearBody` the frustum. The helper does the toe cut first, identifying
its cone face by the toe edge's world **midpoint** best-first across the frustum's cone faces
(`[PB-FACE-BY-MIDPOINT]` — endpoints sit near the apex singularity) and trying each candidate as the
actual split tool; then it selects the keeper (`[PB-REMOVE-PIECES]`); then the heel cut on the keeper
alone. The toe cut must split and its failure propagates; only the heel cut is lenient, and only
through the typed `NonIntersectError` the helper raises when the heel cone does not reach the keeper
at all.

SUBSTITUTE IN THE PROOF. decad refuses a boolean whose operand is a loft — "tessellation does not
support payload decad.loftPayload" — so the split cannot be performed on the lofted tooth with any
tool. Its RESULT can be drawn: `stepCutConicalEnds` builds the band of the tooth between the toe and
heel cone distances directly. It asserts that the toe cut falls strictly inside the uncut tooth,
since the toe cut has to bite and its failure is not lenient; reports whether the heel cut falls
inside the tooth or past it, which is the `NonIntersectError` branch; and measures the band against
the exact volume that stretch of the tooth holds. What it costs: the two cuts are drawn as PLANES
where the build cuts with CONES, so the band's ends are flat where the trimmed tooth's are conical,
and the flushness against the gear base rests on the cone half-angles step 12's proof reads off the
bands themselves. The face search, the candidate ordering and the keeper selection are not reached at
all.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCutConicalEnds, assertCutConicalEnds) -->

**From:** `spec/bevelgear/instructions.md` L336–341, L653–657, L683–699;
`.claude/skills/generate-gear/PLAYBOOK.md` L667–691

## 22 `[GO]` Circular-pattern the tooth

Pattern the trimmed tooth about the **shaft-axis edge** — the profile sketch's first edge, not the §2
construction line: `circularPatternFeatures.createInput(bodies, axis)`, then `quantity` set to this
gear's Teeth Number, `totalAngle = ValueInput.createByString('360 deg')` and `isSymmetric = False`,
then `add(input)`. Pin all three explicitly (`[PB-CIRCULAR-PATTERN]`). The count is the gear's Teeth
Number, never the virtual tooth number the tooth was drawn at.

Although the pitch diameter shrinks from heel toward apex, the angular spacing about the shaft axis
stays `360 / N` for the whole face width; the radial taper is already in the loft, so the pattern only
rotates that one tapered tooth into N copies. The pattern's `bodies` collection already holds the
seed alongside the copies, so do not re-add the seed, and copy them into a fresh
`adsk.core.ObjectCollection.create()` before the combine, because `combineFeatures.createInput`
rejects a `BRepBodies` (`[PB-PATTERN-BODIES]`).

SUBSTITUTE IN THE PROOF. decad's read-only interference pass cannot tessellate a loft payload, so two
teeth whose bounding boxes meet cannot be judged as a pair — measured on two copies of this tooth at
every involute sample count from three to fifteen, and on a bare test loft where an eight-sided
section is already refused while a seven-sided one passes. The copies are therefore slid apart ALONG
the shaft axis, which is the axis the pattern turns about, so every azimuth about it, every radius
from it and every volume is exactly the pattern's; measured, four teeth placed that way verify Sound
with no diagnostics. `stepPatternTeeth` asserts the count against Teeth Number, each copy's volume
against the seed's, each copy's distance from the axis against the seed's, and each copy's azimuth
against a full turn divided by Teeth Number. What it costs is the clearance: the copies no longer sit
where they really would, so the gap between neighbours is asserted from the seed's own angular width
against the tooth pitch instead of from the copies meeting or not.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepPatternTeeth, assertPatternTeeth) -->

**From:** `spec/bevelgear/instructions.md` L701–703;
`.claude/skills/generate-gear/PLAYBOOK.md` L714–719, L595–599

## 23 `[GO]` Combine-Join the teeth into the Gear Body

Join all patterned tooth pieces with the Gear Body in a single Combine-Join:
`combineFeatures.createInput(gearBody, toolCollection)` with the frustum as the target and the
patterned bodies as the tools, `operation` set to
`adsk.fusion.FeatureOperations.JoinFeatureOperation`, then `combineFeatures.add(input)`.

SUBSTITUTE IN THE PROOF. decad refuses a union whose operand is a loft, and the joined gear is a
shape neither a loft nor a revolve draws in one piece — its blank has a three-segment meridian and
its teeth stand on the root cone, so no single pair of sections sweeps it. So `stepCombineTeeth`
builds the join's two operands side by side, laid apart, and asserts what the join has to produce and
what it needs. It produces the blank's volume plus this gear's Teeth Number times one tooth's, read
back as the signed sum of the blank's three bands and off the tooth body that was actually built. It
needs SEATING: every tooth's root corners lying on the blank's root cone, which is the one thing that
decides whether the join closes or leaves the gap the crown's root anchoring exists to prevent. They
sit on it to within half a module, which is exactly the error the floored virtual tooth count
introduces, and a tooth further off than that would stand clear of the blank. What it costs is the
boolean: a defect only a union would surface — a tool body that fails to fuse, a lump that comes out
in two pieces — is out of reach.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCombineTeeth, assertCombineTeeth) -->

**From:** `spec/bevelgear/instructions.md` L705;
`.claude/skills/generate-gear/PLAYBOOK.md` L667–671

## 24 `[GO]` Bore

Skip this step entirely when Enable Bore is unchecked. Otherwise the diameter is this gear's Bore
Diameter when non-zero and `Pitch Diameter / 4` otherwise.

Build the bore plane normal to the shaft at its start with
`setByDistanceOnPath(<shaft-axis edge>, ValueInput.createByReal(0.0))`, passing the in-sketch edge
and not the §2 construction line. In a sketch named `{gearLabel} Bore`, draw the bore circle centred
at the sketch origin — the plane is rooted at the shaft start, so the origin is on the axis — fix its
centre and add a diameter dimension of the bore diameter (`[PB-CIRCLE-CENTER]`). Cut it as a
symmetric through-cut restricted to this Gear Body: `extrudeFeatures.createInput(profile,
adsk.fusion.FeatureOperations.CutFeatureOperation)`, then
`setSymmetricExtent(ValueInput.createByReal(2 * coneDistance), False)` — the second argument false
means that is the half-length per side — then `participantBodies` set to this body, then
`extrudeFeatures.add(input)` (`[PB-THROUGH-CUT]`). Gate the Bore sketch's `isFullyConstrained` and
raise.

SUBSTITUTE IN THE PROOF. The cut's target is the revolve, which decad will not tessellate for a
boolean, and a through-all sweep cannot even decide a path against a body whose extent is known only
to that 200 per cent bound. The cut's RESULT is drawable: `stepBoreCut` builds each band the bore
passes through as a loft between two ANNULAR sections, the hole being the bore. It asserts the
resolved diameter against the rule it comes from, and each bored band against the volume left once
the bore's cylinder is taken out of it. A band the bore is wider than is asserted as consumed rather
than drawn, since an annulus whose hole outgrows its rim closes no region — and the blank tapers to
nothing at its toe corner, which sits ON the shaft axis, so a bore of any diameter takes the toe end
with it. NOTHING IN THE SPEC BOUNDS THE BORE AGAINST THE BLANK, so the step logs which bands each
case's bore consumes. What it costs is the boolean and the extent: the symmetric through-cut and its
`participantBodies` restriction go unexercised.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepBoreCut, assertBoreCut) -->

**From:** `spec/bevelgear/instructions.md` L707–709;
`.claude/skills/generate-gear/PLAYBOOK.md` L484–492, L672–676

## 25 `[GO]` Meshing rotation

**Driving gear only**, and here, in the Design component, before the body is moved out: rotate the
driving body by `180 / drivingTeeth` degrees — half a tooth pitch — about its shaft axis, through
`rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)`, which takes the rotation
axis and origin from the B→I profile edge's **world** endpoints (`[PB-MOVE-ROTATE]`). Both gears are
patterned from a starting tooth in the axial plane, so without the offset a driving tooth and a
pinion tooth would both sit at the axial-plane crossing and visually collide. It runs before the move
because a construction axis cannot be added in the moved-out gear component
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation has to use the edge's world geometry while the
body is still in Design. The pinion additionally gets `_pinionMeshPhase(pinionTeeth)`, which returns
`_PINION_MESH_PHASE_TEETH * 2 * pi / pinionTeeth` radians and is zero by default, because the spiral
tooth's mid-face section is unrotated and already meshes.

SUBSTITUTE IN THE PROOF. The rotation is applied to one tooth rather than to the finished joined
gear, since no gate here holds a joined gear. What the rotation is for survives that exactly: it
moves every tooth by half a pitch about the shaft axis, and one tooth is where that is visible — a
rotationally symmetric blank could be turned by any angle with nothing to measure.
`stepMeshRotate` asserts the turn is `180 / Teeth Number` degrees, that this is half of the
`360 / Teeth Number` the pattern steps by, and that the body's distance from the shaft axis and its
volume are both untouched, since the rotation is about that axis. The lay-apart slide runs along the
same axis and changes none of the three. What it costs is the framework helper's own edge handling:
the rotation is built from the axis directly rather than from a profile edge's world endpoints.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepMeshRotate, assertMeshRotate) -->

**From:** `spec/bevelgear/instructions.md` L342–350, L711–713;
`.claude/skills/generate-gear/PLAYBOOK.md` L708–713

## 26 `[PROSE]` Move the finished bodies into the gear component

Relocate each finished body with `body.moveToComponent(targetOccurrence)`, which preserves world
position and needs no activation (`[PB-NO-CROSS-SIBLING]`). Every feature ran in Design because
Fusion rejects cross-sibling sketch and project references; this is the step that puts the result
where the user sees it.

No harness reaches this step, and no substitute was attempted, because neither harness has a
component tree at all: `decad.Document` owns a flat list of bodies with no owner and no assembly
context, so there is no relocation to perform and nothing that would change if one were. The
substitutes elsewhere in this list all replace one geometric operation with another that reaches the
same solid; there is no solid here.

**From:** `spec/bevelgear/instructions.md` L659–667, L711–713;
`.claude/skills/generate-gear/PLAYBOOK.md` L747–752

## 27 `[PROSE]` Cleanup

Call `hide_construction_geometry(bevelComponent)`. It walks the Bevel Gear component tree
recursively, deduping by `entityToken`, and sets `isLightBulbOn = False` on every sketch,
construction plane and construction axis — construction planes and axes are **not** hidden by
`isVisible` (`[PB-HIDE-AFTER-USE]`, `[PB-TREE-CLEANUP]`, `[BEVEL-F-CLEANUP]`). Do not re-implement
the walk, and do not add a sketch-only mode or a per-mode guard: bevel always builds solids. Only the
two finished gear bodies are left visible.

No harness reaches this step, and no substitute was attempted: the step changes no geometry at all.
Its whole effect is `isLightBulbOn` on sketches, planes and axes, and neither harness has a
visibility flag to set — `sketch.Sketch` and `decad.Body` carry no such property, so there is nothing
a substitute could build that would differ from what is already built.

**From:** `spec/bevelgear/instructions.md` L721–723; `spec/bevelgear/fusion.md` L151–156;
`.claude/skills/generate-gear/PLAYBOOK.md` L607–613, L743–746
