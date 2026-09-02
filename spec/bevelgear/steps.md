# Bevel gear — compiled step list

The proof for this step list is `proof/bevelgear/bevel_test.go`, `proof/bevelgear/lattice_test.go`,
`proof/bevelgear/gearprofiles_test.go`, `proof/bevelgear/tooth_test.go`,
`proof/bevelgear/sketches_test.go`, `proof/bevelgear/solids_test.go`,
`proof/bevelgear/bodies_test.go`, `proof/bevelgear/spiral_test.go` and the generated
`proof/bevelgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/bevelgear/instructions.md` | `5553a86f44036bfd1a6aad20b1d49bb6e922b842` |
| `spec/bevelgear/fusion.md` | `f4c93cac37dc49e16bfc1672a7e5fc601a4ff925` |
| `spec/bevelgear/spiral-tooth-trace.md` | `c9ec08561ced7975aa0ed9ad6a330186259c0d08` |
| `spec/spurgear/instructions.md` | `560549a350c789b6598ee49b72d3cd7b4460e214` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `aa97523b214dbc98b679e8eea73c56c5115c4e54` |

## S01 `[PROSE]` Command dialog inputs

Add the 17 dialog inputs to `cmd.commandInputs` in this display order, from
`BevelGearCommandInputsConfigurator.configure(cmd)`. The order is fixed: Target Plane first so it
wins Fusion's auto-focus (`[PB-AUTOFOCUS-FIRST]`), then Center Point, then the pre-selected Parent
Component, then the numeric and boolean fields.

| # | label | id | call | unit | default | filters / tooltip |
|---|---|---|---|---|---|---|
| 1 | Target Plane | `targetPlane` | `addSelectionInput(id, name, tooltip)` | — | — | `ConstructionPlanes`, `PlanarFaces`; limits 1,1; tooltip `Plane the bottom of the driving gear sits flush against` |
| 2 | Center Point | `centerPoint` | `addSelectionInput(id, name, tooltip)` | — | — | `ConstructionPoints`, `SketchPoints`; limits 1,1; tooltip `Point the driving bevel gear is centered on` |
| 3 | Parent Component | `parentComponent` | `addSelectionInput(id, name, tooltip)` | — | root pre-selected | `Occurrences`, `RootComponents`; limits 1,1; tooltip `Component the gear pair is created under` |
| 4 | Module | `module` | `addValueInput(id, name, unit, default)` | `''` | `createByReal(1)` | — |
| 5 | Shaft Angle | `shaftAngle` | `addValueInput(id, name, unit, default)` | `deg` | `createByString('90 deg')` | — |
| 6 | Driving Gear Teeth | `drivingTeeth` | `addValueInput(id, name, unit, default)` | `''` | `createByReal(31)` | — |
| 7 | Pinion Gear Teeth | `pinionTeeth` | `addValueInput(id, name, unit, default)` | `''` | `createByReal(31)` | — |
| 8 | Driving Gear Base Height | `drivingBaseHeight` | `addValueInput(id, name, unit, default)` | `mm` | `createByReal(to_cm(0))` | — |
| 9 | Pinion Gear Base Height | `pinionBaseHeight` | `addValueInput(id, name, unit, default)` | `mm` | `createByReal(to_cm(0))` | — |
| 10 | Enable Bore | `boreEnable` | `addBoolValueInput(id, name, isCheckBox, folder, initial)` | — | `True` | checkbox |
| 11 | Driving Gear Bore Diameter | `drivingBore` | `addValueInput(id, name, unit, default)` | `mm` | `createByReal(to_cm(0))` | — |
| 12 | Pinion Gear Bore Diameter | `pinionBore` | `addValueInput(id, name, unit, default)` | `mm` | `createByReal(to_cm(0))` | — |
| 13 | Face Width | `faceWidth` | `addValueInput(id, name, unit, default)` | `mm` | `createByReal(to_cm(0))` | — |
| 14 | Tooth Spacing | `toothSpacing` | `addValueInput(id, name, unit, default)` | `mm` | `createByReal(to_cm(0))` | — |
| 15 | Mean Spiral Angle | `spiralAngle` | `addValueInput(id, name, unit, default)` | `deg` | `createByString('35 deg')` | — |
| 16 | Hand of Spiral | `spiralHand` | `addDropDownCommandInput(id, name, style)` | — | items `Right` selected, `Left` | `DropDownStyles.TextListDropDownStyle` |
| 17 | Cutter Radius | `cutterRadius` | `addValueInput(id, name, unit, default)` | `mm` | `createByReal(to_cm(0))` | — |

The 17 id strings are held in module constants `INPUT_ID_PLANE`, `INPUT_ID_CENTER_POINT`,
`INPUT_ID_PARENT`, `INPUT_ID_MODULE`, `INPUT_ID_SHAFT_ANGLE`, `INPUT_ID_DRIVING_TEETH`,
`INPUT_ID_PINION_TEETH`, `INPUT_ID_DRIVING_BASE_HEIGHT`, `INPUT_ID_PINION_BASE_HEIGHT`,
`INPUT_ID_BORE_ENABLE`, `INPUT_ID_DRIVING_BORE`, `INPUT_ID_PINION_BORE`, `INPUT_ID_FACE_WIDTH`,
`INPUT_ID_TOOTH_SPACING`, `INPUT_ID_SPIRAL_ANGLE`, `INPUT_ID_HAND`, `INPUT_ID_CUTTER_RADIUS`, in
row order, plus `_HAND_RIGHT = 'Right'` and `_HAND_LEFT = 'Left'`. There are no `PARAM_*`
constants: bevel registers no Fusion user parameters and writes every value numerically
(`[PB-PRECOMPUTED-MODE]`).

Each selection input takes its filter set with `addSelectionFilter(filter)` and its limits with
`setSelectionLimits(1, 1)`, both declared per input, never improvised. A filter is the enum
attribute `adsk.core.SelectionCommandInput.ConstructionPlanes` and its kin; a quoted literal in its
place raises (`[PB-SELECTION-FILTER-ENUM]`). The Parent input pre-selects the root component with
`addSelection(entity)`. The `mm` and `deg` defaults are passed in Fusion internal units
(`[PB-DIALOG-DEFAULT-UNITS]`). The Hand dropdown adds `Right` selected and `Left` unselected
through its `listItems` collection. `configure(cmd)` calls
`cls._updateSpiralInputVisibility(inputs)` as its last step, so the initial state is right.

`configure` is a method the entry point binds by name and calls; the module defines it rather
than calling it.

<!-- check-step-calls: ignore configure -->

**From:** `spec/bevelgear/instructions.md` L25–115, `spec/bevelgear/instructions.md` L177–203

## S02 `[PROSE]` Conditional visibility of the spiral-only inputs

Hand of Spiral and Cutter Radius are relevant only for a curved bevel, so they are hidden while
the Mean Spiral Angle is 0 and shown once it is above 0. Mean Spiral Angle itself is always
visible — it is how the user reaches a curved bevel.

Realise it with the `isVisible` property; the Fusion API has no declarative show-if. A
`@classmethod _updateSpiralInputVisibility(cls, inputs)` reads the spiral-angle input's
`expression` through `evaluateExpression(expression, 'rad')` — the expression, not the input's
`value` — and sets `isVisible` on the Hand and Cutter Radius inputs to whether that value is
above 0. Guard it: return early if any of the three `itemById(id)` lookups is `None`, and wrap
the evaluation in try/except, leaving both inputs shown on failure. Hiding is cosmetic — the
inputs still exist and are read normally.

The dialog's `inputChanged` event drives the update through
`BevelGearCommandInputsConfigurator.handle_input_changed(args)`, which delegates one line to the
helper and recomputes on every change. The entry point binds it by name; the module defines it
rather than calling it, which is why it is exempted below.

<!-- check-step-calls: ignore handle_input_changed -->
<!-- check-compile: ignore handle_input_changed -->

**From:** `spec/bevelgear/instructions.md` L116–134, `spec/bevelgear/instructions.md` L227–240

## S03 `[PROSE]` Read and validate every input, and derive the cone geometry

`_readInputs(inputs)` runs first, before anything creates an occurrence, and reads every input in
one pass. Selections come from `get_selection(inputs, id)` and the checkbox from
`get_boolean(inputs, id)`, each matched to the type its input was declared with
— a boolean input has no expression, so reading it with a value helper raises. Every numeric and
angular input is read by evaluating its expression with
`evaluateExpression(expression, units)`, which returns Fusion internal units — centimetres and
radians — whatever unit string is passed (`[PB-EVAL-EXPRESSION]`). The Hand dropdown is read
through `itemById(id)` and its `selectedItem.name`, defaulting to `Right`.

Units, and this is where a bevel build goes ten times wrong. The `mm` and `deg` inputs come back
already in internal units; use them as they are. `Module` is read with the unit string `''`, so
it comes back as a raw number that means MILLIMETRES. Every length derived from Module is
therefore `to_cm`-converted before it touches geometry: both pitch diameters, the cone distance,
the dedendum `1.25 * Module`, the module-length extensions, and the default face width. The
virtual spur proxy receives Module in millimetres and converts the radii it serves.

Validate: module above 0, both teeth counts coerced with `int(round(...))` and at least 3, shaft
angle converted with `math.degrees(...)` and inside 30 to 150 degrees, mean spiral angle
converted the same way and inside 0 up to but not including 60 degrees, and non-negative base
heights, bore diameters, face width, tooth spacing and cutter radius.

Return the 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth,
pinionTeeth, shaftAngle_deg)` and stash the rest on `self`: `_drivingBaseHeight_cm`,
`_pinionBaseHeight_cm`, `_boreEnable`, `_drivingBore_cm`, `_pinionBore_cm`, `_faceWidth_cm`,
`_toothSpacing_cm`, `_spiralAngle_rad`, `_hand`, `_cutterRadius_cm`. Then derive, in Python, and
stash: both pitch diameters `Module * teeth`, the Variables section's Cone Distance
`sqrt((Module * drivingTeeth)**2 + (Module * pinionTeeth)**2)`, the two pitch cone half angles
`tan gamma_p = sin(Sigma) * PPD / (DPD + PPD * cos(Sigma))` and `gamma_g = Sigma - gamma_p`, and
the resolved bore diameters.

This step is `[PROSE]` because it builds no geometry: it reads a dialog and does arithmetic, and
neither harness has anything to gate. Every derived quantity it produces IS proved, one step
later, off the solved §2 lattice that consumes it — `proof/bevelgear/gearprofiles_test.go`
measures the two pitch cone angles, the cone distance and both along-shaft lengths on the sketch
rather than restating them, and `proof/bevelgear/tooth_test.go` measures the virtual tooth
numbers and the millimetre conversion the same way.

`generate(inputs)` is the entry point's own binding, and `deleteComponent()` is what the entry
point calls on failure; the module defines both rather than calling them.

<!-- check-step-calls: ignore generate deleteComponent -->

**From:** `spec/bevelgear/instructions.md` L29–77, `spec/bevelgear/instructions.md` L136–176,
`spec/bevelgear/instructions.md` L204–226, `spec/bevelgear/instructions.md` L320–331

## S04 `[PROSE]` The Bevel Gear and Design components

With every selection already read, create the occurrence tree with
`addNewComponent(transform)` on each parent's `occurrences`, passing
`adsk.core.Matrix3D.create()` (`[PB-OCCURRENCE-TREE]`). Under the user's Parent Component create
the component named `Bevel Gear`; under that, the component named `Design`, which owns every
sketch, construction plane, axis and feature the build authors. The two per-gear components are
created later, in the body step, as children of Bevel Gear.

Never activate any occurrence (`[PB-NEVER-ACTIVATE]`): the Anchor sketch is authored on the
user's external, root-owned target plane, and an activated occurrence resolves that plane in its
own local frame, collapsing the whole build onto world XY. The one exception is the spiral
crown's scale, which is its own step. Because every feature runs in Design, no cross-sibling
reference is ever needed (`[PB-NO-CROSS-SIBLING]`).

Nothing here is geometry either harness models — an occurrence tree is a Fusion document
structure, not a sketch or a solid — so the step is `[PROSE]`.

**From:** `spec/bevelgear/instructions.md` L364–372, `spec/bevelgear/instructions.md` L177–203,
`spec/bevelgear/fusion.md` L143–150

## S05 `[GO]` Anchor sketch

Add the sketch named `Anchor` directly on the user-selected target plane with `add(planarEntity)`
on the Design component's `sketches`, whether the selection is a construction plane or a planar
face. Do not re-derive or offset it (`[PB-USE-SELECTED-PLANE]`); re-deriving collapses the gear
onto XY.

Mark the centre by projecting the user-selected centre point into the sketch with
`sketch.project(entity)`.

Draw the Anchor Line through the projection with `addByTwoPoints(startPoint, endPoint)` on
`sketch.sketchCurves.sketchLines`, seeding its two endpoints at exactly plus and minus 0.5 cm from
the projected centre along the sketch's local X, so the seeded length is 10 mm. Then apply, in
this order:

- `addCoincident(point, entity)` pinning the projected centre onto the line;
- `addMidPoint(point, midPointCurve)` making the centre bisect it;
- `addDistanceDimension(pointOne, pointTwo, orientation, textPoint)` on the two endpoints,
  locking the seeded 10 mm without assigning `dimension.parameter.value` — the value is arbitrary,
  the line is only a reference;
- `addHorizontal(line)`, which is sketch-local and so survives a tilted target plane
  (`[PB-REFLINE-DIRECTION]`); an absolute world-axis lock would mis-orient the figure.

The line's absolute direction is arbitrary — §2 takes every direction relative to it — but it must
not be a free degree of freedom. Stash the projected-centre sketch point on `self`, since §2
re-projects THAT point rather than the raw user selection. Gate the finished sketch on
`isFullyConstrained` and raise, naming the sketch (`[PB-FULL-CONSTRAINT]`,
`[BEVEL-F-FULL-CONSTRAINT]`).

The proof function is `stepAnchorSketch`. It records one defect: the coincident and the midpoint
are redundant, because a midpoint already puts the point on the line. It adds the midpoint alone
and measures that the coincident's own condition holds regardless.

<!-- proof-run: proofkit.Run(anchorCases, stepAnchorSketch) -->

**From:** `spec/bevelgear/instructions.md` L374–378, `spec/bevelgear/fusion.md` L19–33

## S06 `[PROSE]` Gear Profiles plane

Create the construction plane named `Gear Profiles Plane` through the Anchor Line, at 90 degrees
to the target plane: `createInput()` on the Design component's `constructionPlanes`, then
`setByAngle(linearEntity, angle, planarEntity)` with the Anchor Line, `createByString('90 deg')`
and the ORIGINAL `targetPlane` as the reference, then `add(input)`. Build it off the user's
selected plane, not off a re-derived copy (`[PB-USE-SELECTED-PLANE]`): this is the second place
the target plane's orientation reaches the bodies, and substituting another plane here collapses
the gear onto XY as surely as the first.

Pass the sketch line straight to `setByAngle`; never wrap it in `Path.create(curve, ...)` first
(`[PB-CONSTRUCTION-PLANES]`).

This step is `[PROSE]`. proofkit gates a sketch and proofkit3d gates a solid; neither models a
construction plane, so there is no substitute geometry that carries the plane's angle. The
orientation it produces is what the next step's sketch is expressed in, and that sketch is proved.

**From:** `spec/bevelgear/instructions.md` L382–383, `spec/bevelgear/instructions.md` L472–473

## S07 `[GO]` Gear Profiles sketch — the §2 lattice

One sketch, named `Gear Profiles`, on the Gear Profiles Plane, holding the whole lattice. Every
line in it is a construction line: set `isConstruction` on each. Solid features later consume the
per-gear Profile sketches, never a §2 curve.

Two construction rules govern every line here. Build each from raw `Point3D.create(x, y, z)`
coordinates and pin each endpoint that meets existing geometry with exactly one
`addCoincident(point, entity)` — never pass an existing sketch point into `addByTwoPoints` to
share it, and never do both (`[BEVEL-F-COINCIDENT-STYLE]`, `[PB-SHARE-XOR-COINCIDENT]`). This
covers the short reference and connector lines too. And each named line is created once and
reused; a second line over the same segment carries its own constraints and over-determines the
net (`[BEVEL-F-LINE-ONCE]`).

Project the ANCHOR SKETCH's stashed centre point with `sketch.project(entity)`, not the raw
user selection: the projection keeps the chain inside Design.

Place the figure in the sketch's own 2-D coordinates, never through a world round-trip
(`[BEVEL-F-APEX-LOCAL]`). With `c` the projected centre and `d` the projected anchor line's unit
direction, the in-plane perpendicular is `(-d.y, d.x)`, and its SIGN is chosen so it points toward
the target plane's normal — read as `targetPlane.geometry.normal`, which is a `core.Plane`
carrying `.normal` for both selection kinds — not by the sketch's local +Y
(`[BEVEL-F-GROW-SIDE]`).

Then, in order:

1. **Centre to Apex.** A construction line from the projected centre, its far end seeded at
   `c + perp * (Driving Gear Pitch Diameter)`, with `addCoincident(point, entity)` on the start and
   `addPerpendicular(lineOne, lineTwo)` against the projected anchor line. No length constraint.
   The far end is the Apex.
2. **Driving Gear Shaft Axis.** From the Apex back toward the anchor line, seeded at
   `c - perp * length`, coincident at the Apex and `addParallel(lineOne, lineTwo)` to the
   centre-to-apex line. Do not use `addVertical(line)` — it forces the sketch's world-vertical,
   which is wrong on a tilted plane. The far end is point B; do not dimension the length.
3. **Pinion Gear Shaft Axis.** From the Apex, coincident at the Apex, at the Shaft Angle from the
   driving axis. Choose the sense by forming BOTH candidates — the driving direction rotated about
   the apex by plus and by minus the Shaft Angle — and keeping the one whose endpoint has the
   greater sketch X. Compare the two; never rotate one fixed sense and flip only on a negative X.
   The far end is point A; do not dimension the length. Dimension the angle with
   `addAngularDimension(lineOne, lineTwo, textPoint)`, its text point inside the wedge so it
   measures the shaft angle and not its supplement: put it on the interior bisector of the two
   shaft directions, a quarter of the Pinion Gear Pitch Diameter out from the apex along the
   normalised sum of the two unit directions (`[PB-ANGULAR-DIM]`).
4. **The two drops to Apex 2.** From A, a line perpendicular to the pinion shaft, aimed at the
   OTHER shaft — pick the perpendicular sense by the sign of its dot product with the A-to-B
   direction — with `addPerpendicular(lineOne, lineTwo)` and a distance dimension of Pinion Gear
   Pitch Diameter over 2. From B the mirror, aimed at point A by the sign of its dot with the
   B-to-A direction. Never choose the B-side sense against a toward-the-anchor-line reference: the
   driving shaft is itself parallel to that direction, so the test is degenerate and picks a side
   at random, and a wrong side makes the solver flip the entire frame to the mirror answer. Close
   the two drops with `addCoincident(point, entity)` on their far ends; that point is Apex 2.
   Throughout this list, "A to Apex 2" always names this drop line, never the Apex-to-A shaft axis.
5. **Seeds for the along-shaft lengths.** Seed `|Apex to A| = R * cos(gamma_p)` and
   `|Apex to B| = R * cos(gamma_g)` from the closed form, with `R = (PPD / 2) / sin(gamma_p)`.
   They stay undimensioned (`[BEVEL-F-DRIVEN-DIMS]`); the seed only picks the branch.
6. **Pitch Line.** Apex to Apex 2, coincident at both ends.
7. **The two dedendum lines.** From Apex 2 to either side, each perpendicular to the Pitch Line
   with `addPerpendicular(lineOne, lineTwo)` and a distance dimension of `Module * 1.25`. The one
   drawn toward the anchor line is the Driving Gear Dedendum, ending at point D; the one away is
   the Pinion Gear Dedendum, ending at point C.
8. **The two root axes.** Apex to D and Apex to C, coincident at both ends.
9. **The module-length extensions.** From A, collinear with Apex to A by
   `addCollinear(lineOne, lineTwo)`, seeded one module long and NOT dimensioned, ending at E; then
   C to E, coincident at both ends, with `addPerpendicular(lineOne, lineTwo)` between A-to-E and
   C-to-E. The mirror on the driving side gives F. Then E to G collinear with A to E, C to H
   collinear with Apex 2 to C, G to H coincident at both ends with E-to-G perpendicular to H-to-G;
   and the driving mirror F to I, D to J, I to J.
10. **The two base-height offsets.** `addOffsetDimension(line, entityTwo, textPoint)` between the
    B-to-Apex-2 DROP and I-to-J, its value the Driving Gear Base Height if given, otherwise
    `module * drivingTeeth / 8`; and between the A-to-Apex-2 drop and G-to-H, its value the Pinion
    Gear Base Height if given, otherwise the RESOLVED driving offset times
    `pinionTeeth / drivingTeeth`. The two lines are already parallel by construction, so add no
    parallel constraint — a redundant one over-constrains the sketch (`[PB-OFFSET-DIM]`).
11. **A to G**, coincident at both ends, and **point I coincident with the projected centre**,
    which is what fixes the figure's position along the grow direction.
12. **Resolve the Maximum Face Width.** A, B, C, D, H and J are now solved, so read their
    `geometry` — the SOLVED positions, never the seed coordinates (`[PB-SOLVED-GEOMETRY]`) — and
    take 0.95 times the smaller of the perpendicular distance from A to the line through C and H
    and from B to the line through D and J. Cap the auto face width, `min(coneDistance / 6,
    maximum)`, at it, and reject a user value above it with a message naming the maximum.
13. **K, and the tooth-centre point K prime.** From G along Apex-to-A extended, its far end pinned
    with two point-on-line `addCoincident(point, entity)` constraints — onto the Apex-to-A line and
    onto the Apex-2-to-C dedendum line — never with `addCollinear`, which over-constrains here
    because G and C are already fixed. That end is K. Draw C to K for reference. When Tooth Spacing
    is above 0, draw one more line from K, its far end seeded on the far side of K from C, pinned
    onto the dedendum line the same way, with a length dimension equal to Tooth Spacing; that end
    is K prime, and C to K prime is the §3 reference line. At Tooth Spacing 0, K prime IS K and the
    existing C-to-K line is reused — a zero-length line would be degenerate and a second line over
    C-to-K a duplicate. The driving mirror gives L, L prime and D to L prime.
14. **The two toe lines.** Seed M near its solved position and N slid along C-to-H far enough to
    reach the A-to-Apex-2 drop (`[PB-SEED-NEAR]`), then apply exactly four constraints:
    `addCoincident(point, entity)` putting M on the Apex-to-C root axis, `addCoincident(point,
    entity)` putting N on the A-to-Apex-2 DROP — not the shaft axis, which would put N on the axis
    of revolution — `addParallel(lineOne, lineTwo)` to C-to-H, and
    `addOffsetDimension(line, entityTwo, textPoint)` from C-to-H set to the Face Width, its text
    point in the gap on the apex side. The driving mirror gives O and P. Then the reference lines
    M to C, N to A, O to D, P to B and B to I.

Gate the sketch on `isFullyConstrained` and raise (`[BEVEL-F-FULL-CONSTRAINT]`). Full constraint
comes from the missing constraint, never from dimensioning a driven length
(`[PB-NO-OVERCONSTRAIN]`); every dimension added here is driving, so never pass the trailing
`isDriving` argument as false (`[PB-DRIVING-DIM]`). Curve collections live under
`sketch.sketchCurves` (`[PB-SKETCHCURVES]`), the constraint spellings are exact — `addCollinear`
has two l's (`[PB-API-SPELLING]`) — and a helper that turns a 2-D coordinate into a `Point3D` must
take both raw tuples and objects with `.x`/`.y`, because this step feeds it both seeds and solved
geometry (`[PB-POINT-HELPER]`).

`addVertical` and `addCollinear` appear above only where the step forbids them for a particular
line; `addCollinear` is required elsewhere in the same step and stays required.

The proof function is `stepGearProfiles`. It reaches DOF 0 with nothing redundant and no discrete
ambiguity, and measures every lattice point, both cone angles, the cone distance, the face-width
bound and the toe-line pins against the closed form. It records four defects it had to work
around: the apex seed, the toe-line seeds, the redundant half of the I-onto-centre closure, and
the mirror answers the unsigned perpendicular constraints leave open.

<!-- check-step-calls: ignore addVertical -->
<!-- proof-run: proofkit.Run(gearProfilesCases, stepGearProfiles) -->

**From:** `spec/bevelgear/instructions.md` L384–468, `spec/bevelgear/instructions.md` L57–77,
`spec/bevelgear/fusion.md` L74–142

## S08 `[PROSE]` Tooth plane, per gear

Once per gear, pinion first: create the construction plane named `{gearLabel} Plane` through that
gear's tooth-centre reference line — C to K prime for the pinion, D to L prime for the driving
gear — perpendicular to the Gear Profiles sketch plane. Use the framework helper
`plane_by_angle(component, line, refPlane, angleDeg)` with 90 degrees; it passes the sketch line
directly to `setByAngle` and never wraps it in a path (`[PB-CONSTRUCTION-PLANES]`).

`[PROSE]` for the reason S06 gives: neither harness carries a construction plane. The frame this
plane defines is the frame the next step's tooth is drawn in, and that tooth is proved.

**From:** `spec/bevelgear/instructions.md` L481, `spec/bevelgear/instructions.md` L472–476

## S09 `[GO]` Virtual spur tooth sketch, per gear

Once per gear, in a sketch named `{gearLabel} Tooth` on that gear's tooth plane.

First the virtual tooth number, from the closed form and never by measuring the lattice: the
virtual pitch radius is this gear's pitch radius over `cos(gamma)`, with gamma the pitch cone half
angle from §2. Pin the unit conversion — the stashed pitch diameters are internal CENTIMETRES
while Module is the raw MILLIMETRE number — so
`virtualPitchRadius_mm = (pitchDia_cm * 10 / 2) / cos(gamma)` and
`virtualTeeth = floor(2 * virtualPitchRadius_mm / Module)` as an int. Dropping the factor of ten
makes the count about ten times wrong.

Then draw the tooth with the borrowed spur generator. Construct
`VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)`, hand it to
`SpurGearInvoluteToothDesignGenerator(sketch, proxy)`, and call
`drawer.draw(anchorPoint, angle=math.radians(180))`. Import the framework's proxy; a local copy of
it is a contract violation. The 180 degree rotation is delivered through
that angle argument, which rotates the whole tooth as it is drawn; it is not a later move or
sketch rotation. The drawer lays down the four circles — root solid, tip, base and pitch as
construction, all centred on the tooth-centre point — and one involute tooth.

After `draw` returns, read `proxy._lastToothEmbedded` back and thread it to the profile selection:
the drawer writes that slot during the draw, and it is the deterministic selector for the tooth
loop's line count, not optional bookkeeping.

Do NOT gate this sketch. Log it if `isFullyConstrained` is false and never raise: the drawer
labels its four circles with along-path sketch text, and sketch text placed along a path carries
its own unpinned position, so a tooth whose geometry is completely determined still reads false
(`[BEVEL-F-FULL-CONSTRAINT]`).

The proof function is `stepToothProfile`. It draws the four circles and the tooth outline from the
shared involute math, measures the virtual tooth number and the millimetre conversion, and reads
the tooth region's curve mix out of the detected profiles — two NURBS, two arcs and the line count
the embedded flag decides. It substitutes the drawer's rib-and-spine constraint scheme for a
recreate-and-fix outline, which is spur's scheme to prove and which this gate exempts anyway.

<!-- proof-run: proofkit.Run(toothCases, stepToothProfile) -->

**From:** `spec/bevelgear/instructions.md` L477–484, `spec/bevelgear/instructions.md` L332–360,
`spec/bevelgear/fusion.md` L34–61

## S10 `[PROSE]` Tooth axis, per gear

Create the construction axis named `{gearLabel} Tooth Axis` through the tooth-centre point, normal
to the plane the tooth was drawn on: `createInput()` on the Design component's `constructionAxes`,
then `setByTwoPlanes(planarEntityOne, planarEntityTwo)` with the Gear Profiles plane and a helper
plane built `setByDistanceOnPath(pathEntity, distance)` on the tooth-centre reference line at 1.0 —
perpendicular to that line at its far end, the tooth-centre point — then `add(input)`. Their
intersection is the line through the tooth centre normal to the tooth plane
(`[PB-CONSTRUCTION-AXES]`). `setByPerpendicularAtPoint` is the obvious alternative and is not
usable here: it needs a `BRepFace` this step does not have.

Adding this axis in the never-activated Design component is proven to work, so keep it
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`).

`[PROSE]` for the reason S06 gives: neither harness carries a construction axis.

<!-- check-step-calls: ignore setByPerpendicularAtPoint -->

**From:** `spec/bevelgear/instructions.md` L485, `spec/bevelgear/instructions.md` L472–473

## S11 `[GO]` Gear profile sketch, per gear

Once per gear, a FRESH sketch on the axial Gear Profiles plane, named `Pinion Profile` or
`Driving Profile`, holding exactly one hexagon loop. Do not draw both gears' hexagons in the
shared §2 sketch: that would leave two identically shaped loops to disambiguate.

Build the six vertices by the recreate-share-fix recipe (`[PB-PROJECT-NOT-FIXED]`), never by
projecting the §2 points: recreate each of the six at its exact world-mapped position with
`add(point)` on `sketch.sketchPoints`, using `modelToSketchSpace(modelCoordinate)` on the source
point's `worldGeometry` — a point-transforming method, not a matrix (`[PB-SPACE-METHODS]`) — then
draw the closed hexagon as six `addByTwoPoints(startPoint, endPoint)` lines SHARING those points,
and only then set `isFixed` on the lines' endpoints. Order matters: fixing a bare point before a
line consumes it does not leave the sketch fully constrained.

The draw order is A, G, H, C, M, N, back to A for the pinion, and B, I, J, D, O, P, back to B for
the driving gear. The FIRST edge — A to G, or B to I — is the gear's shaft axis for the revolve,
the pattern, the bore plane and the meshing rotation, so it has to carry a trustworthy world
position; fixed endpoints give it one, and a free edge resolves against a default frame and
silently moves the body onto world XY (`[PB-WORLDGEO-CONSTRAINED]`). Gate the sketch on
`isFullyConstrained` and raise (`[BEVEL-F-FULL-CONSTRAINT]`).

The proof function is `stepGearHexagon`. It checks the sketch holds exactly one valid six-line
loop, that the first edge runs along the gear's shaft direction, and that every vertex sits on one
side of that edge — the condition the revolve needs. It measures the Maximum Face Width bound
sharp, by pushing the offset two per cent past the reach and watching the toe end change sides,
and it records a second way across the axis that the spec does not bound at all: the heel edge
itself, once the resolved base height passes `(pitchRadius - 1.25 * module * cos(gamma)) *
tan(gamma)`.

<!-- proof-run: proofkit.Run(hexagonCases, stepGearHexagon) -->

**From:** `spec/bevelgear/instructions.md` L579–597, `spec/bevelgear/fusion.md` L19–33

## S12 `[GO]` Revolve the gear body, per gear

Create the gear's own component first — `addNewComponent(transform)` under the BEVEL GEAR
component, not under the user's Parent Component — named `Pinion Gear` or `Driving Gear`. The
finished bodies land there at the end; every feature below runs in Design and the bodies are moved
across afterwards, because Fusion rejects cross-sibling sketch and project calls
(`[PB-NO-CROSS-SIBLING]`).

The profile sketch holds exactly one closed loop, so take `sketch.profiles.item(0)` directly
(`[PB-SINGLE-PROFILE]`). Resist inventing a search that filters by loop or curve type. Revolve it
around the hexagon's first edge:
`createInput(profile, axis, operation)` on the Design component's `revolveFeatures`,
`setAngleExtent(isSymmetric, angle)` with false and `createByString('360 deg')`, then `add(input)`
(`[PB-REVOLVE]`). The result is the Gear Body, the frustum. The axis is the in-sketch edge, never
the §2 Apex-to-A or Apex-to-B construction line, which lives in a different sketch.

The profile must not cross the axis of revolution or Fusion aborts the revolve; that is what the
Maximum Face Width exists to guarantee, and S11 proves both the bound and the gap in it.

Because the toe edge is one edge of the revolved profile, the body already carries the conical face
that edge sweeps, and likewise the heel edge's cone. Those two faces are the cutting tools the next
trim step uses.

The proof function is `stepRevolveGearBody`. It records the harness limit that shapes it: decad's
revolve is the one feature verb whose bodies neither proofkit3d gate accepts, so the proof lofts
the frustum's root-cone band between two coaxial circles instead — the same surface the hexagon's
C-to-M edge sweeps, over the same toe-to-heel span — and measures its radii and volume.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepRevolveGearBody, assertRevolveGearBody) -->

**From:** `spec/bevelgear/instructions.md` L593–600, `spec/bevelgear/instructions.md` L579–592

## S13 `[GO]` Loft the tooth body, per gear

Loft the §2 Apex sketch point to this gear's §3 tooth profile:
`createInput(operation)` on the Design component's `loftFeatures`, then `add(entity)` on the
input's `loftSections` for the apex point and then for the tooth profile, then `add(input)`. A
loft section may be a single sketch point, and lofting one to a profile yields the tapered,
pointed body the bevel tooth is (`[PB-LOFT]`). Section order is loft order.

Use the §2 Apex SKETCH point — `centerToApex.endSketchPoint` from the Gear Profiles sketch —
directly. Do not create a construction point for it: construction geometry needs an active
component and Design is never activated (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`).

Select the tooth cross-section with `find_profile_by_curve_counts(sketch, nurbs, arcs, lines)`,
asking for 2 NURBS, 2 arcs and a line count DETERMINED by the embedded flag read back in S09 —
0 when embedded, 2 when not. Never accept either count: an unrelated loop between the drawer's
circles can carry the same NURBS and arc counts with the other line count, and selecting it makes
this loft fail with no tool body.

The proof function is `stepLoftTooth`. It substitutes a section a fixed two per cent out from the
apex for the point section decad has no verb for, and measures the taper: the body's volume is a
third of its section's area times the apex's perpendicular distance to the section plane. It also
measures where the tooth profile sits along the cone element, and how far the virtual tooth
number's floor leaves its root from the dedendum corner.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepLoftTooth, assertLoftTooth) -->

**From:** `spec/bevelgear/instructions.md` L601–602, `spec/bevelgear/instructions.md` L268–277

## S14 `[GO]` Conical end trims — the straight tooth-body branch

This is the tooth-body hook. Its first line is the gate: when the Mean Spiral Angle is 0 or below,
return `cut_conical_ends(component, toothBody, gearBody, toeMid, heelMid, apexWorld, label)` and
stop — that is the straight bevel, byte for byte the prior behaviour. Above 0, steps S15 to S23
run in its place, and this trim runs again at the end of them.

The helper is the framework's; do not re-implement the cut machinery. Two different bodies are
involved and conflating them is the classic error: the cutting TOOLS are the cone-type faces of
the GEAR BODY, the revolved frustum, and the TARGET being split is the TOOTH BODY, the loft. The
lofted tooth has no cone faces at all, so searching it for one finds none.

What the helper does, and what the caller owes it. It cuts the toe first, finding its cone face by
the toe edge's world MIDPOINT rather than an endpoint, because an endpoint sits near the cone's
apex singularity where the distance is unevaluable; it orders the candidate faces best-first and
tries each as the actual split tool, keeping the first that splits. After each cut it drops the
apex-containing pieces and keeps the largest
(`[PB-REMOVE-PIECES]`). Then it cuts the heel on the keeper alone, which is what makes it exactly
two split features for every gear ratio. A heel cone that does not reach the keeper is raised as a
typed non-intersect error and caught, and the keeper is returned whole; the toe cut is strict and
its failure propagates, because an untrimmed tooth is unusable. Every failure carries its measured
per-face distances, and a piece list that comes back empty is checked at the point it is produced,
never fed into a later max.

The caller passes `toeMid` as the toe edge's world midpoint — half of M plus N for the pinion, half
of O plus P for the driving gear — `heelMid` as the heel edge's world midpoint, half of C plus H
and half of D plus J, `apexWorld` as the §2 Apex sketch point's world geometry, and `gearBody` as
the revolved frustum.

The proof function is `stepTrimToothBand`. decad has no split verb and refuses booleans on both
payloads involved, so the proof lofts the flush band directly between the tooth's sections at the
toe and heel cone distances, and measures its volume, that the heel really is the outer end, and
that the corner-to-corner extent along the cone element is the face width over the sine of the
angle the dedendum line makes with that element.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepTrimToothBand, assertTrimToothBand) -->

**From:** `spec/bevelgear/instructions.md` L603–629, `spec/bevelgear/instructions.md` L260–266,
`spec/bevelgear/instructions.md` L487–490

## S15 `[GO]` Cone element sketch, per gear, when the spiral angle is above 0

Build the frame this branch works in. From the geometry already constructed for this gear:
`axisDir` is the shaft axis direction taken from the two WORLD endpoints of the profile sketch's
first edge; `coneVec` is the root cone element, the unit direction from the apex to the heel
dedendum corner — C for the pinion, D for the driving gear, never H or J, which lie a module beyond
it and skew the vector; `v` is `axisDir` crossed with `coneVec`, the circumferential direction; and
a point's cone distance is its offset from the apex projected on `coneVec`. Take every one of these
from world geometry, never from a sketch-local frame (`[PB-WORLD-FRAME]`).

Guard the ends before anything else: if the apex is nearer the passed heel midpoint than the toe
midpoint, swap toe and heel — both the midpoints and the cone points — and rebuild `coneVec`. A
negative span silently inverts the whole spiral, flipping the cutter arc, the slice direction and
every segment's twist, with no error. Then read the toe and heel cone distances off the two
midpoints, the mean as their average, and the span as heel minus toe.

Now the sketch. In a sketch named `{gear} Cone Element` on the axial Gear Profiles plane, draw one
construction line from the apex to the apex plus the heel cone distance along `coneVec`, using
`addByTwoPoints(startPoint, endPoint)`. This is a transient auxiliary sketch: do NOT gate it on
`isFullyConstrained` (`[BEVEL-F-FULL-CONSTRAINT]`).

The proof function is `stepConeElementSketch`. proofkit gates every sketch it is handed, so the
proof pins the free end the recreate-and-fix way and measures the line's length against the heel
cone distance and its direction against the root cone element.

<!-- proof-run: proofkit.Run(spiralSketchCases, stepConeElementSketch) -->

**From:** `spec/bevelgear/instructions.md` L491–519, `spec/bevelgear/instructions.md` L531,
`spec/bevelgear/spiral-tooth-trace.md` L30–65, `spec/bevelgear/fusion.md` L62–73

## S16 `[PROSE]` Trace plane, per gear, when the spiral angle is above 0

Build the cone's tangent plane as the axial plane rotated 90 degrees about the cone-element line:
`plane_by_angle(component, line, refPlane, angleDeg)` with the `{gear} Cone Element` line, the
axial plane and 90. Name it `{gear} Trace Plane`. The helper passes the sketch line straight to
`setByAngle` (`[PB-CONSTRUCTION-PLANES]`).

`[PROSE]` for the reason S06 gives: neither harness carries a construction plane. The 2-D frame it
establishes — apex at the origin, x along the cone element so a point's x IS its cone distance, y
circumferential — is the frame the next step's sketch is expressed in, and that sketch is proved.

**From:** `spec/bevelgear/instructions.md` L531, `spec/bevelgear/spiral-tooth-trace.md` L30–65

## S17 `[GO]` Tooth trace sketch, per gear, when the spiral angle is above 0

First the cutter-arc geometry, in the tangent plane's 2-D frame. The cutter radius is the Cutter
Radius input when it is non-zero and the mean cone distance otherwise. The hand sign is plus 1 for
`Right` and minus 1 for `Left`, then NEGATED for the pinion, so the pair meshes with opposite
hands. The centre is `Cx = R_mean - r_c * sin(psi)` and `Cy = handSign * r_c * cos(psi)`. The hand
sign belongs on the cosine term: on the sine term it mirrors the centre about `x = R_mean` instead
of across the cone element, which is a different curve and gives the two gears unequal twist.

The arc's two ends are circle-against-circle intersections taken a hair past the face, so the kept
arc reaches cleanly past the end trims: `circle_intersect_nearest(R, Cx, Cy, r_c, refX, refY)` with
`R_lo = R_toe - 0.06 * span` and `R_hi = R_heel + 0.06 * span`. The helper keeps the solution
nearest the mean point, which is the branch the mean point sits on.

Then the sketch, named `{gear} 2D Tooth Trace`, on the trace plane, with
`combine_point(base, a, e1, b, e2)` mapping a 2-D coordinate to a world point:

- the cutter circle, `addByCenterRadius(centerPoint, radius)` at the centre with the cutter radius,
  marked `isConstruction`, its centre pinned by setting `isFixed` on `centerSketchPoint` rather than
  coinciding it to the sketch origin (`[PB-CIRCLE-CENTER]`), and a
  `addDiameterDimension(entity, textPoint)` of twice the cutter radius;
- the trace arc, `addByThreePoints(startPoint, point, endPoint)` through the toe point, the mean
  point on the cone element and the heel point, with `addCoincident(point, entity)` tying its
  centre to the cutter circle's and a `addRadialDimension(entity, textPoint)` of the cutter radius,
  so it is the genuine cutter circle and not a look-alike spline.

Both dimension text points must be off-centre, on or near their curve — the mean point for the
arc's radius, a point on the circle such as centre plus the cutter radius along x for the
diameter — because a text point AT the centre is rejected (`[PB-RADIAL-DIM]`).

The world points go straight into the sketch calls and are consumed as sketch-space input, with no
`modelToSketchSpace` conversion. That is deliberate and harmless: no downstream feature consumes
this sketch — the twist is computed analytically in S20 — and it exists only so the genuine cutter
arc is inspectable before cleanup hides it. The sketch is deliberately left with free degrees of
freedom and is NOT gated (`[BEVEL-F-FULL-CONSTRAINT]`).

There is no 3-D projection anywhere in this branch. Never project the arc onto the root cone with
`projectToSurface(faces, curves, projectType)`: for unequal-ratio pairs it wraps and comes back as
disjoint fragments, and the measured azimuth collapses to a fraction of the true sweep.

The proof function is `stepCutterArcSketch`. It pins the arc's two ends and the one component of
its centre the arc's own construction leaves free, then MEASURES the rest: that the centre is the
cutter centre, that the radius is the cutter radius, that the arc passes through the mean point,
that its ends sit at the toe and heel cone distances, that the spiral angle is realised at the mean
point, that flipping the hand mirrors the construction across the cone element and changes nothing
else, and that the zero-spiral-angle limit puts the centre due circumferential of the mean point.

<!-- check-step-calls: ignore projectToSurface -->
<!-- proof-run: proofkit.Run(spiralSketchCases, stepCutterArcSketch) -->

**From:** `spec/bevelgear/instructions.md` L520–541, `spec/bevelgear/spiral-tooth-trace.md` L90–185,
`spec/bevelgear/spiral-tooth-trace.md` L218–239

## S18 `[GO]` Slice the tooth into slabs, when the spiral angle is above 0

Split the uncut apex-to-heel tooth body into cross-section slabs with a FIXED scheme of eight
planes; the count is not user-configurable. The first cut plane is the parent transverse tooth
plane — this gear's `{label} Plane` from S08, passed into the hook — offset toward the apex by a
sixth of the span. Choose the offset sign per gear so it really moves apex-ward, by testing the
sign of the apex-minus-plane-origin vector against the plane's normal; the two gears' normals point
opposite ways. The eight offsets are then `sign * (k + 1) * span / 6` for k from 0 to 7.

Split with `slice_body_by_offset_planes(component, body, basePlane, offsets)`, the framework
helper: it splits piece by piece and leaves a piece whole where a plane misses it.

The slice MUST actually split the tooth. If the body is still in one piece after the loop, the sign
was wrong or the parent plane sits outside the tooth's span — retry the whole cut once with the
opposite sign. If it is STILL one piece, raise a self-diagnosing error naming the gear, the final
piece count, the span and the sign tried, so the message names the measured quantities rather than
a bare count. Never return an unsliced result:
the next step then drops that single piece as scrap, leaves the segment list empty, and the crown
crashes far from the cause.

The proof function is `stepSliceTooth`; its case table walks the segment index, so each run builds
the one slab that index names. decad has no split verb and refuses booleans on a lofted payload, so
the proof lofts each slab between the tooth's sections at the two cut planes and measures its
volume, the step between planes against a sixth of the span, and that the scheme reaches past the
toe at one end and inside the heel at the other.

<!-- proof-run: proofkit3d.RunSolid(segmentCases, stepSliceTooth, assertSliceTooth) -->

**From:** `spec/bevelgear/instructions.md` L542–543, `spec/bevelgear/instructions.md` L268–277

## S19 `[GO]` Order the slabs and drop the apex scrap, when the spiral angle is above 0

Sort the slabs by the cone distance of their centroid, read from `physicalProperties` and its
`centerOfMass`. The first, apex-most piece is the long scrap below the toe: re-slice the list
first and delete afterwards — take `segments[1:]`, THEN remove the scrap with `add(itemToRemove)`
on the Design component's `removeFeatures`, which is timeline-visible, rather than a bare delete
(`[PB-REMOVE-PIECES]`).

After the drop the segment list must be non-empty. If it is, the slice failed in S18 — raise a
clear error rather than proceeding into the twist and the crown, both of which assume at least one
segment. A collection that can legitimately come back empty is guarded where it is produced,
never three steps later.

The proof function is `stepDropApexScrap`. It builds the scrap piece and measures that its centroid
really is apex-most, that it lies below the toe, and that eight segments remain after it goes.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepDropApexScrap, assertDropApexScrap) -->

**From:** `spec/bevelgear/instructions.md` L544–545

## S20 `[GO]` Twist the slabs about the shaft axis, when the spiral angle is above 0

Rotate each segment about the shaft axis through the apex so the tooth follows the trace, centred
on the mean cone distance so the mid-face section stays UNROTATED — that section then meshes
exactly as the straight tooth does, which is why the pinion needs no extra mesh phase.

The total toe-to-heel twist comes from the conjugate crown-gear generation law, analytically, with
no projection and no curve sampling:

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])
total     = abs(phi_crown) / math.sin(gamma)
```

`phi_crown` is the angle the cutter arc's toe and heel endpoints subtend at the apex in the flat
2-D crown frame — the pairs from S17. `gamma` is this gear's PITCH cone angle, `_gamma_p` or
`_gamma_g` from §2. Never use the root cone angle, which is what the arccosine of `coneVec` against
`axisDir` measures: it is a different, smaller angle and inflates the twist. The two members of a
meshing pair legitimately get different twists, because the same spiral angle and cutter meet
different pitch cone angles.

Each segment's rotation is a linear share keyed on the cone distance of its HEEL FACE, not its
centroid — the loft samples that face, and centroid keying leaves the mid-face section rotated by
half a segment:

```
ang = -handSign * total * (R_mean - R_heelFace(seg)) / span
```

Find a slab's heel face as the face whose centroid has the GREATEST cone distance, searched across
ALL of the slab's faces with NO surface-type filter; its toe face is the least. A sliced slab is
bounded by a mix of planar cut faces and ruled side faces, and a type filter picks the wrong one or
misses the cut face. The same all-faces rule is used everywhere a slab end face is needed: here,
in the crown, and in the loft.

Apply the rotation as a free move: build the matrix with
`setToRotation(angle, axis, origin)` on `adsk.core.Matrix3D.create()`, then
`moveFeatures.createInput2(inputEntities)` on the Design component with an
`ObjectCollection.create()` holding the body, `defineAsFreeMove(transform)`, and `add(input)`
(`[PB-MOVE-ROTATE]`).

The proof function is `stepTwistSegments`; its case table walks the segment index. It measures the
rotated body's azimuth against the untwisted one, the developed crown azimuth against the arc's own
endpoints, the total against the law, this segment's share against the linear rule, that the pitch
and root cone angles differ enough for the choice between them to matter, and that heel-face keying
differs from centroid keying by exactly half a segment's share.

<!-- proof-run: proofkit3d.RunSolid(segmentCases, stepTwistSegments, assertTwistSegments) -->

**From:** `spec/bevelgear/instructions.md` L546–560, `spec/bevelgear/spiral-tooth-trace.md` L186–217

## S21 `[GO]` Lengthwise crown, when the spiral angle is above 0

Crown the tooth by scaling each segment EXCEPT the outermost down by a monotonic factor: full at
the heel, growing smoothly toward the toe. For each segment take the heel-distance fraction
`u = (R_heel - R_heelFace) / span`, with the heel face found by the S20 all-faces rule but
RECOMPUTED here, after the twist has moved the slabs. Then
`factor = 1 - _CROWN_PER_RAD * (abs(total) / 2) * u`, with `_CROWN_PER_RAD` a class constant
defaulting to 0.5. `u` runs from 0 at the held-full heel to 1 at the toe, so the relief grows
monotonically and the slab heights stay strictly ordered heel to toe. Key it on `u`, never on the
twist magnitude, which is symmetric about the mid-face and would make the slab just inside the held
heel the most relieved one — a notch that reverses the taper. If a computed factor comes out at or
below 0, raise a self-diagnosing error naming the gear, the segment's `u` and the factor; never
scale by a non-positive factor.

Three things the scale itself needs.

The base must be a sketch point — one added on the heel face, or a BRep vertex — because
construction points need an active component. And `scaleFeatures` is the one exception to
never-activate: call `activate()` on the DESIGN OCCURRENCE before the crown scales and restore the
root in a `finally` with `activateRootComponent()` on the design. Only an occurrence has
`activate`; a component does not, and reaching for it raises an attribute error
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`).

Skip the outermost segment — the one with the greatest post-twist heel-face cone distance, found by
sorting the segments on that value. Its heel face is the loft's heel end and has to stay full so the
heel cone trims it flush with the gear base.

Anchor the scale on the heel face's ROOT edge, not its centroid. A uniform scale keeps every line
through its base point where it is, so a base on the root keeps the root edge on the seating cone
while the tip is relieved; a base at the face's centroid, at mid tooth height, lifts the root by the
shrinkage times half the tooth height and the tooth floats off the gear base. Take the heel face's
vertices, keep the TWO whose perpendicular distance to the shaft axis is smallest — the root corners,
since the tip corners are farthest from the axis — and put the base point at their midpoint, mapped
into a sketch on the heel face with `modelToSketchSpace(modelCoordinate)` (`[PB-SPACE-METHODS]`).

Then `createInput(inputEntities, point, scaleFactor)` on the Design component's `scaleFeatures`
with an `ObjectCollection.create()` holding the segment, the base sketch point and
`createByReal(factor)`, and `add(input)`.

The proof function is `stepCrownSegments`; its case table walks the segment index. It measures the
factor against the law, that the relief grows monotonically outward-to-inward, that the outermost
segment is held full, that the scale base sits on the root edge rather than at mid height, and that
the centroid anchor would lift the root by a positive amount.

<!-- proof-run: proofkit3d.RunSolid(segmentCases, stepCrownSegments, assertCrownSegments) -->

**From:** `spec/bevelgear/instructions.md` L561–574

## S22 `[GO]` Loft the curved tooth, when the spiral angle is above 0

Re-sort the segments by their heel-face cone distance HERE, after the twist and the crown; do not
reuse the pre-twist slice order. The twist rotates each slab about the shaft axis, and for a
high-twist unequal-ratio pair that rotation changes the slabs' along-cone order enough to reorder
adjacent ones. Lofting in the stale order assembles the cross-sections out of sequence and the
crowned tooth comes out distorted — which is why an equal-teeth pair meshes with the stale order
while a ratio pair does not.

Then loft a new body through, in that order: first the toe-most segment's apex-side face, added
first so the loft pushes past the toe cone and the toe trim bites, then the heel-facing face of
every segment iterated in order, each found by the S20 all-faces rule. Use
`createInput(operation)` on `loftFeatures`, `add(entity)` on `loftSections` per section in order,
then `add(input)` (`[PB-LOFT]`). Name the result `{gear} Spiral Tooth`, then remove the segment
scaffolding with `add(itemToRemove)` on `removeFeatures`; the loft has captured their faces.

The proof function is `stepLoftSpiralTooth`. decad's loft rules between two profiles, so the proof
lofts between the two ends of the recomputed order, each carrying its own twist, and reports which
order came out. It measures that the two ends lie on opposite sides of the unrotated mid-face and
that the twist across them matches the law. The case table stops at 57 degrees rather than the 60
the range reaches: at 59 the two end sections are turned far enough apart that the evaluator refuses
the single loft as degenerate, which is a limit of the two-section substitute — S18, S20 and S21
each carry a 59 degree case, where every loft spans one slab.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepLoftSpiralTooth, assertLoftSpiralTooth) -->

**From:** `spec/bevelgear/instructions.md` L575–576

## S23 `[GO]` Flush trim of the curved tooth, when the spiral angle is above 0

Return `cut_conical_ends(component, toothBody, gearBody, toeMid, heelMid, apexWorld, label)` on the
curved tooth — the same toe-then-heel two-cone trim S14 describes, with the same caller obligations
— so the curved tooth's ends sit flush on the gear base. The toe and heel mesh phasing is not done
here: it belongs to the mesh-rotate step, and the pinion's extra phase is 0 by default precisely
because the mid-face section came out of S20 unrotated and already meshes.

The proof function is `stepTrimSpiralTooth`, substituted the way S14's is. It measures that the
trimmed tooth's toe-to-heel twist is the full total, that the mid-face section is unrotated, and
that the opposite hand comes out as an exact mirror — same magnitude, opposite sense.

<!-- proof-run: proofkit3d.RunSolid(spiralSolidCases, stepTrimSpiralTooth, assertTrimSpiralTooth) -->

**From:** `spec/bevelgear/instructions.md` L577–578, `spec/bevelgear/instructions.md` L603–629

## S24 `[GO]` Circular pattern of the tooth, per gear

Circular-pattern the remaining tooth piece around the SHAFT-AXIS EDGE — the same in-sketch profile
edge the revolve used, never the §2 construction line. `createInput(inputEntities, axis)` on the
Design component's `circularPatternFeatures` with an `ObjectCollection.create()` holding the tooth
and that edge, then pin all three inputs explicitly: `quantity` from `createByReal(teeth)`,
`totalAngle` from `createByString('360 deg')`, and `isSymmetric` false. Do not rely on Fusion's
defaults matching them (`[PB-CIRCULAR-PATTERN]`). Then `add(input)`.

The angular spacing stays a constant 360 degrees over the tooth number across the whole face width,
even though the pitch diameter shrinks from heel toward apex: the radial taper is already produced
by the loft from the apex to the heel-end profile, so the pattern only rotates that one tapered
tooth into evenly spaced copies.

The pattern's `bodies` already include the seed body plus the copies, so do not re-add the seed
(`[PB-PATTERN-BODIES]`).

The proof function is `stepPatternTeeth`. It substitutes a prism over the tooth's transverse
footprint for the tapered tooth, because decad's booleans take a prism where they refuse a loft,
and measures each copy's volume and azimuth. The gate carries the real content: decad reports any
proven overlap between two bodies as a diagnostic, so a pattern whose teeth collided would fail
here, and the proof also measures the tooth's angular half-width against half the angular pitch.

<!-- proof-run: proofkit3d.RunSolid(patternCases, stepPatternTeeth, assertPatternTeeth) -->

**From:** `spec/bevelgear/instructions.md` L631–632

## S25 `[GO]` Combine-join the teeth to the gear body, per gear

Join all patterned tooth pieces with the Gear Body in a SINGLE Combine-Join: the Gear Body is the
target and the patterned tooth bodies are the tools. `createInput(targetBody, toolBodies)` on the
Design component's `combineFeatures`, with the tools copied into an `ObjectCollection.create()` —
the pattern's `bodies` is a `BRepBodies` collection and the input rejects it (`[PB-PATTERN-BODIES]`)
— then set the input's `operation` to the join and `add(input)`.

The proof function is `stepCombineTeeth`. It joins the same transverse prisms the pattern step
builds to a cylinder standing in for the gear body, and measures that one lump comes out with every
tooth attached and that the joined volume lies between the body alone and the body plus every whole
tooth. The stand-in reaches slightly past the tooth roots and past the teeth at both ends, because
two bodies that merely touch, or whose end faces are coplanar, are not a boolean the evaluator will
take.

<!-- proof-run: proofkit3d.RunSolid(patternCases, stepCombineTeeth, assertCombineTeeth) -->

**From:** `spec/bevelgear/instructions.md` L633–634

## S26 `[PROSE]` Bore plane, per gear

Skip this and the two steps after it entirely when Enable Bore is unchecked.

Build the plane normal to the shaft at its start: `createInput()` on the Design component's
`constructionPlanes`, `setByDistanceOnPath(pathEntity, distance)` with the SHAFT-AXIS EDGE — the
in-sketch profile edge, not the §2 construction line — and `createByReal(0.0)`, then `add(input)`.
Pass the edge directly; never wrap it in a path (`[PB-CONSTRUCTION-PLANES]`).

`[PROSE]` for the reason S06 gives: neither harness carries a construction plane. What the plane
decides — that the sketch origin lands on the shaft axis — is what the next step's circle is
centred on, and that circle is proved.

**From:** `spec/bevelgear/instructions.md` L635–636

## S27 `[GO]` Bore sketch, per gear

In a sketch named `{gearLabel} Bore` on the bore plane, draw the bore circle centred at the sketch
origin — the plane is rooted at the shaft's start, so the origin is on the axis. Use
`addByCenterRadius(centerPoint, radius)`, then FIX the circle's centre by setting `isFixed` on its
`centerSketchPoint` and add `addDiameterDimension(entity, textPoint)` set to the bore diameter. A
circle's centre is free even when it is created at the origin, and coinciding it to the sketch
origin has been observed to fail the solve, so `isFixed` is the reliable pin (`[PB-CIRCLE-CENTER]`).

The bore diameter is this gear's Bore Diameter input when non-zero, and this gear's Pitch Diameter
over 4 otherwise. Gate the sketch on `isFullyConstrained` and raise
(`[BEVEL-F-FULL-CONSTRAINT]`).

The proof function is `stepBoreSketch`. It measures the resolved diameter on both sides of the auto
branch and that it stays inside the pitch diameter, and reports the unchecked Enable Bore case as
one with no sketch to gate.

<!-- proof-run: proofkit.Run(boreCases, stepBoreSketch) -->

**From:** `spec/bevelgear/instructions.md` L635–636

## S28 `[GO]` Bore through-cut, per gear

Extrude-cut the bore circle as a SYMMETRIC through-cut restricted to this gear's body:
`createInput(profile, operation)` on the Design component's `extrudeFeatures` with the cut
operation, `setSymmetricExtent(distance, isFullLength)` with `createByReal(2 * coneDistance)` and
false — so that distance is the half-length PER SIDE, generously past any face width — and the
input's `participantBodies` set to this Gear Body alone. Pass no taper argument
(`[PB-THROUGH-CUT]`). Then `add(input)`.

The proof function is `stepCutBore`. It cuts the bore cylinder from a cylinder standing in for the
gear body, and measures the bored volume against the analytic annulus and that the bore stays
inside the root radius. The stand-in is the same one S25 uses, for the same reason.

<!-- proof-run: proofkit3d.RunSolid(boreSolidCases, stepCutBore, assertCutBore) -->

**From:** `spec/bevelgear/instructions.md` L635–636

## S29 `[GO]` Meshing rotation, per gear

Do this HERE, in the Design component, before the body is moved out. A construction axis cannot be
added in the moved-out gear component, so the rotation has to use the profile edge's world geometry
while the body is still in Design.

Rotate the DRIVING body by 180 degrees over the Driving Gear Teeth Number — half a tooth pitch —
about its own shaft axis, with `rotate_body_about_edge(component, body, edge, angleRad)`, the
framework helper, which takes the rotation axis and origin from the B-to-I profile edge's WORLD
endpoints (`[PB-MOVE-ROTATE]`). Without the offset a driving tooth and a pinion tooth would both
sit at the axial-plane crossing and visually collide; with it a driving valley sits where the
pinion tooth crosses, which is the interlocked meshing look.

The pinion additionally gets `_pinionMeshPhase(pinionTeeth)`, which returns
`_PINION_MESH_PHASE_TEETH * 2 * pi / pinionTeeth` in radians. `_PINION_MESH_PHASE_TEETH` is 0 by
default and 0 for every straight bevel: the spiral build leaves the mid-face section unrotated, so
the pinion already meshes where the straight tooth did.

The proof function is `stepMeshRotate`. It measures that the rotation moves the body's azimuth by
exactly the phase and leaves its radius alone, and that the phase is half a tooth pitch for the
driving gear and zero for the pinion.

<!-- proof-run: proofkit3d.RunSolid(meshCases, stepMeshRotate, assertMeshRotate) -->

**From:** `spec/bevelgear/instructions.md` L637–639, `spec/bevelgear/instructions.md` L260–267

## S30 `[PROSE]` Move the finished bodies into the gear component

Relocate each finished gear body into its own `{gearLabel} Gear` component with
`moveToComponent(target)`, which preserves world position and needs no activation. Every feature
above ran in Design because Fusion rejects cross-sibling sketch and project calls even when the
target is activated or the entities are wrapped in assembly-context proxies
(`[PB-NO-CROSS-SIBLING]`); the visible end state is identical.

`[PROSE]`: relocating a body between components changes no geometry, and neither harness has
components for it to change nothing in.

**From:** `spec/bevelgear/instructions.md` L593–594, `spec/bevelgear/instructions.md` L579–592

## S31 `[PROSE]` Hide the construction geometry

Call `hide_construction_geometry(component)` on the Bevel Gear component. The framework helper
walks the component tree recursively, dedupes by `entityToken`, and sets `isLightBulbOn` to false
on every sketch, construction plane and construction axis. Construction planes and axes are NOT
hidden by `isVisible` — that is the property for sketches, and crossing the two leaves the planes
on screen (`[PB-TREE-CLEANUP]`, `[BEVEL-F-CLEANUP]`). There is no
sketch-only mode and no per-mode guard: bevel always builds solids. Leave only the two finished
gear bodies visible.

`[PROSE]`: visibility is a display property of entities neither harness has.

**From:** `spec/bevelgear/instructions.md` L641–644, `spec/bevelgear/fusion.md` L151–156
