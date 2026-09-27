# Screw Gear — Fusion API mechanics

Sidecar to `spec/screwgear/instructions.md`. Cross-gear conventions live in `PLAYBOOK.md` as
`[PB-…]`; this file holds only what is specific to the screw gear, under `[SCREW-F-…]` anchors that
the spec cites by name.

Every anchor below is stated from the API reference. **None of them has been through a Fusion load
yet** — this gear has never been generated. The first load's verdict belongs here, per the
"When Fusion gives a verdict" rule in `CLAUDE.md`.

## `[SCREW-F-SCREW-STEP]` — the screw step matrix

A screw step is a rotation about the gear's axis composed with a translation along that same axis.
Build it from two matrices rather than by setting a rotation matrix's translation component:

```python
rot = adsk.core.Matrix3D.create()
rot.setToRotation(k * pitch / lam, axisVector, axisPoint)   # axisVector unit, axisPoint on the axis
shift = axisVector.copy()
shift.scaleBy(k * pitch)
mov = adsk.core.Matrix3D.create()
mov.translation = shift
rot.transformBy(mov)
```

Build the translation vector first and assign it whole. `Matrix3D.translation` is a read/write
`Vector3D` property, and the API reference does not say whether its getter hands back the
matrix's own vector or a copy; `mov.translation.scaleBy(...)` scales whatever the getter returned
and, if that is a copy, leaves the matrix at zero translation with nothing raised. Scaling a
vector of the build's own and then assigning it is correct either way, and it is how every
vector in this repository's shipped modules is scaled (`alongVec.scaleBy(along)` in
`bevelgear.py`).

⚠️ **Do NOT build the rotation and then assign `matrix.translation`.** `setToRotation(angle, axis,
origin)` already writes a translation component — the part that carries the rotation off the world
origin and onto `origin` — and assigning `translation` overwrites it, silently turning a rotation
about the gear's axis into a rotation about a parallel line through the world origin. The body then
lands somewhere else entirely and nothing raises.

The two commute here because the translation runs along the rotation axis, so `rot.transformBy(mov)`
and `mov.transformBy(rot)` give the same matrix. That is a property of this particular step, not of
`Matrix3D`; do not carry the freedom to another gear.

Apply it with `moveFeatures.createInput2(bodies)` → `defineAsFreeMove(matrix)` → `add(input)`
(`[PB-MOVE-ROTATE]`).

## `[SCREW-F-COPY-BODY]` — taking the copy

`component.features.copyPasteBodies.add(sourceBody)` returns a **`CopyPasteBody` feature**, not a
body. The new body is `copyPasteFeature.bodies[0]` — `CopyPasteBody` derives from `Feature`, whose
`bodies` collection holds what the feature created. `CopyPasteBody`'s own `sourceBody` property
returns the *original*, so a build that reads `sourceBody` gets the body it already had and then
screw-moves the original instead of the copy. The symptom is a ribbon that walks away from its axis
one tooth at a time while never growing past one cell.

## `[SCREW-F-CELL-LOFT]` — lofting the rotated rectangles

The tooth cell lofts rectangles, eleven at the defaults, each rotated a little further about the
axis than the last and each a slightly different width. Add them with `loftSections.add(profile)`
**in station order** (`[PB-LOFT]`); the order of the calls is the loft order. The loft is
`loftFeatures.createInput(NewBodyFeatureOperation)` → the sections → `loftFeatures.add(input)`,
and the cell is the one body in the feature's `bodies`, which the build checks for
(`[PB-EMPTY-RESULT]`).

Select each section's profile by curve count (`utilities.find_profile_by_curve_counts(sketch,
lines=4)`) — each section sketch holds exactly one rectangle, so the count is unambiguous and there
is no need for a positional pick.

The rectangles turn by 1.91° between neighbours at the default proportions. Fusion pairs the
sections' vertices by proximity, and that pairing is what stays correct as long as the step stays
well under a quarter turn. A spec change that cuts the section count to three would put 9.5° between
neighbours, which still pairs, but the failure when it does not is a lofted body with a twisted
crease rather than an error, so the count stays where the spec pins it.

**The loft is smooth between its sections, not ruled.** `LoftFeatureInput` has no ruled option:
its properties are the operation, the sections, `isSolid`, `isClosed`, `isTangentEdgesMerged`,
the two end-edge alignments and the centre line or rails, and a `LoftSection` carries only an
end condition (free, tangent, smooth, direction, point-sharp, point-tangent), which applies at
the loft's two ends. A loft through more than two sections passes through every section and is
fitted smoothly between them. Only a loft through exactly two sections is ruled, which is the
case the helical and bevel gears build and have loaded (`spec/helicalgear/contract.json`
records their ruled walls). The spec's chord arithmetic (§2, "What the loft is") is about the
ruled loft; the cell and the bores are single lofts through all their sections, so it bounds a
body other than the one built, and the spec says so.

## `[SCREW-F-TWISTED-SLOT]` — a collar and its bore

Cut the opening with a lofted clearance ribbon, not a swept one. `SweepFeatureInput.twistAngle` (with
`solidTwistAxis`) is the natural tool and would build the exact helicoid from one section, but the
sweep needs an `adsk.fusion.Path` and `Path.create` on a sketch curve raises `RuntimeError …
InternalValidationError : Utils::getObjectPath(sketchCurve, …)` when the owning sketch is not
trivially resolvable in the current multi-component context. The screw gear builds everything in a
`Design` sub-component, so it is always in that context.

The collar is built the same way, first, on construction planes of its own. On each of them draw
the bore's rectangle grown by the wall — a rounded rectangle, four lines and four arcs of
`collarWall` radius, which `find_profile_by_curve_counts(sketch, lines=4, arcs=4)` picks out — and
loft those into the collar's solid; then loft the plain rectangles on the bore's own planes into a
second body and cut the frame with it. Both lofts twist, because the ribbon turns while it is
inside the collar. A collar's loft spans `±collarHalf` and the bore's spans a millimetre more each
end, so the cut runs clean through the collar's flat ends and through whatever of a rod stands
inside the wall; the two plane sets are separate because one evenly spaced set cannot land on both
the collar's ends and the bore's. Each set is `setByDistanceOnPath` on the gear's axis line
(`[PB-CONSTRUCTION-PLANES]`), evenly spaced over its own span, at the count the spec derives for
it (10 and 15 at the defaults).

The collar section's rounded rectangle is drawn over the bore's rectangle as construction: the
bore's four sides `L1..L4` by the spec's rectangle scheme, all construction, then four solid sides
`O1..O4`, `Oi` parallel to `Li` with an offset dimension of `collarWall` and each of its ends
coincident on the infinite line of the neighbouring construction side (`addCoincident(point,
line)`), and four three-point arcs, each from the end of `Oi` to the start of `O(i+1)` through the
construction corner pushed out by `collarWall` along the corner's diagonal, with one `addTangent`
to `Oi`. The arc's centre then falls on the construction corner and its radius is `collarWall`;
neither is dimensioned, and a second tangent would over-constrain (`[PB-NO-OVERCONSTRAIN]`).

One loft per bore, not one per gear. The two collars of a gear do stand on the same axis, so a single
clearance ribbon through both is the obvious build, but it would have to span the 24 mm between
their far ends and turn 262° on the way, and a loft's accuracy is set by the angle between
neighbouring sections. Lofting only a collar's own length with its margin is 6 mm and 65°, and needs
under a third of the sections for a better channel. `TestBoreLoftKeepsItsClearance` in the proof
measures what is left.

`twistAngle` is also ignored outright when a guide rail or guide surface is set, per the API
reference — worth knowing before anyone reaches for a rail to shape the teeth instead.

## `[SCREW-F-ROUND-FRAME]` — the ring, the rods and the loop

Every other part of the frame is a round section, and none of them twists. No construction axis
is made anywhere in this build, since one needs an active component
(`[PB-CONSTRUCTION-NEEDS-ACTIVE]`); every revolve axis is a sketch line.

Every sketch here is built from the reference points of `[SCREW-F-REFERENCES]` and nothing
free: lines between reference points and circles centred on them. Sketch-local frames are
never used for a position; every point is a world point of the spec's §1 frame, computed on the
sketch's plane and mapped in with `modelToSketchSpace`.

- The ring is a `revolveFeatures` full revolution (`[PB-REVOLVE]`: `createInput(profile, axis,
  NewBodyFeatureOperation)` → `setAngleExtent(False, ValueInput.createByString('360 deg'))` →
  `add`) of a circle sketched on the **Ring Plane**, the plane through the Anchor Line square to the
  selected plane (`setByAngle(anchorLine, '90 deg', targetPlane)`), which therefore holds `C`, `ê`
  and `n̂`, in a sketch named **Ring**. The revolve axis `An` is a construction line between two
  reference points, `C` and `C + cageRise*n̂`, sharing both, and both are set `isFixed` once the
  line is drawn; the revolve uses the whole line the segment lies on. The wire is a circle
  `addByCenterRadius` at `C + cageRise*n̂ + ringRadius*ê` with radius `ringWire/2`, its
  `centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`: the centre is created free, and
  `isFixed` is the pin Fusion accepts) and a diameter dimension of `ringWire` with its text
  point on the circle (`[PB-RADIAL-DIM]`). The circle never reaches the axis, since
  `ringRadius > ringWire/2` follows from the rod checks. **Its profile** is the one closed loop
  in the sketch: raise unless `sketch.profiles.count == 1`, then `sketch.profiles.item(0)`
  (`[PB-SINGLE-PROFILE]`). `find_profile_by_curve_counts` cannot pick a circle: it counts lines,
  arcs and NURBS curves and treats a `Circle3DCurveType` curve as a type that disqualifies the
  loop, so it is used only for the loops of lines and arcs below.
- A rod is an `extrudeFeatures` extrusion of a circle sketched on the selected plane itself,
  `setSymmetricExtent(ValueInput.createByReal(cageRise), False)` — `False` makes the value each
  side's length (`[PB-THROUGH-CUT]` for the argument's meaning), so the rod runs from `-cageRise` to
  `+cageRise`. Sketch all four on one sketch named **Rods**, in the collar order of the spec's
  rod search: each is `addByCenterRadius` at its foot
  `C + ringRadius*(cos(psi)*ê + sin(psi)*k̂)`, `k̂ = n̂ × ê`, radius `rodDiameter/2`, its
  `centerSketchPoint.isFixed = True` and a diameter dimension of `rodDiameter`. Nothing else is
  in the sketch. Each circle is its own profile and the four never overlap, since the rods stand
  at least `rodDiameter + clearance` apart: raise unless `sketch.profiles.count == 4`, put all
  four `sketch.profiles.item(i)` into one `ObjectCollection`, and extrude them in one feature as
  new bodies, raising unless the feature's `bodies.count` is 4. The order of the four profiles
  does not matter, since every rod gets the same extent.
- A bar of the loop is the same extrusion between two rod feet: a solid line from foot to foot
  on a **Loop** sketch on the **Loop Plane**, offset `-cageRise` from the selected plane. The four
  feet are reference points at `C - cageRise*n̂ + ringRadius*(cos(psi)*ê + sin(psi)*k̂)`, in
  the spec's foot order (foot 0 the smallest azimuth in `[0°, 360°)` from `ê` about `+n̂`, the
  rest increasing); bar `i` is `addByTwoPoints(foot_i, foot_((i+1) mod 4))` sharing both, and
  after the four bars are drawn all four feet are set `isFixed`. The sketch holds nothing free
  and no dimension. Then a plane square to bar `i` at its middle (`setByDistanceOnPath(bar_i,
  0.5)`), named **Loop Bar `i` Plane**; on it a sketch **Loop Bar `i`** with a circle
  `addByCenterRadius` at the bar's middle `(foot_i + foot_((i+1) mod 4))/2`, which is where the
  bar pierces that plane, radius `ringWire/2`, its `centerSketchPoint.isFixed = True` and a
  diameter dimension of `ringWire`; its profile is the sketch's one loop, `profiles.count == 1`
  then `item(0)`, as the ring's. It is extruded `setSymmetricExtent` by half the bar's `length`.
  It is not a sweep: a sweep needs a `Path`, and `Path.create` on a sketch curve raises in this
  multi-component build (`[SCREW-F-TWISTED-SLOT]`). The ball at foot `i` is a half-disc revolved
  about a line through the foot along `n̂`, on the plane through bar `i` — the bar that starts at
  that foot — and `n̂` (`setByAngle(bar_i, '90 deg', loopPlane)`), named **Loop Ball `i`
  Plane**, in a sketch **Loop Ball `i`**: the solid line `Bl` runs between two reference points,
  `foot_i - (ringWire/2)*n̂` (start) and `foot_i + (ringWire/2)*n̂` (end), sharing both; then a
  three-point arc `addByThreePoints(Bl.startSketchPoint, through, Bl.endSketchPoint)` with
  `through` the point `ringWire/2` from the foot on the side away from the bar,
  `foot_i - (ringWire/2)*unit(foot_((i+1) mod 4) - foot_i)`, which lies on the plane; then both
  reference points are set `isFixed`, and `addCoincident(arc.centerSketchPoint, Bl)` puts the
  arc's centre on the chord, where the half-disc's centre is. With its two ends fixed the arc has
  one freedom, its bulge, and the coincident takes it; the side of the bulge is the seed's. The
  profile is the half-disc, one line and one arc, `find_profile_by_curve_counts(sketch, lines=1,
  arcs=1)`, and `Bl` is the revolve axis, which a profile may lie against.

Join every piece into one body with a `combineFeatures` join as it is made, collars included, and
cut the bores last: the bore's loft then passes through the collar and through the part of its rod
that stands inside the wall in one operation. A join or cut is `combineFeatures.createInput(target,
tools)` with the tools in an `ObjectCollection`, `operation` set to `JoinFeatureOperation` or
`CutFeatureOperation`, `isKeepToolBodies = False`, then `add`. **Each leaves one body**, which
means the combine feature's `bodies.count` — `Feature.bodies`, the bodies the feature created or
modified — is exactly 1; the build raises with the piece's name and the count when it is not
(`[PB-EMPTY-RESULT]`, `[PB-SELF-DIAGNOSING]`), and `bodies.item(0)` is the target from then on.
The same count gates the ribbon joins of §3. The pieces are always made as new bodies and
combined explicitly, because a join operation on the extrude or loft itself would join into
whatever it touches, the ribbons included. The rods' azimuths are the angles the proof's
`TestRodsStandBesideTheirCollars` derives, and the build recomputes them by the same search rather
than reading them from a table, because they move with every ribbon dimension.

## `[SCREW-F-REFERENCES]` — how fixed geometry enters a sketch

`[PB-PROJECT-NOT-FIXED]` is the rule: `sketch.project(...)` brings geometry in with free degrees
of freedom, so a sketch whose curves hang off shared projected points reports under-constrained,
and every sketch of this build is gated on `isFullyConstrained`. This gear therefore projects
**once**, in the Anchor sketch, and nowhere else, and never calls `intersectWithSketchPlane`.

**The Anchor sketch** projects the user's selected point, `sketch.project(point).item(0)`, and
binds the Anchor Line to it by `addCoincident` and `addMidPoint` together, plus `addHorizontal`
and a horizontal distance between the line's ends. That is the bevel gear's Anchor sketch, which
Fusion reports fully constrained (`PLAYBOOK.md` `[PB-SETTLE-DISPLAY]` records the measurement),
and it is the one place a projection is anchored by constraints rather than replaced. The
compiled API reference declares only `project2(entities, isLinked)`, so the repo's gates report
`project` as unverified; this gear keeps `project` anyway, because every add-in that has loaded
in Fusion (spur, bevel, cycloidal) calls it and none calls `project2`. After the sketch's gate
passes, `C` is the projected point's `worldGeometry` and `ê` runs from the line's
`startSketchPoint.worldGeometry` to its `endSketchPoint.worldGeometry`
(`[PB-WORLDGEO-CONSTRAINED]`); `n̂` is read from the Gear A Axis Plane (`[SCREW-F-NORMAL-SIGN]`).

**Every other sketch** takes its references as **reference points**, by the recreate-share-fix
recipe of `[PB-PROJECT-NOT-FIXED]` (b), which is how the bevel gear builds each per-gear profile
sketch and the shaft axis its revolve, pattern and section planes stand on:

1. Compute the point in world space from `C`, `ê`, `n̂` and the §1 frame, **on the sketch's own
   plane** — the axis point of a section at `origin_g + s*dir_g`, a rod's foot on the ring's
   circle, a bar's end on the Loop Plane.
2. `local = sketch.modelToSketchSpace(worldPoint)`, then `local.z = 0`, then
   `pt = sketch.sketchPoints.add(local)` (`[PB-SKETCH-ZERO-Z]`). Being on the plane in world
   arithmetic does not put the point at zero height in the sketch: the first Fusion load
   (2026-09-27) put every point of `Gear A Collar -R Section 0` at `z = 4.4e-6` cm, because
   Fusion's `setByDistanceOnPath` plane sat that far from the station the build computed, and
   the sketch read not fully constrained with every dimension present. The same zeroing applies
   to every other point this build maps into a sketch: a raw seed passed to `addByTwoPoints`,
   an arc's through point, a circle's centre, a dimension's text point.
3. Draw every curve that uses it **sharing** `pt`: `addByTwoPoints(pt, ...)`,
   `addByThreePoints(pt, ...)` (`[PB-SHARE-XOR-COINCIDENT]`: shared, so no coincident to it).
4. After the last such curve exists, and before any constraint or dimension is added, set
   `pt.isFixed = True`. The order is the playbook's: a bare point fixed before a curve consumes it
   does not leave the sketch fully constrained.

A line between two reference points is fully constrained by the two fixed ends and takes no
dimension. A circle centred on a reference position is not built on a reference point: it is
`addByCenterRadius` at that position and its own `centerSketchPoint` is set `isFixed = True`
(`[PB-CIRCLE-CENTER]`, measured in Fusion on a `setByDistanceOnPath` plane, which is where the
Loop Bar circles sit), with a diameter dimension; a coincident from a circle's centre to a fixed
point is not used, since the playbook records a solve failure for the coincident form.

The world points are the numbers the source geometry was dimensioned to, so they are the solved
positions (`[PB-SOLVED-GEOMETRY]`): the Anchor sketch is the only sketch whose geometry a later
sketch depends on, and its `worldGeometry` is read once, after its gate, into `C` and `ê`. No
sketch reads another sketch's points after that.

**What the proof does with this.** The sketch engine's `Fix` is the analogue of `isFixed`
(`[PB-SKETCH-FIRST]`, constraint mapping), so the compiled proof fixes each reference point and
proves the rest of the scheme against it; for the Anchor sketch it writes the midpoint alone,
because the engine's midpoint already carries the coincident row, as `proof/bevelgear` records.
Whether Fusion's `isFullyConstrained` agrees is what the first load will say, and it goes here.

## `[SCREW-F-NORMAL-SIGN]` — reading `n̂` from a plane the build made

`setByOffset(targetPlane, ValueInput.createByReal(d))` offsets along the selected entity's own
normal, which for a planar face may point either way. The build never assumes that sign. After the
`Gear A Axis Plane` is made at `-A/2`, it reads `plane.geometry` — a `Plane` with an `origin` and a
`normal` — and sets `n̂ = normal` if `(C - origin) · normal > 0`, else `-normal`, so that `C` lies
`+A/2` along `n̂` from gear A's plane by construction; it then checks that `|(C - origin) · normal|`
is `A/2` for both axis planes and raises otherwise. Every later offset from the selected plane —
the Loop Plane at `-cageRise` — is signed the same way, so it lands on `-n̂` with gear A's plane.

## `[SCREW-F-NO-SOLID-TWIST]` — why the ribbon is not built straight and then twisted

The straightforward mental model is to build the toothed rack flat, with its sinusoidal edge as one
spline in one sketch and one extrude, and then twist the solid. Fusion's parametric environment has
no twist feature for a solid body. `SweepFeatureInput` does document a solid twist axis, but a solid
sweep sweeps the body's *volume* along the path, which smears each tooth into a continuous ridge and
destroys the very feature being built.

That is why the build lofts one tooth cell and repeats it by a screw step. It is the same shape as
the involute gears' "draw one tooth, pattern it", with the circular pattern replaced by a
transformation Fusion has no pattern feature for: a rectangular pattern only translates, a circular
pattern only rotates, and no nesting of the two produces the diagonal a screw step needs.
