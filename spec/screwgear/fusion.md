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
mov = adsk.core.Matrix3D.create()
mov.translation = axisVector.copy()
mov.translation.scaleBy(k * pitch)
rot.transformBy(mov)
```

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

- The ring is a `revolveFeatures` full revolution (`[PB-REVOLVE]`: `createInput(profile, axis,
  NewBodyFeatureOperation)` → `setAngleExtent(False, ValueInput.createByString('360 deg'))` →
  `add`) of a circle sketched on the **Ring Plane**, the plane through the Anchor Line square to the
  selected plane (`setByAngle(anchorLine, '90 deg', targetPlane)`). In that sketch a construction
  line from the projected centre point along `n̂`, perpendicular to the projected Anchor Line with a
  length dimension of `cageRise`, is the revolve axis; a construction spoke from its end,
  perpendicular to it with a length dimension of `ringRadius`, carries the circle at its end, with
  a diameter dimension of `ringWire`. The circle never reaches the axis, since `ringRadius >
  ringWire/2` follows from the rod checks.
- A rod is an `extrudeFeatures` extrusion of a circle sketched on the selected plane itself,
  `setSymmetricExtent(ValueInput.createByReal(cageRise), False)` — `False` makes the value each
  side's length (`[PB-THROUGH-CUT]` for the argument's meaning), so the rod runs from `-cageRise` to
  `+cageRise`. Sketch all four on one sketch; each circle is its own profile, and the four are
  extruded in one feature as new bodies, whose count the build checks.
- A bar of the loop is the same extrusion between two rod feet: a line from foot to foot on a
  **Loop** sketch on the plane offset `-cageRise` from the selected plane, each foot the projected
  centre of a rod's circle; a plane square to that line at its middle (`setByDistanceOnPath(bar,
  0.5)`); a circle of `ringWire` on that plane centred on the line's intersection with it
  (`intersectWithSketchPlane`), extruded `setSymmetricExtent` by half the line's `length`. It is not
  a sweep: a sweep needs a `Path`, and `Path.create` on a sketch curve raises in this
  multi-component build (`[SCREW-F-TWISTED-SLOT]`). A ball at each foot is a half-disc revolved
  about a line through the foot along `n̂`: on a plane through the bar and `n̂`
  (`setByAngle(bar, '90 deg', loopPlane)`), a line of `ringWire` through the foot at its midpoint,
  perpendicular to the projected bar, and a three-point arc from its one end to the other on the
  side away from the bar, its centre coincident on the line; the profile is the half-disc, and the
  line is the revolve axis, which a profile may lie against.

Join every piece into one body with a `combineFeatures` join as it is made, collars included, and
cut the bores last: the bore's loft then passes through the collar and through the part of its rod
that stands inside the wall in one operation. A join or cut is `combineFeatures.createInput(target,
tools)` with the tools in an `ObjectCollection`, `operation` set to `JoinFeatureOperation` or
`CutFeatureOperation`, `isKeepToolBodies = False`, then `add`; each leaves one body, and the build
raises with the piece's name when it does not (`[PB-EMPTY-RESULT]`). The pieces are always made as
new bodies and combined explicitly, because a join operation on the extrude or loft itself would
join into whatever it touches, the ribbons included. The rods' azimuths are the angles the proof's
`TestRodsStandBesideTheirCollars` derives, and the build recomputes them by the same search rather
than reading them from a table, because they move with every ribbon dimension.

## `[SCREW-F-REFERENCES]` — how fixed geometry enters a sketch

A point or line of the anchor sketch enters any other sketch as
`sketch.project(entity).item(0)`, as `[PB-PROJECT-NOT-FIXED]` writes it. The compiled API
reference declares only `project2(entities, isLinked)`, so the repo's gates report `project` as
unverified; this gear keeps `project` anyway, because every add-in that has loaded in Fusion
(spur, bevel, cycloidal) calls it and none calls `project2`. A projected line is set
`isConstruction = True` wherever it must not bound a profile.

A gear's axis line enters a section sketch, whose plane is normal to it, as
`sketch.intersectWithSketchPlane([axisLine])[0]`, a sketch point at the axis; the build raises when
the list is empty (`[PB-EMPTY-RESULT]`). The same call gives a bar's middle on its section plane.

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
