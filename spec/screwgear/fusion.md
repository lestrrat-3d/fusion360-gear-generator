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

The tooth cell lofts nine rectangles, each rotated a little further about the axis than the last and
each a slightly different width. Add them with `loftSections.add(profile)` **in station order**
(`[PB-LOFT]`); the order of the calls is the loft order.

Select each section's profile by curve count (`utilities.find_profile_by_curve_counts(sketch,
lines=4)`) — each section sketch holds exactly one rectangle, so the count is unambiguous and there
is no need for a positional pick.

The rectangles turn by 1.97° between neighbours at the default proportions. Fusion pairs the
sections' vertices by proximity, and that pairing is what stays correct as long as the step stays
well under a quarter turn. A spec change that cuts the section count to three would put 4.7° between
neighbours, which still pairs, but the failure when it does not is a lofted body with a twisted
crease rather than an error, so the count stays where the spec pins it.

## `[SCREW-F-TWISTED-SLOT]` — the collar's opening

Cut the opening with a lofted clearance ribbon, not a swept one. `SweepFeatureInput.twistAngle` (with
`solidTwistAxis`) is the natural tool and would build the exact helicoid from one section, but the
sweep needs an `adsk.fusion.Path` and `Path.create` on a sketch curve raises `RuntimeError …
InternalValidationError : Utils::getObjectPath(sketchCurve, …)` when the owning sketch is not
trivially resolvable in the current multi-component context. The screw gear builds everything in a
`Design` sub-component, so it is always in that context.

The same loft cuts both collars' openings, one per gear, and the opening is a twisted channel
because the ribbon turns while it is inside the collar.

`twistAngle` is also ignored outright when a guide rail or guide surface is set, per the API
reference — worth knowing before anyone reaches for a rail to shape the teeth instead.

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
