# Screw Gear — Fusion API mechanics

Sidecar to `spec/screwgear/instructions.md`. Cross-gear conventions live in `PLAYBOOK.md` as
`[PB-…]`; this file holds only what is specific to the screw gear, under `[SCREW-F-…]` anchors that
the spec cites by name.

Every anchor below is stated from the API reference and from what Fusion did when asked. The
loads and diagnostics that asked are recorded under `[SCREW-F-FIRST-LOAD]` and
`[SCREW-F-DIAGNOSTIC]`, per the "When Fusion gives a verdict" rule in `CLAUDE.md`; a sentence that
gives a date is a measurement, and one that gives none is read from the reference.

## `[SCREW-F-FIRST-LOAD]` — what the Fusion loads said

- **2026-09-27.** The first load built both gears and failed at `Gear A Collar -R Section 0`,
  the first collar section, whose points all sat at `z = 4.4e-6` cm in sketch space and read
  under-constrained. The fix is `[PB-SKETCH-ZERO-Z]`.
- **2026-09-28.** The second load completed and its geometry looked right: the ribbons, the
  frame, the collars and the bores as `proof/screwgear/README.md` draws them. It took about
  five minutes and made 136 sketches, 134 construction planes and 79 features, one sketch and
  one plane per loft section — 100 of each for the four collars and four bores alone, lofted
  through 10 and 15 sections, and 22 of each for the two tooth cells. That is the build this
  file's earlier text described, and it is the reason every construction below changed: a
  collar and a bore are now one sweep each, the tooth cell's sections share one sketch, and the
  loop's bars and balls stand on one plane. The spec's "What the build makes" states the counts
  the new construction is expected to reach, 23 sketches, 12 planes and 53 features at the
  defaults.
- **2026-09-28, third load.** The first load of the new construction (sweeps, one 3D-sketch cell
  loft in 4-tooth cells, the loop on its own plane) completed at the defaults in seconds, against
  about five minutes for the second, and no runtime check of `[SCREW-F-SWEEP-CHECK]` raised.
  Counted in the finished document: 23 sketches and 12 construction planes, as "What the build
  makes" predicts; 62 features (59 in `Design`, one each in `Gear A`, `Gear B` and `Cage`)
  against the 53 predicted; 96 timeline items; three bodies. The nine extra features are not yet
  accounted for, and the prediction is what is wrong, not the build. A side-by-side look against
  the second load's geometry was not reported.

## `[SCREW-F-DIAGNOSTIC]` — what the diagnostics of 2026-09-28 measured

Two scripts ran in the user's Fusion on 2026-09-28, each in a scratch document holding the
add-in's own component tree (root → `Screw Gearing` → `Design`, `Gear A`) under a plane tilted
30° off XY, never activating a component, at the spec's defaults. Their outputs are the numbers
quoted through this file. The cross-gear facts they settled are playbook anchors; the numbers
specific to this gear are:

- **The frame.** The Anchor recipe read fully constrained, and `C` sat 0.482000 cm from the
  Gear A Axis Plane, which is `A/2`.
- **The Paths sketch.** Three overlapping collinear lines between six fixed points on the axis
  plane read fully constrained. `Path.create` raised for the line native, as a proxy in the
  `Design` occurrence, and in an `ObjectCollection`; `features.createPath(line, False)` returned
  a path of one curve, open, and `features.createPath(collection, False)` the same
  (`[PB-PATH-FROM-SKETCH]`).
- **The collar sweep.** `setByDistanceOnPath(collarLine, 0)` put the plane's origin 0.0000 mm
  from station `sc - collarHalf`. A rounded-rectangle section drawn from eight fixed points with
  four tangent arcs took 0.10 s and read fully constrained with one profile. The sweep with
  `twistAngle = +43.64°` took 0.04 s and gave a solid of ten faces and sixteen vertices, volume
  0.40091 cm³ against 0.40091 computed, its far-end vertices 0.0047 mm from the section turned
  by the twist and its near-end vertices 0.0000 mm; with `-43.64°` the far end missed by
  4.8979 mm. At stations a quarter, a half and three quarters of the way along, probes 0.05 mm
  inside and outside the outer wall, the back wall, a face wall and a corner diagonal all fell
  on the right side, so the twist is linear along the path (`[PB-SWEEP-TWIST]`).
- **The bore cut.** With the collar joined to its rod and a thin stand-in body run through the
  channel, the bore sketch by the rectangle scheme read fully constrained, and the sweep cut
  with `participantBodies = [cage]` took 0.05 s, left the stand-in's volume unchanged to six
  decimals, and took 0.13144 cm³ from the cage, which is the channel through the collar plus
  the rod's intrusion.
- **The ribbon as temporary BRep.** Six ruled sheets built in 0.09 s, a base feature in the
  non-activated `Gear A` component took them (`startEdit` true, active component unchanged
  afterwards), and the stitch raised `TOOLBODY_CREATION_FAIL_ERROR`; the next
  `TemporaryBRepManager.get()` raised the same. That construction is out
  (`[PB-TEMP-BREP-STITCH]`).
- **The Cell Sections sketch and loft.** One sketch on the Gear A Axis Plane holding every
  section of a cell as fixed points with their `z` kept (|z| from 3.2 to 5.2 mm) and four lines
  each: at one tooth, 11 sections, 0.88 s undeferred and 0.09 s with `isComputeDeferred`, fully
  constrained, 11 profiles; the loft through `features.createPath` sections took 0.07 s, solid,
  six faces, 0.041022 cm³ against the ruled loft's 0.041117 (−0.233%). At four teeth, 41
  sections, deferred: 0.37 s, fully constrained, 41 profiles; loft 0.23 s, 0.164158 cm³ against
  0.164470 (−0.190%). At eight teeth, 81 sections: 1.12 s and 0.59 s, −0.064%. At every size,
  probes 0.04 mm inside and outside the toothed edge and a face, at the midpoint between every
  pair of sections, all fell on the right side of the built surface: 40 of 40, 160 of 160,
  320 of 320. `Path.create(collection)` raised as above (`[PB-3D-SKETCH-SECTIONS]`,
  `[PB-SKETCH-DEFER]`). The four-tooth cell is what `cellTeeth = 4` rests on.
- **The loop.** The Loop Plane sat −1.350 cm along `n̂` from `C`. The four-bar Loop sketch read
  fully constrained, and `features.createPath(bar0, True)` chained it into a closed path of four
  curves. A pipe along it was solid with the exact volume of four straight bars, but a probe
  1.1 wire radii out from a corner along its outward bisector was inside the pipe: the mitred
  corner reaches past a ball of the wire's radius, so the pipe is not used
  (`[PB-PIPE-CORNER]`). A bar revolved from a rectangle on the Loop Plane and a ball revolved
  from a half-disc on it were both solid at their exact volumes, 0.08472 and 0.00818 cm³.
- **The rectangle scheme's cost.** The collar section by the spec's rectangle scheme, about 45
  solver-visible calls, took 0.45 s, and 0.23 s with computing deferred, fully constrained with
  one profile either way.
- **A two-section loft does not twist.** A loft between the collar span's two end rectangles,
  with and without `centerLineOrRails.addCenterLine(collarLine)`, put mid-station probes that
  should be inside the helicoid outside it, in both forms alike
  (`[PB-LOFT-TWO-SECTIONS-STRAIGHT]`).

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

`k` is a number of teeth: `m*cellTeeth` for a body of `m` cells (spec §3). Build the translation
vector first and assign it whole. `Matrix3D.translation` is a read/write `Vector3D` property,
and the API reference does not say whether its getter hands back the matrix's own vector or a
copy; `mov.translation.scaleBy(...)` scales whatever the getter returned and, if that is a copy,
leaves the matrix at zero translation with nothing raised. Scaling a vector of the build's own
and then assigning it is correct either way, and it is how every vector in this repository's
shipped modules is scaled (`alongVec.scaleBy(along)` in `bevelgear.py`).

⚠️ **Do NOT build the rotation and then assign `matrix.translation`.** `setToRotation(angle, axis,
origin)` already writes a translation component — the part that carries the rotation off the world
origin and onto `origin` — and assigning `translation` overwrites it, silently turning a rotation
about the gear's axis into a rotation about a parallel line through the world origin. The body then
lands somewhere else entirely and nothing raises.

The two commute here because the translation runs along the rotation axis, so `rot.transformBy(mov)`
and `mov.transformBy(rot)` give the same matrix. That is a property of this particular step, not of
`Matrix3D`; do not carry the freedom to another gear.

Apply it with `moveFeatures.createInput2(bodies)` → `defineAsFreeMove(matrix)` → `add(input)`
(`[PB-MOVE-ROTATE]`). The second Fusion load ran seven such rounds per gear and the ribbons
came out whole.

## `[SCREW-F-COPY-BODY]` — taking the copy

`component.features.copyPasteBodies.add(sourceBody)` returns a **`CopyPasteBody` feature**, not a
body. The new body is `copyPasteFeature.bodies[0]` — `CopyPasteBody` derives from `Feature`, whose
`bodies` collection holds what the feature created. `CopyPasteBody`'s own `sourceBody` property
returns the *original*, so a build that reads `sourceBody` gets the body it already had and then
screw-moves the original instead of the copy. The symptom is a ribbon that walks away from its axis
one tooth at a time while never growing past one cell.

## `[SCREW-F-CELL-LOFT]` — one sketch of every section, one loft

The tooth cell is `cellTeeth` teeth of ribbon, 4 at the defaults, lofted through `c*n + 1`
rotated rectangles — 41 at the defaults, each rotated 1.91° further about the axis than the last
and each a slightly different width. All of them are drawn in **one sketch**, on the gear's Axis
Plane, and the loft is fed a path per section.

**The sketch.** Every corner is `sketch.sketchPoints.add(sketch.modelToSketchSpace(world))` with
the mapped point's `z` **kept**, which is the one place this build departs from
`[PB-SKETCH-ZERO-Z]`: those corners are meant to lie off the plane, at every height above and
below it, and the rule zeroes only a point that is meant to lie on it. The four lines of a
section are `sketchLines.addByTwoPoints(corner, nextCorner)` sharing the points
(`[PB-SHARE-XOR-COINCIDENT]`), and every point is set `isFixed = True` after the last line of
the last section exists (`[PB-PROJECT-NOT-FIXED]` (b): fix after use). Draw it with
`isComputeDeferred = True` from just after `sketches.add` until just after the last `isFixed`,
then set it `False` (`[SCREW-F-DEFER]`). Fusion reads such a sketch fully constrained and finds
one profile per section, each planar in its own station's plane — measured at 11, 41 and 81
sections (`[SCREW-F-DIAGNOSTIC]`, `[PB-3D-SKETCH-SECTIONS]`). The build raises unless
`isFullyConstrained` and unless `profiles.count == c*n + 1`; the profiles are not used, because
nothing says which profile is which station, and the sections are made from the lines.

**The sections.** For section `k`, put its four lines in an `adsk.core.ObjectCollection` and
call `component.features.createPath(collection, False)` on the `Design` component, which owns
the sketch (`[PB-PATH-FROM-SKETCH]`). `adsk.fusion.Path.create(collection, noChainedCurves)`
raises `InternalValidationError` here, measured; do not fall back to it. Add the paths with
`loftSections.add(path)` **in station order** (`[PB-LOFT]`); the order of the calls is the loft
order. The loft is `loftFeatures.createInput(NewBodyFeatureOperation)` → the sections →
`loftFeatures.add(input)`, and the cell is the one body in the feature's `bodies`, which the
build checks for along with `isSolid` (`[PB-EMPTY-RESULT]`). The loft took 0.23 s at 41
sections.

**The loft is smooth between its sections, not ruled.** `LoftFeatureInput` has no ruled option:
its properties are the operation, the sections, `isSolid`, `isClosed`, `isTangentEdgesMerged`,
the two end-edge alignments and the centre line or rails, and a `LoftSection` carries only an
end condition, which applies at the loft's two ends. A loft through more than two sections
passes through every section and is fitted smoothly between them; only a loft through exactly
two is ruled, and a two-section loft cannot carry a twist at all
(`[PB-LOFT-TWO-SECTIONS-STRAIGHT]`). The spec's chord arithmetic (§2, "What the loft is") is about
the ruled loft, and the built surface was measured against the helicoid on 2026-09-28: within
0.04 mm at every midpoint between sections, at one, four and eight teeth, with the built volume
0.06% to 0.23% under the ruled loft's (`[SCREW-F-DIAGNOSTIC]`).

Fusion pairs the sections' vertices by proximity, and that pairing is what stays correct as long
as the step stays well under a quarter turn. A spec change that cut the steps per tooth to three
would put 6.4° between neighbours, which still pairs, but the failure when it does not is a
lofted body with a twisted crease rather than an error, so the count stays where the spec
derives it.

**Why the cell is four teeth and not the whole ribbon.** The sketch and the loft both grow with
the section count — 0.37 s and 0.23 s at 41 sections, 1.12 s and 0.59 s at 81 — while a
doubling round is three features whatever the body holds. Four teeth cuts the rounds from seven
to five at 80 teeth; the whole ribbon in one loft would be 801 sections and no rounds, and was
not measured. `cellTeeth` is the module constant that decides it, and 1 is the measured
fallback.

## `[SCREW-F-TWISTED-SLOT]` — a collar and its bore as twisted sweeps

A collar is **one sweep** of its rounded-rectangle section along a straight line on the gear's
axis, twisted by the ribbon's own turn over that length; its bore is one sweep of the clearance
rectangle along a slightly longer line, as a cut. Both are the exact helicoid: a section turning
rigidly about the axis at `1/Lambda`. Nothing is lofted here any more, and no section count is
derived for the build.

**The path.** The sweep needs an `adsk.fusion.Path`, and `Path.create` on a sketch curve raises
`RuntimeError … InternalValidationError : Utils::getObjectPath(sketchCurve, …)` in the `Design`
sub-component, in every form tried (`[SCREW-F-DIAGNOSTIC]`). `component.features.createPath(line,
False)` on the same line returns the path (`[PB-PATH-FROM-SKETCH]`), and that is the maker. The
line is the collar's or bore's own line in the gear's Paths sketch (`[SCREW-F-REFERENCES]`),
drawn from its negative station to its positive one, so its start is where the profile sits and
it runs along `+dir_g`.

**The plane and the profile.** One construction plane per collar and per bore,
`setByDistanceOnPath(line, ValueInput.createByReal(0))` on that line (`[PB-CONSTRUCTION-PLANES]`,
the line passed directly), square to the axis at the span's negative end; Fusion put its origin
on the station to four decimals of a millimetre. On it, the section sketch by the spec's
rectangle scheme (§4), which draws the bore's rectangle at that station's own angle
`s/Lambda + Phi_g`. For a bore the four lines are solid and the profile is the rectangle,
`find_profile_by_curve_counts(sketch, lines=4)`. For a collar the four sides `L1..L4` are
construction, and round them go four solid sides `O1..O4`, `Oi` parallel to `Li` with an offset
dimension of `collarWall` and each of its ends coincident on the infinite line of the
neighbouring construction side (`addCoincident(point, line)`), and four three-point arcs, each
from the end of `Oi` to the start of `O(i+1)` through the construction corner pushed out by
`collarWall` along the corner's diagonal, with one `addTangent` to `Oi`. The arc's centre then
falls on the construction corner and its radius is `collarWall`; neither is dimensioned, and a
second tangent would over-constrain (`[PB-NO-OVERCONSTRAIN]`). The profile is
`find_profile_by_curve_counts(sketch, lines=4, arcs=4)`. Both sketches are drawn with computing
deferred (`[SCREW-F-DEFER]`), and both read fully constrained on 2026-09-28.

**The sweep.** `sweepFeatures.createInput(profile, path, operation)`, then
`input.twistAngle = adsk.core.ValueInput.createByReal(twist)` with `twist` in radians, then
`sweepFeatures.add(input)`. Set nothing else: not `orientation`, which defaults to
`PerpendicularOrientationType` and is moot along a straight path, not `solidTwistAxis`, which is
for a solid sweep (`[SCREW-F-NO-SOLID-TWIST]`), and no guide rail or surface, which would make
`twistAngle` ignored per the reference. The operation is `NewBodyFeatureOperation` for a collar
and `CutFeatureOperation` for a bore, and a bore also sets `input.participantBodies = [cageBody]`
before `add`, so the cut touches the cage and nothing else; the ribbons run through the channel
and are left whole, measured (`[PB-SWEEP-TWIST]`). The twist is `+2*collarHalf/Lambda` for a
collar, 43.64° at the defaults, and `+2*(collarHalf + 1 mm)/Lambda` for a bore, 65.45°: the
span divided by `Lambda`, positive. **Positive is the spec's sense**, measured: with the profile
at the line's start and the line running along `+dir_g`, a positive `twistAngle` turns the
section the way `s/Lambda + Phi_g` grows, and the far end of the collar landed 0.0047 mm from
the spec's own section there, against 4.9 mm under the opposite sign; the turn is linear along
the path (`[SCREW-F-DIAGNOSTIC]`). The collar body has ten faces — four sides, four corner
strips, two ends — and sixteen vertices, and the build checks its ends
(`[SCREW-F-SWEEP-CHECK]`).

**One sweep per bore, not one per gear.** The two collars of a gear stand on one axis, so a
single channel through both would be one sweep of 24 mm and 262°; each bore is its own line and
its own sweep instead, spanning its collar plus a millimetre each end, because the cut has to
reach only the collar and the part of its rod inside the wall, and the space between the two
collars is open frame. At the defaults that is 6 mm and 65° per bore.

**What the proof does with this.** `decad` has no twisted sweep. The compiled step proof stands
in a ruled loft through rotated rectangles for each sweep, at the count the spec's "What the
proof's stand-in costs" derives (15 at the defaults), and the hand-written
`TestBoreSubstituteKeepsItsClearance` bounds what that stand-in costs against the exact channel:
0.004 mm of the 0.3 mm clearance. The sense and the linearity of the twist are Fusion's, and the
runtime checks below are what keep them pinned on every build.

## `[SCREW-F-SWEEP-CHECK]` — checking each sweep at build time

Both checks carry their measured quantities in the error they raise (`[PB-SELF-DIAGNOSING]`).

**A collar's ends.** After the sweep, read `body.vertices` (each `vertex.geometry` a world
`Point3D`, the `Design` occurrence being at identity) and compute the sixteen outline points the
spec names in §4 — the eight side ends of the rounded rectangle at the near station
`sc - collarHalf` and at the far station `sc + collarHalf`, each set turned by its own station's
angle. For each computed point take the least distance to any vertex; raise, naming the collar,
the end and the worst distance, when any exceeds 0.05 mm. Measured: 0.0047 mm at the far end and
0.0000 mm at the near end with the right sign, 4.8979 mm at the far end with the wrong one, so
the tolerance sits an order of magnitude above the pass and two below the failure.

**A bore's channel.** After the cut, the sweep feature's `bodies.count` must be 1, and
`cageBody.pointContainment(point)` must return `PointOutsidePointContainment` at the two probes
`origin_g + s*dir_g ± (W/2 + clearance/2)*û(s)` for `s = sc + collarHalf/2`, where `û(s)` is
`cos(theta)*û_g + sin(theta)*v̂_g` with `theta = s/Lambda + Phi_g`: the toothed-side and
back-side middles of the channel in the far half of the collar, clear of the ribbon by
`clearance/2` and of the wall by the same. Raise naming the bore otherwise. A channel that is
open there was cut; and when `(W/2 + clearance/2)*|sin(2*(1.5*collarHalf + 1 mm)/Lambda)|`
exceeds `T/2 + clearance` — 5.14 mm against 1.55 mm at the defaults — a channel turned the
wrong way would stand 87° off the probes at that station and both would sit in the wall, so the
probes tell the senses apart there too. The build does not gate on that inequality; the
collar's own end check is what pins the sense.

## `[SCREW-F-DEFER]` — where sketch computing is deferred

`sketch.isComputeDeferred = True` is set on exactly ten sketches per build: the two Cell
Sections sketches (`[SCREW-F-CELL-LOFT]`), the four collar sections and the four bore sections
(`[SCREW-F-TWISTED-SLOT]`). It goes on right after `sketches.add(plane)` and `sketch.name`, before
the first `sketchPoints.add`, and comes off after the last `isFixed`, constraint or dimension
and before `isFullyConstrained` or `profiles` is read (`[PB-SKETCH-DEFER]`). Measured on
2026-09-28: the rectangle-scheme collar section fell from 0.45 s to 0.23 s, the one-tooth Cell
Sections sketch from 0.88 s to 0.09 s, and every deferred sketch read fully constrained with its
profiles found once computing was back on. No other sketch defers: the Anchor sketch projects
(`[SCREW-F-REFERENCES]`), and deferral under a projection has not been measured; the Paths,
Ring, Rods, bar and ball sketches are a few fixed points with lines, arcs or circles, and were
not measured. Extending it to them is a measurement, not a rule change.

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
- The loop's bars and balls are all revolved on the **Loop Plane**, offset `-cageRise` from the
  selected plane, which holds every foot; no plane is made per bar or per ball, and there is no
  Loop sketch of bare lines. The four feet are `C - cageRise*n̂ + ringRadius*(cos(psi)*ê +
  sin(psi)*k̂)`, in the spec's foot order (foot 0 the smallest azimuth in `[0°, 360°)` from `ê`
  about `+n̂`, the rest increasing); bar `i` runs from `foot_i` to `foot_((i+1) mod 4)`.
  - **Bar `i`** is a sketch **Loop Bar `i`** on the Loop Plane holding one rectangle from four
    reference points: `foot_i`, `foot_j`, `foot_j + (ringWire/2)*m̂` and `foot_i +
    (ringWire/2)*m̂`, with `j = (i+1) mod 4` and `m̂` the unit vector in the Loop Plane square
    to the bar and pointing away from the loop's centre `C - cageRise*n̂` (the component of
    `(foot_i + foot_j)/2 - (C - cageRise*n̂)` square to `foot_j - foot_i`, normalised). Four
    solid lines share the points in that order, `B0` from `foot_i` to `foot_j` first, and all
    four points are set `isFixed` after the last line. The sketch holds nothing free and no
    dimension. The profile is the one loop, `profiles.count == 1` then `item(0)`; the revolve
    is `createInput(profile, B0, NewBodyFeatureOperation)` → `setAngleExtent(False, '360 deg')`
    → `add`, about the bar's own side `B0`, which the profile lies against, as a revolve profile
    may. Measured: solid, 0.08472 cm³ against the cylinder's 0.08472.
  - **Ball `i`** is a sketch **Loop Ball `i`** on the Loop Plane: the solid line `Bl` runs
    between two reference points, `foot_i - (ringWire/2)*ê` (start) and `foot_i +
    (ringWire/2)*ê` (end), sharing both; then a three-point arc
    `addByThreePoints(Bl.startSketchPoint, through, Bl.endSketchPoint)` with `through` the point
    `foot_i + (ringWire/2)*k̂`, which lies on the plane; then both reference points are set
    `isFixed`, and `addCoincident(arc.centerSketchPoint, Bl)` puts the arc's centre on the
    chord, where the half-disc's centre is. With its two ends fixed the arc has one freedom, its
    bulge, and the coincident takes it; the side of the bulge is the seed's. The profile is the
    half-disc, `find_profile_by_curve_counts(sketch, lines=1, arcs=1)`, and `Bl` is the revolve
    axis. Measured: solid, 0.00818 cm³ against the sphere's 0.00818. The chord's direction in
    the plane does not matter to a sphere; `ê` and `k̂` are what was measured.
  - A pipe along the four bars in one feature was measured and rejected: its mitred corners
    reach past the balls (`[PB-PIPE-CORNER]`).

**Joins, in groups.** The ring is the first body. Each later group is made as new bodies and
joined by one `combineFeatures` join whose `tools` collection holds the whole group: the four
rods, as now; the eight loop pieces, four bars and four balls; the four collars. A join is
`combineFeatures.createInput(target, tools)` with the tools in an `ObjectCollection`, `operation`
set to `JoinFeatureOperation`, `isKeepToolBodies = False`, then `add`. **Each leaves one body**,
which means the combine feature's `bodies.count` — `Feature.bodies`, the bodies the feature
created or modified — is exactly 1; the build raises with the group's name and the count when it
is not (`[PB-EMPTY-RESULT]`, `[PB-SELF-DIAGNOSING]`), and `bodies.item(0)` is the target from then
on. The same count gates the ribbon joins of §3. The pieces are always made as new bodies and
combined explicitly, because a join operation on the extrude, revolve or sweep itself would join
into whatever it touches, the ribbons included. The bores are then cut last, each by its own
sweep with the cage as its only participant (`[SCREW-F-TWISTED-SLOT]`); there is no bore body and
no combine cut. The rods' azimuths are the angles the proof's `TestRodsStandBesideTheirCollars`
derives, and the build recomputes them by the same search rather than reading them from a table,
because they move with every ribbon dimension.

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
Both diagnostics of 2026-09-28 built this sketch first and read it fully constrained.

**Every other sketch** takes its references as **reference points**, by the recreate-share-fix
recipe of `[PB-PROJECT-NOT-FIXED]` (b), which is how the bevel gear builds each per-gear profile
sketch and the shaft axis its revolve, pattern and section planes stand on:

1. Compute the point in world space from `C`, `ê`, `n̂` and the §1 frame — the ends of a sweep
   path on the gear's axis, the axis point of a section at `origin_g + s*dir_g`, a rod's foot on
   the ring's circle, a bar's corner on the Loop Plane, a cell section's corner wherever the
   section puts it.
2. `local = sketch.modelToSketchSpace(worldPoint)`, then `local.z = 0` **when the point is meant
   to lie on the plane**, then `pt = sketch.sketchPoints.add(local)` (`[PB-SKETCH-ZERO-Z]`).
   Being on the plane in world arithmetic does not put the point at zero height in the sketch:
   the first Fusion load (2026-09-27) put every point of `Gear A Collar -R Section 0` at
   `z = 4.4e-6` cm, because Fusion's `setByDistanceOnPath` plane sat that far from the station
   the build computed, and the sketch read not fully constrained with every dimension present.
   The same zeroing applies to every other point this build maps into a sketch that is meant to
   lie on its plane: a raw seed passed to `addByTwoPoints`, an arc's through point, a circle's
   centre, a dimension's text point. The Cell Sections sketch is the one exception: its corners
   are meant to lie off the plane and keep their `z` (`[SCREW-F-CELL-LOFT]`).
3. Draw every curve that uses it **sharing** `pt`: `addByTwoPoints(pt, ...)`,
   `addByThreePoints(pt, ...)` (`[PB-SHARE-XOR-COINCIDENT]`: shared, so no coincident to it).
4. After the last such curve exists, and before any constraint or dimension is added, set
   `pt.isFixed = True`. The order is the playbook's: a bare point fixed before a curve consumes it
   does not leave the sketch fully constrained.

A line between two reference points is fully constrained by the two fixed ends and takes no
dimension; the Paths sketch is four such lines, overlapping on one axis, and read fully
constrained that way (`[SCREW-F-DIAGNOSTIC]`). A circle centred on a reference position is not
built on a reference point: it is `addByCenterRadius` at that position and its own
`centerSketchPoint` is set `isFixed = True` (`[PB-CIRCLE-CENTER]`, measured in Fusion on a
`setByDistanceOnPath` plane), with a diameter dimension; a coincident from a circle's centre to
a fixed point is not used, since the playbook records a solve failure for the coincident form.

The world points are the numbers the source geometry was dimensioned to, so they are the solved
positions (`[PB-SOLVED-GEOMETRY]`): the Anchor sketch is the only sketch whose geometry a later
sketch depends on, and its `worldGeometry` is read once, after its gate, into `C` and `ê`. No
sketch reads another sketch's points after that.

**What the proof does with this.** The sketch engine's `Fix` is the analogue of `isFixed`
(`[PB-SKETCH-FIRST]`, constraint mapping), so the compiled proof fixes each reference point and
proves the rest of the scheme against it; for the Anchor sketch it writes the midpoint alone,
because the engine's midpoint already carries the coincident row, as `proof/bevelgear` records.
Fusion agreed on every sketch of the second load and of both diagnostics.

## `[SCREW-F-NORMAL-SIGN]` — reading `n̂` from a plane the build made

`setByOffset(targetPlane, ValueInput.createByReal(d))` offsets along the selected entity's own
normal, which for a planar face may point either way. The build never assumes that sign. After the
`Gear A Axis Plane` is made at `-A/2`, it reads `plane.geometry` — a `Plane` with an `origin` and a
`normal` — and sets `n̂ = normal` if `(C - origin) · normal > 0`, else `-normal`, so that `C` lies
`+A/2` along `n̂` from gear A's plane by construction; it then checks that `|(C - origin) · normal|`
is `A/2` for both axis planes and raises otherwise. Every later offset from the selected plane —
the Loop Plane at `-cageRise` — is signed the same way, so it lands on `-n̂` with gear A's plane;
the diagnostic read the Loop Plane at −1.350 cm along `n̂` from `C`.

## `[SCREW-F-NO-SOLID-TWIST]` — why the ribbon is not built straight and then twisted

The straightforward mental model is to build the toothed rack flat, with its sinusoidal edge as one
spline in one sketch and one extrude, and then twist the solid. Fusion's parametric environment has
no twist feature for a solid body. `SweepFeatureInput` does document a solid twist axis, but a solid
sweep sweeps the body's *volume* along the path, which smears each tooth into a continuous ridge and
destroys the very feature being built. A profile sweep with `twistAngle` builds the exact twisted
blank in one feature, and is what the collars and bores now are (`[SCREW-F-TWISTED-SLOT]`), but
the tooth gaps are not invariant under the continuous screw motion, only under its discrete
step, so no sweep can cut them.

That is why the build lofts one tooth cell and repeats it by a screw step. It is the same shape as
the involute gears' "draw one tooth, pattern it", with the circular pattern replaced by a
transformation Fusion has no pattern feature for: a rectangular pattern only translates, a circular
pattern only rotates, and no nesting of the two produces the diagonal a screw step needs. Two
other ways of getting the whole ribbon in fewer features were measured on 2026-09-28 and are
out: the ruled ribbon built as temporary-BRep sheets and stitched fails at the stitch
(`[PB-TEMP-BREP-STITCH]`), and a two-section loft with a centreline does not turn between its
ends (`[PB-LOFT-TWO-SECTIONS-STRAIGHT]`).
