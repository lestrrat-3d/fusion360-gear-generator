# Screw Gear — Fusion API mechanics

Sidecar to `spec/screwgear/instructions.md`. Cross-gear conventions live in `PLAYBOOK.md` as
`[PB-…]`; this file holds only what is specific to the screw gear, under `[SCREW-F-…]` anchors that
the spec cites by name.

Every anchor below is stated from the API reference and from what Fusion did when asked. The
loads and diagnostics that asked are recorded under `[SCREW-F-FIRST-LOAD]` and
`[SCREW-F-DIAGNOSTIC]`, and the three prints of the sleeve under `[SCREW-F-PRINT-MESH]`,
`[SCREW-F-PRINT-2]` and `[SCREW-F-PRINT-3]`, per the "When Fusion gives a verdict" rule in
`CLAUDE.md`; a sentence that
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
  Counted in the finished document: 23 sketches, 12 construction planes, 96 timeline items and
  three bodies, exactly as "What the build makes" predicts once the three `moveToComponent`
  relocations are in its table (they were missing when this load ran). `Component.features.count`
  summed over the five components read 62 (59 in `Design`, one each in `Gear A`, `Gear B` and
  `Cage`). The timeline settles the real count: 96 items = 23 sketches + 12 planes + 5 component
  creations + 56 features, which is the table's 53 plus the three relocations. The feature
  collections report six more than that, all in `Design`, for a reason not measured. By eye the
  user found its geometry the same as the second load's.
- **2026-09-28, the print.** The user printed the third load's geometry at the defaults of that
  day (ribbon 10 by 2.5 mm, pitch 1.75 mm, teeth 1.2 mm, 80 teeth, 140 mm) and found the teeth
  far too small, the ribbon too narrow and the part short. Every default in `instructions.md`
  was scaled 1.5×, the teeth deepened to one pitch and the ribbon lengthened to 68 teeth because
  of it; `instructions.md` "What the print showed" records the verdict and the new table. Every
  measurement in this file dated 2026-09-28 was made at the old defaults.
- **2026-09-29, the first load at the 1.5× defaults.** The add-in compiled from the resized spec
  (15 mm ribbon, 2.625 mm teeth one pitch tall, 68 teeth) built at the defaults in Fusion and the
  user reported that it "runs well". No counts were read at this size. A print of the resized
  ribbon was under way.
- **2026-09-30, the frame printed.** The user printed the video's frame — the ring, the loop,
  the rods and the collars this file used to describe for §4 — several times
  and found it not practical to print: the thin rods, the wire ring and the loop wobble while the
  printer lays them down. `instructions.md` "What the print showed" records the report; the
  frame is now the sleeve of §4 (`[SCREW-F-SLEEVE]`). The first sleeve printed was built by the
  add-in at c8a63b5 (`[SCREW-F-PRINT-MESH]`); no counts or timings were read from that load.

## `[SCREW-F-PRINT-MESH]` — the printed pair did not mesh, 2026-10-02

**What was printed.** The sleeve from the add-in at commit c8a63b5, at that day's defaults: a
0.45 mm clearance, a 0.75 mm engagement, 15° on both mounting angles and an assembly phase of
−1.31 mm. The two ribbons came from the earlier build at f328813, whose ribbon is the same part:
15 by 3.75 mm, 2.625 mm teeth one pitch deep, 68 teeth, 49.5 mm lead.

**What happened.** Both ribbons screwed through their bores, and the teeth did not mesh.

**Why.** The geometry agrees between the proof and both add-ins; the cause is play. A bore is
the ribbon's crest rectangle plus the clearance all round, so at 0.45 mm each ribbon could move
0.45 mm toward or away from the other ribbon, 0.45 mm sideways, and roll 3.44° about its own
axis. Moving toward the other ribbon adds to the engagement, and a roll adds to the mounting
angle. The mesh at those defaults drove only within about 0.6 mm of engagement and, at equal
mounting angles, from 11° to 16°, so 15° was 1° from the angle past which the teeth no longer
box each other in. Over the poses the bores allowed, a scratch study found 128 of 324 pose
pairs jamming or letting the teeth pass without boxing each other. The proof had measured the
mesh at one pose, each ribbon on its nominal axis, where the pair drives. The teeth meet 2–3 mm
from where the centre lines cross, which is where the mounting angle puts them.

**The fix.** The defaults became a 0.20 mm clearance, a 0.90 mm engagement, 14° on both
mounting angles and an assembly phase of −1.30 mm (`instructions.md`, "What the print
showed"). The ribbon is unchanged, so the printed ribbons fit; only the sleeve is reprinted. At
those values each ribbon moves 0.20 mm toward, away or sideways and rolls 1.53°, and the
nominal free window is 0.433–0.473 mm. `proof/screwgear/bore_play_test.go` runs the mesh over the
play:

- `TestPairDrivesUnderBorePlay` takes each ribbon at its limits toward and away at every whole
  degree of roll and at its two roll limits, 8 poses each, and runs all 64 pose pairs: 63 drive,
  with windows from 0.052 to 0.932 mm, and one jams, both ribbons pushed the whole clearance
  toward each other.
- `TestPairDrivesUnderSidewaysPlay` takes each ribbon at its sideways limits against the other
  at rest, at its limits along the axis between them and at its own sideways limits: 16 of 16
  drive, with windows from 0.066 to 1.142 mm.
- `TestPrintedFitFailsUnderBorePlay` runs the same check at the printed values and holds that it
  fails them: 8 of its 16 pose pairs fail in a way the check refuses.

An offline run sampled sideways and diagonal moves combined with each roll as well, 26 poses a
ribbon and 676 pose pairs: 6 jammed, each with both ribbons pushed toward each other by 0.204 mm
or more in all, and none failed another way. The user accepted those jams as the cost of the
fix. The reprinted sleeve is `[SCREW-F-PRINT-2]`.

**Could the proof have caught it?** Yes. The bores and the mesh were both in the proof, and no
case joined them. `bore_play_test.go` does, and at the printed values it fails.

## `[SCREW-F-PRINT-2]` — the second sleeve: tight `-R` bores, a mesh that slips, 2026-10-02

**What was printed.** The sleeve at the defaults `[SCREW-F-PRINT-MESH]` moved to: a 0.20 mm
clearance all round every bore, a 0.90 mm engagement, 14° on both mounting angles and an
assembly phase of −1.30 mm. The ribbons were the ones printed for the first sleeve, the same part
the defaults still describe.

**What happened.**

- The two `-R` bores were too tight to pass the ribbons. The two `+R` bores, cut to the same
  0.20 mm, passed them.
- The user chiselled the two `-R` bores open, and the ribbons then went through.
- The teeth meshed only sometimes. They mostly slipped past each other, even with the ribbons
  pressed toward each other by hand.

**Why the `-R` bores were tight.** Each `-R` bore passes through level inside the wall: its
long faces lie flat at station −14.30 mm, so with the sleeve standing on an end the printer
bridges its 15.4 mm roof across the hole. The `+R` bores' long faces come no nearer level than
11.3° inside the wall, and are level only at their inner mouth, station 10.45 mm. The likeliest
cause is that the bridged roofs sagged into the 0.20 mm of room. Nothing measured the sag; the
user's report of which bores bound is the evidence.

**Why the teeth slip.** Two things, found by a study whose scratch harness and logs are kept
in the worktree's `.tmp/meshfix/` (`band1.log` to `band5.log`, `map1.log`, `union1.log` to
`union3.log`, `tilt.log`, `blunt1.log`, `allow1.log`, `allow2.log`, `tube2.log`); they are
working notes and are not tracked.

- **The mesh drives only in a narrow band of mounting angle, and a ribbon rolls in its bores.**
  What decides whether the pair drives is the sum of the two mounting angles, not either alone.
  At the 0.90 mm engagement, with the tips 0.35 mm short as a print makes them, equal angles drive
  from 11.5° to 15.5°, sampled every 0.5°. A 0.20 mm bore lets each ribbon roll 1.53°, and
  every degree of roll is a degree of mounting angle, so the bores keep the pair between 12.5°
  and 15.5°: a degree inside the edge where it jams, and at the edge where the teeth stop boxing
  each other. At a 0.50 mm clearance each ribbon rolls 3.85°, and the rolls alone span wider
  than the band.
- **Deeper teeth make the band narrower.** At 1.4 mm of engagement the band is about 1° wide
  and past about 1.6 mm it is gone: the pair goes straight from jamming to letting the teeth
  pass. At 0.90 mm of engagement, thinner ribbons (2.5 and 3.0 mm), other crossing angles (60°
  and 100°), leads from 35 to 70 mm, taller teeth (3.0 and 3.5 mm), a 3.0 mm pitch and a 20 mm
  ribbon each left between two and six of the 1° samples driving, against four at the
  defaults. So the fix is not a deeper mesh but a fit that keeps the roll the bores allow
  inside the band.
- **A printed tip is short.** The model's crest is an exact cosine. Over the 0.20 mm bores as
  drawn, `proof/screwgear` finds the mesh boxing the teeth at every pose pair with the tips as
  much as 0.40 mm short, and letting them pass, with both ribbons pulled the whole clearance
  apart, at 0.45 mm (`TestSecondPrintMeshIsMarginal`). That is 0.05 mm of margin past the
  0.35 mm the proof now takes for a printed tip. The chiselled `-R` bores were no longer the
  bores drawn, and a ribbon in a bore opened by hand can move in ways no pose of the drawn bore
  reaches.

**The fix.** The mesh, the clearance and the ribbon stay as they are, so the printed ribbons
stay usable and only the sleeve is reprinted. Each gear's level bore, its `-R` bore at the
defaults, gets a **roof allowance** of 0.30 mm on the long face that is its roof when the sleeve
stands on its `-n̂` end, the end below the selected plane: 0.50 mm of room under the bridged roof
and 0.20 mm everywhere else (`instructions.md` §4, "The roof allowance"). The 0.30 mm is the
least the user asked for. The sleeve has to be printed standing on that end; the build logs
which end it is. In `proof/screwgear`, at that fit, the straight tooth at 14°, before
`[SCREW-F-PRINT-3]` moved the tooth and the mounting angles:

- `TestRoofAllowanceAddsOnlyATilt` holds that the allowance adds no move along or across the
  axes and no roll, 0.200 mm and 1.53° with it as without it, and measures the one thing it adds:
  a tilt of the ribbon about the axis square to it and to the common perpendicular, up to 0.96°
  one way against 0.56° without the allowance. At a tilt of 0.60° with the best move along the
  axes that goes with it, gear A's crossing stands 0.273 mm toward gear B and gear B's 0.273 mm
  away from gear A, against 0.200 mm without the allowance.
- `TestPairDrivesUnderBorePlay` runs the mesh with both ribbons' tips 0.35 mm short over each
  ribbon's limits toward and away at every whole degree of roll, its roll limits and those two
  tilts, 100 pose pairs: 96 drive and 4 jam, every jam with gear A tilted 0.273 mm toward gear B
  and gear B pushed toward it or at its roll limit, and none lets the teeth pass.
  `TestPairDrivesUnderSidewaysPlay` finds 16 of 16 sideways pose pairs driving at the same tips.
- The tilt is the allowance's cost. With the roof at 0.28 mm of allowance or more and no sag at
  all, gear A's tilt toward gear B jams against gear B at its roll limit; that jam is accepted,
  since it closes the axes by more than the clearance with neither ribbon pulled away. The
  tilt stops at 0.273 mm however much room the roof has, because the other bore holds it.

**Could the proof have caught it?** For the tight `-R` bores, no: the proof drew the bores as
the spec cuts them, logged that their roofs are bridged, and has no model of what a printer
makes of a bridge (`instructions.md` "What the proof cannot reach"). For the slipping teeth,
partly: the proof modelled no tip loss, and a print's tips are short. It now blunts both
ribbons' tips by 0.35 mm wherever it judges the mesh under play, and
`TestSecondPrintMeshIsMarginal`, in `bore_play_test.go`, records that the fit printed in the
second sleeve boxes the teeth at 0.40 mm of tip loss and lets them pass at 0.45 mm, with the
answer to this question beside it. What it cannot reach is a bore opened by a chisel. The third
sleeve is `[SCREW-F-PRINT-3]`.

## `[SCREW-F-PRINT-3]` — the third sleeve: the bores fit, the teeth touch at a point, 2026-10-03

**What was printed.** The sleeve at the defaults `[SCREW-F-PRINT-2]` moved to: a 0.20 mm
clearance, a 0.30 mm roof allowance on each gear's `-R` bore, a 0.90 mm engagement, 14° on both
mounting angles and an assembly phase of −1.30 mm, with ribbons of the straight-ridge tooth. That
table is `thirdPrintParams` in `proof/screwgear/geometry_test.go`.

**What happened.** The ribbons passed through the bores: the roof allowance fixed the fit. The
teeth still slipped. The user saw that "they meet at a single _point_ rather than mating at the
tooth surface".

**Why.** Each ribbon's tooth ridges ran straight across its thickness, square to its own axis,
and the two axes cross at 80°. Where the teeth touched, the two ridges stood 74–80° apart, so a
corner of one tooth dug into the other's flank: less than 0.13 mm of each 3.75 mm ridge came
within 0.10 mm of the other ribbon, and the two flank normals stood 107–118° apart instead of
facing each other. The study that found it is
`mesh-search.md` "The search for line contact"; its scratch harness and logs are kept in the
worktree's `.tmp/linecontact/`, working notes that are not tracked.

**The fix.** The ridges lean so that the two ribbons' ridges lie along each other where they
touch: the toothed edge becomes
`Utooth(v, s) = W/2 - H/2 + (H/2)*cos(2*pi*(s + tan(Slant)*v - Z)/P) - Bow*v^2` with a 25.8°
slant and a 0.048 mm⁻¹ bow, two new dialog inputs. Both mounting angles went to 0°, the
engagement to 1.05 mm and the assembly phase to −1.31 mm; the clearance, the roof allowance and
the ribbon blank are unchanged (`instructions.md`, "What the print showed"). The ribbons and the
sleeve are both reprinted. §2 of `instructions.md` draws each section's toothed side as a fitted
spline through eleven points of the edge (`[SCREW-F-CELL-LOFT]`) and checks the slant's sign on
the lofted cell. At those values in `proof/screwgear`:

- `TestTeethTouchAlongALine` finds the touching ridge within 0.05 mm of the other flank over
  2.67 mm at the least, 97% of it, at all 24 driving poses of a pitch, the ridges within 0.3° of
  parallel and the normals within 1.0° of facing.
- `TestPairDrivesOneToOne` finds a 1.063–1.103 mm window winding 1:1 with a 0.018 mm departure.
- `TestPairDrivesUnderBorePlay` drives 100 of 100 pose pairs with the tips 0.35 mm short, the
  touching ridge within 0.10 mm of the other flank over 1.39 mm at the least, and
  `TestPairDrivesUnderSidewaysPlay` 16 of 16.

**What it costs at the bores.** At 0° every bore passes through level inside the wall, at
stations ±12.37 mm, so the printer bridges all four roofs, and only the `-R` bores carry the roof
allowance; the `+R` bores' roofs are bridged with the 0.20 mm that closed up on the second
sleeve. A roof allowance on both bores of each gear lets gear A move 0.344 mm toward gear B and
gear B 0.344 mm away from gear A, instead of 0.200 mm, and over that play, in a scratch run of the full sample, 4 of
100 pose pairs let the teeth pass and the least contact fell to 1.02 mm. Whether the `+R` bores
pass the ribbons is the next print's to say.

**Could the proof have caught it?** Yes, by the contact's length. Every quantity the analysis
used was in the model when the straight tooth was printed; the proof asked only whether the two
ribbons clear each other, and a point contact clears. `TestTeethTouchAlongALine` in
`proof/screwgear/contact_test.go` now measures how much of the touching ridge lies near the
other flank, and `TestStraightRidgesTouchAtAPoint` beside it holds that the check fails the
printed tooth: no stretch of ridge within 0.05 mm, at most 0.13 mm within 0.10 mm, the ridges 76°
and 79° apart.

## `[SCREW-F-PRINT-4]` — the square-marked bores reject the ribbons, 2026-10-05

The user printed a new sleeve from the Add-In deployed on 2026-10-04, standing on its flat,
unmarked end. Both square-marked `-R` bores stop the ribbons at their mouths, whether approached
from the tube's outside or inside. The Roof Allowance value in the generating dialog was not
recorded. Its default was 0.30 mm, but this report alone does not show whether the cut contained
that allowance or how much the printed opening closed. The two `+R` bores were not reported as
blocking.

The next print uses a 0.60 mm default roof allowance, giving 0.80 mm of drawn room at each `-R`
roof while keeping 0.20 mm at its other faces. At 0.60 mm the focused bore-play proof still
finds all six sampled pose pairs driving, and the roof allowance does not increase the measured
translation or roll; the opposite bore limits those moves. The build must also check the solved
sketch corners and both sides of the roof allowance in its cut. These checks can reject a wrong
Fusion cut; they cannot measure shrink or sag in the printed sleeve. A new print decides whether
0.80 mm of drawn roof room is enough.

## `[SCREW-F-DIAGNOSTIC]` — what the diagnostics of 2026-09-28 measured

Two scripts ran in the user's Fusion on 2026-09-28, each in a scratch document holding the
add-in's own component tree (root → `Screw Gearing` → `Design`, `Gear A`) under a plane tilted
30° off XY, never activating a component, at the spec's defaults of that day, the 10 mm ribbon
(the defaults were scaled 1.5× after the print, `[SCREW-F-FIRST-LOAD]`). Their outputs are the
numbers quoted through this file. The cross-gear facts they settled are playbook anchors; the
numbers specific to this gear are:

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
rotated sections — 41 at the defaults, each rotated 1.91° further about the axis than the last.
Each section is a rectangle whose toothed side is a fitted spline through eleven points of the
leaned edge, since the third sleeve's print (`[SCREW-F-PRINT-3]`); until then it was four lines.
All of them are drawn in **one sketch**, on the gear's Axis Plane, and the loft is fed a path
per section.

**The sketch.** Every point is `sketch.sketchPoints.add(sketch.modelToSketchSpace(world))` with
the mapped point's `z` **kept**, which is the one place this build departs from
`[PB-SKETCH-ZERO-Z]`: those points are meant to lie off the plane, at every height above and
below it, and the rule zeroes only a point that is meant to lie on it. The three straight sides
of a section are `sketchLines.addByTwoPoints(point, nextPoint)` sharing the points
(`[PB-SHARE-XOR-COINCIDENT]`), and the toothed side is
`sketch.sketchCurves.sketchFittedSplines.add(fitPoints)`, `fitPoints` an
`adsk.core.ObjectCollection` of the side's eleven sketch points in order across the thickness,
the two toothed corners at its ends (`instructions.md` §2 states the order, the checks and the
signatures). Every point is set `isFixed = True` after the last curve of the last section exists
(`[PB-PROJECT-NOT-FIXED]` (b): fix after use), each spline's `fitPoints` items included, since
the reference does not say whether the spline keeps the points it was given. No tangent handle is
activated. Draw it with
`isComputeDeferred = True` from just after `sketches.add` until just after the last `isFixed`,
then set it `False` (`[SCREW-F-DEFER]`). Fusion reads such a sketch fully constrained and finds
one profile per section, each planar in its own station's plane — measured at 11, 41 and 81
sections of four lines (`[SCREW-F-DIAGNOSTIC]`, `[PB-3D-SKETCH-SECTIONS]`); a sketch of spline
sections has not been loaded. The build raises unless `isFullyConstrained` and unless
`profiles.count == c*n + 1`; the profiles are not used, because nothing says which profile is
which station, and the sections are made from the curves.

**The sections.** For section `k`, put its four curves, line, spline, line, line, in an
`adsk.core.ObjectCollection` and
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
to five at 68 teeth, as it did at the 80 the measurement was made with; the whole ribbon in one
loft would be 681 sections and no rounds, and was not measured. `cellTeeth` is the module
constant that decides it, and 1 is the measured fallback.

## `[SCREW-F-TWISTED-SLOT]` — a bore as a twisted sweep cut

A bore is **one sweep** of the clearance rectangle along a straight line on the gear's axis,
twisted by the ribbon's own turn over that length, as a cut from the cage. It is the exact
helicoid: a section turning rigidly about the axis at `1/Lambda`. Nothing is lofted here, and no
section count is derived for the build. The video frame's collars were built the same way as new
bodies, a rounded rectangle swept along a shorter line, and that is where the measurements below
were made.

**The path.** The sweep needs an `adsk.fusion.Path`, and `Path.create` on a sketch curve raises
`RuntimeError … InternalValidationError : Utils::getObjectPath(sketchCurve, …)` in the `Design`
sub-component, in every form tried (`[SCREW-F-DIAGNOSTIC]`). `component.features.createPath(line,
False)` on the same line returns the path (`[PB-PATH-FROM-SKETCH]`), and that is the maker. The
line is the bore's own line in the gear's Paths sketch (`[SCREW-F-REFERENCES]`), drawn from its
negative station to its positive one, so its start is where the profile sits and it runs along
`+dir_g`.

**The plane and the profile.** One construction plane per bore,
`setByDistanceOnPath(line, ValueInput.createByReal(0))` on that line (`[PB-CONSTRUCTION-PLANES]`,
the line passed directly), square to the axis at the span's negative end; Fusion put its origin
on the station to four decimals of a millimetre. On it, the section sketch by the spec's
rectangle scheme (§4), which draws the bore's rectangle at that station's own angle
`s/Lambda + Phi_g`; its four lines are solid and the profile is the rectangle,
`find_profile_by_curve_counts(sketch, lines=4)`. The sketch is drawn with computing deferred
(`[SCREW-F-DEFER]`); the collar sections drawn by the same scheme read fully constrained on
2026-09-28. After solving, compare each of the four sketch corner points with its mapped world
seed to within 0.001 mm. A fully constrained sketch with the roof allowance on the opposite
face must fail this check before it can cut the sleeve.

**The sweep.** `sweepFeatures.createInput(profile, path, CutFeatureOperation)`, then
`input.twistAngle = adsk.core.ValueInput.createByReal(twist)` with `twist` in radians, then
`input.participantBodies = [cageBody]`, then `sweepFeatures.add(input)`. Set nothing else: not
`orientation`, which defaults to `PerpendicularOrientationType` and is moot along a straight
path, not `solidTwistAxis`, which is for a solid sweep (`[SCREW-F-NO-SOLID-TWIST]`), and no guide
rail or surface, which would make `twistAngle` ignored per the reference. The participant list
makes the cut touch the cage and nothing else; the ribbons run through the channel and are left
whole, measured (`[PB-SWEEP-TWIST]`). The twist is the span divided by `Lambda`, positive:
`+(sOut - sIn)/Lambda`, 81.41° at the defaults. **Positive is the spec's sense**, measured on a
collar: with the profile at the line's start and the line running along `+dir_g`, a positive
`twistAngle` turned the section the way `s/Lambda + Phi_g` grows, and the far end of the collar
landed 0.0047 mm from the spec's own section there, against 4.9 mm under the opposite sign; the
turn is linear along the path (`[SCREW-F-DIAGNOSTIC]`). The measured sweeps turned 43.64° (a
collar) and 65.45° (a bore at the 10 mm ribbon) and started on a collar's face; a sleeve's bore
turns 80° and its profile starts in air, in the hollow or outside the tube. The add-in at c8a63b5
built such bores at that day's 82.31°, and both printed ribbons screwed through them
(`[SCREW-F-PRINT-MESH]`); nothing else was measured on them. The build checks each cut (`[SCREW-F-SWEEP-CHECK]`).

**One sweep per bore, not one per gear.** The two bores of a gear stand on one axis, 30 mm apart
at the defaults with the hollow between them, so a single channel through both would cut the
tube only where each bore does and turn 262° for nothing; each bore is its own line and its own
sweep instead, spanning the wall plus a millimetre each end beyond where the channel first and
last meets it.

**What the proof does with this.** `decad` has no twisted sweep. The compiled step proof builds each sweep's
channel as a chain of two-section lofts through rotated rectangles, at the count the spec's "What the proof's
stand-in costs" derives (18 at the defaults). The hand-written `TestSleeveBoreSubstituteKeepsItsClearance`
holds a ruled wall through those sections to 0.007 mm of the 0.20 mm clearance; `decad` walls each cell with
two flat triangles, up to 0.32 mm off that ruled wall, so no clearance is read off the stand-in. The sense and
the linearity of the twist are Fusion's, and the runtime check below is what keeps them pinned on every build.

## `[SCREW-F-SWEEP-CHECK]` — checking each sweep at build time

The check carries its measured quantities in the error it raises (`[PB-SELF-DIAGNOSING]`).

**A bore's channel.** After the cut, the sweep feature's `bodies.count` must be 1, and
`cageBody.pointContainment(point)` must return `PointOutsidePointContainment` at the two probes
`origin_g + sc*dir_g ± (W/2 + clearance/2)*û(sc)` at the crossing itself, `sc = ±cageRadius`,
where `û(s)` is `cos(theta)*û_g + sin(theta)*v̂_g` with `theta = s/Lambda + Phi_g`: the
toothed-side and back-side middles of the channel, clear of the ribbon by `clearance/2` and of the
channel's wall by the same, 16.80 mm and 16.30 mm from the frame's axis at the defaults, inside
the wall. Raise naming the bore and the containment read otherwise. A channel that is open there
was cut, and the probes also tell the two senses apart: under the wrong sense the channel at the
crossing stands turned `2*(sc - s0)/Lambda` from the right one, `s0` being the station the profile
sits at — 103° for a `+R` bore and 58° for a `-R` bore — and both probes then sit 7.39 and
6.46 mm across a channel 2.075 mm half thick, in the wall. `TestSleeveBoreProbesTellTheTwistSense`
holds both halves. The video frame's collars also had their end vertices checked against the
turned outline, 0.0047 mm with the right sign and 4.8979 mm with the wrong one; a sleeve has no
swept body of its own to read vertices from, so the probes are the whole check.

For a level bore with `roofAllowance > 0`, also probe the cut at its crossing station with
`u = 0` and `v = roofSign*(ht + roofAllowance/2)`, where `roofSign` is `+1` for a `+v` roof and
`-1` for a `-v` roof. That point must be outside the cage. At
`v = -roofSign*(ht + roofAllowance/2)` the point must remain inside the cage. The pair detects
an allowance missing from the roof, put on the floor, or applied to both faces; the existing
toothed-side and back-side probes would miss those mistakes. Raise with the bore name, the
side, and the containment result.

## `[SCREW-F-DEFER]` — where sketch computing is deferred

`sketch.isComputeDeferred = True` is set on exactly six sketches per build: the two Cell Sections
sketches (`[SCREW-F-CELL-LOFT]`) and the four bore sections (`[SCREW-F-TWISTED-SLOT]`). It goes on
right after `sketches.add(plane)` and `sketch.name`, before the first `sketchPoints.add`, and
comes off after the last `isFixed`, constraint or dimension and before `isFullyConstrained` or
`profiles` is read (`[PB-SKETCH-DEFER]`). Measured on 2026-09-28: the rectangle-scheme collar
section fell from 0.45 s to 0.23 s, the one-tooth Cell Sections sketch from 0.88 s to 0.09 s, and
every deferred sketch read fully constrained with its profiles found once computing was back on.
No other sketch defers: the Anchor sketch projects (`[SCREW-F-REFERENCES]`), and deferral under a
projection has not been measured; the Paths, Sleeve and Window sketches are a few fixed points
with lines or circles, and were not measured. Extending it to them is a measurement, not a rule
change.

## `[SCREW-F-SLEEVE]` — the tube and the window cuts

None of this has been built in Fusion; the calls are the API reference's, and the checks the
spec's §4 names are what a first load reads. No construction axis is made anywhere in this build,
since one needs an active component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`). Every sketch here is built
from the reference points of `[SCREW-F-REFERENCES]` and nothing free: world points of the spec's
§1 frame, mapped in with `modelToSketchSpace` and given `z = 0`.

- **The tube** is an `extrudeFeatures` extrusion of the ring between two circles sketched on the
  selected plane, in a sketch named **Sleeve**: each circle `addByCenterRadius` at `C`, radius
  `Ri` or `Ro`, its `centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`: the centre is
  created free, and `isFixed` is the pin Fusion accepts) and a diameter dimension with its text
  point on the circle (`[PB-RADIAL-DIM]`). The two centre points are separate points at the same
  place, both fixed, with no coincident between them. The sketch has two profiles, the disc
  inside `Ri` and the ring; the ring is the one whose `profileLoops.count` is 2. Iterate
  `sketch.profiles`, take the one profile with two loops, and raise with the counts when there is
  not exactly one (`[PB-EMPTY-RESULT]`). `find_profile_by_curve_counts` cannot pick it: it counts
  lines, arcs and NURBS curves and treats a `Circle3DCurveType` curve as a type that disqualifies
  the loop. The extrude is `extrudeFeatures.createInput(ring, NewBodyFeatureOperation)` →
  `setSymmetricExtent(ValueInput.createByReal(cageRise), False)` → `add`; `False` makes the
  value each side's length (`[PB-THROUGH-CUT]` for the argument's meaning), so the tube runs
  from `-cageRise` to `+cageRise`. The video frame's rods were extruded the same way from circles
  on the selected plane, and the third load built them.
- **The Window Plane** holds `C` and `n̂` and stands square to the direction the windows face.
  For windows facing `±k̂` it is `constructionPlanes.createInput()` →
  `setByAngle(anchorLine, ValueInput.createByString('90 deg'), targetPlane)` → `add`: the plane
  through the Anchor Line square to the selected plane. The video frame's Ring Plane was made by
  this call and the third load built it. For windows facing `±ê`, past a 90° crossing, it is
  `setByDistanceOnPath(anchorLine, ValueInput.createByReal(0.5))`: square to the Anchor Line at
  its midpoint, which is `C`. That call is the one the bore planes use, at fraction 0; it has not
  been run at 0.5.
- **A window** is a sketch **Window {d}** on the Window Plane: one reference point per hexagon
  corner, a solid line from each corner to the next sharing the points
  (`[PB-SHARE-XOR-COINCIDENT]`), every point set `isFixed` after the last line, nothing else.
  The one loop is the one profile (`[PB-SINGLE-PROFILE]`). The cut is
  `extrudeFeatures.createInput(profile, CutFeatureOperation)` →
  `setOneSideExtent(DistanceExtentDefinition.create(ValueInput.createByReal(Ro + 0.1)),
  direction)` → `participantBodies = [cageBody]` → `add`, lengths in cm. `direction` is
  `ExtentDirections.PositiveExtentDirection` when `sketch.modelToSketchSpace(C + d)` has a
  positive `z`, else `NegativeExtentDirection`; the reference does not say in so many words which
  way a profile's positive extent runs, so the build checks the result by the probe the spec
  names, a point in the middle of the wall where the window goes, inside before the cut and
  outside after it (`[PB-SELF-DIAGNOSING]`). The cycloidal gear's extrusions call
  `setOneSideExtent` the same way (`spec/cycloidal/fusion.md`).
- **Order and bodies.** The tube is the first body; each bore cut and each window cut has the
  cage alone as its participant, and each has to leave exactly one body, the feature's
  `bodies.count`, which the build raises on with the piece's name (`[PB-EMPTY-RESULT]`).

## `[SCREW-F-JOIN]` — joining a body

A join is `combineFeatures.createInput(target, tools)` with the tools in an `ObjectCollection`,
`operation` set to `JoinFeatureOperation`, `isKeepToolBodies = False`, then `add`. **It leaves
one body**, which means the combine feature's `bodies.count` — `Feature.bodies`, the bodies the
feature created or modified — is exactly 1; the build raises with the piece's name and the count
when it is not (`[PB-EMPTY-RESULT]`, `[PB-SELF-DIAGNOSING]`), and `bodies.item(0)` is the target
from then on. The ribbon joins of §3 use it. A piece is always made as a new body and combined
explicitly, because a join operation on the loft or the move itself would join into whatever it
touches. The video frame joined its rods, loop and collars into its ring this way, and the
second and third loads built them.

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
   path on the gear's axis, the axis point of a section at `origin_g + s*dir_g`, a window's
   corner on the Window Plane, a cell section's corner wherever the section puts it.
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
dimension; the Paths sketch is two such lines on one axis, and a Paths sketch of three and of four
such lines, overlapping, read fully constrained that way (`[SCREW-F-DIAGNOSTIC]`). A circle
centred on a reference position is not built on a reference point: it is `addByCenterRadius` at
that position and its own `centerSketchPoint` is set `isFixed = True` (`[PB-CIRCLE-CENTER]`,
measured in Fusion on a `setByDistanceOnPath` plane), with a diameter dimension; a coincident from
a circle's centre to a fixed point is not used, since the playbook records a solve failure for the
coincident form.

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
is `A/2` for both axis planes and raises otherwise. No later plane is offset from the selected
plane. The video frame's Loop Plane was, signed the same way, and the diagnostic read it at
−1.350 cm along `n̂` from `C`, on gear A's side as intended.

## `[SCREW-F-NO-SOLID-TWIST]` — why the ribbon is not built straight and then twisted

The straightforward mental model is to build the toothed rack flat, with its sinusoidal edge as one
spline in one sketch and one extrude, and then twist the solid. Fusion's parametric environment has
no twist feature for a solid body. `SweepFeatureInput` does document a solid twist axis, but a solid
sweep sweeps the body's *volume* along the path, which smears each tooth into a continuous ridge and
destroys the very feature being built. A profile sweep with `twistAngle` builds the exact twisted
blank in one feature, and is what the bores are (`[SCREW-F-TWISTED-SLOT]`), but
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

## `[SCREW-F-BORE-MARKS]` — raised signs on the sleeve

Follow "Bore identification marks" in `instructions.md` after the bore and window cuts. Offset
one plane from Gear B Axis Plane to 0.1 mm or less inside the top end. Use one fully constrained
sketch and one new-body extrusion per mark. Map the mark centre plus `nHat` to the sketch to
choose the extent direction that rises above
the top face; the selected plane can orient its local normal either way. Each extrusion begins
inside solid end-wall material, rises 0.4 mm above the face, and joins only to the cage with
`[SCREW-F-JOIN]`. Check the plane offset, each profile and feature body count, and each join's
single body in Fusion. The bottom end stays flat for printing. The marker proof checks the
sketches and a marked uncut sleeve; the full sleeve surface and render still omit the marks.
