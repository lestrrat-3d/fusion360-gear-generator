# The bevel gear, one step at a time

These pictures are the gear the command builds, taken step by step. Each heading carries the
number [`spec/bevelgear/steps.md`](../../spec/bevelgear/steps.md) gives that step. The solid
pictures are meshed by [render_test.go](render_test.go)'s model, the one that draws the bevel
picture in the repository README; the lattice is the sketch `stepGearProfiles` builds, which is
the sketch the generator builds in Fusion. `TestStepSnapshots` in
[snapshots_test.go](snapshots_test.go) writes them all, and it runs only when `-snapshot.out`
names a directory, so an ordinary proof run writes no images.

Every picture is of one gear: the shipped dialog default with the Mean Spiral Angle at 0, which
is module 1, an equal 31/31 pair at a 90 degree shaft angle, taken on the pinion — the member
the generator builds first. The last picture is the pair.

Each picture is the gear as that step leaves it, so the sequence is cumulative. Where a step
adds a body the new body is drawn in the lighter shade until the Combine-Join, after which the
gear is one body and takes one colour.

## What the pictures show, and what they do not

No image here comes from Fusion. Loading a gear into Fusion is still the only check that sees
the real thing, and it is the one this repository cannot run.

The picture model does four things the geometry proof in this directory cannot. It chords
the Gear Body's revolve at 240 segments where the proof sweeps a 32-sided polygon. It trims the
tooth on the two cones the body's toe and heel faces lie on, computing each section's trim
station in closed form. It patterns all 31 teeth. It takes the bore out of the revolve profile
before sweeping it. No body is moved aside from where the gear puts it.

One thing it simplifies. Fusion draws the virtual spur tooth on the `{gearLabel} Plane`, which
is tilted from the axis-perpendicular plane by the pitch cone angle, and lofts to it from the
Apex point. These pictures put that same tooth outline on axis-perpendicular sections scaled
about the Apex, which is the Tredgold mapping the tooth's own construction already applies. The
tooth's size, its curve inventory, its taper and both conical trims are the real ones; the tilt
of the plane it is drawn on is not.

## S10 — Gear Profiles sketch, the section 2 lattice

![The lattice: construction lines between the section 2 points](images/s10-gear-profiles.svg)

The step draws the whole lattice in the axial plane. The orange line at the left is S8's Anchor
Line, projected into this sketch as reference geometry. The dashed lines are construction
geometry, and the red points are the vertices both gears' profiles are later recreated from,
each labelled with the letter
[`spec/bevelgear/instructions.md`](../../spec/bevelgear/instructions.md) calls it by.

Two things the sketch holds are left off the picture. The thirteen dimensions that hold the
lattice are not drawn, because thirteen labels inside one 30 mm figure overlap into a block of
text. The step also gives every construction line a name like `Apex->B` and each of its
endpoints one like `Apex->B.start`. Those hundred names are cleared on the picture's own copy of
the sketch before it is drawn.

## S16 — Revolve the hexagon into the Gear Body

![The revolved gear body, a shallow dish seen from above the toe](images/s16-gear-body.png)

The hexagon is spun a full turn. The dish facing the camera is the toe end, the end the teeth
taper to; the heel is the wide face underneath. The bore is not in it yet — S30 draws it and
S31 cuts it.

## S17 — Loft the Apex point to the tooth profile

![One long tooth running from the Apex across the gear body and out past it](images/s17-tooth-loft.png)

One tooth is lofted from the Apex point to the tooth profile at the heel. It is drawn in the
lighter shade because it is a separate body until the Combine-Join, and the camera stands on its
side of the gear. Both of its ends run past the Gear Body, which is what S18 cuts back: the
Apex end carries on past the toe into open space, and the heel end stands out past the body's
rim.

## S18 — Conical end trims, the flush band

![The same tooth cut flush with the gear body at both ends](images/s18-conical-trims.png)

The same tooth is trimmed by the two cones the Gear Body's own toe and heel faces lie on. What
is left is the flush band: the tooth now starts and ends where the body does, and its two end
faces are conical rather than flat, because the surfaces that cut them are cones.

## S28 — Circular pattern

![The gear body carrying all 31 teeth](images/s28-circular-pattern.png)

That one tooth is patterned into all 31 of them, 360/31 degrees apart. The spacing stays the
same the whole way along the face width even though the pitch diameter shrinks toward the toe,
because the taper is already in the lofted tooth.

## S29 — Combine-Join

![The same gear in a single colour](images/s29-combine-join.png)

The same geometry is drawn in one colour. Nothing moved between this picture and the last, and
the colour is the whole difference, because the join changes which bodies the gear is made of
and not its shape.

## S31 — Bore through-cut

![The finished single gear, seen down the shaft axis, with the bore through it](images/s31-bore-cut.png)

The bore is cut along the shaft axis, through the whole body. The camera stands steeper for this
one picture, because the bore comes out in the floor of the toe dish and the dish's own rim
hides nearly all of it from the viewpoint the rest of the sequence uses. The diameter is
7.75 mm, which is what S30's "0 means auto" branch resolves to: this gear's pitch diameter
divided by four.

## S32 — Meshing rotation

![The finished pair meshing at a right angle](images/s32-meshing-rotation.png)

The meshing rotation shows only in the pair. The driving gear is turned half a tooth pitch about
its own shaft, so that its valley meets the pinion's tooth rather than tooth meeting tooth. Both
members come out of the one case, exactly as the command builds them.

## Steps with no picture

**S19 to S27, the spiral path.** At Mean Spiral Angle 0 the command does not run them at all:
the tooth-body hook returns the conical end trims of S18 and no trace, no slices, no twist and
no crown are built. A spiral bevel builds its tooth through those steps instead, and nothing
here shows that.

**S8, S12, S15 and S30, the other sketches.** The Anchor Line is a line, the frustum hexagon is
the outline S16 revolves and then shows in the round, the bore is a circle, and the tooth
section is a 2.25 mm tooth in a 16 mm frame. The lattice is the one sketch drawn here, because
every length the gear is built from is in it and no solid picture shows those lengths.

**The steps that build no geometry.** S1 to S4 and S6 are the module layout, the command dialog,
the conditional inputs and the input reading. S5 resolves the derived values and the input
bounds as numbers. S7, S9, S11, S13, S14, S33 and S34 create components, planes, axes and the
occurrence tree, or move the finished body and clean up.

## Regenerating

```sh
cd proof
GOWORK=off go test ./bevelgear -run '^TestStepSnapshots$' -snapshot.out=./images -count=1
```

`./images` is relative to this directory rather than to the one the command is run from, because
`go test` runs a test binary in its own package's directory.

`GOWORK=off` is what [render_examples.sh](../render_examples.sh) uses and for the same reason:
the images are then built against the engine revisions `proof/go.mod` pins, out of the module
cache, rather than against whatever checkout sits beside the repository. `-count=1` is needed
because a cached PASS writes no files.

Nothing checks these images. They are regenerated by hand when a step changes what it builds,
and the test that writes them fails to compile if a step is renamed or removed.
