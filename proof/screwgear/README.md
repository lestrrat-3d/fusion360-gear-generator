# The screw gearing, in pictures

These are the parts [`spec/screwgear/instructions.md`](../../spec/screwgear/instructions.md)
describes, drawn at the defaults that spec's table gives. Every section in every picture comes
from `Gear.section` in [geometry_test.go](geometry_test.go), the same function the meshing proof
samples, so a change that moves the proved geometry moves these pictures with it. The three
`TestRender` cases in [render_test.go](render_test.go) write them, and they run only when
`-render.out` names a directory, so an ordinary proof run writes no images.

The mechanism is a screw/screw gearing: two racks, each twisted into a helix about its own centre
line, each moving by a screw motion — turning as it advances — in a cage that holds them. Pushing
one along its axis drives the other, 1:1. It comes from Henry Segerman's video
[Screw/screw gearing](https://www.youtube.com/watch?v=0ZPo3HxR0KI), which is the only known
example of one.

## What the pictures show, and what they do not

No image here comes from Fusion, and nothing in this directory builds a Fusion body. Loading a
gear into Fusion is still the only check that sees the real thing.

The pictures draw the **ideal** ribbon: an exact cosine edge on an exact helicoid, which is what
the proof reasons about. The part Fusion builds lofts nine rectangles per tooth, and
`TestLoftSectionCountHoldsTheHelicoid` bounds the difference between the two at 0.6 µm against a
0.25 mm backlash. That bound is arithmetic about the loft rather than a measurement of one.

The cage's slots are drawn on a grid: a cell of the tube's wall is dropped when its own midpoint
falls inside a gear's clearance rectangle, which is the same local mapping the meshing proof uses.
Each opening's long edges are then walked onto the true cut line, but its two short ends still
step at the grid, because a boundary running across the rows cannot be fixed by moving a vertex
around the tube. Those steps are in the drawing and not in the geometry.

## The part

![A twisted toothed rack, its wavy edge spiralling once along its length](images/part.png)

One gear. It is a flat plate 10 mm wide and 2.5 mm thick with a cosine tooth form cut into one
long edge, 24 teeth at a 3.5 mm pitch, twisted about its own centre line at 90 mm per turn. The
toothed edge is the one that spirals: it faces the camera at the left, turns away through the
middle, and comes back at the right, which is a little over one full turn across the 84 mm.

The teeth are 1.2 mm from crest to root, a little over a tenth of the plate's width, which is why
they read as a ripple rather than as gear teeth at this scale.

## The pair

![Two ribbons crossing, passing through the cage tube between them](images/plan.png)

Both gears, seen from almost overhead, which is the only view that shows the angle their axes
cross at. That angle is 38.5°, and it is not free: the crossed-helical rule makes it twice the
toothed edge's own helix angle, which the plate's width and the twist lead fix between them.

![The same pair from the side](images/pair.png)

The same assembly from the side. The two axes are 9.28 mm apart, measured along the cage's axis,
and the two gears are the same part rather than mirror images.

## The frame

![The cage tube with two twisted slots cut through the near wall](images/cage.png)

The frame on its own, looked at straight down gear A's axis. Neither gear is drawn, because a
ribbon on this line of sight fills the frame — it is exactly what the opening is cut to pass.

**The slots are what make this a frame rather than a pair of bearings.** A round hole would let
its ribbon turn freely as it slid, and the mechanism would have three degrees of freedom instead
of one; a slot shaped like the ribbon's own cross-section forces the ribbon to turn as it
advances, the way a twisted-bar screwdriver does. Each slot is a twisted channel of the same lead
as its gear, because the ribbon turns as it crosses the wall — which is why the two openings here
lean, and why a straight slot would bind.

A ribbon crossing the tube in front of the wall hides part of it, which reads in the assembly
pictures like a ribbon passing through solid material. `TestRibbonsPassThroughTheirSlots` is what
settles that: it walks both parts end to end and asserts neither ever meets the wall.

Two openings are in view and two more are cut in the far wall, one per gear per side. They come
close to each other at the middle of the tube, where the gears mesh, and the wall between them is
what the cage cases are there to protect. Below 19.25 mm across, each gear's two piercings
reach around and join into one opening, and the wall is left standing on two arms; below 7 mm the
slots cut the tube into pieces and it stops being a frame at all.

## The mesh

![The two toothed edges meeting, crests of one in the roots of the other](images/mesh.png)

The crossing, close up. The crests of the lower gear sit in the roots of the upper one, engaged
0.72 mm of the 1.2 mm tooth height. This is the picture that settles whether the teeth engage at
all, which is the one thing about this gear that no number on a page shows.

![The same engagement seen from across it](images/mesh-across.png)

The same engagement from across the crossing. The two tooth rows run at an angle to each other
rather than along one line, so what this pair carries is a **point contact**, like a crossed
helical pair, and not the line contact a spur pair has. That angle is also what the Mounting
Angle is there to work around: with both toothed edges pointing straight at each other, two or
three tooth pairs land in the engaged zone at once and cannot all interdigitate, and the pair
jams. `TestSymmetricMountJams` holds that.

## What the proof measures on these parts

| Quantity | Value |
|---|---|
| Ratio | 1:1, the free window advancing exactly one pitch per pitch |
| Backlash | 0.21–0.26 mm |
| Departure from the 1:1 line | 0.058 mm, 1.7% of the pitch |
| Clearance away from the teeth | 0.081 mm at the closest approach |
| Engaged zone | ±6 mm, about 3.4 tooth pairs |
| Smallest cage tube that stays one body | 7 mm across |
| Smallest tube with four distinct openings | 19.25 mm across, against a 24 mm default |

`TestPairDrivesOneToOne` is where the first four come from. It tracks the interval of gear B's
tooth phase that clears gear A through a full pitch of A, and requires three things of it: that
it is never empty, that it is narrower than a pitch, and that its centre advances by exactly one
pitch. The third is what separates a gear from two parts that merely touch, and an arrangement
tried earlier passed the first two and failed it.

## What has no picture

**The tooth cell and its screw step.** The spec builds a ribbon as one tooth cell repeated by a
screw step, and `TestRibbonIsInvariantUnderItsScrewStep` is what licenses that, but the pictures
draw the whole ribbon from its sections and never form a cell.

**Anything a generator does.** There is no `lib/geargen/screwgear.py` yet, and no compiled step
list, so there is no build sequence to show step by step the way
[`proof/bevelgear/README.md`](../bevelgear/README.md) does.

## Regenerating

```sh
cd proof
GOWORK=off go test ./screwgear -run '^TestRender' -render.out=./images -count=1
```

`./images` is relative to this directory rather than to the one the command is run from, because
`go test` runs a test binary in its own package's directory.

`GOWORK=off` is what [render_examples.sh](../render_examples.sh) uses and for the same reason:
the images are then built against the engine revisions `proof/go.mod` pins, out of the module
cache, rather than against whatever checkout sits beside the repository. `-count=1` is needed
because a cached PASS writes no files.

Nothing checks these images. They are regenerated by hand when the geometry changes.
