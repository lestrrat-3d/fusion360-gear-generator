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
`TestLoftSectionCountHoldsTheHelicoid` bounds the difference between the two at 0.7 µm against a
0.28 mm backlash. That bound is arithmetic about the loft rather than a measurement of one.

The collars' openings are drawn on a grid: a cell of the disc is dropped when it falls in the
channel the turning ribbon sweeps, which is the same local mapping the meshing proof uses. Each
opening's edges are then walked onto the true cut line, though a boundary running across the grid's
rows can still step. Those steps are in the drawing and not in the geometry.

## The part

![A twisted toothed rack, its wavy edge spiralling twice along its length](images/part.png)

One gear. It is a flat plate 10 mm wide and 2.5 mm thick with a cosine tooth form cut into one
long edge, 48 teeth at a 1.75 mm pitch, twisted about its own centre line at 40 mm per turn — a
little over two full turns across its 84 mm.

The toothed edge is the one that spirals, and the twist is what the crossing angle is made of:
`Sigma = 2*Beta` ties the two axes' angle to this edge's own helix angle, so a slower twist would
give a straighter part AND a pair whose axes lie almost side by side.

The teeth are 1.2 mm from crest to root, a little over a tenth of the plate's width.

## The pair

![Two ribbons crossing near a right angle, with the frame's two rings at their centre](images/plan.png)

Both gears, seen from almost overhead, which is the only view that shows the angle their axes
cross at. That angle is 76.3°, and it is not free: the crossed-helical rule makes it twice the
toothed edge's own helix angle, which the plate's width and the twist lead fix between them.

![The same pair from the side](images/pair.png)

The same assembly from the side. The two axes are 9.40 mm apart along the frame's own axis, and the
two gears are the same part rather than mirror images — what differs is the angle each is held at,
30° for one and 0° for the other.

## The frame

![The two collars of the frame, each threaded on one ribbon and fused to the other](images/cage.png)

The frame on its own: two collars, one threaded on each gear, fused where their rims overlap. Each
collar is a disc standing across its gear's axis with an opening cut to that ribbon's own
cross-section.

**The opening is what makes this a frame rather than a pair of bearings.** A round hole would let
its ribbon turn freely as it slid, and the mechanism would have three degrees of freedom instead of
one; an opening shaped like the ribbon forces the ribbon to turn as it advances, the way a
twisted-bar screwdriver does. `TestCollarAdmitsOnlyTheScrewMotion` is the measurement behind that
claim: it turns a gear out of step with its own advance and finds that it jams 3.55° later.

The opening is a **twisted** channel, not a straight hole. The ribbon turns while it is inside the
collar — 13.5° over the default 1.5 mm of depth — and a straight hole would not pass it at all.

The two rims overlap by 2 mm, which is what makes the frame one body. Their centres stand 14 mm
apart, set by where each collar sits along its own gear: far enough out to clear the ±5.3 mm the
gears engage over, close enough in that the rings still meet.

## The mesh

![The two toothed edges meeting, crests of one in the roots of the other](images/mesh.png)

The crossing, close up. The crests of the lower gear sit in the roots of the upper one, engaged
0.60 mm of the 1.2 mm tooth height. This is the picture that settles whether the teeth engage at
all, which is the one thing about this gear that no number on a page shows.

![The same engagement seen from across it](images/mesh-across.png)

The same engagement from across the crossing. The two tooth rows run at an angle to each other
rather than along one line, so what this pair carries is a **point contact**, like a crossed
helical pair, and not the line contact a spur pair has. That angle is also what the Mounting
Angles are there to work around: with both toothed edges pointing straight at each other, about
four tooth pairs land in the engaged zone at once and cannot all interdigitate, and the pair jams.
`TestSymmetricMountJams` holds that.

## What the proof measures on these parts

| Quantity | Value |
|---|---|
| Ratio | 1:1, the free window advancing exactly one pitch per pitch |
| Backlash | 0.245–0.298 mm |
| Departure from the 1:1 line | 0.047 mm, 2.7% of the pitch |
| Clearance away from the teeth | 0.136 mm at the closest approach |
| Engaged zone | ±5.3 mm, about 4 tooth pairs |
| Play in the frame | the collar jams a gear 3.55° out of step |
| Frame rim overlap | 2 mm, on centres 14 mm apart |

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
