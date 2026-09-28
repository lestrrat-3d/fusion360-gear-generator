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
the proof reasons about. The part Fusion builds is a four-tooth cell lofted through 41 rotated
rectangles, ten to the tooth, repeated by a screw step, and
`TestLoftSectionCountHoldsTheHelicoid` bounds how far a *ruled* loft through those sections
would fall from the ideal: 1.0 µm at the crest, where a ruled surface cuts the corner of the
helicoid, and 0.064 mm on the toothed edge, which it draws as a chord of the cosine between
sections, against a 0.82 mm backlash. Fusion's loft through more than two sections is smooth
between them rather than ruled, so those bounds are arithmetic about a body other than the one
built; the built surface passes through the same sections, and nothing here measures how far it
departs between them. Fusion measured it on 2026-09-28: within 0.04 mm at every midpoint
between sections, at the 10 mm ribbon the defaults were then (`spec/screwgear/fusion.md`
`[SCREW-F-CELL-LOFT]`); the 1.5× ribbon drawn here has not been loaded yet.

The frame's parts are drawn one by one and laid over each other rather than joined: the ring as a
torus, the rods and the loop's bars as plain cylinders with a ball at each corner, and each collar
as the bore's outline grown by the wall and swept through thirty-two stations along its ribbon.
Where a rod runs into a collar's wall the picture shows both surfaces, and the part has one.

The channel's wall drawn here is the ideal, and so is the one Fusion cuts: each collar and each
bore is one sweep of its section along the axis with a twist, a rectangle turning rigidly, with
no sections and no facets. The proof's solid engine has no twisted sweep, so the compiled step
proof stands in a ruled loft through thirteen rotated rectangles for each; `TestBoreSubstituteKeepsItsClearance`
measures what that stand-in costs against the true channel, with the wall flat between sections
and every facet a little inside it: 0.007 mm of the 0.45 mm clearance. It is the one case in the
hand-written proof that reasons about a stand-in rather than about the ideal shape, and it is
what says a measurement made on the stand-in holds for the swept channel to within that much.

## The part

![A twisted toothed rack, the same plain section from end to end, its wavy edge spiralling along its length](images/part.png)

One gear. It is a flat plate 15 mm wide and 3.75 mm thick with a cosine tooth form cut into one
long edge, 68 teeth at a 2.625 mm pitch, twisted about its own centre line at 49.5 mm per turn —
three and a half full turns across its 178.5 mm. It is the same twisted rack from end to end, with
nothing added where it passes through the frame, as in the video.

The toothed edge is the one that spirals, and the twist is what the crossing angle is made of:
`Sigma = 2*Beta` ties the two axes' angle to this edge's own helix angle, so a slower twist would
give a straighter part AND a pair whose axes lie almost side by side.

The teeth are 2.625 mm from crest to root, one pitch and 0.175 of the plate's width, which is
the depth the model in the video reads (0.15–0.2 widths, about one pitch). They were 1.2 mm on a
10 mm plate until 2026-09-28, when a print at that size showed teeth far too small;
the spec's "What the print showed" records the verdict and the 1.5× defaults that came of it.

## The pair

![Two ribbons crossing, with the frame's cage between them](images/plan.png)

Both gears, seen from almost overhead, which is the only view that shows the angle their axes
cross at. That angle is 80°. The crossed-helical rule would make it twice the toothed edge's helix
angle, which is 87.2° here, and the search picks 80° instead: at 90° the pair departs from the
1:1 line by 0.195 mm, 7.4% of the pitch, against the proof's bound of 6% of the pitch.

![The same pair from the side](images/pair.png)

The same assembly from the side. The two axes are 14.25 mm apart along the frame's own axis, and the
two gears are the same part, held at the same 15° angle, which is the only equal angle that
drives at this twist.

## The frame

![The frame with one gear through it: a round ring on top, a rectangular loop underneath, four rods between them and a twisted collar round the ribbon at each crossing, with the ribbon's teeth running through the collar](images/cage.png)

The frame with one gear left in it. It is the frame in the video: an open skeleton of round rods,
with no wall, no plate and no post. A round wire **ring** stands at one end and a smaller **loop**
with straight sides at the other, four thin **rods** run between them, and a short **collar** sits
round each ribbon where it crosses. The ring is 37.5 mm across, 2.5 ribbon widths, and the frame
is 44.25 mm tall; the spec's "What the video shows" table puts both against the video's readings.

**The collar is what holds a gear to its screw motion.** Its bore is the ribbon's crest rectangle
plus a 0.45 mm clearance all round, 15.9 by 4.65 mm, twisted at the ribbon's own lead, so a gear
that turns without advancing jams in it. `TestBoresAdmitOnlyTheScrewMotion` measures that: 3.55°
out of step and the gear's crests and back corners lock. A frame of round holes would report no
jam at any angle, which is the case that rules out. The collar's outside is that bore grown by a
3 mm wall in every direction of its own section, which rounds every corner off, and it turns with
the ribbon over its 6 mm length. Two collars sit low and two high, so the two gears pass at
different heights and their teeth meet in the middle.

**A rod stands beside its collar, not on it.** The ribbon runs on through the point where it
crosses the frame, so a rod there would cut it. Each rod stands on the ring's circle, turned round
the ring counter-clockwise, seen from the ring's end, by the least angle at which it clears both
ribbons at every phase of the travel — 34.6° for a gear's collar at `-CageRadius` and 34.4° for
the one at `+CageRadius` — and runs through its collar's 3 mm wall, which is what joins the two.
All four are turned the same way round, so they
land in the gaps between the ribbons, and the loop that joins their feet has a corner at each
rod: 25.89 and 25.82 mm by 21.69 mm inside a 33.75 mm circle, which is why the loop reads smaller
than the ring. It is an isosceles trapezoid rather than a rectangle, because the rods for the
collars at −CageRadius and at +CageRadius turn by angles 0.19° apart, and the two long sides
share that difference.
`TestRodsStandBesideTheirCollars` derives the angles and `TestFrameIsOnePiece` walks ring to rods
to loop and rod to collar.

**Nothing of the frame touches a ribbon but the bore.** `TestRibbonsClearTheFrameOverTheTravel`
walks everything both ribbons reach at any phase of the travel against the ring, the loop, the
rods and the collars: 4.10 mm to the ring, 5.69 mm to the loop, 0.45 mm to the rods, which is
the clearance itself, because each rod stands at the least angle that clears.

**The teeth run through the collars.** The crest of a cosine rack is the ribbon's outer edge, so
the crest rectangle holds every point of the ribbon and the bore needs nothing shaped like a
tooth: the frame is plain round bar and plain rectangular openings. What bears on the bore's
toothed side is the crests, one every pitch; the back edge and the faces run the clearance from
the wall at every station. `TestRibbonsStayInsideTheirBoresOverTheTravel` holds every point of
the ribbon inside every bore by the clearance at every phase of the travel, and holds each side
of the bore to the clearance, so the bore is cut to the ribbon and not merely round it.

**The travel is most of the ribbon.** Nothing on the ribbon limits it, since any stretch of the
ribbon fits a collar; a gear runs until an end reaches a collar's far face, which is 142.5 mm for a
ribbon in its own two collars, and 141.2 mm for the pair, 53.8 teeth, because gear B sits 1.31 mm
along its axis at the assembly phase and reaches one collar that much sooner. The teeth are still
meshing at both ends of that; an end would leave the engaged zone at 80.8 mm.
`TestTravelIsTheRibbonBetweenItsCollars` walks both limits. In the video, at 5:53 the frame sits
near one end of a ribbon where at 6:00 it sits near the middle, so that gear too travels most of
its length.

## The mesh

![The two toothed edges meeting, crests of one in the roots of the other](images/mesh.png)

The crossing, close up. The crests of the lower gear sit in the roots of the upper one, engaged
0.75 mm of the 2.625 mm tooth height. This is the picture that settles whether the teeth engage at
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
| Backlash | 0.761–0.892 mm |
| Departure from the 1:1 line | 0.063 mm, 2.4% of the pitch |
| Slack between the ribbons at the assembly phases | 0.484 mm at the closest approach, in the mesh, with gear B in the middle of a 0.762 mm window |
| Bore | 15.9 by 4.65 mm, the crest rectangle plus 0.45 mm all round; the ribbon at 0.450 mm on every side over the travel |
| Play in the frame | a collar jams a gear 3.55° out of step |
| Travel | 141.2 mm, 53.8 teeth, 79% of the ribbon: 69.9 mm back and 71.3 mm forward of the assembly position |
| Ring | 37.5 mm across, 2.5 ribbon widths and 0.76 leads; the frame 1.18 ring widths tall |
| Rods | 34.61° (at −CageRadius) and 34.43° (at +CageRadius) round the ring from their collars, 10.0 mm from the crossings, crossing the ribbon at stations 13.89 and 13.92 mm, their axes 1.49 and 1.38 mm outside the bore inside the 3 mm wall |
| Loop | 25.89 and 25.82 mm by 21.69 mm between the rods' feet, an isosceles trapezoid |
| Frame to ribbon | 4.10 mm at the ring, 5.69 mm at the loop, 0.45 mm at the rods, over the travel |
| Ribbon to ribbon outside the engaged zone | 1.96 mm at every phase of the travel, at station −7.14 mm, crest rectangle against crest rectangle |
| Bore wall | the swept channel is the exact helicoid; the proof's ruled stand-in through thirteen sections leaves 0.443 mm of the 0.45 mm clearance, so a measurement on it holds for the sweep to within 0.007 mm |

`TestPairDrivesOneToOne` in [pair_test.go](pair_test.go) is where the first three come from, and
`TestFullRibbonsClearOutsideTheEngagement` beside it gives the fourth. It tracks the interval of
gear B's tooth phase that clears gear A through a full pitch of A, and requires three things of
it: that it is never empty, that it is narrower than a pitch, and that its centre advances by
exactly one pitch. The third is what separates a gear from two parts that merely touch, and an
arrangement tried earlier passed the first two and failed it. The sampling every number in the
table was taken with is fixed at the top of that file and quoted in the spec's "Defaults", and
so are its bounds, which are fractions of the pitch: the departure under 6% of it and the window
within 7% of it of the backlash quoted above.

## What has no picture

**The tooth cell and its screw step.** The spec builds a ribbon as one four-tooth cell repeated
by a screw step, five copy-move-join rounds at the defaults, and
`TestRibbonIsInvariantUnderItsScrewStep` is what licenses that: it carries the corners of every
cross-section through one step and requires them to land on the next tooth's.
`TestDoublingScheduleCoversTheRibbon` holds the schedule. The pictures draw the whole ribbon from
its sections and never form a cell.

**Anything a generator does.** There is no `lib/geargen/screwgear.py` yet, and no compiled step
list, so there is no build sequence to show step by step the way
[`proof/bevelgear/README.md`](../bevelgear/README.md) does. The four files named here —
[geometry_test.go](geometry_test.go), [pair_test.go](pair_test.go), [cage_test.go](cage_test.go)
and [render_test.go](render_test.go) — are the hand-written mechanism proof; `/compile-gear`
writes the step proof beside them and leaves them alone.

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
