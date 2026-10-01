# The screw gearing, in pictures

These are the parts [`spec/screwgear/instructions.md`](../../spec/screwgear/instructions.md)
describes, drawn at the defaults that spec's table gives: the ribbons and the printable sleeve that
holds them. Every ribbon section in every picture comes from `Gear.section` in
[geometry_test.go](geometry_test.go), the same function the meshing proof samples, and the
sleeve's surface comes from `sleeve.inFrame` in [sleeve_test.go](sleeve_test.go), the same test
the sleeve's proof walks, so a change that moves the proved geometry moves these pictures with it.
The three `TestRender` cases in [render_test.go](render_test.go) write them, and they run only
when `-render.out` names a directory, so an ordinary proof run writes no images.

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

The sleeve is a tube with four holes and two windows cut through it, and the pictures have no
boolean to cut them with. `sleeveMesh` in [render_test.go](render_test.go) meshes the surface of
the set `sleeve.inFrame` describes, by marching tetrahedra on a 0.2 mm grid, and places every
vertex on that surface by bisection, so the holes drawn are the channels the proof walks. The
grid leaves its own marks: the sharp rims come out bevelled across up to one grid cell, and the
holes' twisted walls show short dark dashes where the grid cuts them into small triangles at odd
angles.

The channel's wall drawn here is the ideal, and so is the one Fusion cuts: each bore is one sweep
of its section along the axis with a twist, a rectangle turning rigidly, with no sections and no
facets. The proof's solid engine has no twisted sweep, so the compiled step proof stands in a
ruled loft through eighteen rotated rectangles for each bore.
`TestSleeveBoreSubstituteKeepsItsClearance` measures what that stand-in costs against the true
channel, with the wall flat between sections and every facet a little inside it: 0.007 mm of the
0.45 mm clearance. It is the one case in the hand-written proof that reasons about a stand-in
rather than about the ideal shape, and it is what says a measurement made on the stand-in holds
for the swept channel to within that much.

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

![Two ribbons crossing inside the sleeve, seen from almost overhead](images/plan.png)

Both gears in the sleeve that "The frame" below describes, seen from almost overhead, which is the
only view that shows the angle their axes cross at. That angle is 80°. The crossed-helical rule
would make it twice the toothed edge's helix angle, which is 87.2° here, and the search picks 80°
instead: at 90° the pair departs from the 1:1 line by 0.195 mm, 7.4% of the pitch, against the
proof's bound of 6% of the pitch.

![The same pair in the sleeve, from the side](images/pair.png)

The same assembly from the side. The two axes are 14.25 mm apart along the frame's own axis, and the
two gears are the same part, held at the same 15° angle, which is the only equal angle that
drives at this twist.

## The frame

The frame was first the one in the video, an open skeleton of thin round rods. On 2026-09-30 the
user reported that it is not practical to print: printed several times, its thin parts wobbled
while the printer laid them down. The sleeve replaced it. The four bores did not change, because
they hold each gear to its screw motion, and the mesh still has to be visible from the top and
the bottom. [sleeve_test.go](sleeve_test.go) proves the sleeve.

![The sleeve alone: a thick tube standing on one flat end, with two twisted rectangular holes through its wall, one high on the left and one low on the right, and the edge of a slanted window at the right](images/sleeve-frame.png)

The sleeve is one thick-walled tube about the frame's axis with the four bores and two slanted
windows cut through its wall, and nothing else. Its inner radius is `CageRadius − CollarHalf`,
12 mm, and its outer radius `CageRadius + CollarHalf`, 18 mm, so on each bore's centre line the
6 mm wall runs from station 12 to station 18 mm of the gear's axis, and the far face of the bore
at 18 mm is what limits the travel. It stands 18.75 mm either side of the middle plane, 37.5 mm
tall: its `CageRise` is 18.75 mm, 1.25 ribbon widths.

Each bore is the ribbon's crest rectangle plus 0.45 mm all round, 15.9 by 4.65 mm, turned with
the ribbon at its own lead. The tube's inside is a cylinder, not a plane square to the ribbon, so
the channel's corners reach into the wall before its centre line does. The cut runs from 1 mm
before a corner first touches the inner cylinder, station 7.68 mm of the gear's axis, to 1 mm
outside the tube, station 19 mm, and turns 82.3° over those 11.3 mm.

![One gear through the sleeve: the ribbon enters through a hole on the left and leaves through one on the right, with gear B's upper hole facing the camera](images/sleeve-cage.png)

One gear passes through the sleeve by its two holes, which sit on opposite sides of the tube at
the gear's own height, 7.125 mm below the middle for gear A and as far above it for gear B, so
the two gears pass at different heights and their teeth meet in the middle. The two gears' holes
are the same either way up: a half turn about the line that bisects the two axes carries gear A's
holes onto gear B's.

**The bores hold each gear to its screw motion.** Each bore is twisted at the ribbon's own lead,
so a gear that turns without advancing jams in it. `TestSleeveAdmitsOnlyTheScrewMotion` measures
that: 3.55° out of step and the gear's crests and back corners lock. A frame of round holes would
report no jam at any angle, which is the case that rules out.

**The teeth run through the bores.** The crest of a cosine rack is the ribbon's outer edge, so
the crest rectangle holds every point of the ribbon and the bore needs nothing shaped like a
tooth. What bears on the bore's toothed side is the crests, one every pitch; the back edge and
the faces run the clearance from the wall at every station.
`TestRibbonsStayInsideTheirBoresOverTheTravel` holds every point of the ribbon inside every bore
by the clearance at every phase of the travel, and holds each side of the bore to the clearance,
0.450 mm, so the bore is cut to the ribbon and not merely round it.
`TestRibbonsClearTheSleeveOverTheTravel` walks both ribbons' crest rectangles, grown by the
clearance, over the whole travel and finds no point of them in the sleeve; outside the bores'
cuts the ribbons keep 1.00 mm from the tube.

**The travel is most of the ribbon.** Nothing on the ribbon limits it, since any stretch of the
ribbon fits a bore; a gear runs until an end reaches a bore's far face, which is 142.5 mm for a
ribbon in its own two bores, and 141.2 mm for the pair, 53.8 teeth, because gear B sits 1.31 mm
along its axis at the assembly phase and reaches one bore's end that much sooner. The teeth are
still meshing at both ends of that; an end would leave the engaged zone at 80.8 mm.
`TestTravelIsTheRibbonBetweenItsBores` walks both limits. In the video, at 5:53 the frame sits
near one end of a ribbon where at 6:00 it sits near the middle, so that gear too travels most of
its length.

**The mesh is seen down the hollow.** The two ribbons cross and mesh in the middle of the tube,
and the tube is open at both ends.

![Looking straight down the hollow from the top end: gear B's ribbon crosses it on edge, and beside it gear A's teeth sit in gear B's](images/sleeve-top.png)

![The same view from the bottom end, where gear A's ribbon is the nearer one](images/sleeve-bottom.png)

`TestSleeveKeepsTheMeshVisibleAlongTheAxis` projects everything either ribbon reaches within the
engaged zone onto the plane square to the axis. That footprint reaches 10.43 mm from the axis,
11.21 mm once grown by the clearance, inside a hollow of radius 12 mm, and the test follows the
line along the axis through every point of it from one end of the sleeve to the other without
meeting material. The build refuses an input whose footprint and clearance would reach the inner
radius; `TestSleeveInputsAreChecked` reaches that refusal with a 1.5 mm engagement.

**The mesh is also seen from the side, through two slanted windows.**

![The sleeve seen square on to one window: a long six-sided slot leaning at 45 degrees through the wall, with the matching window on the far side showing through it, and the mouths of two bores at either edge](images/sleeve-window.png)

![The same view with both ribbons in place: through the window, gear B's teeth run down the upper edge and gear A's along the lower, meeting in the middle](images/sleeve-side.png)

The four bores sit round the tube at 40°, 140°, 220° and 320°. Across the +X and −X sides the
neighbouring bores are 80° apart and their channels come within 6.81 and 4.83 mm of each other,
which leaves no room for a window with a 3 mm `CollarWall` on both sides of it. Across +Y and −Y
they are 100° apart, one bore low and the other high, and the wall between them is a band that
runs at about 45° from above the low bore down to below the high one. Each window is cut along
that band, one facing +Y and one facing −Y; the −Y window is the +Y window turned half a turn
about X, as the bores are.

A window is a six-sided hole drawn on the plane through the frame's axis square to the direction
it faces and pushed straight out through the wall on that side. Its two long sides are 45° lines
6.95 mm apart, 9.83 mm measured up the axis. Its two ends are upright, 23.07 mm apart across the
window. Two 45° lines trim its long corners. Every edge is upright or at 45°, so standing on
either end the printer lays the window's roof on the layer below it and bridges nothing; that is
why the window is slanted rather than a level slot, whose roof would be a bridge.

Every dimension comes from the sleeve's own geometry. `newWindow` in
[sleeve_test.go](sleeve_test.go) finds how far each of the two flanking bores' channels reaches
across the band, where it lies in the wall, and sets the long sides a `CollarWall` beyond. It
stands each end as far out as it can while the end's corner lines through the wall, and its
upright edges on the inner and the outer face, stay a `CollarWall`, plus 0.076 mm for what the
sampling can miss, from the other two bores. At the defaults the corners are what hold the ends;
at a 0.2 mm clearance the corners alone would have let an end's inner edge come 2.60 mm from a
bore, and the edges are what hold them. No end stands further than `SleeveOuter/√2` from the
middle, which keeps its roofs off the outer face at the angle the next paragraph describes for
the inner one. The trims keep the window within the 15.01 mm of the middle
the channels already reach, so both end bands keep the 3.74 mm end wall. They also keep each long
side from meeting the inner face more than 45° round from the window's facing direction. Where a
45° roof meets the curved inner face, the line they meet on descends more gently than the roof,
and past 45° round it is flatter than the 0.25 mm grid's 45° rule accepts.

`TestSleeveWindowsKeepTheirWalls` holds the walls round the windows. Each flanking bore's channel
lies exactly 3.000 mm beyond one of the window's long sides, measured on the window's plane. That
settles the bore without sampling: a point is never nearer the window in space than its
projection is on the plane. The other two
bores are measured in space from every face of the window's cut, sampled every 0.1 mm, and keep
3.076 mm. The windows reach 14.98 mm from the middle against the channels' 15.01 mm; every edge
rises at 45° or more; the window's faces meet the tube at edges of 49.6° or more; and the lines
where its roofs meet the tube descend at 35.3° or more, one cell diagonally on the grid. Each
window is 208.1 mm² on its plane and takes 1412 mm³ of wall.

`TestSleeveWindowPostsStandFirm` cuts level sections every 0.1 mm and walks each round the inner
face, the middle of the wall and the outer face at 0.1° steps. The narrowest post beside a window
is 3.44 mm, between gear B's +R bore and the +Y window. Where a post is narrower than two
`CollarWall`s, 6 mm, the tallest unbroken stretch is 3.2 mm tall on 3.77 mm: 0.85 times its width,
against a limit of 2.

`TestSleeveWindowsShowTheMeshFromTheSide` takes the 4176 points of the mesh zone that the axial
test projects into the footprint, sampled every 0.25 mm of station, and asks of each whether a
straight line from it leaves the tube through a window without touching either ribbon's crest
rectangle. 3756 of them can be seen, 89.9%: 52.9% through the +Y window and 53.0% through the −Y
window. Along a level line, as someone beside the frame at the mesh's height would look, 59.2% can
be. The plain tube has no opening in its side but the bores, and the ribbons fill those.

**It prints standing on either end, with no support.** Every outside face is a vertical cylinder
or a flat end ring of 565.5 mm², and every face of a window is upright or at 45°.
`TestSleevePrintsStandingOnEitherEnd` samples the sleeve on a 0.25 mm grid and finds every cell
with no material within one cell under it, which is a 45° rule at that size. Printed either way
up, there are 1325 such cells, 83 mm², and every one is in the roof of a bore, the same count as
before the windows were cut. Nowhere else does the printer lay material on air. The same test
holds the end wall above and below the channels, 3.74 mm, and the two nearest channels, gear A's
and gear B's −R bores 4.83 mm apart, over the 3 mm `CollarWall`, and logs the edge each hole's
mouth leaves: 46.4° on the inner face and 62.6° on the outer. `TestSleeveIsOnePiece` flood-fills
the same grid and reaches every cell: one piece of 16,503 mm³, about 20 g of PLA, against 19,325
mm³ before the windows.

| Quantity | Value |
|---|---|
| Sleeve | radius 12 to 18 mm, a 6 mm wall, 37.5 mm tall |
| Bore | 15.9 by 4.65 mm, cut over stations 7.683 to 19 mm of its gear's axis and turning 82.31° over them |
| Play in the frame | the bores jam a gear 3.55° out of step |
| Frame to ribbon | no point of either ribbon, grown by the clearance, in the sleeve over the travel; 1.00 mm from the tube outside the bores' cuts |
| Mesh footprint | 10.43 mm from the axis, 11.21 mm with the clearance, inside the 12 mm inner radius |
| Windows | two, facing +Y and −Y; long sides at 45°, 6.95 mm apart (9.83 mm up the axis); upright ends 23.07 mm apart; 208.1 mm² and 1412 mm³ each |
| Window to channel | 3.000 mm to the flanking bores, measured on the window's plane; 3.076 mm to the others, sampled in space |
| Beside the windows | the narrowest post 3.44 mm; stretches under 6 mm wide at most 0.85 times as tall as they are wide |
| Side view | 89.9% of the mesh zone seen through a window past both ribbons, 59.2% along a level line |
| Base | a 565.5 mm² flat ring at either end |
| Material laid on air | 1325 cells of 0.25 mm, 83 mm², all in the bores' roofs, printed either way up |
| End wall | 3.74 mm above and below the channels and the windows |
| Between channels | 4.83 mm at the nearest, gear A's and gear B's −R bores; the build's check (`channelSeparation`) holds them 4.64 mm apart |
| Hole mouths | 46.4° edges on the inner face, 62.6° on the outer |
| Volume | 16,503 mm³, about 20 g of PLA |
| Build's twist check | probes at stations ±15 mm, 16.86 and 16.31 mm from the axis, inside the channel under the right twist sense; under the wrong one they sit 6.56 and 7.41 mm across a channel 2.325 mm half thick, in the wall |
| Bore wall stand-in | a ruled loft through 18 sections leaves 0.443 mm of the 0.45 mm clearance |

**The windows follow the sleeve's size.** `TestSleeveWindowsFollowTheSize` builds the sleeve at 33
inputs away from the defaults — the whole ribbon and frame scaled from 2/3 to 1.75 with the 3 mm
wall and the 0.45 mm clearance held, and the width, thickness, lead, cage radius, rise, clearance,
collar half length, `CollarWall`, crossing angle and mounting angles moved one or two at a time —
and on every window it cuts runs every check the default windows pass, with the one-piece and
printing checks on the same 0.25 mm grid. Past a right-angled crossing the wider gaps between the
bores are the ones across ±X, and the windows face those. Every window passes. A window the bores
leave no room for is left out; no input the build accepts reaches that, and the test leaves both
windows out at a 7 mm `CollarWall`. The same run found that the sleeve itself needed a fourth
input check: at five of the inputs two neighbouring bores come nearer each other than `CollarWall`
(2.74 mm at 2/3 scale, 3.15 mm for a 4 mm wall at a 70° crossing), which no closed form on the
inputs predicts. `channelSeparation` is the build's check. It projects the two bores' sampled
outlines onto the plane across their gap and takes how far apart the two convex hulls stand, which
can only be less than the distance between the outlines; the build refuses the input when that is
under `CollarWall`. It is 4.64 mm at the defaults, and of the 33 inputs it refuses exactly those
five. The test takes about 15 s on 24 cores. Finding each end walks the far bores' channels at 2
µm stations; `wallGap` now builds each bore's stations once and walks out from the nearest,
stopping where no further station could come nearer, which gives the same distances and cuts
building both windows from 1.2 s to 0.07 s.

**What the proof cannot reach.** Each −R bore passes through level: its 15.9 mm-wide roof is flat
at station −14.43 mm, inside the wall, and the printer has to bridge it across the wall. The +R
bores' roofs are flat at station 10.31 mm, near the inner mouth, where 1.8 mm at each end of the
roof is in the wall and the rest in the hollow. Whether a 15.9 mm bridge sags into the 0.45 mm
clearance, and how the +R roofs come out, only a print settles; the proof logs the roofs and
enforces nothing about them. The sleeve has not been built in Fusion either: no Fusion load of
the add-in's sleeve has been reported, so none of the steps that build it, the tube's sketch and
extrude, the four twisted sweep cuts of 82° that start in air, and each window's one sketch of
the hexagon on the plane through the axis and one cut extruded from it through the wall on one
side, has run in Fusion. Where a window's roof meets the inner face, the line it meets on descends
at 35.3° at the flattest; the grid takes that as one cell diagonally, and whether a slicer lays a
short unsupported edge there only a print settles.

**The spec and the add-in build the sleeve.**
[`spec/screwgear/instructions.md`](../../spec/screwgear/instructions.md) §4 builds the sleeve and
sizes its windows by `newWindow`'s search, and its "What the print showed" records the
2026-09-30 report on the video's frame. `lib/geargen/screwgear.py` is emitted from that spec's
compiled step list and builds the sleeve too. The verdict of the first print of the sleeve
belongs in "What the print showed".

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
| Travel | 141.2 mm, 53.8 teeth, 79% of the ribbon: 69.9 mm back and 71.3 mm forward of the assembly position |
| Ribbon to ribbon outside the engaged zone | 1.96 mm at every phase of the travel, at station −7.14 mm, crest rectangle against crest rectangle |

`TestPairDrivesOneToOne` in [pair_test.go](pair_test.go) is where the first three come from, and
`TestFullRibbonsClearOutsideTheEngagement` beside it gives the fourth. It tracks the interval of
gear B's tooth phase that clears gear A through a full pitch of A, and requires three things of
it: that it is never empty, that it is narrower than a pitch, and that its centre advances by
exactly one pitch. The third is what separates a gear from two parts that merely touch, and an
arrangement tried earlier passed the first two and failed it. The sampling every number in the
table was taken with is fixed at the top of that file and quoted in the spec's "Defaults", and
so are its bounds, which are fractions of the pitch: the departure under 6% of it and the window
within 7% of it of the backlash quoted above. `TestRibbonsStayInsideTheirBoresOverTheTravel`,
`TestTravelIsTheRibbonBetweenItsBores` and `TestRibbonsClearEachOtherOutsideTheEngagement` in
[sleeve_test.go](sleeve_test.go) give the last three.

## What has no picture

**The tooth cell and its screw step.** The spec builds a ribbon as one four-tooth cell repeated
by a screw step, five copy-move-join rounds at the defaults, and
`TestRibbonIsInvariantUnderItsScrewStep` is what licenses that: it carries the corners of every
cross-section through one step and requires them to land on the next tooth's.
`TestDoublingScheduleCoversTheRibbon` holds the schedule. The pictures draw the whole ribbon from
its sections and never form a cell.

**Anything a generator does.** `lib/geargen/screwgear.py` builds the gear from the compiled step
list `spec/screwgear/steps.md`, but nothing here draws that build sequence step by step the way
[`proof/bevelgear/README.md`](../bevelgear/README.md) does. The four files named here —
[geometry_test.go](geometry_test.go), [pair_test.go](pair_test.go),
[sleeve_test.go](sleeve_test.go) and [render_test.go](render_test.go) — are the hand-written
mechanism proof; `/compile-gear` writes the step proof, the `compiled_*_test.go` files, beside
them and leaves them alone.

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

`TestRenderSleeve` takes about 30 seconds, most of it sampling the sleeve on its 0.2 mm grid;
`-run '^TestRenderSleeve$'` regenerates every picture with the sleeve in it, `plan.png` and
`pair.png` included.

Nothing checks these images. They are regenerated by hand when the geometry changes.
