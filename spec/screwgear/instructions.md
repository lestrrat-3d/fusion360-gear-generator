# Screw Gear Creation Instructions

The **design & geometry intent** is here; the **Fusion-API realization** is in the sidecar `fusion.md`
(cited by anchor `[SCREW-F-…]`); the **search that produced the meshing numbers** is in
`mesh-search.md` (cited by name). Cross-gear conventions are `[PB-…]` in `PLAYBOOK.md`. Read all
three together.

This generator does **not** subclass any gear class and shares no involute math. It subclasses
`base.Generator` directly and uses the generic occurrence plumbing only.

## What this builds

A **screw/screw gearing**: a three-part mechanism in which both gears move by a screw motion —
rotation about an axis combined with translation along that same axis — relative to a frame. Every
gear elsewhere in this repository rotates in its frame and translates not at all; a rack translates
and rotates not at all. This is the remaining case, and the mechanism comes from Henry Segerman's
video *Screw/screw gearing* (https://www.youtube.com/watch?v=0ZPo3HxR0KI), which is the only known
example.

The three parts are:

- **Gear A** and **Gear B**, each an ordinary rack — a flat plate with teeth cut into one long edge —
  twisted about its own centre line into a helix. The two are the same part.
- **The Cage**, the frame that holds them: a **sleeve**, one thick-walled tube about the frame's
  axis, standing on a flat end. Each ribbon passes through the tube's wall twice, once on each
  side, through a **bore** cut in the wall. The two bores of one gear sit low and the two of the
  other high, so the gears meet in the middle of the hollow, which is open at both ends; the
  mesh is seen down the hollow from either end, and from the side through two slanted
  **windows** cut through the wall in the two widest gaps between the bores. The video's own
  frame, a skeleton of round rods, is not built: it printed too wobbly ("What the print
  showed").

  Each bore is what makes its gear's motion a screw motion rather than a free slide. It is cut to
  the ribbon's crest rectangle, a clearance larger all round, and twisted at the ribbon's own
  lead, so a gear that turns without advancing jams in it. One bore of each gear lies level in
  the wall, and the printer bridges its roof; that bore has a **roof allowance** more room on
  its roof face, so the sleeve is printed standing on one particular end (§4, "The roof
  allowance"). A round hole would not: it would leave
  the mechanism three degrees of freedom instead of one. The teeth run straight through the
  bores, as they do through the video's collars.

The gears' toothed edges meet in the middle of the cage, between the two bore heights. Pushing
one gear along its axis drives the other, at a **1:1 ratio** — one tooth pitch of advance each.
The mechanism has one degree of freedom.

## What the video shows, and where this spec departs from it

The table compares this spec with frames of the video, so that a departure is not taken for an
oversight. Timestamps are minutes:seconds. Pixel ratios were read off the 0:09 overhead shot,
where the mechanism lies flat and the camera looks down its cage axis, using the ring's outer
diameter (108 px) and the widest face-on stretch of a ribbon arm (46–55 px) as the units; the
twist was read from where that arm's silhouette pinches, every 62–66 px along it. Every ratio
carries about ±20%, because a 1280×720 frame puts a ribbon width on some fifty pixels.

| Aspect | Video | This spec |
|---|---|---|
| The part | A flat ribbon with teeth on one long edge, twisted about its centre line, cut square at both ends with the teeth running to the end (0:09, 6:10) | Same |
| Hand | Both ribbons twist the same way. Read from which edge-on stretches show their teeth from above at 0:09, the hand is right-handed, at moderate confidence | Same hand on both, right-handed (`Hand = +1` in the proof) |
| Teeth per turn | 20–26, counting the teeth on one face-on stretch, which is half a turn (0:09, 6:12, 6:14) | 18.9 |
| Twist lead | 2.8–3.5 ribbon widths (0:09) | 3.3 widths |
| Tooth depth | 0.15–0.2 widths, about one pitch (0:09, 6:14) | 0.175 widths, one pitch |
| Ribbon thickness | 0.2–0.3 widths (6:14) | 0.25 widths |
| Ribbon length | About 12 widths (6:10, both ends in frame against the ring) | 11.9 widths, 68 teeth |
| Crossing angle | 85–100° between the arms in the overhead shots (0:09 reads 89°, 6:10 reads 102°) | 80° |
| Cage size | Ring outer diameter 2.2–2.8 widths, about 0.8 of a twist lead (0:09, 6:10); the frame is about as tall as the ring is wide (5:26, 5:34) | The sleeve's outer diameter 2.4 widths, 0.73 leads; 1.04 of its diameter tall |
| The frame | An open skeleton of round rods, described below the table (0:09, 5:23, 5:26, 5:28, 5:31, 5:34, 5:36, 5:37, 6:10, 6:16, 6:42) | Departs: a sleeve, one tube with four bores and two slanted windows through its wall ("What the print showed") |
| What passes through a collar | The plain ribbon: the teeth run through the collars (0:09, 6:12, 6:14) | Same through a bore. The bore is the crest rectangle plus the clearance, and the crests bear on its toothed side |
| Travel | Most of the ribbon: at 5:53 the frame sits near one end of a ribbon, at 6:00 near its middle | 141 mm, 79% of the ribbon, 54 teeth |
| Tooth form | "Based on a sine wave" (6:29). At print resolution the crests read flat and the flanks straight (6:21 and 0:09 alike) | A pure cosine |

The video's frame is one round ring of round wire at one end, a smaller loop with straight sides
at the other, four thin rods between them, and a short smooth collar round each ribbon where it
crosses. It has no wall, no plate and no post. The frames settle this much of its layout:

- The ring is at one end and the loop at the other, never both alike. At 5:34 the ring is on
  top and the loop underneath; at 5:36, two seconds on, the model has been turned over and the
  loop with its straight sides is on top.
- Each rod is one straight line from the ring to the loop, parallel to the frame's axis, and
  passes one collar on the way: at 5:26 and 5:34 the left rod is one line above its collar and
  below it, and the collar sits at about the middle of the frame's height.
- The rods stand in the gaps between the ribbons, not on the ribbons' own lines. In the 0:09
  overhead shot each rod's end at the ring is 20–30° round from the arm nearest it.
- The loop is a rectangle with rounded corners, and it reads smaller than the ring because its
  corners are on the ring's circle and its sides are chords: in the 0:09 shot it is about 0.7 of
  the ring's width across.
- The rods and the ring's wire are of similar thickness, 0.2–0.3 of a ribbon width, with the rods
  the thinner (0:09, 5:34, 6:10).

Where exactly a rod meets its collar, how long the collar is along the ribbon and how thick its
wall is, the frames do not settle: the collar is white on white against the ribbon inside it,
and at 1280×720 it is some thirty pixels long. The collar length here is a choice, 6 mm. In the
sleeve it is the thickness of the tube's wall, which each bore runs through, and "Travel"
records what it costs.

Two departures are deliberate, and each has its reason elsewhere in this spec. **The crossing
angle** and **the lead** come from the meshing search, not from the video. With `CrossAngle`
set to 90° and nothing else moved, `TestPairDrivesOneToOne` reports a 0.166 mm departure from
the 1:1 line, 6.3% of the pitch, against its bound of 6% of the pitch; with the lead at 60 mm
it reports 0.109 mm, 4.2%, with the free window narrowed to 0.30–0.38 mm. 80° at 49.5 mm
reports 0.042 mm, 1.6%, with a 0.43–0.47 mm window. **The frame** is a sleeve rather than the
video's skeleton, because the skeleton printed too wobbly ("What the print showed").

The cage radius is 15 mm, the value at which the video frame's rods, on a 16.875 mm ring, could reach
their collars; the sleeve keeps it. It is where each bore sits on its gear's own axis, and the sleeve's
wall runs `collarHalf` either side of it, from radius 12 to 18 mm. Inside that, the mesh zone's
footprint along the frame's axis reaches 10.92 mm from it. Grown by the clearance as a box, the crest
rectangle widened on every side and the zone lengthened at both ends, it reaches 11.26 mm
(`TestSleeveKeepsTheMeshVisibleAlongTheAxis`), so the hollow shows the whole mesh from either end. An
earlier version of this spec, at the 10 mm width, carried a smooth boss on each ribbon at the collars,
0.6 mm proud, so that no tooth ever entered a bore; the boss is gone, because the video's ribbons carry
none and the crest rectangle already holds the whole ribbon ("Why the cage needs no tooth-shaped cut").

`TestProportionsFollowTheVideo` holds the ratios this spec follows for the ribbon — teeth per
turn, thickness, tooth depth and length against the width, the tooth depth against the pitch,
and one hand for both gears — inside the video's ranges widened by the ±20% the readings carry;
the 18.9 teeth per turn sit just under the 20–26 read. It also holds the sleeve inside the
widened ranges read for the video frame's ring and height, reading the ring's outer diameter as
the sleeve's and the frame's height as its own: 2.40 ribbon widths across and 1.04 of its
diameter tall. Those two ratios no longer bind the design, since the user dropped the video
frame's look as a requirement; the test holds them to catch a sleeve far off them.

## What the print showed

Four prints have been reported. The first sized the ribbon, the second replaced the frame,
the third, of the sleeve with the resized ribbons, changed the fit of the bores and the mesh, and
the fourth, of the sleeve at that fit, added the roof allowance.

**The second sleeve, 2026-10-02.** The user printed the sleeve at the fit "The mesh" below
describes, a 0.20 mm clearance, a 0.90 mm engagement and 14° on both gears, with the ribbons
printed for the first sleeve (`[SCREW-F-PRINT-2]`). Its two `-R` bores were too tight to pass the
ribbons and its two `+R` bores were not. Each `-R` bore passes through level inside the wall, at
station −14.30 mm, so the printer bridges its 15.4 mm roof, and the likeliest cause is that the
roof sagged into the 0.20 mm of room. The user chiselled the `-R` bores open and the ribbons went
through; then the teeth meshed only sometimes, even with the ribbons pressed toward each other.

A search for a mesh that survives a print (`mesh-search.md`, "The search for a mesh that
survives a print") found that the pair drives only while the sum of the two mounting angles
stays in a band a few degrees wide, that every degree a ribbon rolls in its bores is a degree of
mounting angle, and that a deeper engagement narrows the band rather than widening it. A
0.50 mm bore lets each ribbon roll 3.85°, more than the band holds, and no arrangement of this
ribbon tried meshed over that play without jamming almost everywhere. So the defaults changed
this way, and nothing else did:

- Each gear's level bore, its `-R` bore at the defaults, has a **0.30 mm roof allowance** on the
  long face that is its roof when the sleeve stands on its `−n̂` end: 0.50 mm of room under the
  bridged roof, 0.20 mm everywhere else (§4, "The roof allowance"). The 0.30 mm is the least the
  user asked for. The allowance adds no move and no roll; it lets the ribbon tilt, which carries
  one crossing 0.273 mm instead of 0.200 mm, and the mesh holds over that tilt.
- The sleeve has to be printed standing on its `−n̂` end, the end below the selected plane.
- The clearance, the mesh and the ribbon are unchanged, so the printed ribbons stay usable and
  only the sleeve is reprinted.
- The proof now takes a printed tooth's tip to be 0.35 mm short of the model's crest wherever
  it judges the mesh under play ("What the proof checks"). At the bores as drawn the mesh boxes
  the teeth with the tips up to 0.40 mm short and not at 0.45 mm.

**The mesh, 2026-10-02.** The user printed the sleeve built by the add-in at commit c8a63b5 and
two ribbons from the earlier build at f328813, whose ribbon is the same part. Both ribbons
screwed through their bores, and the teeth did not mesh. The geometry agreed between the proof
and both add-ins; the cause was play (`[SCREW-F-PRINT-MESH]`). Each 0.45 mm bore let its ribbon
move 0.45 mm toward or away from the other ribbon and roll 3.44° about its own axis, which
changes the engagement and the mounting angle the mesh sees. At that day's defaults, a 0.75 mm
engagement and 15° on both gears, the pair drove only within about 0.6 mm of engagement and from
11° to 16° of equal mounting angle, so 15° was 1° from the angle past which the teeth no longer
box each other in. Over the poses the bores allowed, 128 of 324 pose pairs jammed or let the
teeth pass without boxing each other. The proof had measured the mesh at the nominal pose
alone, where the pair drives.

The user chose a tighter bore and a deeper mesh, and the defaults changed:

- The clearance went from 0.45 mm to 0.20 mm, so each ribbon moves 0.20 mm toward, away or
  sideways in its bores and rolls 1.53°.
- The engagement went from 0.75 mm to 0.90 mm and both mounting angles from 15° to 14°, which
  puts the pair 2° inside both edges of the band of equal angles that drive ("Both Mounting
  Angles are 14°"). The nominal window narrowed from 0.761–0.892 mm to 0.433–0.473 mm.
- The assembly phase went from −1.31 mm to −1.30 mm, the middle of the new free window. It moves
  gear B's ribbon 0.01 mm along its own axis and changes nothing about the part.
- The ribbon did not change, so the printed ribbons fit the new sleeve and only the sleeve is
  reprinted.

At those values `TestPairDrivesUnderBorePlay`, as it stood then, ran the mesh over 64 pose pairs
of the play with the model's own tips, and 63 drove with windows from 0.052 to 0.932 mm; the one
that jammed had both ribbons pushed the whole clearance toward each other. An offline run that sampled sideways and diagonal moves as
well found 6 of 676 pose pairs jamming, each with both ribbons pushed toward each other by
0.204 mm or more in all, and none failing another way; the user accepted those jams as the
cost. The proof could have caught the print's failure and now does: run at the printed values,
`TestPrintedFitFailsUnderBorePlay` finds pose pairs failing ("What the proof checks").
The reprinted sleeve is "The second sleeve" above.

**The frame, 2026-09-30.** The user printed the video's frame several times — the ring, the
loop, the four rods and the collars that §4 used to build — and reported on 2026-09-30 that it
is not practical to print: the thin rods, the wire ring and the loop hanging on them wobble
while the printer lays them down. The frame is there to show the two ribbons meshing, not to
copy the video, so the rules for a replacement were that the four bores may not change, since
they hold each gear to its screw motion, and that the mesh stays visible from the top and the
bottom. The sleeve of §4 keeps both rules and prints standing on either flat end with no
support. `proof/screwgear/sleeve_test.go` proves it and its two side windows; the user accepted
the design and asked for the windows to be sized from the cage and the bores rather than fixed,
which §4's window search does. The sleeve was first printed on 2026-10-02 ("The mesh",
above).

**The ribbon, 2026-09-28.** The first physical test was a print of the third Fusion load's geometry
(`[SCREW-F-FIRST-LOAD]`), which the user made at the defaults of that day: ribbon width 10 mm,
thickness 2.5 mm, tooth pitch 1.75 mm, tooth height 1.2 mm, 80 teeth, 140 mm long, with the
frame in proportion (ring radius 11.25 mm, cage radius 10 mm, 0.3 mm clearance). On 2026-09-28
the user reported that the printed model's teeth were far too small, that the ribbon should be
wider with bigger teeth, and that the ribbon could be about 30% longer.

The defaults changed because of that report, and this is what changed:

- Every length grew by 1.5: the ribbon to 15 by 3.75 mm, the pitch to 2.625 mm, the lead to
  49.5 mm, the frame (ring, cage radius, cage rise, ring wire, rods, collars) and the clearance
  (0.45 mm) with it. Every ratio the video table holds is unchanged by a uniform scale.
- The teeth deepened from 0.69 of the pitch to one pitch, 2.625 mm, which is the video's own
  depth (0.175 widths against its 0.15–0.2). The tooth depth thereby moved from the ratios this
  spec departed from to the ones it follows.
- The ribbon lengthened from 140 mm to 178.5 mm, 68 teeth at the new pitch: 27.5% longer, the
  nearest multiple of `cellTeeth` to the 30% asked for, so the ribbon is still whole four-tooth
  cells with no remainder (§3) and still five doubling rounds. 72 teeth would be 189 mm, 35%.
- The mesh arrangement was searched again at the new size and tooth (`mesh-search.md`, "The
  search at the 1.5× size"): the crossing angle and both mounting angles stayed at 80° and 15°,
  the engagement became 0.75 mm, and the assembly phase −1.31 mm. The print of 2026-10-02
  moved the mounting angles, the engagement and the assembly phase again ("The mesh", above).

The window that print was made with was 0.44–0.57 mm; the 1.5× one was 0.76–0.89 mm. The print
of 2026-10-02 found the 1.5× ribbons screwing through the sleeve and not meshing ("The mesh",
above).

## Geometry

### The part

Work in each gear's own frame: `s` runs along its axis, `u` across the plate's width, `v` across its
thickness. Before the twist, the rack occupies

```
-W/2 <= u <= Utooth(s),    |v| <= T/2
Utooth(s) = W/2 - H/2 + (H/2)*cos(2*pi*(s - Z)/P)
```

`W` is Ribbon Width, `T` is Ribbon Thickness, `H` is Tooth Height, `P` is Tooth Pitch and `Z` is the
**tooth phase** — the one coordinate that moves. The toothed edge is a **pure cosine**, with no flat
crest and no flat root: crest at `u = W/2`, root at `u = W/2 - H`. Segerman's model uses a sine wave
too, and says explicitly that he did nothing cleverer; working out the conjugate tooth shape for
this mesh is an open question, and this spec does not attempt it.

The twist takes the cross-section at station `s` and rotates it about the axis by `s/Lambda + Phi`,
where

```
Lambda = TwistLead / (2*pi)      screw parameter, mm of advance per radian
Phi    = that gear's Mounting Angle, its cross-section angle at the crossing station
```

**Each gear has its own `Phi`.** They are not equal at the defaults, and the meshing search is what
says so.

**The screw motion is a shift of `Z` and nothing else.** Advancing the gear by `dz` translates it by
`dz` and rotates it by `dz/Lambda`; the twisted blank is invariant under exactly that motion, so the
only thing that changes in the body's own frame is the tooth phase. Every step below relies on this:
it is why the whole ribbon is one tooth cell repeated by a screw step, and why the proof can pose
meshing as a search over two numbers.

### Why the cage needs no tooth-shaped cut

The ribbon is the same twisted rack along its whole length, and the crest of a cosine rack is the
ribbon's outer edge: `Utooth` peaks at `u = W/2` and falls to `W/2 - H` at the root, while the
back edge stays at `-W/2` and the faces at `±T/2` whatever the phase. So the **crest rectangle**,
`W` by `T`, holds every point of the ribbon at every station and every phase, and the bore is
that rectangle plus the clearance all round, `(W + 2*Clearance)` by `(T + 2*Clearance)`, 15.4 by
4.15 mm at the defaults, twisted at the ribbon's lead; each gear's level bore has the roof
allowance more on its roof face, 4.45 mm through (§4). Nothing in the frame is shaped like a
tooth, and the teeth run through the bores as they run through the video's collars.

What bears in a bore is the crests. The back edge and both faces run the clearance from the wall
at every station, the level bore's roof face the clearance and the roof allowance; on the toothed side a crest comes to the clearance every pitch and the root
between falls `H` short of it, so the bore's toothed side bears on a crest line and never on a
flank. `TestRibbonsStayInsideTheirBoresOverTheTravel` holds every point of the ribbon inside
every bore by the clearance through the whole travel, and holds the crests, the back edge and
the faces to the clearance, so the bore is cut to the ribbon rather than merely round it.

The twist is what holds the motion, not the tooth. A rectangle twisted at the ribbon's lead
admits the screw motion and nothing else: `TestSleeveAdmitsOnlyTheScrewMotion` turns a gear out
of step with its advance and finds it jamming in the sleeve's bores at 1.60°.

### Travel

Nothing on the ribbon limits the travel, since any stretch of it fits a bore; the ribbon's
length does. The sleeve's wall is `2*collarHalf` thick, the video's collar length, and on its
centre line each bore runs through the wall from station `cageRadius - collarHalf` to
`cageRadius + collarHalf` of its gear's axis, where a collar ran. A gear has to keep both its
bores full, and its end reaches a bore's far face after `N*P/2 - CageRadius - CollarHalf` of
advance, 71.25 mm at the defaults, so **a ribbon runs `N*P - 2*(CageRadius + CollarHalf)`,
142.5 mm, through its bores**. Gear B is assembled 1.30 mm along its axis (the assembly phase),
so it reaches one bore's end that much sooner than gear A does, and the pair travels **141.2 mm:
70.0 mm back and 71.3 mm forward** of the assembly position, 53.8 teeth, 79% of the ribbon. The
engaged zone is nearer the middle than the bores, so the teeth are still meshing at both ends of
that: an end would leave the ±7.79 mm zone at 80.2 mm. `TestTravelIsTheRibbonBetweenItsBores`
walks both limits through the sleeve's bores, each over the wall's span on its centre line.
That span is the one the video frame's collars covered, and `TestSleeveBoresAreTheSameChannels`
holds the bores to the collars' channels, so the travel is the one that frame had.

The wall's thickness comes straight out of the travel, two millimetres of travel per millimetre
of wall, and the video does not settle it; 6 mm is kept.

### The pair

Let `n̂` be the common perpendicular of the two axes and `C` the centre of the mechanism. Then

```
Beta  = atan(pi*W / TwistLead)        helix angle of the toothed edge
Sigma = Crossing Angle                angle between the two axes, the crossAngle input
A     = W - Engagement                distance between the two axes
```

Gear A's axis passes through `C - (A/2)*n̂`, gear B's through `C + (A/2)*n̂`, and the two directions
sit at `+Sigma/2` and `-Sigma/2` about `n̂` from a shared reference direction in the plane. Each
gear's `u` axis at its crossing station points at the other gear, then turns by `Phi`.

**The build takes `Sigma` from the `crossAngle` input, 80° by default, and derives nothing from
`Beta`.** `Beta` is 43.6° at the defaults and `2*Beta` is 87.2°, and that number is the
**crossed-helical rule**, which is where the search for the crossing angle started: two helical
tooth rows run parallel where they meet when the shaft angle equals the sum of the two helix
angles, and the two members here have the same helix angle and the same hand. Deriving it for this
pair: the crest of gear A traces the helix `axis(s) + (W/2)*û(s)`, whose tangent is
`â + (W/2/Lambda)*v̂(s)`; setting A's and B's tangents parallel gives
`tan(Sigma/2) = pi*W/TwistLead = tan(Beta)`. `TestCrossedHelicalRuleMakesTheCrestHelicesParallel`
holds the rule as a fact about the geometry at `Sigma = 2*Beta`, not as the angle the pair is built
at. **Both gears have the same hand.**

**The rule is a starting point, not a proof that this pair meshes and not the angle it is built
at.** The two crest helices run parallel at exactly one station on each ribbon — the one whose
cross-section angle is zero, which the mounting angle puts `Phi*Lambda` back from the axes'
closest approach — and they cross everywhere else. The proof measures contact between 1.2 mm short
of the closest approach and 1.1 mm past it, which is nowhere near that station, so what this pair
actually carries is a point contact like a crossed-helical pair rather than the line contact the
rule describes. The meshing search moved the angle off the rule: at 90°, `TestPairDrivesOneToOne`
reports a 0.166 mm departure from the 1:1 line, 6.3% of the pitch, against its bound of 6% of
the pitch, and at 80° it reports 0.042 mm, 1.6% ("What the video shows"). At 2*Beta, 87.2°, it
reports 0.120 mm, 4.6%, inside the bound at today's defaults; at the arrangement the search ran
at, before 2026-10-02, 90° departed by 7.4% and 80° by 2.4%. The search is what
settles it, and 80° is what it settled on, at both sizes the pair has been searched at.

### Why the mounting angles are not zero

The obvious arrangement points both toothed edges straight at each other at the crossing station
(`Phi = 0` on both). **That arrangement jams.** A meshing sweep over crossing angle, mounting angle,
hand and engagement found that at `Phi = 0` no phase of gear B clears gear A at every phase of gear
A, at any engagement past roughly a quarter of the tooth height, because the engaged zone spans
about four tooth pairs whose ridges cross at an angle and cannot all interdigitate at once. Turning
the ribbons about their own axes moves the crossing to a station where they do clear.

The same sweep is what the defaults below come from; `mesh-search.md` records its model, its
criterion and what it covered. **The proof owns these numbers** and must re-derive them (see "What
the proof must check").

### Defaults, and what they were measured to do

| Quantity | Default |
|---|---|
| Ribbon Width `W` | 15 mm |
| Ribbon Thickness `T` | 3.75 mm |
| Tooth Pitch `P` | 2.625 mm |
| Tooth Height `H` | 2.625 mm |
| Tooth Count `N` | 68 |
| Twist Lead | 49.5 mm per turn |
| Crossing Angle `Sigma` | 80° |
| Engagement | 0.90 mm |
| Mounting Angle, both gears | 14° |
| Assembly Phase | −1.30 mm |
| Cage Radius | 15 mm |
| Cage Rise | 18.75 mm |
| Collar Half Length | 3 mm |
| Collar Wall | 3 mm |
| Clearance | 0.20 mm |
| Roof Allowance | 0.30 mm |

Derived: `Beta` = 43.6°, `A` = 14.10 mm, ribbon length = 178.5 mm, twist per tooth = `P/Lambda` =
19.1°, ribbon turn between a gear's two bores = `2*CageRadius/Lambda` = 218°. The sleeve runs
from radius 12 to 18 mm and stands 37.5 mm tall. These are the ribbon's defaults since the print
of 2026-09-28, the frame's since the print of 2026-09-30, and the clearance, engagement,
mounting angles and assembly phase since the print of 2026-10-02 ("What the print showed"); the
earlier tables are recorded there; the roof allowance came in with the second sleeve's print of
2026-10-02. The proof's `defaultParams`, in
`geometry_test.go`, is this table; `sleeveParams` returns the same table under the name the
compiled step proof calls.

**The frame does not set the lead.** An earlier printable frame tied the cage radius to half the
twist lead, so that its bores stood near upright, and tied the two mounting angles to each other.
The sleeve takes each bore at whatever angle the ribbon passes its wall — 56° off upright for the
`+R` bores and 86° for the `-R` bores at the defaults, whose roofs the printer bridges ("What the
proof cannot reach") — so the lead stands on its own: 3.3 ribbon widths, read from the video
(`mesh-search.md`). At two widths per turn the ribbon is twisted past the point of looking like a
rack; at four widths (60 mm) the pair still drives, with the window narrowed to 0.30–0.38 mm and
the departure raised to 0.109 mm, and at 4.4 widths (66 mm) the window is 0.18–0.26 mm.

At those values `proof/screwgear` measures a free window in B's tooth phase that is **0.433–0.473 mm
wide** and that **advances by exactly one tooth pitch for each pitch A advances**, departing from
the 1:1 line by 0.042 mm, which is 1.6% of the pitch. That window width is the backlash, and the
winding is what makes this a 1:1 gear rather than two parts that merely touch.

**The sampling those numbers were taken with.** They are `TestPairDrivesOneToOne`'s, at the
sampling `pair_test.go` fixes, and a model at another sampling moves their last digit or two:

- Each ribbon's boundary is sampled at stations within `±axialWindow` of the crossing,
  `1.5 * sqrt(W^2 - A^2) / sin(Sigma)`, ±7.79 mm at the defaults, every 0.01 mm of station; at each
  station, five points across the thickness on the toothed edge and five on the back edge, and
  seven across the width on each face.
- A sample is inside the other ribbon when its coordinates in that ribbon's section at its own
  station, twist undone, satisfy the four inequalities of "The part". The pair is clear at a
  phase pair when no sample of either ribbon is inside the other.
- The free window at a phase of A is scanned in B's phase in steps of `P/200`, 0.0131 mm,
  outward from the previous window's middle, so its ends are quoted to that step. A advances
  through one pitch in 12 equal steps; the width is the narrowest and widest of those 12 windows,
  and the departure is the largest gap, over the 12, between a window's middle and the straight
  1:1 line from the first middle to the last.

**The proof's bounds on the mesh are fractions of the pitch, not lengths.** The model is an exact
cosine on an exact helicoid, so a pair scaled by `k` has its window and its departure scaled by
`k` too, and the search found the departure tracking the pitch at every pitch it tried
(`mesh-search.md`); a bound in millimetres would pass or fail a scaled gear on its size alone.
`TestPairDrivesOneToOne` holds the departure under 6% of the pitch (0.1575 mm here; the bound
was written as 0.10 mm when the pitch was 1.75 mm) and the window within 7% of the pitch of the
number this spec quotes. The clearances below are lengths, because the frame's clearance is a
length the dialog sets.

Two other clearances are quoted in this spec, and each is a different measurement:

- **0.361 mm** is how far the two ribbons stand from each other at the assembly phases, gear A at
  tooth phase 0 and gear B at −1.30 mm, with nothing moved. `TestFullRibbonsClearOutsideTheEngagement`
  walks the toothed edge and the back edge of each whole ribbon, every 0.01 mm of station at five
  points across the thickness, and takes the least slack of any sample against the other ribbon:
  the slack is measured in that ribbon's own section at the sample's station, along `u` to its
  toothed or back edge and along `v` to its faces, whichever is least, so it is a slack rather than
  a Euclidean distance, and only its sign is exact. The least is 0.361 mm at station −2.50 mm, in
  the mesh: gear B sits in the middle of a window 0.433 mm wide, so it has 0.22 mm each way along
  its own axis before a flank touches, and the slack across the flank is more than that.
- **1.95 mm** is the least the two ribbons keep from each other outside the engaged zone at any
  phase of the travel. `TestRibbonsClearEachOtherOutsideTheEngagement` walks each ribbon's crest
  rectangle, `W` by `T`, which is everything the ribbon reaches at any phase, over every station
  the travel carries it through but outside `±axialWindow` of the crossing, every 0.02 mm of
  station at five points along each side of the rectangle, and takes the least slack of any sample
  against the other ribbon's crest rectangle, measured as above. It is 1.95 mm at station
  −7.80 mm of the sampled ribbon, just outside the zone.

**Both Mounting Angles are 14°.** At the 0.90 mm engagement, equal angles drive from 12° to 16°:
at 11° and below the pair jams, and at 17° and above B is never boxed in. 16° departs from the
1:1 line by 0.345 mm, 13% of the pitch, past the proof's bound, so the band the proof accepts is
12° to 15° (measured on 2026-10-02 at the sampling above, in 1° steps). A ribbon rolls in its
bores, 1.53° either way at the 0.20 mm clearance, and every degree of roll is a degree of
mounting angle to the mesh; a sideways move in the bores acts as a roll as well
(`bore_play_test.go`). 14° stands 2° from both ends of the band that drives, and 1° inside the
15° the proof's departure bound still accepts. With the tips 0.35 mm short, as a print makes
them, the band is 11.5° to 15.5° (`mesh-search.md`), and the 1.53° of roll keeps the pair inside
it; `TestPairDrivesUnderBorePlay` judges the play with those tips. The earlier 15°, at
a 0.75 mm engagement and a 0.45 mm clearance, was 1° from the edge, and the print of 2026-10-02
did not mesh ("What the print showed"). Unequal angles drive too — 0° on gear A and 30° on gear
B gives a 0.735–0.919 mm window and a 0.031 mm departure at the 0.90 mm engagement — but the two
gears are the same part held alike, and at equal angles a half turn about `ê` carries each
gear's bores onto the other's, so the sleeve and its two windows are the same either way up but
for the roof allowance (§4); 14° on both is kept. The arrangement is an input on both gears, and an unequal pair is a
change of two dialog values.

**Assembly phase.** With gear A at tooth phase 0, gear B is built at **−1.30 mm**: its teeth and
its ends are gear A's advanced by that much along its own axis under its own screw motion ("The
part"), so its ribbon runs from station −90.55 mm to +87.95 mm where gear A's runs ±89.25 mm. The
number is the middle of the free window the proof measures at gear A's phase 0, −1.523 to −1.090 mm
at the sampling above and −1.527 to −1.078 mm found by bisection on 2026-10-02, whose middle is
−1.303 mm,
and `TestAssemblyPhaseSitsInTheFreeWindow` holds it there. It was −1.31 mm until 2026-10-02; the
phase moves gear B's ribbon along its axis and does not change the part. The build cannot
measure that window, so the phase is the `assemblyPhase` input, defaulting to the proof's number
as the mounting angles and the engagement do; an arrangement the proof has not measured needs its
own.

## Architecture

The module `lib/geargen/screwgear.py` defines exactly these public classes (the command wiring binds
to them by name; exported via `lib/geargen/__init__.py`):

- **`ScrewGearCommandInputsConfigurator`** — classmethod `configure(cls, command)` adds the dialog
  inputs in the order and groups of "Variables" below. No conditional visibility.
- **`ScrewGearGenerator(base.Generator)`** — 1-arg constructor `(design)` (inherited); implements
  `generate(self, inputs)` and the call graph below; relies on inherited `deleteComponent()` for
  error cleanup. Overrides `prefixBase()` to return `'ScrewGear'`.

**Generation Context: none.** Carry handles on `self`: `self.designOcc`, `self.gearOccs` (list of
two), `self.cageOcc`, `self.gearBodies` (list of two), `self.cageBody`, `self.pathLines` (list of
two dicts, one per gear, keyed `'bore-'` and `'bore+'`, each the sketch line of §1 that one bore's
sweep of §4 runs along), and `self.windows` (the windows the window search of §4 finds room for,
zero to two, each its facing direction and its corners).

**Dependencies: none.** Imports only the framework (`base`, `misc`, `utilities`, `solids`,
`fusion360utils`). Use `solids.hide_construction_geometry(component)` for the final cleanup — do
NOT re-implement it.

**Entry wiring:** `commands/screwgear/entry.py` constructs `GearCommand(gear_type='ScrewGear',
name='Screw Gear Generator', …)`, binding the two classes above by name (PLAYBOOK.md
"Command-entry wiring").

**Parameter mode: all-Python-precomputed** (`[PB-PRECOMPUTED-MODE]`). Every value is computed in
Python in internal cm and written numerically; the generator registers **no** named user parameters.
The trigonometry here (per-station rotation angles, the screw step matrix) has no useful expression
form in the parameter table. The two searches of §4 that run before any feature — the check on
the wall between the bores and the window search — work in millimetres, with the step sizes §4
states in millimetres, and every length they hand on is divided by 10.

## Component Setup

One command invocation creates a **`Screw Gearing`** component under the user-selected Parent
Component, holding four sub-components (`[PB-OCCURRENCE-TREE]`). The top occurrence is the
inherited `self.getOccurrence()`, which creates it under `self.parentComponent` and is what the
inherited `deleteComponent()` deletes on failure; the build names its component and never calls
`addNewComponent` for it itself. The four children are each
`occurrences.addNewComponent(adsk.core.Matrix3D.create())` under that component:

- **`Design`** — every sketch, construction plane and feature runs here.
- **`Gear A`**, **`Gear B`**, **`Cage`** — empty until the end, when the finished bodies are
  relocated into them with `body.moveToComponent` (`[PB-NO-CROSS-SIBLING]`).

**NEVER call `occurrence.activate()`** (`[PB-NEVER-ACTIVATE]`). Place sketches directly on the
user-selected plane (`[PB-USE-SELECTED-PLANE]`).

The user selects a **plane** and a **point**. The plane's normal is the cage axis and the common
perpendicular `n̂`; the point is the mechanism's centre `C`.

## Variables

User inputs in dialog order: first by importance, then in named groups. All linear inputs are
mm; the crossing and mounting angles are degrees.

The frame's inputs keep the names the video frame gave them, and mean this for the sleeve:
`cageRadius` is where each bore sits on its gear's own axis and the middle of the sleeve's wall;
`cageRise` is half the sleeve's height, to its flat end faces; `collarHalf` is half the wall's
thickness, so the sleeve runs from radius `cageRadius - collarHalf` to `cageRadius + collarHalf`;
`collarWall` is the least material the build leaves round every bore and every window, at the
end faces, between two bores and beside a window; `clearance` is added all round the bore's
rectangle; `roofAllowance` is added on one face of one bore of each gear, the roof the printer
bridges (§4, "The roof allowance"). The video frame's `ringRadius`, `ringWire` and `rodDiameter` are gone with its ring,
loop and rods.

The dialog opens with where the mechanism goes, because nothing can be built without it and
Fusion focuses the first selection input (`[PB-AUTOFOCUS-FIRST]`). Then three groups, each a
`GroupCommandInput` made with `command.commandInputs.addGroupCommandInput(groupId, groupLabel)`,
whose inputs are added to that group's `children` collection instead of the top-level
`commandInputs`. The groups run from what a user changes most to what they should rarely touch:
the ribbon's size, then the frame, then the mesh arrangement, whose values come from the meshing
search (`mesh-search.md`) and jam when changed carelessly, so that group starts collapsed
(`isExpanded = False`); the other two start expanded. Within a group the rows run from most to
least important, as listed.

| Group (group id, label) | Dialog label | input id | unit | default |
|---|---|---|---|---|
| none (top level) | Target Plane | `plane` | selection | — |
| none (top level) | Centre Point | `point` | selection | — |
| none (top level) | Parent Component | `parent` | selection | — |
| `ribbonGroup`, Ribbon | Ribbon Width | `ribbonWidth` | mm | 15 |
| `ribbonGroup`, Ribbon | Tooth Count | `toothCount` | — | 68 |
| `ribbonGroup`, Ribbon | Twist Lead | `twistLead` | mm | 49.5 |
| `ribbonGroup`, Ribbon | Ribbon Thickness | `ribbonThickness` | mm | 3.75 |
| `ribbonGroup`, Ribbon | Tooth Pitch | `toothPitch` | mm | 2.625 |
| `ribbonGroup`, Ribbon | Tooth Height | `toothHeight` | mm | 2.625 |
| `frameGroup`, Frame | Cage Radius | `cageRadius` | mm | 15 |
| `frameGroup`, Frame | Cage Rise | `cageRise` | mm | 18.75 |
| `frameGroup`, Frame | Clearance | `clearance` | mm | 0.20 |
| `frameGroup`, Frame | Roof Allowance | `roofAllowance` | mm | 0.30 |
| `frameGroup`, Frame | Collar Half Length | `collarHalf` | mm | 3 |
| `frameGroup`, Frame | Collar Wall | `collarWall` | mm | 3 |
| `meshGroup`, Mesh (from the mesh search) | Crossing Angle | `crossAngle` | deg | 80 |
| `meshGroup`, Mesh (from the mesh search) | Engagement | `engagement` | mm | 0.90 |
| `meshGroup`, Mesh (from the mesh search) | Mounting Angle A | `mountAngleA` | deg | 14 |
| `meshGroup`, Mesh (from the mesh search) | Mounting Angle B | `mountAngleB` | deg | 14 |
| `meshGroup`, Mesh (from the mesh search) | Assembly Phase | `assemblyPhase` | mm | −1.30 |

Every input is read back by id with `inputs.itemById(id)` on the command's top-level
`commandInputs`, grouped or not; input ids are unique across the whole command, which is what
lets that lookup reach into a group. `processInputs` raises naming the id if any lookup returns
`None`, so a lookup that does not reach into a group fails at once and by name.

Module-level constants for every input id: `INPUT_ID_RIBBON_WIDTH = 'ribbonWidth'` and so on.
One more module-level constant is not a dialog input: `CELL_TEETH = 4`, the number of teeth the
lofted cell of §2 holds, written `cellTeeth` in this spec. Every count that depends on it is
derived from it in `processInputs`, and this spec states those counts for `cellTeeth = 4` and,
as the fallback the first cell measurement was made at, for `cellTeeth = 1`.
Value inputs are `addValueInput(id, label, unit, ValueInput.createByReal(default))` with the
default in internal units (`[PB-DIALOG-DEFAULT-UNITS]`): `mm/10` for a length, radians for an
angle, the bare count for `toothCount`.

The three selection inputs are `addSelectionInput(id, label, tooltip)`, each with these filters
(`[PB-SELECTION-FILTER-ENUM]`) and `setSelectionLimits(1, 1)` (`[PB-SELECTION-DECL]`):

- Target Plane: `ConstructionPlanes` + `PlanarFaces`; tooltip `Plane the cage's axis is normal to`.
- Centre Point: `ConstructionPoints` + `SketchPoints`; tooltip `Centre of the mechanism`.
- Parent Component: `Occurrences` + `RootComponents`; pre-selects `get_design().rootComponent`;
  tooltip `Component the mechanism is created under`.

Read raw values with `design.unitsManager.evaluateExpression(input.expression, units)`, which
returns internal units — cm for length and **radians** for angle (`[PB-EVAL-EXPRESSION]`).

**Range checks, all raised as a clear error naming the offending field and the bound:**

- `ribbonWidth`, `ribbonThickness`, `toothPitch`, `twistLead`, `collarHalf`, `collarWall` and
  `clearance` must be `> 0`; `roofAllowance` must be `>= 0`, zero cutting every bore to the
  clearance alone; `toothCount` must be a whole number `>= 4`.
- `toothHeight` must be `> 0` and `< ribbonWidth/2`.
- `engagement` must be `> 0` and `<= toothHeight`. Past the tooth height the two blanks foul each
  other rather than meshing.
- `twistLead` has no upper bound: a very long lead approaches two straight racks pushing each
  other, which is the degenerate case Segerman names, and nothing here forbids it. The section
  floor in §2 is what keeps the tooth at a long lead.
- `crossAngle` must lie strictly between 0° and 180°. At either end the axes are parallel and the
  engaged zone below has no length. Only 80° has been proved to drive; the search covered
  38.5°–100° (`mesh-search.md`), and nothing here clamps to it.
- `assemblyPhase` must lie strictly within `±toothPitch`. A phase a pitch further on is the same
  phase with the ribbon's ends moved a pitch. Only the default has been measured to sit in the
  free window.
- Both bores, each cut a millimetre past the wall (§4), have to lie within the ribbon's length:
  `cageRadius + collarHalf + 1 mm < toothCount*toothPitch/2`, the millimetre being the cut's
  margin. The message names `cageRadius`.

The sleeve's own four checks come next, in this order, each naming the field given and the
bound; `sleeveRefusal` in `sleeve_test.go` is the same four in the same order, and
`TestSleeveInputsAreChecked` reaches each with an input that passes every check before it.
`Ri = cageRadius - collarHalf` is the sleeve's inner radius and `c = hypot(W/2 + clearance,
T/2 + clearance + roofAllowance)` the corner radius of the level bore's roof side, the furthest
any bore's corner stands from its axis, 8.058 mm at the defaults; every check takes it for all
four bores.

- **The channel starts in the hollow**, naming `cageRadius`: `hypot(c, 1 mm) < Ri`. Each bore's
  cut starts a millimetre before the channel's corner first reaches the inner face, at station
  `sIn = sqrt(Ri^2 - c^2) - 1 mm` (§4), and the check is `sIn > 0`. That needs the corner inside
  the inner radius by more than the millimetre allows: with `c < Ri` alone, a corner within about
  0.04 mm of a 12 mm inner radius would put `sIn` at or before the middle, so a gear's `bore-` and
  `bore+` lines would overlap or run backwards and the build would fail inside Fusion. A 4 mm
  clearance fails it, and so does a cage radius 0.02 mm past `collarHalf + c`, where `sIn` is
  −0.44 mm.
- **The mesh stays visible along the axis**, naming `cageRadius`:
  `hypot(axialWindow, hypot(W/2, T/2)) + clearance <= Ri`, with
  `axialWindow = 1.5 * sqrt(W^2 - A^2) / sin(Sigma)`, 7.79 mm at the defaults, the reach the
  proof's `axialWindow` samples. The left side bounds how far the engaged zone's footprint along
  the frame's axis reaches from it, 10.98 mm, and with the clearance 11.18 mm against the 12 mm
  inner radius at the defaults; `TestSleeveKeepsTheMeshVisibleAlongTheAxis` walks the footprint
  itself, 10.92 mm. The bound is at least `axialWindow`, so it also keeps the engaged zone
  inside the hollow. A 1.5 mm engagement fails it.
- **The end faces keep `collarWall`**, naming `cageRise`:
  `cageRise >= A/2 + c + collarWall`, 18.11 mm at the defaults. No point of a channel is further
  from the middle plane than its axis, `A/2`, plus `c`. That is a sufficient bound, not the gap:
  the channels reach 14.54 mm from the middle inside the wall, which leaves 4.21 mm of end wall
  at the 18.75 mm default, and a 17.5 mm rise, which this check refuses, leaves 2.96 mm. The
  check also refuses rises from about 17.55 mm up to 18.11 mm, which the channels would allow.
- **The wall between two neighbouring bores keeps `collarWall`**, naming `collarWall`: the
  separation §4 computes under "The wall between the bores" must be at least `collarWall`; the
  message names the two bores and the separation. It is 5.078 mm at the defaults, across the gap
  between the two `-R` bores. It depends on nearly every input at
  once, so it is computed rather than bounded by a closed form: `TestSleeveWindowsFollowTheSize`
  finds it refusing a 4 mm `collarWall` at a 70° crossing (3.585 mm) or with a 0.9 mm clearance
  (3.294 mm), and accepting a sleeve scaled to 2/3 with the 3 mm wall kept (3.064 mm), a 5 mm
  `collarWall` (5.078 mm), and a 4 mm `collarWall` at the defaults (5.078 mm) or at a 14.5 mm
  cage radius (4.584 mm).

`collarHalf` trades grip against travel. A thicker wall holds the gear closer to its screw motion
and shortens the travel by twice its own growth, because a ribbon runs
`toothCount*toothPitch - 2*(cageRadius + collarHalf)` through its bores.

After the four checks the window search of §4 runs. It refuses nothing: a gap it finds no room in
gets no window, and the build logs which and why (§4, "When a gap has no room").

**No range is enforced on either Mounting Angle, and none is asserted here.** At the 0.90 mm
engagement 12° on both is the smallest equal pair that drives at this twist and 14° the default;
what happens elsewhere is the proof's to map, and
this spec does not clamp what has not been measured.

## Sketch Discipline

Every sketch must report `isFullyConstrained` before it is consumed, and the build raises naming the
sketch when one does not (`[PB-SKETCH-FIRST]`, `[PB-FULL-CONSTRAINT]`). No sketch here carries text,
so none is exempt. Six rules hold in every sketch of this build:

- **Only the Anchor sketch projects anything, and every other sketch's references are fixed
  points of its own** (`[SCREW-F-REFERENCES]`, `[PB-PROJECT-NOT-FIXED]`). The Anchor sketch
  projects the selected point and binds the Anchor Line to the projection exactly as the bevel
  gear's Anchor sketch does, which reads fully constrained in Fusion. Every later sketch takes
  its references — the ends of a sweep path, the axis point of a section, the corners of a cell
  section, the corners of a window — as
  **reference points**: each is a world point of the frame of §1, mapped in with
  `sketch.modelToSketchSpace`, given `z = 0` when it is meant to lie on the sketch's plane
  (`[PB-SKETCH-ZERO-Z]`; the one exception is the next rule) and added with
  `sketch.sketchPoints.add`. The sketch's curves are drawn **sharing** those points
  (`[PB-SHARE-XOR-COINCIDENT]`), and after the last curve that uses a reference point is drawn,
  and before any constraint or dimension is added, the point is set `isFixed = True`. A
  **reference line** is a construction line between two reference points, so it has no freedom
  and carries no dimension. Nothing is projected into these sketches and
  `intersectWithSketchPlane` is not called: the point where a sweep path pierces its section
  plane is the path's own start, `origin_g + s*dir_g` for the span's first station `s`, and
  that is what the reference point is placed at. A circle centred on a reference point is
  instead created at that point's position with its `centerSketchPoint` set `isFixed = True`
  (`[PB-CIRCLE-CENTER]`), and no reference point is added for it.
- **The Cell Sections sketch is the one sketch whose points lie off its plane, and it keeps
  their `z`** (`[SCREW-F-CELL-LOFT]`, `[PB-3D-SKETCH-SECTIONS]`). Its points are the corners of
  every section of the tooth cell (§2), which stand at every height above and below the Gear
  Axis Plane the sketch is drawn on. `[PB-SKETCH-ZERO-Z]` zeroes the `z` of a point that is
  meant to lie on the plane and that rounding has pushed off it; a point meant to lie off the
  plane keeps the `z` that `modelToSketchSpace` gives it. Fusion accepted such a sketch on
  2026-09-28, read it fully constrained at 11, 41 and 81 sections, and found one profile per
  section.
- **Every angular dimension is taken against whichever of two perpendicular reference lines puts
  it between 45° and 135°** (`[PB-ANGULAR-DIM]`). Fusion cannot dimension the angle between two
  lines that are nearly parallel, and the one angle this build dimensions — a section's twist at
  its station, in the rectangle scheme of §4 — runs through 0° and 180° along the ribbon. The
  scheme names its two references; the build computes the angle it is about to seed, takes the
  reference that keeps it inside that range, and puts the dimension's text point inside the
  wedge it measures. The value written is the angle between two **rays**, each named in the
  step, from the point where the two lines meet; the text point sits inside that wedge.
- **Every seed is the solved position** (`[PB-SEED-NEAR]`), computed in Python in the frame of §1
  and mapped in with `modelToSketchSpace`, so the solver has nothing to move and a dimension's
  side is the seed's (`[PB-DIM-VALUE-SEMANTICS]`). **Every point mapped in that is meant to lie
  on the plane has its `z` set to 0 before it is used** — a reference point, a raw seed, an
  arc's through point, a circle's centre and a dimension's text point alike
  (`[PB-SKETCH-ZERO-Z]`). Fusion's section planes do not sit exactly where this build's
  arithmetic puts them, and the first Fusion load failed on a collar section whose points all
  landed `4.4e-6` cm off its plane.
- **No `addPerpendicular` on a line that only one of its ends anchors.** A line drawn from a
  fixed point, given a length and made perpendicular to a reference has two solutions, one each
  side of the reference, and the proof's sketch gate refuses a sketch that admits a mirror
  image. `addPerpendicular` is used only in the rectangle scheme of §4, where the line it turns
  already has both ends tied to other lines.
- **Sketch computing is deferred while the two heavy kinds of sketch are drawn, and nowhere
  else** (`[PB-SKETCH-DEFER]`, `[SCREW-F-DEFER]`): the Cell Sections sketch of §2, and the four
  bore section sketches of §4. In those, set
  `sketch.isComputeDeferred = True` right after the sketch is created and named and before its
  first point, and set it back to `False` after the last curve, constraint, dimension or
  `isFixed`, before `isFullyConstrained` or `profiles` is read. Measured in Fusion on
  2026-09-28: a collar section fell from 0.45 s to 0.23 s and a one-tooth Cell Sections
  sketch from 0.88 s to 0.09 s, each still reading fully constrained with its profiles found.
  The Anchor sketch does not defer, because deferral has not been measured under a projection;
  the Paths, Sleeve and Window sketches do not, because they are a few fixed points with lines
  or circles and were not measured.

The four bore section sketches of §4 share one rectangle scheme, stated there, that leaves no
freedom, so there is no under-constrained case to exempt, unlike the bevel tooth profile. Every
other sketch is built from reference points alone: lines between them and circles centred on
them, with nothing left to solve.

## Method contract — call graph

```
generate(inputs)
  → processInputs(inputs)                    # read, check, precompute; each gear's level bore, the bore-wall check and the window search run here
  → buildComponentTree()                     # Screw Gearing + Design + 3 children
  → buildAnchor()                            # anchor sketch, centre point, reference direction, axis planes, n̂
  → buildGear(index)   x2                    # per gear: paths → cell → repeat → one body
      → buildSweepPaths(index)               # Paths sketch: the two bore-path lines on the gear's axis
      → buildToothCell(index)                # one Cell Sections sketch (41 sections at the defaults) → one loft
      → repeatCellByDoubling(index)          # copy + screw-move + join, in cells; a remainder cell when N is not a multiple of cellTeeth
  → buildCage()                              # the sleeve, one twisted sweep cut per bore, one extrude cut per window; logs the end to print on
  → relocateBodies()                         # moveToComponent into Gear A / Gear B / Cage
  → solids.hide_construction_geometry(self.designOcc.component)
```

### What the build makes

Counted at the defaults, in timeline entries, so a Fusion load can be checked against it. The
first two Fusion loads (`[SCREW-F-FIRST-LOAD]`) built the video frame's geometry as 136 sketches,
134 construction planes and 79 features, one sketch and one plane per loft section, and took
about five minutes; the third built it as 23 sketches, 12 planes and 56 features by replacing
every per-section sketch and plane with one sweep or one sketch. The gears' rows below are that
construction's, which the diagnostics of 2026-09-28 measured piece by piece; the sleeve's rows
replace the ring, rods, loop and collars and have not been built in Fusion.

| Part | Sketches | Planes | Features, `cellTeeth = 4` | Features, `cellTeeth = 1` |
|---|---|---|---|---|
| Anchor (§1) | 1 | 0 | 0 | 0 |
| Gear axis planes (§1) | 0 | 2 | 0 | 0 |
| Per gear: Paths sketch (§1) | 1 | 0 | 0 | 0 |
| Per gear: Cell Sections sketch and loft (§2) | 1 | 0 | 1 | 1 |
| Per gear: doubling (§3), 3 features a round | 0 | 0 | 15 (5 rounds) | 21 (7 rounds) |
| Both gears, subtotal | 4 | 0 | 32 | 44 |
| Sleeve (§4): Sleeve sketch, extrude | 1 | 0 | 1 | 1 |
| Bores (§4): 4 planes, 4 sketches, 4 sweep cuts | 4 | 4 | 4 | 4 |
| Windows (§4): Window Plane, 2 sketches, 2 extrude cuts | 2 | 1 | 2 | 2 |
| Relocate (§5): 3 `moveToComponent` | 0 | 0 | 3 | 3 |
| **Total** | **12** | **7** | **42** | **54** |

That is 61 timeline entries at `cellTeeth = 4` and 73 at `cellTeeth = 1`, plus the five
component creations: 66 and 78 in all, against 96 and 108 for the video frame. The third Fusion
load (`[SCREW-F-FIRST-LOAD]`) counted exactly 96 timeline items for that frame. Check a load
against the timeline count, not against `Component.features.count`: at that load, summed over
the five components, it read 62, six more than the 56 features the timeline held, all six in
`Design`, for a reason not yet measured. A remainder cell (§3) adds one sketch, one loft and one
join per gear that needs one; the defaults need none. A window the search finds no room for
(§4) takes one sketch and one feature off the count, and when neither window has room the
Window Plane is not made either. The heaviest sketches measured are the bore sections of §4, at
0.23 s each for the video frame's collar sections drawn by the same scheme, and the two Cell
Sections sketches at 0.37 s each with computing deferred; each collar sweep took 0.04 s, each
cell loft 0.23 s, and the doubling's copies, moves and joins were not timed on their own. The
two searches of §4 that run before any feature took 2.2 s together at the defaults in a CPython
port of them, run outside Fusion, at the defaults before 2026-10-02, and 4.4 s at every length
scaled by 1.5; Fusion has not run
them.

## Instructions

### 1: Anchor and frame

Create the anchor sketch, named `Anchor`, on the user-selected plane and project the selected
point into it, `sketch.project(point).item(0)`; this is the one projection in the build
(`[SCREW-F-REFERENCES]`). Draw the **Anchor Line** through it: a line from two raw `Point3D`
seeds 0.5 cm either side of the projected point along the sketch's own x axis, so it is 10 mm
long with its end to the right of its start, and constrain it with four things and nothing else,
which is the bevel gear's Anchor sketch and reads fully constrained in Fusion:

- `addCoincident(projectedPoint, line)` — the centre lies on the line — **and**
  `addMidPoint(projectedPoint, line)` — the centre bisects it. Both, not the midpoint alone,
  as the bevel spec requires of its own Anchor sketch. The proof's sketch engine emits the
  coincident row as part of its midpoint constraint, so the compiled proof writes the midpoint
  alone, as `proof/bevelgear` does.
- `addHorizontal(line)` — sketch-local, so it survives a tilted plane (`[PB-REFLINE-DIRECTION]`).
- A **horizontal** distance dimension from the line's start to its end
  (`addDistanceDimension(start, end, HorizontalDimensionOrientation, textPoint)`), value 10 mm.
  Not an aligned one: midpoint, horizontal and an aligned length are satisfied by the line in
  either of its two end-for-end orientations, and the proof's sketch gate refuses that as
  ambiguous; a horizontal distance from start to end runs one way, so only the seeded orientation
  satisfies it.

Midpoint, horizontal and the horizontal distance take the line's four degrees of freedom, so it
has none, and its absolute direction is arbitrary. The build raises unless the sketch reports
`isFullyConstrained`, and only then reads the frame from world geometry
(`[PB-WORLDGEO-CONSTRAINED]`, `[PB-WORLD-FRAME]`): `C` is the projected point's
`worldGeometry`, and `ê` is the unit vector from the line's `startSketchPoint.worldGeometry` to
its `endSketchPoint.worldGeometry`. The plane's normal is `n̂`, read in the next paragraph but
one. No later sketch projects the point or the line: every other sketch takes `C` and `ê` as
numbers, and the Anchor Line itself is passed once more, as the line the Window Plane of §4 is
built on.

Compute both axes from `ê` and `n̂`:

```
dirA = rotate(ê, +Sigma/2, about n̂)      originA = C - (A/2)*n̂
dirB = rotate(ê, -Sigma/2, about n̂)      originB = C + (A/2)*n̂
```

Gear A's cross-section frame is `û_A = +n̂`, `v̂_A = dirA × û_A`; gear B's is `û_B = -n̂`,
`v̂_B = dirB × û_B`. **`û` points at the other gear** — that is what makes `Phi` mean the same thing
for both parts, and it is why the two gears are the same part rather than mirror images. A point
of gear `g` at station `s` with section coordinates `(u, v)` is
`origin_g + s*dir_g + (u*cos theta - v*sin theta)*û_g + (u*sin theta + v*cos theta)*v̂_g`, with
`theta = s/Lambda + Phi_g`; every seed below is that point. The third direction of the frame is
`k̂ = n̂ × ê`, level and square to `ê`; the proof's `pair` puts `ê`, `k̂` and `n̂` on its X, Y
and Z axes with `C` at the origin, and §4 names directions by them.

**The axis planes, and the sign of `n̂`.** Two construction planes are offset from the selected
plane itself (`[PB-USE-SELECTED-PLANE]`, `[PB-CONSTRUCTION-PLANES]`): `Gear A Axis Plane` by
`-A/2` and `Gear B Axis Plane` by `+A/2`, both with `setByOffset`. Fusion offsets along the
selected entity's own normal, and the build does not assume its sign: `n̂` is read from the Gear
A Axis Plane after it is made — the unit normal of its `geometry`, signed so that `C` lies `+A/2`
along it — and the build checks that `C` is `A/2` from each of the two planes
(`[SCREW-F-NORMAL-SIGN]`). No later plane is offset from the selected plane.

**The Paths sketches.** `Gear A Paths` on the Gear A Axis Plane and `Gear B Paths` on gear B's,
each holding the two lines that gear's bores are swept along (§4), both on the gear's axis,
which lies in that plane. The `+R` bore's cut spans stations `sIn` to `sOut` of the gear's axis
and the `-R` bore's `-sOut` to `-sIn`, with `sIn = sqrt(Ri^2 - c^2) - 1 mm` and
`sOut = cageRadius + collarHalf + 1 mm` (§4): 7.892 and 19 mm at the defaults. The sketch holds
four reference points (Sketch Discipline), at `origin_g + s*dir_g` for `s` = `-sOut`, `-sIn`,
`sIn` and `sOut`, all on the plane, and two solid lines — `bore-` from `-sOut` to `-sIn` and
`bore+` from `sIn` to `sOut` — each drawn from its negative station to its positive one, sharing
both points, so its start is its negative end and it runs along `+dir_g`; then all four points
are set `isFixed = True`. The lines carry no dimension and no constraint, and nothing else is in
the sketch. Both lie on one line with a gap between them; Fusion read a sketch of four such
lines, overlapping, fully constrained on 2026-09-28, and a path made from one of its lines with
chaining off held that one line (`[PB-PATH-FROM-SKETCH]`). A line whose two ends are fixed has a
`worldGeometry` the build can trust (`[PB-WORLDGEO-CONSTRAINED]`), and a line held by dimensions
from a projected point does not. The section plane of each bore is `setByDistanceOnPath` on its
own line at fraction `0` (`[PB-CONSTRUCTION-PLANES]`; pass the line directly), so it stands at
the span's negative end, square to the axis; the point where the line pierces it is the span's
first station, `origin_g + s*dir_g`, and Fusion put that plane's origin on the station to four
decimals of a millimetre. The sweep's path is `features.createPath(line, False)` on the same
line (`[SCREW-F-TWISTED-SLOT]`).

### 2: The tooth cell

One cell is `c` tooth pitches of the finished ribbon, at the ribbon's negative end: stations `s`
from `s0 = Z0 - L/2` to `s0 + c*P`, where `Z0` is the gear's tooth phase — `0` for gear A and the
`assemblyPhase` input for gear B ("Defaults") — and `c = min(cellTeeth, N)` is the number of
teeth the cell holds, 4 at the defaults (`cellTeeth`, "Variables"). Gear B's whole ribbon is
gear A's advanced by `Z0` under its own screw motion, ends and teeth alike, so the same cell
built `Z0` further along its axis and repeated the same way is gear B. Build it once per gear.

**The section count is derived, not pinned, and it is not user-configurable.** The cell is lofted
through `c*n + 1` sections, `n` steps to the tooth, where `n` is the larger of two counts:

- the smallest that keeps the twist between neighbouring sections under 2°,
  `ceil((P/Lambda) / 2°)`, 10 at the defaults. A surface ruled straight between two sections
  cuts the corner of the true helicoid at the crest by `(W/2)*(1 - cos(dtheta/2))`, `dtheta`
  the twist per step: 1.0 µm at the defaults' 1.91°, under a hundredth of the 0.46 mm backlash,
  which is the bound the proof holds it to (the shortfall grows with the width and the backlash
  with the pitch, so it is not held to a fixed number of microns);
- eight. A straight chord of the cosine between two sections falls `(H/2)*(1 - cos(pi/n))` short
  of it at the deepest point whatever the twist: 0.064 mm at the defaults' ten steps, 14% of the
  backlash, and 0.100 mm, 3.8% of the tooth height, at eight. The proof holds it under a fifth of
  the backlash; it was 7.8% of the 0.82 mm backlash before 2026-10-02, when the backlash narrowed
  and the printed ribbon, and so its sections, stayed as they were. That floor is what keeps a slow
  twist from lofting the tooth through two or three sections and losing it: at a 400 mm lead the
  twist alone asks for two sections, and the cell is built through nine.

That is 10 steps to the tooth at the defaults: 41 sections in a four-tooth cell, 11 in a
one-tooth cell. `TestLoftSectionCountHoldsTheHelicoid` holds both bounds over leads from 20 mm
to 400 mm.

**What the loft is, and what the count guarantees for it.** Both bounds are arithmetic about a
*ruled* loft, whose surface runs straight between neighbouring sections. Fusion builds a ruled
loft only through exactly two sections; a loft through more passes through every section and is
fitted smoothly between them, and `LoftFeatureInput` has no option to make it ruled — its
sections carry only end conditions (`[SCREW-F-CELL-LOFT]`). The cell is **one loft through all
`c*n + 1` sections**, so it is the smooth kind. What the count guarantees for it is that the
built cell carries the exact rotated rectangle at each of its sections, `dtheta` apart, and that
between stations its surface interpolates those rectangles; the chord figures above are the
departure of the ruled loft through the same sections, and they are the only figures the proof
has. What the smooth surface does between sections was measured in Fusion on 2026-09-28, at
the defaults of that day, the 10 mm ribbon (`[SCREW-F-CELL-LOFT]`): at one, four and eight
teeth, probes 0.04 mm inside and 0.04 mm outside the toothed edge and a face, at the midpoint
between every pair of sections, all fell on the right side of the built surface — 40 of 40,
160 of 160 and 320 of 320 — and the built volume was 0.06% to 0.23% under the ruled loft's. So
at that size the built surface stayed within 0.04 mm of the helicoid between sections, under a
tenth of the 0.44 mm backlash of that day; the 1.5× ribbon has the same section spacing and has
not been measured. The proof cannot measure that ("What the proof cannot reach"), and a Fusion
load is what checks it at other inputs. A chain of `c*n` two-section lofts would be ruled and would carry the bounds
literally, at the cost of that many lofts and joins per cell, and this spec keeps the one loft.

**The Cell Sections sketch.** All the cell's sections go in **one sketch**, named
`{gearLabel} Cell Sections`, on the gear's Axis Plane (`[SCREW-F-CELL-LOFT]`,
`[PB-3D-SKETCH-SECTIONS]`), drawn with sketch computing deferred (Sketch Discipline). No
section plane is made. Section `k`, for `k` from `0` to `c*n`, is the cross-section at station
`s_k = s0 + k*P/n`: a rectangle spanning `u` from `-W/2` to `Utooth(s_k)` and `v` from `-T/2` to
`+T/2`, rotated about the axis by

```
theta_k     = s_k/Lambda + Phi
Utooth(s_k) = W/2 - H/2 + (H/2)*cos(2*pi*(s_k - Z0)/P)
```

Its four corners are the world points of §1 at `(uB, -hv)`, `(uF, -hv)`, `(uF, hv)` and
`(uB, hv)`, with `uB = -W/2`, `uF = Utooth(s_k)` and `hv = T/2`, each mapped in with
`modelToSketchSpace` and added with `sketchPoints.add` **with its `z` kept**: the corners lie
off the sketch's plane on purpose (Sketch Discipline). Four solid lines share them in order —
`L1` from the first corner to the second, `L2` on to the third, `L3` on to the fourth, `L4` back
to the first (`[PB-SHARE-XOR-COINCIDENT]`) — and the build keeps the four lines of each section
together, because they are what the loft is fed. After the last line of the last section is
drawn, every point in the sketch is set `isFixed = True`. Nothing else is in the sketch: no
construction line, no constraint, no dimension. Then set `isComputeDeferred` back to `False`,
raise unless the sketch reports `isFullyConstrained`, and raise unless `sketch.profiles.count`
is `c*n + 1`: Fusion finds one profile per section, each planar in its own station's plane, and
read the sketch fully constrained at 11, 41 and 81 sections on 2026-09-28. The profiles are
not what the loft is fed, because nothing says which profile is which station.

**Loft the sections in station order** (`[PB-LOFT]`, `[SCREW-F-CELL-LOFT]`). Create the loft
with `loftFeatures.createInput(NewBodyFeatureOperation)`; for each section in order of `k`, put
its four lines in an `ObjectCollection`, make its path with `features.createPath(collection,
False)` (`[PB-PATH-FROM-SKETCH]`; never `Path.create`, which raises in this component), and
`loftSections.add(path)`; then `loftFeatures.add(input)`. The result is `c` pitches of the
twisted toothed ribbon in one body with six faces; raise unless the feature leaves exactly one
body and that body `isSolid` (`[PB-EMPTY-RESULT]`). Measured on 2026-09-28 at the 10 mm ribbon:
the four-tooth loft took 0.23 s and held 0.164158 cm³ against the ruled loft's 0.164470.

### 3: Repeat the cell by doubling

The finished ribbon is the cell, `c` teeth, repeated under the **screw step**

```
Step(k) = translate(k*P along the axis) ∘ rotate(k*P/Lambda about the axis)
```

Write `N = q*c + r`, with `q = floor(N/c)` whole cells and a remainder of `r = N mod c` teeth:
`q = 17` and `r = 0` at the defaults, `q = 68` at a one-tooth cell. Build the `q` cells by
doubling rather than by `q` separate placements. A **round** is copy, move, join: copy the
current body, move the copy by `Step(m*c)` where `m` is the number of cells the body holds, join
the two, and the body holds `2m` cells. Doubling alone reaches only a power of two, and copying
the whole body for the rest would overlap it, so the rest is made of unmoved copies put aside on
the way up:

- Write `q` in binary. Start with the cell, `m = 1`.
- For each bit of `q` below its top bit, lowest first: if that bit is set, take a copy of the
  body and keep it unmoved — an **aside** of `m` cells; then double.
- After the last doubling `m` is the top bit's power of two. Move each aside into place, largest
  first, by `Step(m*c)`, join it, and add its cells to `m`. The last join brings `m` to `q`.

That is `floor(log2 q) + popcount(q) - 1` rounds and as many joins: 5 at the defaults — `q = 17`,
one aside of the single cell, taken before the first doubling, then four doublings to 16 cells,
and the aside moved by `Step(64)` — and 7 at a one-tooth cell — `q = 68`, one aside of 4 cells,
taken when the body held 4, six doublings to 64, and the aside moved by `Step(64)` — against 67
placements. When `q = 1` there is no round. Every placement is
exact, because the body genuinely is invariant under `Step`
(`TestRibbonIsInvariantUnderItsScrewStep`). Every move is by a whole number of teeth already
built, so each join meets its neighbour at one shared planar cross-section and leaves no sliver
and no overlap; `TestDoublingScheduleCoversTheRibbon` runs the schedule at every count from 4 to
512 with cells of one, three and four teeth.

**The remainder.** When `r > 0`, the last `r` teeth are a second, shorter cell, built where they
belong rather than moved there: a sketch `{gearLabel} Cell Remainder` and a loft by the recipe of
§2 with `c` replaced by `r`, so `r*n + 1` sections at stations from `s0 + q*c*P` to `s0 + N*P`,
joined into the body after the last round. Its first section is the body's last, so the join
meets at a shared cross-section like every other. The defaults have none; 69 teeth in
four-tooth cells would have one of one tooth.

Copy with `copyPasteBodies.add(body)` and take the copy from the feature's `bodies`
(`[SCREW-F-COPY-BODY]`); move with `moveFeatures.createInput2(bodies)` → `defineAsFreeMove(matrix)`
→ `add(input)` (`[PB-MOVE-ROTATE]`, `[SCREW-F-SCREW-STEP]`); join with a `combineFeatures` join and
check that it leaves exactly one body: the combine feature's own `bodies.count` is 1, and that
body is the ribbon from then on (`[PB-EMPTY-RESULT]`, `[SCREW-F-JOIN]`). After the last
join name the body `Gear A` or `Gear B`.

**A zero-angle matrix is a no-op that Fusion rejects** (`[PB-MOVE-ROTATE]`). A screw step is never
zero for `k >= 1`, so no guard is needed here, but do not "optimize" a `k = 0` case into the loop.

### 4: The cage

The cage is the **sleeve**: one tube about the frame's axis, the four bores cut through its wall
by twisted sweeps, and up to two windows cut through the wall by extrusions. Heights are along
`n̂` from the centre `C`; gear B's axis is on the `+n̂` side and gear A's on the `-n̂` side.
`proof/screwgear/sleeve_test.go` proves the shape, and where the build follows one of its
functions step for step the function is named. Its lengths, with their values at the defaults:

```
Ri   = cageRadius - collarHalf                inner radius, 12 mm
Ro   = cageRadius + collarHalf                outer radius, 18 mm
hw   = W/2 + clearance                        the bore's half-width, 7.70 mm
ht   = T/2 + clearance                        the bore's half-thickness, 2.075 mm
a    = roofAllowance                          the roof allowance, 0.30 mm
c    = hypot(hw, ht + a)                      the furthest any bore's corner stands from its axis, 8.058 mm
sIn  = sqrt(Ri^2 - c^2) - 1 mm                where a +R bore's cut starts on its axis, 7.892 mm
sOut = Ro + 1 mm                              where it ends, 19 mm
```

The sleeve prints standing on its **`−n̂` end**, the end below the selected plane, with no
support. Every outside face is a vertical cylinder or a level end face and every face of a
window is upright or at 45°, so the only material the printer lays on air is the roofs of the
four bores: 1341 cells of the proof's 0.25 mm grid, 84 mm², standing on that end
(`TestSleevePrintsStandingOnEitherEnd`). Standing on the other end it prints too, 1318 cells,
but the roof allowance is then on the floors and the bridged roofs keep only the clearance.

**The tube** (`[SCREW-F-SLEEVE]`). One sketch, `Sleeve`, on the selected plane
(`[PB-USE-SELECTED-PLANE]`): two circles `addByCenterRadius` at `C`, mapped in with
`modelToSketchSpace` and given `z = 0` (`[PB-SKETCH-ZERO-Z]`), of radii `Ri` and `Ro`, each with
its `centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`) and a diameter dimension of `2*Ri`
or `2*Ro` whose text point sits on its circle, off the centre (`[PB-RADIAL-DIM]`). Nothing else is
in the sketch. It has two profiles, the inner disc and the ring between the circles; the ring is
the profile whose `profileLoops.count` is 2, and the build raises unless exactly one profile has
two loops (`[PB-EMPTY-RESULT]`). `find_profile_by_curve_counts` cannot pick it, since it treats a
circle as a curve type that disqualifies a loop. Extrude the ring as a new body with
`setSymmetricExtent(ValueInput.createByReal(cageRise), False)`, `False` making the value each
side's length (`[PB-THROUGH-CUT]` for the argument), so the tube runs from `-cageRise` to
`+cageRise`: 37.5 mm tall with a flat 565.5 mm² ring at each end. Raise unless the feature's
`bodies.count` is 1. That body is the cage from here on.

**The bores.** One per crossing: a gear's `+R` bore runs through the wall where its axis crosses
the circle of radius `cageRadius`, at station `+cageRadius` of the axis, and its `-R` bore at
`-cageRadius`. Each is the crest rectangle plus the clearance, `2*hw` by `2*ht`, 15.4 by 4.15 mm,
turned at every station `s` to the ribbon's own angle `s/Lambda + Phi_g`. That is the channel the
video frame's collars were cut with, and `TestSleeveBoresAreTheSameChannels` pins it: the
rectangle, the stations, the twist, the angles at the crossings, 123.1° and −95.1°, and which
bore carries the roof allowance on which face. Only the span the cut covers is the sleeve's own.
The tube's inside is a cylinder, not a plane square to the ribbon, so the channel's corners reach
into the wall before its centre line does: a corner first touches the inner face by station
`sqrt(Ri^2 - c^2)`, 8.892 mm. The cut runs from a millimetre before that, `sIn`, where the whole
section is in the hollow, to a millimetre past the outer face, `sOut`, where the whole section is
outside: `[sIn, sOut]` for a `+R` bore and `[-sOut, -sIn]` for a `-R` bore, 11.108 mm of axis and
80.78° of turn. The assembly phase moves
gear B's ribbon along its axis and not its bores: any stretch of the ribbon fits a bore, and the
ribbon can go in at any phase a pitch apart, so the frame has no way to know the phase.

**The roof allowance** (`levelBore`, `roofSide`, `boreOpening`). With the sleeve standing on its
`−n̂` end, one bore of each gear passes through level inside the wall, and the printer bridges its
roof across the hole; at the defaults that is each gear's `-R` bore, whose long faces lie flat at
station −14.30 mm. The second sleeve printed those two bores too tight and the two `+R` bores
not (`[SCREW-F-PRINT-2]`). So each gear's **level bore** has its roof face moved out by
`roofAllowance`, and every other face of every bore stays at the clearance:

- The level bore is, of the gear's two bores, the one whose long faces come nearest level over
  the wall's span on its centre line. A long face runs along the section's `u`, which stands
  `theta` from `û_g`, and `û_g` is along `±n̂`, so a long face is level where `cos(theta)` is zero.
  For the bore at `σ*cageRadius`, `σ` being `-1` or `+1`, take `theta` at the two stations
  `σ*(cageRadius - collarHalf)` and `σ*(cageRadius + collarHalf)`; when some `pi/2 + k*pi` lies
  between them its tilt is 0, and otherwise it is the smaller `|cos theta|` of the two. The bore
  with the smaller tilt is the level bore, the `-R` bore when they tie. At the defaults the `-R`
  bores' tilt is 0 and the `+R` bores' is `|cos 101.3°|`, 0.195.
- The roof face is the long face that is up when the sleeve stands on its `−n̂` end: the `+v`
  face when `v̂(sc)·n̂ = -sin(theta(sc))*(û_g·n̂)` is positive at the bore's crossing station
  `sc = σ*cageRadius`, and the `-v` face otherwise. `v̂_g·n̂` is zero, so that is the whole
  product. At the defaults it is gear A's `+v` face and gear B's `-v` face. The sign holds over
  the whole cut: `v̂·n̂` changes sign only where a long face stands upright, 90° of twist, 12.4 mm
  of axis, from where it lies level.
- The level bore's rectangle spans `v` from `vLo = -ht` to `vHi = ht + a` when its roof is the
  `+v` face, and from `vLo = -ht - a` to `vHi = ht` otherwise: 4.45 mm through at the defaults.
  Every other bore spans `-ht` to `ht`, and every bore spans `u` from `-hw` to `hw`.

The allowance adds no move of the ribbon along or across the axes and no roll: the other bore and
the level bore's floor hold the ribbon to the clearance there. It lets the ribbon tilt, its level
bore's end rising into the room while the other bore holds, which carries the crossing 0.273 mm
instead of 0.200 mm toward the other ribbon for gear A and away from it for gear B
(`TestRoofAllowanceAddsOnlyATilt`). After the sleeve is built the build logs, with `futil.log`
(`[PB-LOGGING]`), `Print the cage standing on its end below the selected plane: the roof
allowance is on the bridged roofs that way up.`, since nothing on the part shows which end that
is.

**Cut each bore with a twisted sweep** (`[SCREW-F-TWISTED-SLOT]`, `[PB-SWEEP-TWIST]`): the bore's
rectangle, drawn by the rectangle scheme below on the bore's own plane,
`{gearLabel} Bore {-R|+R} Plane`, `setByDistanceOnPath` at fraction `0` of the bore's `bore-` or
`bore+` line of §1, in the sketch `{gearLabel} Bore {-R|+R}`, at that plane's station, `-sOut` or
`sIn`; then swept along that line as `sweepFeatures.createInput(profile, path,
CutFeatureOperation)`, with `path = features.createPath(line, False)`, `twistAngle` set to
`ValueInput.createByReal(+(sOut - sIn)/Lambda)`, `participantBodies` set to a list holding the
cage body alone, and nothing else set; then `sweepFeatures.add`. The sign is positive: the path
runs along `+dir_g`, the section's angle `s/Lambda + Phi` grows with `s`, and a positive
`twistAngle` turns the profile that way, as measured on 2026-09-28 on the video frame's collars
(`[PB-SWEEP-TWIST]`). Each profile starts in air, in the hollow for a `+R` bore and outside the
tube for a `-R` bore, and each cut turns 80°; the cuts measured in Fusion turned 58–65° and
started on a collar's face. The add-in at c8a63b5 built such cuts at the earlier 82.31°, and both
printed ribbons screwed through them (`[SCREW-F-PRINT-MESH]`). The ribbons pass through the
channel and are not on the participant list, so the cut leaves them whole: measured on
2026-09-28, a body wholly inside the channel and off the list kept its volume to the last digit
while the cut body lost the channel's. The bore is the exact helicoid — the rectangle turning
rigidly at `1/Lambda` about the axis, the channel `TestRibbonsStayInsideTheirBoresOverTheTravel`
and `TestSleeveAdmitsOnlyTheScrewMotion` measure against — with no facets and no fit between
samples. The two bores of one gear stand `2*cageRadius` apart and are two sweeps on two lines, as
§1 draws them.

**The rectangle scheme.** Each of the four bore section sketches is a rectangle spanning `v` from
`vLo` to `vHi` and `u` from `uB` to `uF`, turned by `theta = s/Lambda + Phi_g` about the axis point
`O`, which lies on the line `v = 0`; for a bore other than a level one, at its centre, and for a
level bore `a/2` off it across the thickness. Four lines, two
dimensions, one coincidence and one angle cannot fix `O` inside such a rectangle; a construction
spine through `O` can, and this is the scheme:

- **References.** Two reference points (Sketch Discipline): `O`, the point where the sweep
  path pierces the sketch plane, at `origin_g + s*dir_g` for the plane's station `s`, and
  `Cp`, at `origin_g + s*dir_g + (A/2)*û_g`, which is `A/2` from `O` along the gear's
  unrotated `û` and lies on the plane because `û_g` does. The construction line `Ru` from `O`
  to `Cp`, sharing both, is the sketch's zero of rotation; it has no freedom once its ends are
  fixed. Both points are set `isFixed = True` after `Ru` and the spine `K` are drawn and before
  any dimension is added. Nothing is projected into a section sketch: the Anchor Line's
  projection would run through `Cp` across the rectangle at `u = A/2`, which at the defaults
  is inside the rectangle and would split the profile, and `[PB-PROJECT-NOT-FIXED]` rules a
  projected point out as an anchor in any case.
- **The spine.** A construction line `K` from `O` to `E`, seeded at `(uF, 0)` turned by
  `theta`, with a distance dimension `O`–`E` of `uF`.
- **The angle.** An angular dimension between `Ru` and `K` when `|sin theta| >= sqrt(1/2)`,
  where the angle is `theta` folded into 45°–135°, and otherwise between `Ru` and the toothed
  side `L2`, which stands at `theta + 90°` (Sketch Discipline).
- **The rectangle.** Four lines sharing their corners: `L1` from `(uB, vLo)` to `(uF, vLo)`,
  `L2` on to `(uF, vHi)`, `L3` on to `(uB, vHi)`, `L4` back to the start, every seed the solved
  point (`[PB-SHARE-XOR-COINCIDENT]`: shared, no coincident on a corner). Then `L1` parallel to
  `K` with an offset dimension of `-vLo`, and `L3` on the other side with one of `vHi`; `E` coincident on
  `L2`, and `L2` perpendicular to `K`; `L4` parallel to `L2` with an offset dimension of
  `uF - uB` (`[PB-OFFSET-DIM]`, `[PB-NO-OVERCONSTRAIN]`).

Ten degrees of freedom — `E` and the four corners — against ten rows: five dimensions (the
length, the angle, three offsets) and five constraints (three parallels, a perpendicular, a
coincidence), so the sketch closes fully constrained. In every one of the four sketches
`uB = -hw`, `uF = hw`, and `vLo` and `vHi` are the bore's own from "The roof allowance", and its
four lines are solid and are
the profile, the one loop of four lines, `find_profile_by_curve_counts(sketch, lines=4)`
(`[PB-PROFILE-MATCH]`). The construction lines bound nothing. The scheme is drawn with sketch
computing deferred (Sketch Discipline); the video frame's collar sections, drawn by the same
scheme, read fully constrained in Fusion on 2026-09-28.

**What the proof's stand-in costs.** The proof's solid engine has no twisted sweep, so the compiled step
proof builds each bore's channel as a chain of two-section lofts through rotated rectangles ("What the
proof checks"), and the count it lofts through is derived from the turn and the clearance: no two
neighbouring sections more than 5° of twist apart, nor more than the angle at which the facets between them
take 4% of the clearance, `2*acos(1 - 0.04*clearance/c)`, and the count is `ceil(turn / step) + 1` for the
smaller step. At the defaults the turn is 80.78°, the facet bound 5.1° and 5° governs: 18 sections. At a
clearance of 0.05 mm the turn is 79.58°, the facet bound 2.6° governs, and the count is 32. Those facets
are a ruled wall's: flat between sections, inside the true channel, and 0.007 mm of the 0.20 mm clearance
at the derived count, which `TestSleeveBoreSubstituteKeepsItsClearance` holds at 95% of the clearance from
0.05 mm to 0.9 mm. `decad` does not build that wall: it walls each cell with two flat triangles, which
depart from it by up to a quarter of the cell's twist, 0.32 mm on the long faces at the defaults, more than
the clearance (the same test logs it). So the compiled proof reads the cage's volume and the build's probes
off the stand-in, and no clearance. The build derives no section count: its channel has none.

**The bore has to twist; a straight hole binds.** Over the wall's `2*collarHalf` the ribbon turns
by `2*collarHalf/Lambda`, so its corner sweeps `(W/2)*(2*collarHalf/Lambda)` across the opening.
At the defaults that is 5.7 mm against 0.20 mm of clearance.

**The bore is cut to the crest rectangle, never to a tooth.** The crests are the ribbon's outer
edge, so that rectangle holds the whole ribbon, teeth included, and its toothed side bears on the
crests ("Why the cage needs no tooth-shaped cut"); nothing in the frame is shaped like a tooth.

**The sweep's sense is checked, not assumed** (`[SCREW-F-SWEEP-CHECK]`, `[PB-SELF-DIAGNOSING]`).
After each bore cut the build requires the sweep feature to leave exactly one body and probes the
cage with `pointContainment` at the two points `origin_g + sc*dir_g ± (W/2 + clearance/2)*û(sc)`
at the crossing itself, `sc = ±cageRadius`, where `û(s)` is the section's turned `u` direction,
`cos(theta)*û_g + sin(theta)*v̂_g` with `theta = s/Lambda + Phi_g`. Both must be
`PointOutsidePointContainment`, the channel being open there, and the build raises naming the
bore otherwise. The probes stand 16.80 mm (`-R`) and 16.30 mm (`+R`) from the frame's axis,
inside the wall, and clear of the ribbon and of the channel's wall by `clearance/2`. They tell
the two senses apart on their own. The profile sits at the cut's first station `s0`, `sIn` or
`-sOut`, and under the wrong sense the channel at the crossing is turned `2*(sc - s0)/Lambda`
from the right one — 103° for a `+R` bore, 58° for a `-R` bore — which puts the probes 7.39 and
6.46 mm across a channel 2.075 mm half thick, or 2.375 mm on the level bore's roof side, in the
wall
(`TestSleeveBoreProbesTellTheTwistSense`). With no collar body there is no end-vertex check; the
probes are the whole of the sense check.

**The wall between the bores** (`channelSeparation`). This is the fourth of the sleeve's range
checks ("Variables"), computed in `processInputs` before any feature. Round the tube the bores
alternate between the gears: gear A's `+R` bore at azimuth `Sigma/2` about `+n̂` from `ê`, gear
B's `-R` at `180° - Sigma/2`, gear A's `-R` at `180° + Sigma/2` and gear B's `+R` at `-Sigma/2`,
which is 40°, 140°, 220° and 320° at the defaults. So the four gaps between neighbours face `+k̂`
(gear A's `+R` and gear B's `-R` bores), `-ê` (the two `-R` bores), `-k̂` (gear A's `-R` and gear
B's `+R`) and `+ê` (the two `+R` bores).

1. Sample each bore's outline (`channelOutline`): at stations every 0.1 mm from `σ*sIn` outward
   while `|s| <= sOut`, `σ` being `+1` for a `+R` bore and `-1` for a `-R` bore, take seventeen
   points on each side of the bore's rectangle — `(hw, v)` and `(-hw, v)` for
   `v = vLo + (vHi - vLo)*i/16`, `(u, vHi)` and `(u, vLo)` for `u = -hw + 2*hw*i/16`,
   `i = 0 … 16` —
   turned to the station's angle and placed as the world point of §1. Keep a point when its
   distance from the frame's axis, `hypot((P - C)·ê, (P - C)·k̂)`, lies within `Ri - 0.5 mm` to
   `Ro + 0.5 mm`.
2. For each gap, facing `d`: with `across = n̂ × d`, project the kept points of the gap's two bores
   to `((P - C)·across, (P - C)·n̂)`, and take the convex hull of each bore's projections by
   Andrew's monotone chain.
3. For every edge of either hull, with `m` the unit vector square to it, the separation along
   `m` is the larger of `min(m·q) - max(m·p)` and `min(m·p) - max(m·q)`, `p` over the first
   hull's corners and `q` over the second's. The gap's separation is the largest over all the
   edges; it is negative when the hulls overlap.
4. The least of the four gaps' separations must be at least `collarWall`.

Projecting onto a plane and then onto a line brings no two points nearer, and the hull holds
every projected point, so the separation never exceeds the least distance between the two
outlines, which `nearestChannels` in the proof measures and `TestSleevePrintsStandingOnEitherEnd`
holds to `collarWall`. At the defaults it is 5.078 mm, across the `-ê` gap, where
`nearestChannels` measures 5.187 mm; without the roof allowance it was 5.206 and 5.240 mm.

**The windows.** Down the hollow is one way to see the mesh; two windows through the wall show it
from the side. The windows go across the two wider gaps between neighbouring bores
(`windowFacings`): they face `+k̂` and `-k̂` when `crossAngle <= 90°`, where those gaps are
`180° - Sigma` wide against `Sigma` across `±ê` (100° against 80° at the defaults), and `+ê` and
`-ê` past it. Across a wide gap one flanking bore is low and the other high, and the wall between
them is a band running at about 45° from above the low bore down to below the high one; each
window is cut along that band. Each window's shape is found in `processInputs`, before any
feature, by the search below, which is `newWindow` in the proof step for step; each window is
found the same way from its own facing direction. At equal mounting angles and no roof
allowance the `-k̂` window comes out as the `+k̂` one turned half a turn about `ê`, as the bores
do; the allowance on gear A's `-R` bore, which flanks the `-k̂` window, makes that window's band
narrower, so the two differ. Neither is a shortcut the build takes.

*The window's plane.* For a window facing the level unit direction `d`, let `across = n̂ × d`,
which is `-ê` for `d = +k̂`. A point `P` has plane coordinates `t = (P - C)·across` and
`z = (P - C)·n̂`, and depth `a = (P - C)·d`. The window is the set of points with `a > 0` whose
`(t, z)` lie in its hexagon: a prism pushed straight out through the wall from the plane through
the frame's axis. At `t` the wall runs from `a0(t) = sqrt(max(0, Ri^2 - t^2))` to
`a1(t) = sqrt(max(0, Ro^2 - t^2))`.

*A bore's section in the wall* (`sectionInWall`). For a bore of gear `g` at station `s` with
`|s| < Ro`, in the section plane's coordinates `x` along `û_g` and `y` along `v̂_g`: take the
rectangle's corners `(-hw, vLo)`, `(hw, vLo)`, `(hw, vHi)` and `(-hw, vHi)`, in that order, with
the bore's own `vLo` and `vHi` ("The roof allowance"), which runs counter-clockwise, each turned by `theta = s/Lambda + Phi_g` to
`(u*cos theta - v*sin theta, u*sin theta + v*cos theta)`. Clip it to the heights inside the end
faces: keep the `x` for which `(origin_g + x*û_g - C)·n̂` lies within `±cageRise`. A point at
`(x, y)` stands `hypot(s, y)` from the frame's axis, so clip what is left twice more, once to
`near <= y <= far` and once to `-far <= y <= -near`, with `near = sqrt(max(0, Ri^2 - s^2))` and
`far = sqrt(Ro^2 - s^2)`: those are the section's two **pieces** in the wall, either of which may
be empty. Each clip keeps the part of a convex polygon on one side of a line, walking its edges
in order, keeping each corner on the kept side and adding the point where an edge crosses the
line. At `|s| >= Ro` the section has no piece.

*The long sides.* The two bores whose crossings lie on the window's side, `(crossing - C)·d > 0`,
**flank** the window; the other two are its **far** bores. The low flanking bore is the one whose
crossing is lower along `n̂`, and `lean` is `+1` when the high one's crossing has the larger `t`
and `-1` otherwise. The long sides are the 45° lines `z + lean*t = lo` and `z + lean*t = hi`.
Walk each flanking bore's cut span at stations every 0.001 mm from its lower end, and take every
corner of every piece as the world point `origin_g + x*û_g + y*v̂_g + s*dir_g`, with its
`m = z + lean*t` (`wallCorners`). The low bore's largest `m` is `lowReach` and the high bore's
least is `highReach`; then `lo = lowReach + sqrt(2)*collarWall` and
`hi = highReach - sqrt(2)*collarWall`. `m` grows by `sqrt(2)` per millimetre across the band, so
each long side stands `collarWall` beyond its bore's channel measured on the plane, and a point
of the channel that far from the hexagon on the plane is at least that far from the prism. A
linear measure of a piece is extreme at a corner, so the corners are all the walk needs.

*The trims.* `zLimit` is the furthest any channel reaches from the middle plane in the wall, up or
down (`channelTop`): over all four bores, at stations from `σ*sIn` outward every 0.01 mm while
`|s| <= sOut`, wherever the section reaches the wall — `|s| <= Ro` and `hypot(s, y) >= Ri` for the
largest `y = |u*sin theta + v*cos theta|` over the bore's four corners `(u, v)` — it is the largest
`|(origin_g - C)·n̂ + (u*cos theta - v*sin theta)*(û_g·n̂)|` over those corners, 14.54 mm at the
defaults. Then

```
top    = min(2*zLimit - hi, hi + sqrt(2)*Ri)
bottom = max(-2*zLimit - lo, lo - sqrt(2)*Ri)
```

and the trims are the 45° lines `z - lean*t = top` and `z - lean*t = bottom`. The side `hi` meets
`top` at `t = lean*(hi - top)/2`, and `lo` meets `bottom` at `t = lean*(lo - bottom)/2`. The first
term keeps each long corner inside the height the channels already reach, so both end bands keep
the end wall the channels leave. The second cuts each long side off where it would meet the inner
face more than 45° round from `d`, at `|t| = Ri/sqrt(2)`: where a 45° roof meets the curved inner
face, the line they meet on descends at `atan(cos psi)`, `psi` being how far round the face the
line is from `d`, and past `psi = 45°` that line steps further along the face per layer than the
proof's one-cell rule accepts.

*The ends* (`windowEnd`). Each window has two upright ends, `t = right` and `t = left`, standing
as far out as the far bores allow. For each side `δ = +1` (right) and `δ = -1` (left), bisect
`te` 24 times between 0 and `min(Ri, Ro/sqrt(2))*(1 - 1e-9)`: take the middle, and make it the
new lower end when `clear(te)` holds and the new upper end when it does not. The end is the final
lower end: `right = end(+1)` and `left = -end(-1)`. The limit `Ro/sqrt(2)` keeps a window's roofs
off the outer face where they would meet it more than 45° round. `clear(te)`, at `t = δ*te`:

1. `zLow = max(lo - lean*t, bottom + lean*t)` and `zHigh = min(hi - lean*t, top + lean*t)`, the
   end's two corners; not clear when `zLow > zHigh`.
2. `need = collarWall + 0.1 mm/sqrt(2) + 0.005 mm`, 3.076 mm at the defaults: `collarWall` plus
   the most the true least distance can fall under the sampled one, which is half the diagonal
   of a 0.1 mm sample cell and 5 µm for the stations the channel is walked at (`windowSlack`).
3. The points, each `C + t*across + z*n̂ + a*d`, are: the end's two corner lines through the wall,
   at `z = zLow` and at `z = zHigh`, each at `a = a0, a0 + 0.1 mm, …` and last `a1`, a step that
   would pass `a1` landing on it; and, on the inner face `a = a0(t)` and on the outer face
   `a = a1(t)`, the end's upright edge between the corners, at `z = zLow + (zHigh - zLow)*k/n`
   for `k = 1 … n - 1` with `n = ceil((zHigh - zLow)/0.1 mm)`, and at
   `z = -zLimit + j*0.1 mm` for every `j >= 0` with `z <= zLimit` and `zLow < z < zHigh`.
4. Not clear as soon as any point's distance to either far bore's channel in the wall, by the
   walk below with `reach = need`, is under `need`; clear otherwise.

The upright edges are sampled at the heights the proof's full check of a window samples an end
at: its walk along the edge, and its walk of the openings, which steps up from `-zLimit`. So the
check that places the ends reads the same points the proof then holds to `need`. The corner lines
alone, as the proof placed the ends before this search was written, held at the 0.45 mm clearance
the defaults had until 2026-10-02 and gave the same window there, but at a 0.2 mm clearance, with
the mounting angles and engagement of that time, they let an end's edge on the inner face come
2.60 mm from a far bore against the 3 mm `collarWall`.

*The distance to a channel* (`wallGap`). For a point `P` and one bore of gear `g`: in the gear's
frame `x = (P - origin_g)·û_g`, `y = (P - origin_g)·v̂_g` and `sq = (P - origin_g)·dir_g`. Every
point of a section lies within `c` of the gear's axis, so when `hypot(hypot(x, y), sq -
min(max(sq, span start), span end)) - c >= reach` the distance is taken as `reach` and nothing is
walked. Otherwise walk the bore's **station table**, built once per bore before the first walk:
stations `span start + k*0.002 mm` for `k = 0, 1, …` while inside the span, each with its two
pieces, and, wherever the set of non-empty pieces differs between two neighbouring stations,
stations every 0.0001 mm strictly between those two, all in order of `s`. For each piece keep its
**circle**: the average of its corners, and the largest distance from that to a corner. Start with
`best = reach^2`; walk up from the first station at or above `sq`, then down from the one before
it, and stop each way at the first station with `(sq - s)^2 >= best`. At each station, for each
non-empty piece, pass over the piece when `o = hypot(x - cx, y - cy) - radius` is positive and
`(sq - s)^2 + o^2 >= best`; otherwise set `best = min(best, (sq - s)^2 + d2)`, where `d2` is 0
when `(x, y)` is inside the piece — on the inner side of every edge of the counter-clockwise
polygon, which a piece of fewer than three corners never is — and otherwise the least squared
distance from `(x, y)` to the piece's edges. The distance is `sqrt(best)`. A section's points move
at most 1.42 mm per millimetre of station, and the tube's faces clip them faster only near the few
stations where a face is tangent to the section's plane, so between two 2 µm stations the distance
dips under the nearer by at most 4 µm, which the 5 µm in `need` covers; where a piece appears or
vanishes, the 0.1 µm stations follow the channel's corner first reaching into the wall. The walk
stops only where no station further on could come nearer and passes over only pieces that could
not, so it finds the least a walk of every station would.

*The hexagon.* Its corners are the square `-2*Ro <= t, z <= 2*Ro` clipped, in this order, to
`lean*t + z <= hi`, `-lean*t - z <= -lo`, `-lean*t + z <= top`, `lean*t - z <= -bottom`,
`t <= right` and `-t <= -left` (`clipCorners`), so every edge is upright or at 45°. Drop a corner
that lies within 0.001 mm of the one before it, since a sketch line cannot have zero length. At
the defaults the `+k̂` window has `lo = -3.780`, `hi = 6.730`, `bottom = -20.751`, `top = 22.352`,
`left = -11.523` and `right = 11.703`, and its six corners in `(t, z)` are `(11.70, -4.97)`,
`(-7.81, 14.54)`, `(-11.52, 10.83)`, `(-11.52, 7.74)`, `(8.49, -12.27)` and `(11.70, -9.05)`: long
sides 7.43 mm apart across the band, ends 23.23 mm apart, 220.0 mm² on the plane, reaching
14.54 mm from the middle against the channels' 14.54 mm. The `-k̂` window has `lo = -6.436`,
`hi = 3.780`, `bottom = -22.645`, `top = 20.751`, `left = -11.670` and `right = 11.523`: long
sides 7.22 mm apart, ends 23.19 mm apart, 215.1 mm².

*When a gap has no room* (`room`). A window is not cut when `hi <= lo`, the flanking bores leaving
no band between them, or when `right <= left` or the hexagon has fewer than three distinct
corners or no area, the far bores leaving the band no length. The build then logs
`No window facing {d}: {reason}` with `futil.log` (`[PB-LOGGING]`), `{d}` being `+k`, `-k`, `+e`
or `-e`, and builds the sleeve without that window; a window only takes material away, so every
check the sleeve passes still holds without it. No input `TestSleeveWindowsFollowTheSize`
accepts reaches this rule. It leaves both windows out at a 7 mm `collarWall`, where the flanking
bores leave no band; the build refuses that input for the wall between its bores anyway.

*What the search keeps.* `TestSleeveWindowsKeepTheirWalls` holds the default windows: each
flanking bore's channel lies exactly 3.000 mm beyond a long side, measured on the plane; every
face of the cut keeps 3.076 mm from the far bores' channels, sampled every 0.1 mm; every edge
rises at 45° or more; the faces meet the tube at edges of 49.4° or more; and the lines where the
roofs meet the tube descend at 35.3° or more. `TestSleeveWindowPostsStandFirm` holds every post
beside a window to at least `collarWall` wide, 3.18 mm at the narrowest at the defaults, and any
stretch of post under two `collarWall`s wide to at most twice as tall as it is wide, 0.85 times
at the defaults. Through either window 90.3% of the mesh zone can be seen past both ribbons
(`TestSleeveWindowsShowTheMeshFromTheSide`).
`TestSleeveWindowsFollowTheSize` runs all of those checks, with the one-piece and printing checks,
at 33 inputs across the dialog's ranges, and every window it cuts passes.

*Cutting a window in Fusion* (`[SCREW-F-SLEEVE]`). Both windows lie on one plane through the
frame's axis square to `d`, `Window Plane` (`[PB-CONSTRUCTION-PLANES]`). For `±k̂` windows it is
`setByAngle(anchorLine, ValueInput.createByString('90 deg'), targetPlane)`, the plane through the
Anchor Line square to the selected plane, which holds `C`, `ê` and `n̂`; for `±ê` windows it is
`setByDistanceOnPath(anchorLine, ValueInput.createByReal(0.5))`, the plane square to the Anchor
Line through its midpoint, `C`. It is not made when neither window has room. Each window is a
sketch `Window {d}` on that plane holding one reference point per corner at
`C + t*across + z*n̂`, mapped in with `modelToSketchSpace` and given `z = 0`
(`[PB-SKETCH-ZERO-Z]`), and one solid line from each corner to the next, the last back to the
first, sharing the points (`[PB-SHARE-XOR-COINCIDENT]`); then every point is set `isFixed = True`.
Nothing else is in the sketch, and its profile is its one loop: raise unless `profiles.count` is
1, then take `profiles.item(0)` (`[PB-SINGLE-PROFILE]`). The two windows are two sketches because
on the shared plane their hexagons cross.

Cut each window with one extrude: `extrudeFeatures.createInput(profile, CutFeatureOperation)`,
then `setOneSideExtent(DistanceExtentDefinition.create(ValueInput.createByReal(Ro + 1 mm)),
direction)`, `participantBodies` set to a list holding the cage body alone so that the ribbons in
the hollow are left whole (`[PB-THROUGH-CUT]`), then `extrudeFeatures.add`. `direction` is
`PositiveExtentDirection` when `sketch.modelToSketchSpace(C + d)` has a positive `z`, that is when
`d` points to the sketch's positive side, and `NegativeExtentDirection` otherwise. The cut runs
one way only: the same hexagon on the far side of the axis is the other window's side of the
wall, where the bores stand.

*Checking a window's cut* (`[PB-SELF-DIAGNOSING]`). Before each cut, `cageBody.pointContainment`
at the probe `C + tc*across + zc*n̂ + ((a0(tc) + a1(tc))/2)*d`, with `(tc, zc)` the average of the
hexagon's corners, must be `PointInsidePointContainment`: the middle of the wall where the window
goes, which keeps `collarWall` from every bore. After the cut the same point must be
`PointOutsidePointContainment`, and the feature's `bodies.count` must be 1 (`[PB-EMPTY-RESULT]`).
The build raises naming the window and what it read otherwise. A cut extruded the wrong way
leaves the probe inside.

**Order.** The tube is the first body. The four bores are cut from it, each with the cage as its
only participant, in the order gear A `-R`, gear A `+R`, gear B `-R`, gear B `+R`; then the
windows, the one facing `d` before the one facing `-d`. Every cut has to leave exactly one body,
counted as the feature's `bodies.count`, and that body is the cage from then on; the build raises
with the piece's name otherwise. `TestSleeveIsOnePiece` holds that what remains is one piece:
16,561 mm³ at the defaults, about 21 g of PLA.

### 5: Relocate the bodies

Name the cage body `Cage`, then move the two gear bodies and the cage body into their
sub-components with `body.moveToComponent`, which preserves world position and needs no
activation. Then `solids.hide_construction_geometry(self.designOcc.component)`: the `Design`
sub-component is where every sketch and construction plane of this build was made, and the helper
walks it and anything under it.

## What the proof checks

`proof/screwgear` holds two proofs side by side, and they have different jobs.

The **hand-written mechanism proof** is the `Test` functions listed below, in `geometry_test.go`
(the model, the part and the section count), `pair_test.go` (the mesh), `bore_play_test.go` (the
mesh over the play the bores allow), `sleeve_test.go` (the sleeve this spec builds, its bores,
the travel and its windows) and `render_test.go` (the pictures). It is written by hand, is not compiled from this spec, and is not the compile stage's job to
reproduce: it proves the mechanism — that these two parts drive each other 1:1 and move freely
in this frame — and the compile stage reads its numbers rather than re-deriving them. The whole
risk in this gear is meshing, and no other proof in this repository simulates motion, so these
files model the ribbon and the frame implicitly — a point is inside the ribbon when its
cross-section coordinates satisfy four inequalities, and inside the sleeve when it is in the tube
and in no bore's channel and no window — rather than as solids. That is exact where a boolean
between two lofted solids would be a tangency `decad`'s exact predicates refuse to classify, and
it is cheap enough to run the search a few million times. These five files import neither
engine. `defaultParams` in `geometry_test.go` is this spec's default table.

The **compiled step proof** is what `/compile-gear` writes beside them from the step list, in
the shape the compile contract fixes: one function per build step, no `Test` functions of its
own, registrations generated into `zz_registrations_test.go`, and the `sketch` and `decad`
engines for what it builds. It covers the build steps of "Instructions" — that every sketch
scheme closes fully constrained and unambiguous, and that every solid step yields the body the
next step consumes — and nothing in the list below. A recompile regenerates it and leaves the
hand-written files alone. Four steps of this build are ones the engines cannot build as
Fusion does, and each takes a stand-in, to be named in the proof beside what it stands in for:

- The Cell Sections sketch (§2) holds points off its plane, and the sketch engine is planar. The
  stand-in is one planar sketch per section, on that station's own plane, of the four corners
  as fixed points and four lines, which pins each section's numbers; Fusion's verdict on the one
  3D sketch — fully constrained, one profile per section — was measured on 2026-09-28 and is
  the sketch step's `[PROSE]` part.
- The cell loft (§2) is smooth in Fusion. `decad` lofts two sections at a time, so the stand-in is a chain
  of such lofts through the same `c*n + 1` sections, each wall cell two flat triangles, up to 0.125 mm off
  the ruled loft `TestLoftSectionCountHoldsTheHelicoid` bounds; the cell's volume comes out 2.2% under the
  helicoid's (`TestCellLoft`). Fusion's smooth loft came within 0.23% of the ruled loft's volume.
- A bore's sweep (§4) has no `decad` counterpart. The stand-in is a chain of two-section lofts through the sections
  "What the proof's stand-in costs" derives, 18 at the defaults, built as the sweep's own section turned by
  `s/Lambda + Phi` at each. `TestSleeveBoreSubstituteKeepsItsClearance` holds a ruled wall through those sections
  to 0.007 mm of the 0.20 mm clearance at the defaults, and logs how far `decad`'s two triangles a cell depart from
  it, up to 0.32 mm; the compiled proof reads no clearance off the stand-in. The twist's sense and its linearity
  are Fusion's (`[PB-SWEEP-TWIST]`), measured on 2026-09-28 and checked at build time (`[SCREW-F-SWEEP-CHECK]`).
- A cut with a participant list (§4), a bore's or a window's, is a `decad` cut from the cage
  alone; that the ribbons are left whole is Fusion's, measured on 2026-09-28 for a sweep cut.

The sleeve's tube is an extrude and each window an extrude cut of a planar hexagon, which
`decad` builds as they are. The two searches of §4 that run before any feature, the wall between
the bores and the window search, are the proof's `channelSeparation` and `newWindow`; the
compiled step proof takes their results as numbers.

The mechanism proof's cases:

- `TestPairDrivesOneToOne` is the one everything rests on. It tracks the free window in B's tooth
  phase through a full pitch of A and asserts three things: the window is never empty (no jam), it
  is narrower than a pitch (the teeth box B in, so something is driven), and its centre advances by
  exactly one pitch (the 1:1 ratio). The third is what separates a gear from two parts that merely
  touch, and an earlier arrangement passed the first two and failed it. Its bounds on the
  departure and on the window are fractions of the pitch ("Defaults").
- **What "meshes" means under play.** A printed tooth's tip comes out rounded and short, so
  wherever the proof judges the mesh under play it cuts both ribbons' crests flat 0.35 mm down
  (`printTipLoss`): a 0.4 mm nozzle lays a bead about 0.45 mm wide, the cosine's crest is
  narrower than that bead for its last 0.19 mm, and an outer wall comes out a tenth or two of a
  millimetre off on top. The pair meshes when, at every pose pair the bores allow, the free
  window of `TestPairDrivesOneToOne` is never a pitch wide and advances one pitch per pitch, at
  six phases of A per pitch. A pose pair that jams is accepted only when the two axes stand at
  least one clearance nearer than nominal and neither ribbon is pulled away from the other: the
  two bodies cannot both be in that pose at that phase, so the teeth push the ribbons out of it,
  and only pressing the ribbons together reaches it. A pose pair that lets the teeth pass is
  never accepted.
- **The default sample and the full one.** `TestPairDrivesUnderBorePlay`,
  `TestPairDrivesUnderSidewaysPlay`, `TestPrintedFitFailsUnderBorePlay` and
  `TestSleeveWindowsFollowTheSize` run a smaller sample unless the environment variable
  `SCREWGEAR_FULL` is `1`, and CI runs the smaller one. The full sample runs with
  `SCREWGEAR_FULL=1 proof/run.sh --package ./screwgear -- -count=1`; any value but empty, `0` or
  `1` fails those cases. It is an environment variable rather than a test flag because `go test`
  hands every argument after a flag it does not know to the test binary, and `run.sh` puts the
  package list last. The counts in the entries below are the full sample's, and each entry says
  what the default leaves out. Every case logs which sample it ran. At the defaults the package
  takes about 36 s on 24 cores with the default sample and about 76 s with the full one.
- `TestPairDrivesUnderBorePlay` judges the defaults that way. Each ribbon is moved inside its
  two bores to its limits toward and away from the other ribbon at every whole degree of roll,
  to its two roll limits, and to the two tilts that put its crossing furthest toward and away
  from the other ribbon (`tiltReach`): 10 poses a ribbon, 100 pose pairs. It holds that the
  nominal pose drives, that no pose pair fails in a way the check refuses, and that at most four
  jam. At the defaults 96 of 100 drive, with windows from 0.026 to 1.654 mm and departures up to
  0.315 mm, and the four that jam all have gear A tilted into its roof allowance, its crossing
  0.273 mm toward gear B, against gear B pushed toward it, at 0.199 mm or 0.200 mm, rolled 1°
  and pushed 0.069 mm, or at its roll limit. It does not combine a sideways move with a roll or
  a move along the axis between them, nor tilt a ribbon about the axis through the crossing that
  the roof allowance does not open; an offline run of 676 pose pairs before the roof allowance,
  with the model's own tips, that combined the first found 6 jams of the accepted kind and no
  other failure. The default sample takes six poses a ribbon, the limits toward and away at no
  roll, the two roll limits and the two tilts, and runs each against the same pose of the other
  ribbon: 6 pose pairs, in which the two moves add up rather than cancel. It leaves out the whole
  degrees of roll short of the limits and every pair of two different poses, among them three of
  the four jams. It keeps the nominal pose, every kind of move, and the full sample's narrowest
  window, both ribbons pushed together, and its widest window and worst departure, both pulled
  apart with gear B tilted; 5 of the 6 drive and one jams.
- `TestPairDrivesUnderSidewaysPlay` takes each ribbon to its sideways limits, 0.20 mm either
  way, against the other at rest, at its limits along the axis between them and at its own
  sideways limits, with the tips 0.35 mm short: 16 pose pairs, all of which drive at the
  defaults, with windows from 0.092 to 1.339 mm. The default sample takes both ribbons at the
  same sideways limit, and each at either sideways limit against the other pushed toward it: 6
  pose pairs, which keep both of those windows. It leaves out the other ribbon at rest or pulled
  away, and the two at opposite sideways limits.
- `TestRoofAllowanceAddsOnlyATilt` holds that the roof allowance adds no move and no roll:
  against the same sleeve with no allowance, each ribbon moves 0.200 mm toward, away and either
  way sideways and rolls 1.53°, to a micron and to 1e-4 rad, and both gears' level bores are
  their `-R` bores. What it adds is a tilt about the axis square to the ribbon and to the common
  perpendicular, −0.96° to +0.56° for gear A and −0.56° to +0.96° for gear B against ±0.56°
  without, which carries gear A's crossing 0.273 mm toward gear B and gear B's 0.273 mm away
  from gear A, against 0.200 mm. The test holds that gain under half the allowance. The other
  bore caps the tilt, so a roof with far more room, such as one chiselled open, carries the
  crossing no further.
- `TestPrintedFitFailsUnderBorePlay` runs the same check at the values printed before
  2026-10-02 — a 0.45 mm clearance and no roof allowance, a 0.75 mm engagement, 15° on both
  gears, the model's own tips — over each ribbon's limits along the axis between them and its
  roll limits, 16 pose pairs, and holds that at least one fails in a way the check refuses. The
  nominal pose drives there, which is all the proof used to ask; 6 of the 16 let the teeth pass
  unboxed. Two more jam with one ribbon pushed the whole clearance and the other at its roll
  limit, which the check refused until the roof allowance came in and accepts now. It is the
  case the print of 2026-10-02 asked for (`[SCREW-F-PRINT-MESH]`). The default sample runs each
  limit against the same limit of the other ribbon, 4 pose pairs. Two of them let the teeth pass,
  both ribbons pulled 0.45 mm apart and both rolled +3.46°, so the default still fails the print,
  once on a move along the axis between them and once on a roll. It leaves out every pair of two
  different limits, among them the other four that let the teeth pass.
- `TestSecondPrintMeshIsMarginal` takes the fit the second sleeve was printed at, the defaults'
  0.20 mm clearance with no roof allowance, and the one pose pair whose mesh gives way first as
  the tips shorten: both ribbons pulled the whole clearance apart. With the tips 0.40 mm short
  the pair drives there, and with them 0.45 mm short it lets the teeth pass; over all 64 of its
  reach pose pairs it fails none at 0.40 mm, one at 0.45 mm and four at 0.50 mm. So the drawn
  fit holds 0.05 mm of tip loss past the 0.35 mm the proof takes, and that margin is what the
  case holds. The print it answers meshed only sometimes, after its `-R` bores were chiselled
  open (`[SCREW-F-PRINT-2]`); a bore opened by hand is not the bore drawn, and the proof cannot
  model the chisel. One pose pair at two tip losses is the least that shows an edge, so the case
  has no smaller sample; it runs the two losses side by side.
- `TestSymmetricMountJams` holds the reason the Mounting Angle exists, by failing if the `Phi = 0`
  arrangement ever stops jamming.
- `TestFullRibbonsClearOutsideTheEngagement` walks both parts end to end, so the contact search's
  window is not taken on trust.
- `TestRibbonIsInvariantUnderItsScrewStep` is what licenses building the ribbon as one cell
  repeated. It carries points of the twisted blank and the four corners of every cross-section,
  teeth included, through one screw step and requires them to land on the next cell's own, so
  the body genuinely is invariant under `Step`, not only the twist.
- `TestSleeveBoresAreTheSameChannels` pins what the frame may not change: the bore's rectangle,
  15.4 by 4.15 mm, its stations `±cageRadius`, its twist and its angles at the crossings, that
  the cut covers the wall's span on the bore's centre line, and that one gear's bores sit low
  and the other's high. It also pins the roof allowance: each gear's level bore is its `-R`
  bore, spanning `v` from −2.075 to 2.375 mm for gear A and from −2.375 to 2.075 mm for gear B,
  and each `+R` bore spans ±2.075 mm.
- `TestRibbonsStayInsideTheirBoresOverTheTravel` holds every point of the ribbon, teeth
  included, inside every bore by the clearance at every phase of the travel, over the wall's
  span on the bore's centre line, and holds the bore no looser than that, the crests, the back
  edge and the faces each coming to 0.200 mm, and the level bores' roofs to 0.500 mm.
- `TestTravelIsTheRibbonBetweenItsBores` walks each gear out of the assembly position both ways,
  141.2 mm for the pair, 53.8 teeth, with the mesh limit further out at 80.2 mm.
- `TestRibbonsClearEachOtherOutsideTheEngagement` walks everything each ribbon reaches at any
  phase against the other ribbon outside the engaged zone, 1.95 mm at the least, and holds the
  sleeve's inner face outside the engaged zone.
- `TestSleeveAdmitsOnlyTheScrewMotion` is the frame's own proof. It turns a gear out of step with
  its advance and finds it jamming in the bores at **1.60°** at the defaults. A frame of round
  holes would report no jam at any angle, and that is the case this rules out.
- `TestRibbonsClearTheSleeveOverTheTravel` is the other half of that: the gear moves freely
  through its travel. It walks everything both ribbons reach at any phase, grown by the
  clearance, over every station the travel carries them through, and finds no point of it in the
  sleeve; outside the bores' cuts the ribbons keep 0.98 mm from the tube.
- `TestSleeveBoreProbesTellTheTwistSense` holds the build's check of each bore's sweep: both
  probes at the crossing are in the channel under the right twist sense and in the wall under the
  wrong one.
- `TestSleeveKeepsTheMeshVisibleAlongTheAxis` projects everything either ribbon reaches in the
  engaged zone along the frame's axis, 10.92 mm from it at the most, and follows the line through
  every point of that footprint, grown by the clearance, from one end of the sleeve to the other
  without meeting material.
- `TestSleeveIsOnePiece` flood-fills the sleeve on a 0.25 mm grid and reaches every cell, and
  holds the end wall the channels leave, 4.21 mm, to `collarWall`.
- `TestSleevePrintsStandingOnEitherEnd` holds both ends flat, finds material laid on air only in
  the bores' roofs, printed either way up — 1341 cells standing on the `−n̂` end and 1318 on the
  other — and holds the end wall, the wall between the nearest two bores (5.19 mm,
  `nearestChannels`) and the edge each bore's mouth leaves on the inner and outer face (47.8°
  and 63.4°, against a 30° floor). It logs each bore's flattest roof, the span the printer
  bridges there and whether the bore carries the roof allowance, and enforces nothing about the
  roofs.
- `TestSleeveBoreSubstituteKeepsItsClearance` holds a ruled wall through the sections of the
  compiled proof's stand-in for the bore sweep to 95% of the clearance over the sleeve's cut, at
  clearances from 0.05 mm to 0.9 mm, the 0.20 mm default and the printed 0.45 mm among them. It
  logs how far the two flat triangles `decad` walls each cell with depart from that ruled wall,
  0.32 mm on the long faces and 0.09 mm on the short ones at the defaults, and holds nothing to
  it ("What the proof's stand-in costs").
- `TestSleeveInputsAreChecked` holds the sleeve's four range checks ("Variables"): the defaults
  pass all four, and each is reached by an input that passes every check before it — a 4 mm
  clearance, a 1.5 mm engagement, a 17.5 mm rise, and a 4 mm `collarWall` at a 70° crossing. It
  holds the separation of §4 under `nearestChannels` at the defaults, 5.078 against 5.187 mm.
- `TestSleeveWindowsKeepTheirWalls`, `TestSleeveWindowPostsStandFirm` and
  `TestSleeveWindowsShowTheMeshFromTheSide` hold the default windows (§4, "What the search
  keeps").
- `TestSleeveWindowsFollowTheSize` holds that the windows are sized from the sleeve at every size,
  not only at the defaults. It builds the sleeve at 33 inputs: every length of the ribbon and the
  frame scaled by 2/3, 0.75, 0.8, 1.25, 1.5 and 1.75 with `collarWall` and the clearance held;
  ribbon widths of 10 and 12 mm and thicknesses of 2.5 and 5 mm; leads of 40 and 60 mm; cage
  radii of 14.5, 17, 20 and 25 mm; rises of 18.5 and 25 mm; clearances of 0.45 and 0.9 mm;
  collar half lengths of 2 and 3.5 mm; `collarWall` of 2, 4 and 5 mm; crossing angles of 70°,
  90°, 100° and 120°; mounting angles of 0° and 30°; and a 4 mm `collarWall` with a 0.9 mm
  clearance, a 14.5 mm cage radius or a 70° crossing. `cageRise` is raised where needed to the
  least the end-wall check accepts. The build refuses two of them, each for the wall between two
  bores, and the test holds that no other refusal is reached. At the other 31 it cuts both
  windows and runs on them every check the default windows pass, with the one-piece and printing
  checks on the same grid; every one passes, the mesh zone seen through the windows running from
  62% to 98%. The cage radius of 14 mm and the collar half length of 4 mm it used before
  2026-10-02 put the inner radius at 11 mm, inside the 11.18 mm the mesh check asks for at the
  deeper engagement, so the spread moved them to 14.5 and 3.5 mm. It
  then leaves both windows out at a 7 mm `collarWall`, the rule for a gap with no room. The
  default sample builds five of the 33 inputs: everything scaled by 2/3, the smallest sleeve; a
  120° crossing, past a right angle, where the windows face ±X; a 5 mm `collarWall`, the thickest
  the spread tries on its own; and the two inputs the build refuses. It leaves out the other 28,
  every one an input the build accepts, and still leaves both windows out at a 7 mm
  `collarWall`.
- `TestCrossedHelicalRuleMakesTheCrestHelicesParallel` pins `Sigma = 2*Beta` and the station it
  holds at.
- `TestLoftSectionCountHoldsTheHelicoid` is the arithmetic the section count is bought with: the
  twist chord at the crest and the cosine chord on the toothed edge, over leads from 20 mm to
  400 mm, so the floor of eight steps is held where the twist is slow; it logs the cell's
  section count at `cellTeeth` of 4 and 1, 41 and 11 at the defaults. It also logs how far the
  two flat triangles `decad` walls each cell of the compiled proof's stand-in with depart from
  that ruled loft, 0.125 mm on the 15 mm faces at the defaults, and holds nothing to it.
- `TestDoublingScheduleCoversTheRibbon` runs the doubling schedule of §3 at every tooth count from
  4 to 512 with cells of one, three and four teeth, and holds that the pieces tile the ribbon
  with no overlap, in `floor(log2 q) + popcount(q) - 1` rounds plus one remainder join when
  `N mod c` is not zero: 5 rounds at the defaults, 7 at a one-tooth cell.
- `TestProportionsFollowTheVideo` holds the defaults inside the ranges read off the video, each
  widened by the ±20% the reading carries, for the ratios this spec follows: teeth per turn,
  thickness, tooth depth and length against the width, tooth depth against the pitch, and one
  hand for both gears, and the sleeve's diameter and height against the video frame's ring and
  height. It logs the one ratio the spec departs from, the crossing angle, so a run shows both.

`TestRenderPart`, `TestRenderMesh` and `TestRenderSleeve` in `render_test.go` draw the pictures
from the same section function the mesh proof samples. They are skipped unless `-render.out`
names a directory.

### What the proof cannot reach

It cannot say whether the sine tooth is the *right* tooth. Conjugate flanks for two screw motions
follow from the equation of meshing `n·v_rel = 0` against the relative screw, and nothing here
derives them; the proof measures what this tooth does, not what the best tooth would do.

It proves the ideal ribbon, an exact cosine on an exact helicoid, and not the lofted body Fusion
builds. `TestLoftSectionCountHoldsTheHelicoid` bounds the gap between the helicoid and a *ruled*
loft through the build's sections — 1.0 µm at the crest and 0.064 mm on the toothed edge, where
a ruled loft draws a chord of the cosine — against a 0.46 mm backlash. Fusion's loft through
more than two sections is smooth between them, not ruled (§2, "What the loft is"), so that is a
bound on a body Fusion does not build: the built surface passes through the same sections, and
how far it departs from the helicoid between them is measured by nothing here. A Fusion
measurement on 2026-09-28 put it within 0.04 mm at the 10 mm ribbon (`[SCREW-F-CELL-LOFT]`), and
only a Fusion load sees it at other inputs, the 1.5× defaults included. The bores are exact
helicoids in Fusion and have no such gap; `TestSleeveBoreSubstituteKeepsItsClearance` bounds a
ruled wall through the sections of the proof's own stand-in for them, not the built part, and
not the flat triangles `decad` builds that stand-in's walls from.

It cannot see the sweep. The solid engine has no twisted sweep, so which way Fusion turns a
profile for a positive `twistAngle`, and that it turns it linearly along the path, are Fusion's
facts, measured on 2026-09-28 (`[PB-SWEEP-TWIST]`) and re-checked on every build by the probe
check of §4 (`[SCREW-F-SWEEP-CHECK]`). Nor can it see a sketch whose points lie off its plane:
the sketch engine is planar, so the Cell Sections sketch's own constraint verdict is Fusion's,
measured on 2026-09-28 at 11, 41 and 81 sections.

It cannot see where Fusion puts a plane. The sketch engine draws each section on an exact plane
at `z = 0`, so a point Fusion leaves a few nanometres off its plane — which made the first Fusion
load fail at `Gear A Collar -R Section 0` (`[PB-SKETCH-ZERO-Z]`) — does not exist in the proof.
What the proof can hold is the rule's side of it: the compiled step list must set `z = 0` on
every mapped point that is meant to lie on its plane, the corners of a window included, and a
step that maps one without it is a compile defect.

It has seen the sleeve built once, and measured nothing of it. The add-in at c8a63b5 built the
sleeve the print of 2026-10-02 was made from, and the ribbons screwed through its bores
(`[SCREW-F-PRINT-MESH]`); no count, timing or probe was read from that load, so none of §4 has
been checked in Fusion beyond that: the tube's sketch of two concentric fixed circles and the
choice of its two-loop profile, the four twisted sweep cuts whose profiles start in air, the window's plane made by `setByDistanceOnPath` at the Anchor
Line's midpoint for a crossing past 90°, the one-sided extrude cut and the direction the build
picks for it, and the probe checks that guard the sweeps and the windows. Nor has Fusion run the
two searches of §4; a CPython port of them outside Fusion took 2.2 s at the defaults before
2026-10-02.

It also cannot see print tolerance, friction or how the printer copes with an overhang. The bore
play cases (`bore_play_test.go`) take each bore as drawn: a print that comes out tighter or
looser than drawn has a different play, and a ribbon that bends between its bores moves in a way
no rigid pose reaches. They tilt a ribbon only about the axis the roof allowance opens. Their
tip loss, 0.35 mm, is a reasoned allowance for a 0.4 mm nozzle, not a measurement; the drawn fit
holds 0.40 mm and not 0.45 mm (`TestSecondPrintMeshIsMarginal`). A window of 0.43–0.47 mm is
narrower than the 0.76–0.89 mm the first sleeve was printed with, and only a printed part settles
whether it runs freely. Each `-R` bore passes through level: its 15.4 mm-wide roof is flat at
station −14.30 mm, inside the wall, and the printer bridges it across the wall; the `+R` bores'
roofs are flat at station 10.45 mm, near the inner mouth. The second sleeve's `-R` bores printed
too tight at a 0.20 mm clearance, and the 0.30 mm roof allowance is the answer
(`[SCREW-F-PRINT-2]`). How far a bridged roof sags the proof has no model of: it takes the bore
as drawn, so with the allowance it assumes the roof does not sag at all, which is the case that
gives the ribbon the most room to tilt, and whether 0.50 mm under the roof is enough only a
print settles. The proof logs the roofs and enforces nothing about them. Where a window's roof meets the inner face, the line it meets on descends at 35.3° at the
flattest: the proof's 0.25 mm grid takes that as one cell diagonally, which its 45° rule
accepts, and whether a slicer lays a short unsupported edge there only a print settles. The
first print, at the 10 mm ribbon, settled something the proof could not have caught: the teeth
were far too small ("What the print showed"). The proof has no case for what a tooth looks like
at print resolution, and cannot have one; what it now holds is the ratio the print moved, the
tooth depth against the width and the pitch, inside the video's range
(`TestProportionsFollowTheVideo`), so a later default cannot drift back without a run saying so.
The second, of the video frame, found it too wobbly to print, which no check of that frame's
geometry could have caught; the sleeve's printing checks are what the proof now holds instead.
The third, of the sleeve, found the teeth not meshing over the play the 0.45 mm bores allowed,
which the proof could have caught, since it held both the bores and the mesh; it measured the
mesh at the nominal pose alone. `TestPairDrivesUnderBorePlay` now runs the mesh over that play,
and `TestPrintedFitFailsUnderBorePlay` holds that it fails the printed values. The fourth, of
the sleeve at the 0.20 mm fit, found the bridged `-R` bores too tight, which the proof could not
have caught, and, with them chiselled open, the teeth meshing only sometimes, which it could
have caught in part: it modelled no tip loss. It now blunts the tips wherever it judges the mesh
under play, and `TestSecondPrintMeshIsMarginal` holds where the drawn fit gives way.

It cannot see the video's model either. The ratios in "What the video shows" were read off
1280×720 frames by hand, and `TestProportionsFollowTheVideo` holds the defaults inside those
readings widened by the ±20% they carry; a finer reading needs the model, not the video.
