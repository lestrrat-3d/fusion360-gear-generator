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
- **The Cage**, the frame in the video: an open skeleton of round rods. A round wire **ring**
  stands at one end and a smaller **loop** with straight sides at the other, four thin **rods**
  run between them, and a short smooth **collar** sits round each ribbon where it crosses the
  frame. It has no wall, no plate and no post. The two collars of one gear sit low and the two
  of the other high, so the gears meet in the middle, where nothing of the frame blocks the view
  of them.

  Each collar carries a **bore** its ribbon passes through, and the bore is what makes each
  gear's motion a screw motion rather than a free slide. It is cut to the ribbon's own
  cross-section and twisted at the ribbon's own lead, so a gear that turns without advancing
  jams in it. A round hole would not: it would leave the mechanism three degrees of freedom
  instead of one.

  A rod cannot stand where its ribbon crosses the frame, because the ribbon runs on through that
  point. Each rod stands beside its collar instead, on the ring's circle, turned round the ring
  by the least angle at which it clears both ribbons, and the collar's wall is what joins the two.

The gears' toothed edges meet in the middle of the cage, between the two collar heights. Pushing
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
| Tooth depth | 0.15–0.2 widths, about one pitch (0:09, 6:14) | 0.12 widths, 0.69 pitch |
| Ribbon thickness | 0.2–0.3 widths (6:14) | 0.25 widths |
| Ribbon length | About 12 widths (6:10, both ends in frame against the ring) | 14 widths, 80 teeth |
| Crossing angle | 85–100° between the arms in the overhead shots (0:09 reads 89°, 6:10 reads 102°) | 80° |
| Cage size | Ring outer diameter 2.2–2.8 widths, about 0.8 of a twist lead (0:09, 6:10); the frame is about as tall as the ring is wide (5:26, 5:34) | Outer diameter 2.75 widths, 0.83 leads; 1.07 ring widths tall |
| The frame | An open skeleton of round rods, described below the table (0:09, 5:23, 5:26, 5:28, 5:31, 5:34, 5:36, 5:37, 6:10, 6:16, 6:42) | Same: a ring, a loop with straight sides, four rods and four collars |
| The boss | None. The teeth run through the collars (0:09, 6:12, 6:14) | A smooth boss at each crossing, and the bore is cut to it |
| Travel | Most of the ribbon: at 5:53 the frame sits near one end of a ribbon, at 6:00 near its middle | 4.2 mm, 2.4 teeth |
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
and at 1280×720 it is some thirty pixels long. The collar length here is what the boss allows
(see "The boss"), and the rod's place is derived from the clearance it needs ("The cage").

Two of the departures are deliberate, and each has its reason elsewhere in this spec:

- **The boss, and with it the travel.** The bore is cut to what passes through it, and the boss
  is what keeps that a plain rectangle ("The boss, and why the cage needs no tooth-shaped cut").
  The video's collars are thin, so its teeth pass through them and the gear travels almost its
  whole length; here the collar's own length comes out of the boss's flat top, and the travel is
  4.2 mm. Dropping the boss here would change no bore dimension, since the crests already define
  the ribbon's rectangle, but the bore's toothed side would then bear on crests and
  `TestTeethNeverReachABore` would invert.
- **The crossing angle** and **the lead** come from the meshing search, not from the video.
  With `CrossAngle` set to 90° and nothing else moved, `TestPairDrivesOneToOne` reports a
  0.187 mm departure from the 1:1 line against its 0.10 mm bound; with the lead at 40 mm it
  reports 0.130 mm. 80° at 33 mm reports 0.067 mm.

The ring sits at the top of the video's range rather than in the middle of it, and the boss is
why. A rod has to clear the ribbon beside it by the clearance over the whole stroke, and a
ribbon carrying a boss reaches 5.9 mm from its own axis at its widest, so a rod stands 7 mm
from that axis. On a ring the size of the video's, the point 7 mm beside the crossing falls
inboard of the collar and nothing joins the two; on a 12.5 mm ring it falls at the collar's own
station. `TestRodsStandBesideTheirCollars` holds that join.

The tooth depth is left where it is. A deeper tooth drives in the proof (`ToothHeight` 1.75 with
`Engagement` 0.5 reports a 0.51–0.60 mm window and a 0.038 mm departure, which is better than the
defaults), but the video's depth reads 0.15–0.2 widths with ±20% on it, which does not settle a
value. `TestProportionsFollowTheVideo` holds the ratios this spec does follow — teeth per turn,
thickness and length against the width, the ring against the width, the frame's height against
the ring, and one hand for both gears — inside the video's ranges
widened by the ±20% the readings carry; the 18.9 teeth per turn sit just under the 20–26 read.

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

### The boss, and why the cage needs no tooth-shaped cut

The ribbon carries a **smooth boss** at each of the two places it passes through the cage: a
swelling that stands `BossGrow` proud of the plain section and fades the teeth out of it, running
back into them over `BossTaper` at each end. The bore is cut to the boss, so **no tooth ever enters
a bore** and nothing in the frame has to be cut to the shape of a tooth. Nothing bears on a crest
either.

The boss travels with its gear, so **its length is the stroke**: the mechanism runs while the
boss's flat top still fills the collars, which is `2*(BossHalf - BossTaper - CollarHalf)`, 4.2 mm
or 2.4 teeth, at the defaults. `TestStrokeIsTheBossLength` and `TestTeethNeverReachABore` hold
both halves of that, and `TestCollarBoresHoldTheBossOverTheStroke` holds that every collar sees
the flat top, the clearance away all round, at every phase of the travel.

The collar's length is set by that trade. A collar of the video's proportions, half a ribbon
width or more, would leave under two teeth of travel on this boss; a 4 mm collar leaves 2.4. A
longer boss buys a longer collar, at the price of the boss starting nearer the mesh: it starts
`CageRadius - BossHalf` from the middle, 6 mm at the defaults against the 4.05 mm the teeth
engage over, and `TestBossesClearTheOtherRibbon` measures 0.91 mm between the ribbons with their
bosses on, outside that zone.

The video's ribbons carry no boss and travel most of their length; "What the video shows" above
records what dropping the boss here would and would not change.

### The pair

Let `n̂` be the common perpendicular of the two axes and `C` the centre of the mechanism. Then

```
Beta  = atan(pi*W / TwistLead)        helix angle of the toothed edge
Sigma = 2*Beta                        angle between the two axes
A     = W - Engagement                distance between the two axes
```

Gear A's axis passes through `C - (A/2)*n̂`, gear B's through `C + (A/2)*n̂`, and the two directions
sit at `+Sigma/2` and `-Sigma/2` about `n̂` from a shared reference direction in the plane. Each
gear's `u` axis at its crossing station points at the other gear, then turns by `Phi`.

**`Sigma = 2*Beta` is the crossed-helical rule** — two helical tooth rows run parallel where they
meet when the shaft angle equals the sum of the two helix angles, and the two members here have the
same helix angle and the same hand. Deriving it for this pair: the crest of gear A traces the helix
`axis(s) + (W/2)*û(s)`, whose tangent is `â + (W/2/Lambda)*v̂(s)`; setting A's and B's tangents
parallel gives `tan(Sigma/2) = pi*W/TwistLead = tan(Beta)`. **Both gears have the same hand.**

**The rule is where the crossing angle comes from, not a proof that this pair meshes.** The two
crest helices run parallel at exactly one station on each ribbon — the one whose cross-section angle
is zero, which the mounting angle puts `Phi*Lambda` back from the axes' closest approach — and they
cross everywhere else. The proof measures contact between 1.3 mm short of the closest approach and
1.7 mm past it, which is nowhere near that station, so what this pair actually carries is a point
contact like a crossed-helical pair rather than the line contact the rule describes. The angle is a
starting point that the meshing search confirms, and the search is what settles it.

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
| Ribbon Width `W` | 10 mm |
| Ribbon Thickness `T` | 2.5 mm |
| Tooth Pitch `P` | 1.75 mm |
| Tooth Height `H` | 1.2 mm |
| Tooth Count `N` | 80 |
| Twist Lead | 33 mm per turn |
| Crossing Angle `Sigma` | 80° |
| Engagement | 0.36 mm |
| Mounting Angle, both gears | 15° |
| Cage Radius | 11 mm |
| Ring Radius | 12.5 mm |
| Clearance | 0.3 mm |

Derived: `Beta` = 43.6°, `A` = 9.64 mm, ribbon length = 140 mm, twist per tooth = `P/Lambda` =
19.1°, ribbon turn between a gear's two collars = `2*CageRadius/Lambda` = 240°.

**The frame no longer sets the lead.** An earlier frame had to print a bore through a wall, which
tied the cage radius to half the twist lead and the two mounting angles to each other. The video's
frame has no wall, so the cage radius is read from the video instead and the lead stands on its
own: at 20 mm per turn the ribbon is twisted past the point of looking like a rack, and at 40 mm
the arrangements with equal mounting angles turned out to jam under finer sampling
(`mesh-search.md`). 33 mm is where it stayed.

At those values `proof/screwgear` measures a free window in B's tooth phase that is **0.438–0.569 mm
wide** and that **advances by exactly one tooth pitch for each pitch A advances**, departing from
the 1:1 line by 0.067 mm, which is 3.8% of the pitch. That window width is the backlash, and the
winding is what makes this a 1:1 gear rather than two parts that merely touch. Away from the teeth
the two ribbons clear each other by 0.254 mm at their closest, and by 0.91 mm outside the engaged
zone with their bosses on.

**Both Mounting Angles are 15°.** Unequal angles drive too, and some drive better; 15° on both is
the smallest equal angle that drives at this twist, and the two gears are the same part held
alike. The earlier frame needed the two equal for its bores to stand upright, and this one does
not; the angles stay where the search left them rather than being searched again for a frame
that no longer cares.

**Assembly phase.** With gear A at tooth phase 0, gear B is built at **−0.90 mm**. The number is the
middle of the free window, and `TestAssemblyPhaseSitsInTheFreeWindow` holds it there.

## Architecture

The module `lib/geargen/screwgear.py` defines exactly these public classes (the command wiring binds
to them by name; exported via `lib/geargen/__init__.py`):

- **`ScrewGearCommandInputsConfigurator`** — classmethod `configure(cls, command)` adds the dialog
  inputs in the order of the table below. No conditional visibility.
- **`ScrewGearGenerator(base.Generator)`** — 1-arg constructor `(design)` (inherited); implements
  `generate(self, inputs)` and the call graph below; relies on inherited `deleteComponent()` for
  error cleanup. Overrides `prefixBase()` to return `'ScrewGear'`.

**Generation Context: none.** Carry handles on `self`: `self.designOcc`, `self.gearOccs` (list of
two), `self.cageOcc`, `self.gearBodies` (list of two), `self.cageBody`, `self.axisLines` (list of
two).

**Dependencies: none.** Imports only the framework (`base`, `misc`, `utilities`, `solids`,
`fusion360utils`). Use `solids.hide_construction_geometry(component)` for the final cleanup — do
NOT re-implement it.

**Entry wiring:** `commands/screwgear/entry.py` constructs `GearCommand(gear_type='ScrewGear',
name='Screw Gear Generator', …)`, binding the two classes above by name (PLAYBOOK.md
"Command-entry wiring").

**Parameter mode: all-Python-precomputed** (`[PB-PRECOMPUTED-MODE]`). Every value is computed in
Python in internal cm and written numerically; the generator registers **no** named user parameters.
The trigonometry here (per-station rotation angles, the screw step matrix) has no useful expression
form in the parameter table.

## Component Setup

One command invocation creates a **`Screw Gearing`** component under the user-selected Parent
Component, holding four sub-components (`[PB-OCCURRENCE-TREE]`):

- **`Design`** — every sketch, construction plane and feature runs here.
- **`Gear A`**, **`Gear B`**, **`Cage`** — empty until the end, when the finished bodies are
  relocated into them with `body.moveToComponent` (`[PB-NO-CROSS-SIBLING]`).

**NEVER call `occurrence.activate()`** (`[PB-NEVER-ACTIVATE]`). Place sketches directly on the
user-selected plane (`[PB-USE-SELECTED-PLANE]`).

The user selects a **plane** and a **point**. The plane's normal is the cage axis and the common
perpendicular `n̂`; the point is the mechanism's centre `C`.

## Variables

User inputs in dialog order. All linear inputs are mm; the mounting angles are degrees.

| Dialog label | input id | unit | default |
|---|---|---|---|
| Ribbon Width | `ribbonWidth` | mm | 10 |
| Ribbon Thickness | `ribbonThickness` | mm | 2.5 |
| Tooth Pitch | `toothPitch` | mm | 1.75 |
| Tooth Height | `toothHeight` | mm | 1.2 |
| Tooth Count | `toothCount` | — | 80 |
| Twist Lead | `twistLead` | mm | 33 |
| Crossing Angle | `crossAngle` | deg | 80 |
| Engagement | `engagement` | mm | 0.36 |
| Mounting Angle A | `mountAngleA` | deg | 15 |
| Mounting Angle B | `mountAngleB` | deg | 15 |
| Boss Half Length | `bossHalf` | mm | 5 |
| Boss Taper | `bossTaper` | mm | 0.9 |
| Boss Height | `bossGrow` | mm | 0.6 |
| Cage Radius | `cageRadius` | mm | 11 |
| Ring Radius | `ringRadius` | mm | 12.5 |
| Cage Rise | `cageRise` | mm | 13.5 |
| Ring Wire | `ringWire` | mm | 2.5 |
| Rod Diameter | `rodDiameter` | mm | 2 |
| Collar Half Length | `collarHalf` | mm | 2 |
| Collar Wall | `collarWall` | mm | 2 |
| Clearance | `clearance` | mm | 0.3 |
| Target Plane | `plane` | selection | — |
| Centre Point | `point` | selection | — |
| Parent Component | `parent` | selection | — |

Module-level constants for every input id: `INPUT_ID_RIBBON_WIDTH = 'ribbonWidth'` and so on.

Read raw values with `design.unitsManager.evaluateExpression(input.expression, units)`, which
returns internal units — cm for length and **radians** for angle (`[PB-EVAL-EXPRESSION]`).

**Range checks, all raised as a clear error naming the offending field:**

- `toothHeight` must be `> 0` and `< ribbonWidth/2`.
- `engagement` must be `> 0` and `<= toothHeight`. Past the tooth height the two blanks foul each
  other rather than meshing.
- `toothPitch` must be `> 0`; `toothCount` must be `>= 4`.
- `twistLead` must be `> 0`. There is no upper bound: a very long lead approaches two straight racks
  pushing each other, which is the degenerate case Segerman names, and nothing here forbids it.
- `cageRadius` is where each gear's collars sit on its own axis, so it must clear the engaged zone,
  which the proof measures at ±4.05 mm. It must also leave the boss room: the boss is centred on the
  collar, so `cageRadius - bossHalf` must stay outside the engaged zone too, and
  `TestBossesClearTheOtherRibbon` measures what the two ribbons keep between them with the bosses
  on.
- `ringRadius` must leave every rod a place to stand. A rod on the ring's circle has to clear both
  ribbons over the stroke and still run through its own collar's wall; too small a ring puts that
  place inboard of the collar, and `TestRodsStandBesideTheirCollars` fails on either count.
- `cageRise` must put the ring and the loop clear of both ribbons. The ribbons reach further from
  the middle at the ring's radius than their own width suggests, because a ribbon crosses that
  radius at more than one station. `TestRibbonsClearTheFrameOverTheStroke` walks everything both
  ribbons reach over the stroke against the whole frame rather than arguing it.
- `collarHalf` trades grip against stroke. A longer collar holds the gear closer to its screw
  motion and shortens the travel, because the stroke is `2*(bossHalf - bossTaper - collarHalf)`.
- `collarWall` must be at least `rodDiameter`, or a rod running through it is not held.

**No range is enforced on either Mounting Angle, and none is asserted here.** 15° on both is the
smallest equal pair that drives at this twist; what happens elsewhere is the proof's to map, and
this spec does not clamp what has not been measured.

## Sketch Discipline

Every sketch must report `isFullyConstrained` before it is consumed, and the build raises naming the
sketch when one does not (`[PB-SKETCH-FIRST]`). Each section sketch here is a **rectangle** — four
lines, two dimensions, one anchoring coincidence and one angular dimension against the anchor line —
so there is no under-constrained case to exempt, unlike the bevel tooth profile.

## Method contract — call graph

```
generate(inputs)
  → processInputs(inputs)                    # read, check, precompute
  → buildComponentTree()                     # Screw Gearing + Design + 3 children
  → buildAnchor()                            # anchor sketch, centre point, reference direction
  → buildGear(index)   x2                    # per gear: cell → repeat → one body
      → buildAxis(index)                     # construction line on the gear's axis
      → buildToothCell(index)                # 9 section sketches → one loft
      → repeatCellByDoubling(index)          # copy + screw-move + join, log2 rounds
  → buildCage()                              # ring, loop, rods, collars; one twisted bore cut per collar
  → relocateBodies()                         # moveToComponent into Gear A / Gear B / Cage
  → solids.hide_construction_geometry(design)
```

## Instructions

### 1: Anchor and frame

Create the anchor sketch on the user-selected plane and project the selected point into it. Draw a
reference line through the projected point and fully constrain it exactly as the bevel spec's Anchor
Line is constrained: midpoint plus a length dimension plus `addHorizontal`, so the line has zero
degrees of freedom and its absolute direction is arbitrary. That line's direction is `ê`; the
plane's normal is `n̂`.

Compute both axes from `ê` and `n̂`:

```
dirA = rotate(ê, +Sigma/2, about n̂)      originA = C - (A/2)*n̂
dirB = rotate(ê, -Sigma/2, about n̂)      originB = C + (A/2)*n̂
```

Gear A's cross-section frame is `û_A = +n̂`, `v̂_A = dirA × û_A`; gear B's is `û_B = -n̂`,
`v̂_B = dirB × û_B`. **`û` points at the other gear** — that is what makes `Phi` mean the same thing
for both parts, and it is why the two gears are the same part rather than mirror images.

### 2: The tooth cell

One cell is one tooth pitch of the finished ribbon. Build it once per gear.

Create **`LoftSections` construction planes** perpendicular to the gear's axis, evenly spaced over
one pitch, with `setByDistanceOnPath` against the axis line (`[PB-CONSTRUCTION-PLANES]`; pass the
sketch line directly, never wrapped in `Path.create`).

On plane `k` draw the section sketch `{gearLabel} Section {k}`: a rectangle spanning `u` from `-W/2`
to `Utooth(s_k)` and `v` from `-T/2` to `+T/2`, rotated about the axis by

```
theta_k = s_k/Lambda + Phi
```

with `Utooth(s_k) = W/2 - H/2 + (H/2)*cos(2*pi*(s_k - Z0)/P)` and `Z0 = 0` for gear A, `Z0 = P/2`
for gear B (the assembly phase). Anchor the rectangle to the projected axis point and pin its
rotation with an angular dimension against the projected reference line, so the sketch closes fully
constrained.

**Loft the eleven sections in order** (`[PB-LOFT]`). The result is one pitch of the twisted toothed
ribbon.

**The section count is derived, not pinned, and it is not user-configurable.** It is the smallest
count that keeps the twist between neighbouring sections under 2°, which is 11 at the defaults. The loft's surface between two
sections is ruled, so it cuts the corner of the true helicoid by about `(W/2)*(1 - cos(dtheta/2))`
where `dtheta` is the twist between neighbouring sections. At the defaults that is 1.91° per step
and 0.7 µm of departure, which is nearly three orders below the 0.50 mm backlash. A faster twist
needs more sections for the same departure, which is why the count is derived from the twist per
tooth rather than fixed.

### 3: Repeat the cell by doubling

The finished ribbon is the cell repeated `N` times under the **screw step**

```
Step(k) = translate(k*P along the axis) ∘ rotate(k*P/Lambda about the axis)
```

Build it by doubling rather than by `N` separate placements: copy the current body, move the copy by
`Step(m)` where `m` is the number of teeth built so far, join, and repeat; finish with one partial
round for the remainder. That is `ceil(log2(N))` copy-move-join rounds — 7 at the default 80 teeth,
against 79 — and every placement is exact, because the body genuinely is invariant under `Step`.

Copy with `copyPasteBodies`, move with `moveFeatures` / `defineAsFreeMove(matrix)`
(`[PB-MOVE-ROTATE]`, `[SCREW-F-SCREW-STEP]`), and join with a `combineFeatures` join. Adjacent cells
meet at a shared planar cross-section, so the join leaves no sliver.

**A zero-angle matrix is a no-op that Fusion rejects** (`[PB-MOVE-ROTATE]`). A screw step is never
zero for `k >= 1`, so no guard is needed here, but do not "optimize" a `k = 0` case into the loop.

### 4: The cage

The frame is the open skeleton in the video, built from round sections and joined into one body
(`[SCREW-F-ROUND-FRAME]`). Heights are along `n̂` from the centre `C`; gear B's axis is on the
`+n̂` side and gear A's on the `-n̂` side.

- **The ring** is a torus about `n̂` at height `+cageRise`, of radius `ringRadius` to the centre of
  its wire and wire diameter `ringWire`. Revolve a circle of `ringWire` about `n̂`.
- **The rods** are four cylinders of `rodDiameter`, parallel to `n̂`, each running from
  `-cageRise` to `+cageRise` on the ring's circle. Each serves one collar and stands beside it:
  take the point where that ribbon's axis meets the circle of radius `cageRadius`, and turn round
  `n̂` from there by the least angle at which a rod on the ring's circle clears both ribbons by
  `clearance` over the whole stroke. All four are turned the **same way round** — counter-clockwise
  seen from the ring's end — so they land in the gaps between the ribbons rather than against each
  other. At the defaults the angles are 34.7° for a gear's collar at `+cageRadius` and 33.7° for
  the one at `-cageRadius`, which differ because the ribbon has a different cross-section angle at
  each crossing; `TestRodsStandBesideTheirCollars` derives and logs them, and the build takes them
  from the same search rather than from a table.
- **The loop** is four straight bars of `ringWire` at height `-cageRise`, each from one rod's foot
  to the next round the ring, with a ball of `ringWire` at each foot to round the corner. Its
  corners are the rods, so it is a rectangle inscribed in the ring's circle and reads smaller than
  the ring: 19.3 by 16.1 mm at the defaults, inside a 25 mm circle.
- **The collars** are one per crossing. A collar is the bore's rectangle grown by `collarWall` in
  every direction of its own section — a rounded rectangle — swept along the ribbon over
  `±collarHalf` from the crossing and turning with it, so its ends are flat and square to the
  ribbon's axis. Gear A's collars are centred on stations `±cageRadius` of its axis; gear B's on
  `±cageRadius` plus the assembly phase, which is where its bosses are once it is assembled, so
  they sit 0.9 mm along B's axis from the `cageRadius` circle. Build a collar as a loft of rounded
  rectangles on the same planes its bore is lofted on (`[SCREW-F-TWISTED-SLOT]`).

Each rod runs through the wall of its own collar, and that is the whole of what joins the two;
nothing else is added. The rod's axis passes 1.0 mm outside the bore's rectangle at the
defaults, inside the 2 mm wall, and `collarWall` may not go under `rodDiameter` for that reason.
`TestFrameIsOnePiece` walks ring → rods → loop and rod → collar and fails on any piece the ring
does not reach.

**Bore each collar with a twisted clearance ribbon** (`[SCREW-F-TWISTED-SLOT]`): loft rectangles of
`(W + 2*bossGrow + 2*clearance)` by `(T + 2*bossGrow + 2*clearance)` on planes along that gear's
axis, each rotated by `s/Lambda + Phi` exactly as the tooth cell's sections are.

**One loft per bore, spanning the collar's length plus a millimetre at each end**, so the cut runs
clean through. At the defaults that is 6 mm of axis and 65° of turn. The two bores of one gear
stand 2\*`cageRadius` apart, so one loft covering both would span 26 mm and 284° and need four
times the sections for the same accuracy.

**The section count is derived from the turn**, at no more than 5° between neighbours, which is 15
at the defaults. The tooth cell is held to 2° because its error is measured against the backlash;
a bore's is measured against the clearance, which is twenty times larger.

**A loft is flat between its sections, so the bore's wall is faceted and every facet stands inside
the true channel.** What that costs is clearance, straight out of the gap the boss passes through,
and enough of it binds the gear. At the derived count the facets take 0.005 mm of the 0.3 mm;
at the five sections this spec fixed before they take 0.060 mm. `TestBoreLoftKeepsItsClearance`
holds it at 95% of the clearance.

**The bore has to twist; a straight hole binds.** Over a collar of length `2*collarHalf` the ribbon
turns by `2*collarHalf/Lambda`, so its corner sweeps `(W/2)*(2*collarHalf/Lambda)` across the
opening. At the defaults that is 3.8 mm against 0.3 mm of clearance.

**The bore is cut to the boss, never to a tooth.** That is the whole reason the ribbon carries a
boss: nothing in the frame is shaped like a tooth.

### 5: Relocate the bodies

Move the two gear bodies and the joined cage body into their sub-components with `body.moveToComponent`,
which preserves world position and needs no activation.

## What the proof checks

`proof/screwgear` holds it. The whole risk in this gear is meshing, and no other proof in this
repository simulates motion, so this one models the ribbon implicitly — a point is inside when its
cross-section coordinates satisfy four inequalities — rather than as a solid. That is exact where a
boolean between two lofted solids would be a tangency `decad`'s exact predicates refuse to classify,
and it is cheap enough to run the search a few million times. The package imports neither engine.

- `TestPairDrivesOneToOne` is the one everything rests on. It tracks the free window in B's tooth
  phase through a full pitch of A and asserts three things: the window is never empty (no jam), it
  is narrower than a pitch (the teeth box B in, so something is driven), and its centre advances by
  exactly one pitch (the 1:1 ratio). The third is what separates a gear from two parts that merely
  touch, and an earlier arrangement passed the first two and failed it.
- `TestSymmetricMountJams` holds the reason the Mounting Angle exists, by failing if the `Phi = 0`
  arrangement ever stops jamming.
- `TestFullRibbonsClearOutsideTheEngagement` walks both parts end to end, so the contact search's
  window is not taken on trust.
- `TestRibbonIsInvariantUnderItsScrewStep` is what licenses building the ribbon as one cell repeated.
- `TestBoresAdmitOnlyTheScrewMotion` is the frame's own proof. It turns a gear out of step with its
  advance and finds where it jams in its collars, which is **3.21°** at the defaults. A frame of
  round holes would report no jam at any angle, and that is the case this rules out.
- `TestRibbonsClearTheFrameOverTheStroke` is the other half of that: the gear moves freely through
  its stroke. It walks everything both ribbons reach over the travel — the crest on the toothed
  side, the boss at its fullest for each station — against the ring, the loop, the rods and the
  collars, and logs the least distance to each: 1.59 mm to the ring, 2.53 mm to the loop and
  0.32 mm to the rods at the defaults. It is what sizes `cageRise`.
- `TestCollarBoresHoldTheBossOverTheStroke` holds that every collar sees the boss's flat top at
  every phase of the travel, the clearance away all round: a stroke longer than the boss allows
  would leave the ribbon loose in its collar at the ends of the travel.
- `TestRodsStandBesideTheirCollars` derives where each rod stands, logs the angles, and fails when
  no point of the ring's circle clears the ribbons or when the rod that does runs nowhere inside
  its collar's wall. `TestFrameIsOnePiece` then walks ring → rods → loop and rod → collar and
  fails on any piece the ring does not reach; it also logs the loop's sides.
- `TestBoresSitOnOppositeSidesOfTheMiddle` holds one gear's collars low against the other's high.
- `TestBossesClearTheOtherRibbon` is the frame's reach into the mesh: it walks each ribbon with
  its bosses on, over the stroke, against the other ribbon outside the engaged zone, and holds the
  boss's start outside that zone. The mesh proof walks bare ribbons, and the boss is where the
  frame sits.
- `TestBoreLoftKeepsItsClearance` is the one case that looks at what Fusion will really cut rather
  than at the ideal channel: it builds the bore's loft as the build would and requires 95% of the
  clearance to survive the facets.
- `TestTheMiddleStaysOpen` keeps the frame out of the space the gears mesh in.
- `TestStrokeIsTheBossLength` and `TestTeethNeverReachABore` hold the boss: the travel is 2.4 teeth,
  and no tooth is ever inside a collar over that travel, on either gear.
- `TestCrossedHelicalRuleMakesTheCrestHelicesParallel` pins `Sigma = 2*Beta` and the station it
  holds at.
- `TestLoftSectionCountHoldsTheHelicoid` is the arithmetic the eleven sections are bought with.
- `TestProportionsFollowTheVideo` holds the defaults inside the ranges read off the video, each
  widened by the ±20% the reading carries, for the ratios this spec follows: teeth per turn,
  thickness and length against the width, the ring's diameter against the width, the frame's
  height against the ring, and one hand for both gears. It logs the ratios the spec departs
  from, so a run shows both.

`TestRenderPair`, `TestRenderPart`, `TestRenderMesh` draw the pictures from the same section
function the mesh proof samples. They are skipped unless `-render.out` names a directory.

### What the proof cannot reach

It cannot say whether the sine tooth is the *right* tooth. Conjugate flanks for two screw motions
follow from the equation of meshing `n·v_rel = 0` against the relative screw, and nothing here
derives them; the proof measures what this tooth does, not what the best tooth would do.

It proves the ideal ribbon, an exact cosine on an exact helicoid, and not the lofted body Fusion
builds. `TestLoftSectionCountHoldsTheHelicoid` bounds the gap between the two at 0.7 µm against a
0.50 mm backlash, but that is arithmetic about the loft rather than a measurement of one.

It also cannot see print tolerance or friction. A window of 0.44–0.57 mm is comfortable for fused
filament and tight for resin, and only a printed part settles it.

It cannot see the video's model either. The ratios in "What the video shows" were read off
1280×720 frames by hand, and `TestProportionsFollowTheVideo` holds the defaults inside those
readings widened by the ±20% they carry; a finer reading needs the model, not the video.
