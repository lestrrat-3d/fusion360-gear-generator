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
  gear's motion a screw motion rather than a free slide. It is cut to the ribbon's crest
  rectangle, a clearance larger all round, and twisted at the ribbon's own lead, so a gear
  that turns without advancing jams in it. A round hole would not: it would leave the
  mechanism three degrees of freedom instead of one. The teeth run straight through the
  collars, as they do in the video.

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
| Cage size | Ring outer diameter 2.2–2.8 widths, about 0.8 of a twist lead (0:09, 6:10); the frame is about as tall as the ring is wide (5:26, 5:34) | Outer diameter 2.5 widths, 0.76 leads; 1.18 ring widths tall |
| The frame | An open skeleton of round rods, described below the table (0:09, 5:23, 5:26, 5:28, 5:31, 5:34, 5:36, 5:37, 6:10, 6:16, 6:42) | Same: a ring, a loop with straight sides, four rods and four collars |
| What passes through a collar | The plain ribbon: the teeth run through the collars (0:09, 6:12, 6:14) | Same. The bore is the crest rectangle plus the clearance, and the crests bear on its toothed side |
| Travel | Most of the ribbon: at 5:53 the frame sits near one end of a ribbon, at 6:00 near its middle | 115 mm, 82% of the ribbon, 66 teeth |
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
and at 1280×720 it is some thirty pixels long. The collar length here is a choice, 4 mm, and
"Travel" records what it costs; the rod's place is derived from the clearance it needs ("The
cage").

One departure is deliberate, and it has its reason elsewhere in this spec: **the crossing
angle** and **the lead** come from the meshing search, not from the video. With `CrossAngle`
set to 90° and nothing else moved, `TestPairDrivesOneToOne` reports a 0.187 mm departure from
the 1:1 line against its 0.10 mm bound; with the lead at 40 mm it reports 0.130 mm. 80° at
33 mm reports 0.067 mm.

The ring sits in the middle of the video's range, and the rods tie the cage radius to it. A
rod has to clear the ribbon beside it by the clearance at every phase of the travel, and a
ribbon reaches 5.15 mm from its own axis at its corners, so a rod on the ring's circle stands
6.7 mm from the crossing. It also has to run through its collar's wall, and the collar is 4 mm
long along the ribbon. On an 11.25 mm ring the rod's axis crosses the ribbon 9.26 mm from the
middle for the collar at `-cageRadius` and 9.28 mm for the one at `+cageRadius`, the stations
`TestRodsStandBesideTheirCollars` logs, so its 2 mm diameter starts 8.26 mm out: inside a collar
centred at 10 mm, which runs from 8 to 12 mm, and outside one centred at 10.5 mm, which starts
at 8.5 mm. So the cage radius is 10 mm. An earlier version of this spec carried a smooth boss on each ribbon at
the collars, 0.6 mm proud, so that no tooth ever entered a bore; the boss is gone, because the
video's ribbons carry none and the crest rectangle already holds the whole ribbon ("Why the
cage needs no tooth-shaped cut"). While it was there the rods stood 7.3–7.5 mm from the
crossings and the ring had to be 12.5 mm, the top of the video's range.
`TestRodsStandBesideTheirCollars` holds the join.

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

### Why the cage needs no tooth-shaped cut

The ribbon is the same twisted rack along its whole length, and the crest of a cosine rack is the
ribbon's outer edge: `Utooth` peaks at `u = W/2` and falls to `W/2 - H` at the root, while the
back edge stays at `-W/2` and the faces at `±T/2` whatever the phase. So the **crest rectangle**,
`W` by `T`, holds every point of the ribbon at every station and every phase, and the bore is
that rectangle plus the clearance all round, `(W + 2*Clearance)` by `(T + 2*Clearance)`, 10.6 by
3.1 mm at the defaults, twisted at the ribbon's lead. Nothing in the frame is shaped like a
tooth, and the teeth run through the collars as the video's do.

What bears in a bore is the crests. The back edge and both faces run the clearance from the wall
at every station; on the toothed side a crest comes to the clearance every pitch and the root
between falls `H` short of it, so the bore's toothed side bears on a crest line and never on a
flank. `TestRibbonsStayInsideTheirBoresOverTheTravel` holds every point of the ribbon inside
every bore by the clearance through the whole travel, and holds the crests, the back edge and
the faces to the clearance, so the bore is cut to the ribbon rather than merely round it.

The twist is what holds the motion, not the tooth. A rectangle twisted at the ribbon's lead
admits the screw motion and nothing else: `TestBoresAdmitOnlyTheScrewMotion` turns a gear out
of step with its advance and finds its crests and back corners jamming in the collars at 3.55°.

### Travel

Nothing on the ribbon limits the travel, since any stretch of it fits a collar; the ribbon's
length does. A gear has to keep both its collars full, and its end reaches a collar's far face
after `N*P/2 - CageRadius - CollarHalf` of advance, 58 mm at the defaults, so **a ribbon runs
`N*P - 2*(CageRadius + CollarHalf)`, 116 mm, through its collars**. Gear B is assembled 0.90 mm
along its axis (the assembly phase), so it reaches one collar that much sooner than gear A does,
and the pair travels **115.1 mm: 57.1 mm back and 58.0 mm forward** of the assembly position,
65.8 teeth, 82% of the ribbon. The engaged zone is nearer the middle than the collars, so the
teeth are still meshing at both ends of that: an end would leave the ±4.05 mm zone at 65 mm.
`TestTravelIsTheRibbonBetweenItsCollars` walks both limits.

The collar's length comes straight out of the travel, two millimetres per millimetre of collar,
and the frames do not settle it; 4 mm is kept.

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
closest approach — and they cross everywhere else. The proof measures contact between 1.3 mm short
of the closest approach and 1.7 mm past it, which is nowhere near that station, so what this pair
actually carries is a point contact like a crossed-helical pair rather than the line contact the
rule describes. The meshing search moved the angle off the rule: at 90°, `TestPairDrivesOneToOne`
reports a 0.187 mm departure from the 1:1 line against its 0.10 mm bound, and at 80° it reports
0.067 mm ("What the video shows"). The search is what settles it, and 80° is what it settled on.

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
| Assembly Phase | −0.90 mm |
| Cage Radius | 10 mm |
| Ring Radius | 11.25 mm |
| Clearance | 0.3 mm |

Derived: `Beta` = 43.6°, `A` = 9.64 mm, ribbon length = 140 mm, twist per tooth = `P/Lambda` =
19.1°, ribbon turn between a gear's two collars = `2*CageRadius/Lambda` = 218°.

**The frame no longer sets the lead.** An earlier frame had to print a bore through a wall, which
tied the cage radius to half the twist lead and the two mounting angles to each other. The video's
frame has no wall, so the ring is read from the video, the cage radius is what lets a rod on that
ring reach its collar, and the lead stands on its own: at 20 mm per turn the ribbon is twisted
past the point of looking like a rack, and at 40 mm the arrangements with equal mounting angles
turned out to jam under finer sampling (`mesh-search.md`). 33 mm is where it stayed.

At those values `proof/screwgear` measures a free window in B's tooth phase that is **0.438–0.569 mm
wide** and that **advances by exactly one tooth pitch for each pitch A advances**, departing from
the 1:1 line by 0.067 mm, which is 3.8% of the pitch. That window width is the backlash, and the
winding is what makes this a 1:1 gear rather than two parts that merely touch.

**The sampling those numbers were taken with.** They are `TestPairDrivesOneToOne`'s, at the
sampling `pair_test.go` fixes, and a model at another sampling moves their last digit or two:

- Each ribbon's boundary is sampled at stations within `±axialWindow` of the crossing,
  `1.5 * sqrt(W^2 - A^2) / sin(Sigma)`, ±4.05 mm at the defaults, every 0.01 mm of station; at each
  station, five points across the thickness on the toothed edge and five on the back edge, and
  seven across the width on each face.
- A sample is inside the other ribbon when its coordinates in that ribbon's section at its own
  station, twist undone, satisfy the four inequalities of "The part". The pair is clear at a
  phase pair when no sample of either ribbon is inside the other.
- The free window at a phase of A is scanned in B's phase in steps of `P/200`, 0.00875 mm,
  outward from the previous window's middle, so its ends are quoted to that step. A advances
  through one pitch in 12 equal steps; the width is the narrowest and widest of those 12 windows,
  and the departure is the largest gap, over the 12, between a window's middle and the straight
  1:1 line from the first middle to the last.

Two other clearances are quoted in this spec, and each is a different measurement:

- **0.254 mm** is how far the two ribbons stand from each other at the assembly phases, gear A at
  tooth phase 0 and gear B at −0.90 mm, with nothing moved. `TestFullRibbonsClearOutsideTheEngagement`
  walks the toothed edge and the back edge of each whole ribbon, every 0.01 mm of station at five
  points across the thickness, and takes the least slack of any sample against the other ribbon:
  the slack is measured in that ribbon's own section at the sample's station, along `u` to its
  toothed or back edge and along `v` to its faces, whichever is least, so it is a slack rather than
  a Euclidean distance, and only its sign is exact. The least is 0.254 mm at station 0.30 mm, in
  the mesh: gear B sits in the middle of a window 0.499 mm wide, so it has about half of that each
  way before a flank touches.
- **1.04 mm** is the least the two ribbons keep from each other outside the engaged zone at any
  phase of the travel. `TestRibbonsClearEachOtherOutsideTheEngagement` walks each ribbon's crest
  rectangle, `W` by `T`, which is everything the ribbon reaches at any phase, over every station
  the travel carries it through but outside `±axialWindow` of the crossing, every 0.02 mm of
  station at five points along each side of the rectangle, and takes the least slack of any sample
  against the other ribbon's crest rectangle, measured as above. It is 1.04 mm at station
  −4.40 mm of the sampled ribbon, just outside the zone.

**Both Mounting Angles are 15°.** Unequal angles drive too, and some drive better; 15° on both is
the smallest equal angle that drives at this twist, and the two gears are the same part held
alike. The earlier frame needed the two equal for its bores to stand upright, and this one does
not; the angles stay where the search left them rather than being searched again for a frame
that no longer cares.

**Assembly phase.** With gear A at tooth phase 0, gear B is built at **−0.90 mm**: its teeth and
its ends are gear A's advanced by that much along its own axis under its own screw motion ("The
part"), so its ribbon runs from station −70.9 mm to +69.1 mm where gear A's runs ±70 mm. The number
is the middle of the free window the proof measures at gear A's phase 0, −1.154 to −0.655 mm at
the sampling above, and `TestAssemblyPhaseSitsInTheFreeWindow` holds it there. The build cannot
measure that window, so the phase is the `assemblyPhase` input, defaulting to the proof's number
as the mounting angles and the engagement do; an arrangement the proof has not measured needs its
own.

## Architecture

The module `lib/geargen/screwgear.py` defines exactly these public classes (the command wiring binds
to them by name; exported via `lib/geargen/__init__.py`):

- **`ScrewGearCommandInputsConfigurator`** — classmethod `configure(cls, command)` adds the dialog
  inputs in the order of the table below. No conditional visibility.
- **`ScrewGearGenerator(base.Generator)`** — 1-arg constructor `(design)` (inherited); implements
  `generate(self, inputs)` and the call graph below; relies on inherited `deleteComponent()` for
  error cleanup. Overrides `prefixBase()` to return `'ScrewGear'`.

**Generation Context: none.** Carry handles on `self`: `self.designOcc`, `self.gearOccs` (list of
two), `self.cageOcc`, `self.gearBodies` (list of two), `self.cageBody`, `self.pathLines` (list of
two dicts, one per gear, keyed `'collar-'`, `'bore-'`, `'collar+'` and `'bore+'`, each the sketch
line of §1 that one sweep of §4 runs along).

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

User inputs in dialog order. All linear inputs are mm; the crossing and mounting angles are
degrees.

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
| Assembly Phase | `assemblyPhase` | mm | −0.90 |
| Cage Radius | `cageRadius` | mm | 10 |
| Ring Radius | `ringRadius` | mm | 11.25 |
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

- `ribbonWidth`, `ribbonThickness`, `toothPitch`, `twistLead`, `ringWire`, `rodDiameter`,
  `collarHalf`, `collarWall` and `clearance` must be `> 0`; `toothCount` must be a whole number
  `>= 4`.
- `toothHeight` must be `> 0` and `< ribbonWidth/2`.
- `engagement` must be `> 0` and `<= toothHeight`. Past the tooth height the two blanks foul each
  other rather than meshing.
- `twistLead` has no upper bound: a very long lead approaches two straight racks pushing each
  other, which is the degenerate case Segerman names, and nothing here forbids it. The section
  floor in §2 is what keeps the tooth at a long lead.
- `crossAngle` must lie strictly between 0° and 180°. At either end the axes are parallel and the
  engaged zone below has no length. Only 80° has been
  proved to drive; the search covered 38.5°–100° (`mesh-search.md`), and nothing here clamps to it.
- `assemblyPhase` must lie strictly within `±toothPitch`. A phase a pitch further on is the same
  phase with the ribbon's ends moved a pitch. Only the
  default has been measured to sit in the free window.
- `cageRadius` is where each gear's collars sit on its own axis, so a collar must clear the
  engaged zone: `cageRadius - collarHalf` must exceed the zone's half-length,
  `1.5 * sqrt(W^2 - A^2) / sin(Sigma)`, which is the reach the proof's `axialWindow` samples and
  is 4.05 mm at the defaults. `TestRibbonsClearEachOtherOutsideTheEngagement` holds the same
  inequality along with what the two ribbons keep between them there. Both bores, each a
  millimetre past its collar's ends (§4), also have to lie within the ribbon's length, so
  `cageRadius + collarHalf + 1 mm < toothCount*toothPitch/2`, the
  millimetre being the bore's margin. It is also tied to `ringRadius` by the rods, below.
- `ringRadius` must leave every rod a place to stand. The rod search of §4 runs here, once per
  collar in the order gear A `-R`, gear A `+R`, gear B `-R`, gear B `+R`, and fails this field
  when no azimuth on the ring's circle clears both ribbons, when the rod that does runs nowhere
  inside its own collar's wall (`0 < depth <= collarWall`, §4), when part of its diameter misses
  the collar's length (`|sp - sc| + rodDiameter/2 <= collarHalf`, §4), or, after all four, when
  two rods stand nearer than `rodDiameter + clearance`; the message names the collar, or the
  two rods. `TestRodsStandBesideTheirCollars` fails on the same counts, and
  `TestCoincidentRodsAreRefused` holds an input that reaches the last one. At the defaults the
  rod's axis crosses the ribbon 9.26 mm from the middle for the collar at `-cageRadius` and
  9.28 mm for the one at `+cageRadius`, the stations the proof logs, against a collar centred at
  10 mm and 4 mm long, and a cage radius of 10.5 mm on the same ring already misses.
- `cageRise` must put the ring and the loop clear of both ribbons. No point of a ribbon is further
  from the middle plane than its axis is plus the half-diagonal of its crest rectangle,
  `A/2 + hypot(W/2, T/2)`, 9.97 mm at the defaults, and the ring's wire and the loop's bars start
  `ringWire/2` inside `±cageRise`, so the build requires
  `cageRise - ringWire/2 - (A/2 + hypot(W/2, T/2)) >= clearance`, which leaves 2.28 mm at the
  defaults. That is a sufficient bound, not the gap: the ribbons cross the ring's radius at an
  angle and reach it at more than one station, so the real gap is larger, and
  `TestRibbonsClearTheFrameOverTheTravel` walks everything both ribbons reach over the travel
  against the whole frame — 2.66 mm at the ring, 3.73 mm at the loop — and holds the bound beside
  it.
- `collarHalf` trades grip against travel. A longer collar holds the gear closer to its screw
  motion and shortens the travel by twice its own growth, because a ribbon runs
  `toothCount*toothPitch - 2*(cageRadius + collarHalf)` through its collars.
- `collarWall` must be at least `rodDiameter`, or a rod running through it is not held.

**No range is enforced on either Mounting Angle, and none is asserted here.** 15° on both is the
smallest equal pair that drives at this twist; what happens elsewhere is the proof's to map, and
this spec does not clamp what has not been measured.

## Sketch Discipline

Every sketch must report `isFullyConstrained` before it is consumed, and the build raises naming the
sketch when one does not (`[PB-SKETCH-FIRST]`, `[PB-FULL-CONSTRAINT]`). No sketch here carries text,
so none is exempt. Six rules hold in every sketch of this build:

- **Only the Anchor sketch projects anything, and every other sketch's references are fixed
  points of its own** (`[SCREW-F-REFERENCES]`, `[PB-PROJECT-NOT-FIXED]`). The Anchor sketch
  projects the selected point and binds the Anchor Line to the projection exactly as the bevel
  gear's Anchor sketch does, which reads fully constrained in Fusion. Every later sketch takes
  its references — the centre point, the ends of a sweep path, the axis point of a section, the
  corners of a cell section, a rod's foot, a bar's corners, a ball's chord ends — as
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
  else** (`[PB-SKETCH-DEFER]`, `[SCREW-F-DEFER]`): the Cell Sections sketch of §2, and the eight
  section sketches of §4, the collars' and the bores'. In those, set
  `sketch.isComputeDeferred = True` right after the sketch is created and named and before its
  first point, and set it back to `False` after the last curve, constraint, dimension or
  `isFixed`, before `isFullyConstrained` or `profiles` is read. Measured in Fusion on
  2026-09-28: a collar section fell from 0.45 s to 0.23 s and a one-tooth Cell Sections
  sketch from 0.88 s to 0.09 s, each still reading fully constrained with its profiles found.
  The Anchor sketch does not defer, because deferral has not been measured under a projection;
  the Paths, Ring, Rods, bar and ball sketches do not, because they are a few fixed points with
  lines, arcs or circles and were not measured.

The eight section sketches of §4 — the collars' and bores' — share one rectangle scheme, stated
there, that leaves no freedom, so there is no under-constrained case to exempt, unlike the bevel
tooth profile. Every other sketch is built from reference points alone: lines and arcs between
them and circles centred on them, with nothing left to solve.

## Method contract — call graph

```
generate(inputs)
  → processInputs(inputs)                    # read, check, precompute; the rod search runs here
  → buildComponentTree()                     # Screw Gearing + Design + 3 children
  → buildAnchor()                            # anchor sketch, centre point, reference direction, axis planes, n̂
  → buildGear(index)   x2                    # per gear: paths → cell → repeat → one body
      → buildSweepPaths(index)               # Paths sketch: the four sweep-path lines on the gear's axis
      → buildToothCell(index)                # one Cell Sections sketch (41 sections at the defaults) → one loft
      → repeatCellByDoubling(index)          # copy + screw-move + join, in cells; a remainder cell when N is not a multiple of cellTeeth
  → buildCage()                              # ring, rods, loop, collars, then one twisted sweep cut per collar
  → relocateBodies()                         # moveToComponent into Gear A / Gear B / Cage
  → solids.hide_construction_geometry(self.designOcc.component)
```

### What the build makes

Counted at the defaults, in timeline entries, so a Fusion load can be checked against it. The
first two Fusion loads (`[SCREW-F-FIRST-LOAD]`) built the same geometry as 136 sketches, 134
construction planes and 79 features, one sketch and one plane per loft section, and took about
five minutes; this construction replaces every per-section sketch and plane with one sweep or
one sketch, and the counts are what the diagnostics of 2026-09-28 measured piece by piece.

| Part | Sketches | Planes | Features, `cellTeeth = 4` | Features, `cellTeeth = 1` |
|---|---|---|---|---|
| Anchor (§1) | 1 | 0 | 0 | 0 |
| Gear axis planes (§1) | 0 | 2 | 0 | 0 |
| Per gear: Paths sketch (§1) | 1 | 0 | 0 | 0 |
| Per gear: Cell Sections sketch and loft (§2) | 1 | 0 | 1 | 1 |
| Per gear: doubling (§3), 3 features a round | 0 | 0 | 15 (5 rounds) | 21 (7 rounds) |
| Both gears, subtotal | 4 | 0 | 32 | 44 |
| Ring (§4): Ring Plane, Ring sketch, revolve | 1 | 1 | 1 | 1 |
| Rods (§4): Rods sketch, extrude, join | 1 | 0 | 2 | 2 |
| Loop (§4): Loop Plane, 4 bar and 4 ball sketches, 8 revolves, 1 join | 8 | 1 | 9 | 9 |
| Collars (§4): 4 planes, 4 sketches, 4 sweeps, 1 join | 4 | 4 | 5 | 5 |
| Bores (§4): 4 planes, 4 sketches, 4 sweep cuts | 4 | 4 | 4 | 4 |
| Relocate (§5): 3 `moveToComponent` | 0 | 0 | 3 | 3 |
| **Total** | **23** | **12** | **56** | **68** |

That is 91 timeline entries at `cellTeeth = 4` and 103 at `cellTeeth = 1`, plus the five
component creations: 96 and 108 in all, against about 349 before. The third Fusion load
(`[SCREW-F-FIRST-LOAD]`) counted exactly 96 timeline items. Check a load against the timeline
count, not against `Component.features.count`: summed over the five components that read 62,
six more than the 56 features the timeline holds, all six in `Design`, for a reason not yet
measured. A remainder cell (§3) adds one sketch, one loft
and one join per gear that needs one; the defaults need none. The heaviest sketches are now the
eight section sketches of §4 at 0.23 s each and the two Cell Sections sketches at 0.37 s each
with computing deferred; each collar sweep took 0.04 s, each cell loft 0.23 s, and the doubling's
copies, moves and joins were not timed on their own.

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
numbers, and the Anchor Line itself is passed once more, as the line the Ring Plane of §4 is
built through.

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
`theta = s/Lambda + Phi_g`; every seed below is that point.

**The axis planes, and the sign of `n̂`.** Two construction planes are offset from the selected
plane itself (`[PB-USE-SELECTED-PLANE]`, `[PB-CONSTRUCTION-PLANES]`): `Gear A Axis Plane` by
`-A/2` and `Gear B Axis Plane` by `+A/2`, both with `setByOffset`. Fusion offsets along the
selected entity's own normal, and the build does not assume its sign: `n̂` is read from the Gear
A Axis Plane after it is made — the unit normal of its `geometry`, signed so that `C` lies `+A/2`
along it — and the build checks that `C` is `A/2` from each of the two planes
(`[SCREW-F-NORMAL-SIGN]`). Every later offset from the selected plane uses the same sign as these
two: a negative offset lands on gear A's side, a positive one on gear B's.

**The Paths sketches.** `Gear A Paths` on the Gear A Axis Plane and `Gear B Paths` on gear B's,
each holding the four lines that gear's collars and bores are swept along (§4), all on the
gear's axis, which lies in that plane. A collar at crossing station `sc` (`-cageRadius` or
`+cageRadius`) spans `sc - collarHalf` to `sc + collarHalf`, and its bore spans a millimetre
more at each end, `sc - collarHalf - 1 mm` to `sc + collarHalf + 1 mm`. The sketch holds eight
reference points (Sketch Discipline), at `origin_g + s*dir_g` for those eight stations, all on
the plane, and four solid lines — `collar-`, `bore-`, `collar+`, `bore+` — each drawn from its
negative station to its positive one, sharing both points, so its start is its negative end and
it runs along `+dir_g`; then all eight points are set `isFixed = True`. The lines carry no
dimension and no constraint, and nothing else is in the sketch. A collar's line and its bore's
overlap, and all four lie on one line; Fusion read such a sketch fully constrained on
2026-09-28, and a path made from one of its lines with chaining off held that one line
(`[PB-PATH-FROM-SKETCH]`). A line whose two ends are fixed has a `worldGeometry` the build can
trust (`[PB-WORLDGEO-CONSTRAINED]`), and a line held by dimensions from a projected point does
not. The section plane of each collar and bore is `setByDistanceOnPath` on its own line at
fraction `0` (`[PB-CONSTRUCTION-PLANES]`; pass the line directly), so it stands at the span's
negative end, square to the axis; the point where the line pierces it is the span's first
station, `origin_g + s*dir_g`, and Fusion put that plane's origin on the station to four decimals
of a millimetre. The sweep's path is `features.createPath(line, False)` on the same line
(`[SCREW-F-TWISTED-SLOT]`).

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
  the twist per step: 0.7 µm at the defaults' 1.91°, three orders below the 0.50 mm backlash;
- eight. A straight chord of the cosine between two sections falls `(H/2)*(1 - cos(pi/n))` short
  of it at the deepest point whatever the twist: 0.029 mm at the defaults' ten steps, 6% of the
  backlash, and 0.046 mm, 3.8% of the tooth height, at eight. That floor is what keeps a slow
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
has. What the smooth surface does between sections was measured in Fusion on 2026-09-28
(`[SCREW-F-CELL-LOFT]`): at one, four and eight teeth, probes 0.04 mm inside and 0.04 mm
outside the toothed edge and a face, at the midpoint between every pair of sections, all fell
on the right side of the built surface — 40 of 40, 160 of 160 and 320 of 320 — and the built
volume was 0.06% to 0.23% under the ruled loft's. So at the defaults the built surface stays
within 0.04 mm of the helicoid between sections, under a tenth of the 0.44 mm backlash. The
proof cannot measure that ("What the proof cannot reach"), and a Fusion load is what checks it
at other inputs. A chain of `c*n` two-section lofts would be ruled and would carry the bounds
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
body and that body `isSolid` (`[PB-EMPTY-RESULT]`). Measured at the defaults on 2026-09-28: the
four-tooth loft took 0.23 s and held 0.164158 cm³ against the ruled loft's 0.164470.

### 3: Repeat the cell by doubling

The finished ribbon is the cell, `c` teeth, repeated under the **screw step**

```
Step(k) = translate(k*P along the axis) ∘ rotate(k*P/Lambda about the axis)
```

Write `N = q*c + r`, with `q = floor(N/c)` whole cells and a remainder of `r = N mod c` teeth:
`q = 20` and `r = 0` at the defaults, `q = 80` at a one-tooth cell. Build the `q` cells by
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

That is `floor(log2 q) + popcount(q) - 1` rounds and as many joins: 5 at the defaults — `q = 20`,
four doublings to 16 cells and one aside of 4 cells, taken when the body held 4 cells and moved
by `Step(64)` — and 7 at a one-tooth cell — `q = 80`, six doublings to 64 and one aside of 16,
moved by `Step(64)` — against 79 placements. When `q = 1` there is no round. Every placement is
exact, because the body genuinely is invariant under `Step`
(`TestRibbonIsInvariantUnderItsScrewStep`). Every move is by a whole number of teeth already
built, so each join meets its neighbour at one shared planar cross-section and leaves no sliver
and no overlap; `TestDoublingScheduleCoversTheRibbon` runs the schedule at every count from 4 to
512 with cells of one, three and four teeth.

**The remainder.** When `r > 0`, the last `r` teeth are a second, shorter cell, built where they
belong rather than moved there: a sketch `{gearLabel} Cell Remainder` and a loft by the recipe of
§2 with `c` replaced by `r`, so `r*n + 1` sections at stations from `s0 + q*c*P` to `s0 + N*P`,
joined into the body after the last round. Its first section is the body's last, so the join
meets at a shared cross-section like every other. The defaults have none; 81 teeth in
four-tooth cells would have one of one tooth.

Copy with `copyPasteBodies.add(body)` and take the copy from the feature's `bodies`
(`[SCREW-F-COPY-BODY]`); move with `moveFeatures.createInput2(bodies)` → `defineAsFreeMove(matrix)`
→ `add(input)` (`[PB-MOVE-ROTATE]`, `[SCREW-F-SCREW-STEP]`); join with a `combineFeatures` join and
check that it leaves exactly one body: the combine feature's own `bodies.count` is 1, and that
body is the ribbon from then on (`[PB-EMPTY-RESULT]`, `[SCREW-F-ROUND-FRAME]`). After the last
join name the body `Gear A` or `Gear B`.

**A zero-angle matrix is a no-op that Fusion rejects** (`[PB-MOVE-ROTATE]`). A screw step is never
zero for `k >= 1`, so no guard is needed here, but do not "optimize" a `k = 0` case into the loop.

### 4: The cage

The frame is the open skeleton in the video, built from round sections and joined into one body
(`[SCREW-F-ROUND-FRAME]`). Heights are along `n̂` from the centre `C`; gear B's axis is on the
`+n̂` side and gear A's on the `-n̂` side.

- **The ring** is a torus about `n̂` at height `+cageRise`, of radius `ringRadius` to the centre of
  its wire and wire diameter `ringWire`: a circle of `ringWire` revolved about the line through
  `C` along `n̂` (`[SCREW-F-ROUND-FRAME]`).
- **The rods** are four cylinders of `rodDiameter`, parallel to `n̂`, each running from
  `-cageRise` to `+cageRise` on the ring's circle. Each serves one collar and stands beside it.
  A collar's **crossing** is the point where its ribbon's axis meets the circle of radius
  `cageRadius`, at station `+cageRadius` or `-cageRadius` of that axis, the sign being that of
  the axis direction `dirA` or `dirB` of §1. From the crossing's azimuth about `n̂`, turn
  **counter-clockwise about `+n̂`** — the sense seen from the ring's end, gear B's side — by the
  least angle at which a rod on the ring's circle clears both ribbons by `clearance`, as
  defined next. All four are turned the **same way round**, so they land in the gaps between the
  ribbons rather than against each other. At the defaults the turn is 34.61° for a gear's
  collar at `-cageRadius` and 34.43° for the one at `+cageRadius`, which differ because the
  ribbon has a different cross-section angle at each crossing; `TestRodsStandBesideTheirCollars`
  derives and logs them, and the build takes them from the same search rather than from a
  table. The rods are four circles on one sketch, named `Rods`, on the selected plane, each
  centred on its foot `C + ringRadius*(cos(psi)*ê + sin(psi)*k̂)` with `k̂ = n̂ × ê` and `psi`
  the rod's azimuth, its centre `isFixed` and its diameter dimensioned (Sketch Discipline), and
  all four extruded in one feature symmetrically `cageRise` either side of the plane.

  **What "clears" means.** The rod is parallel to `n̂` and runs past both ribbons, so only
  distance in the selected plane counts, and what a ribbon reaches at a station, at some phase
  of the travel, is its crest rectangle turned to that station's angle. Seen along `n̂` that
  rectangle is a segment across the axis of half-width
  `h(s) = (W/2)*|sin(theta_g(s))| + (T/2)*|cos(theta_g(s))|`, its extreme a corner. With a
  rod's foot `F` on the ring's circle, `sp = (F - origin_g)·dir_g` its station on gear `g` and
  `wp = (F - origin_g)·v̂_g` its offset across that axis, the distance from the rod's axis to the
  ribbon at station `s` is `hypot(sp - s, max(0, |wp| - h(s)))`. A rod **clears** when that
  distance is at least `rodDiameter/2 + clearance` for both gears at every station `s` that is
  a whole multiple of 0.01 mm with `|s - sp| <= rodDiameter/2 + clearance`. A station further
  along the axis than that from `sp` is further than that from the rod on the first term
  alone, so no other station can fail. **The search** walks the azimuth up from the crossing in
  quarter-degree steps to the first that clears — refusing `ringRadius` if a full turn finds
  none — then bisects between that and the last that did not, moving the upper end down when
  the middle clears and the lower end up when it does not, until the two are within 0.001°;
  the turn is the upper end, or 0 when the crossing itself clears. `rodClears` and `rodShift`
  in `cage_test.go` are this search, step for step.
- **The loop** is four straight bars of `ringWire` at height `-cageRise`, each from one rod's
  foot to the next round the ring in azimuth order, with a ball of `ringWire` at each foot to
  round the corner. **Foot 0** is the rod whose azimuth, taken into `[0°, 360°)`
  counter-clockwise about `+n̂` from `ê`, is the smallest; feet 1, 2 and 3 follow in increasing
  azimuth, and bar `i` runs from foot `i` to foot `(i + 1) mod 4`, so bar 3 closes the loop.
  Bars and balls are numbered from 0 by their foot. At the defaults the azimuths are 74.43°
  for gear A's `+R` rod, 174.61° for gear B's `-R`, 254.61° for gear A's `-R` and 354.43° for
  gear B's `+R`, so foot 0 is gear A's `+R` rod. Its corners are the rods, so it is inscribed
  in the ring's circle and reads smaller than the ring: 17.26 and 17.21 mm by 14.46 mm at the
  defaults, inside a 22.5 mm circle. It is not quite a rectangle. The two short bars each join
  a gear A rod to the gear B rod of the same sign of station, and those two rods are turned
  from their crossings by the same angle, so each short bar spans exactly the 80° crossing
  angle round the ring; the two long bars span 100° plus and minus the 0.19° by which the two
  turns differ (34.612° for a collar at `-cageRadius` and 34.426° at `+cageRadius`, which the
  proof logs as 34.61° and 34.43°). Equal short sides on one circle make the long sides
  parallel, so the loop is an isosceles trapezoid, a rectangle to within 0.05 mm on a side;
  `TestFrameIsOnePiece` logs the four sides. A bar is a rectangle `ringWire/2` wide and the
  bar's length long, drawn on the Loop Plane itself from four reference points and revolved a
  full turn about the bar's own line, foot to foot; a ball is a half-disc of `ringWire` on the
  Loop Plane, revolved a full turn about its chord through the foot
  (`[SCREW-F-ROUND-FRAME]`). No plane is made per bar or per ball: the Loop Plane holds every
  foot, and both revolves were measured exact on it on 2026-09-28.
- **The collars** are one per crossing. A collar is the bore's rectangle grown by `collarWall` in
  every direction of its own section — a rounded rectangle — running along the ribbon over
  `±collarHalf` from the crossing and turning with it, so its ends are flat and square to the
  ribbon's axis. Every collar is centred on its crossing, station `±cageRadius` of that gear's
  axis for both gears alike. The assembly phase moves gear B's ribbon along its axis and not its
  collars: any stretch of the ribbon fits a collar, and the ribbon can go in at any phase a pitch
  apart, so the frame has no way to know the phase. Build a collar as **one sweep** of its
  section along its own `collar-` or `collar+` line of §1, twisted by `2*collarHalf/Lambda`,
  43.6° at the defaults, so the section turns with the ribbon (`[SCREW-F-TWISTED-SLOT]`,
  `[PB-SWEEP-TWIST]`). Its one plane is `{gearLabel} Collar {-R|+R} Plane`, `setByDistanceOnPath`
  on that line at fraction `0`, so it stands at `sc - collarHalf`, `-R` for the collar at
  `-cageRadius` and `+R` at `+cageRadius`; the bores' below are named the same with `Bore` for
  `Collar`. On it the sketch `{gearLabel} Collar {-R|+R}` is the collar's section at that
  station: the bore's rectangle below, drawn as construction by the rectangle scheme, with four
  solid sides parallel to it `collarWall` outside and a quarter-circle arc of radius
  `collarWall` at each corner, tangent to one side and centred on the construction corner, so
  the profile is four lines and four arcs (`find_profile_by_curve_counts(sketch, lines=4,
  arcs=4)`). The sweep is `sweepFeatures.createInput(profile, path, NewBodyFeatureOperation)`
  with `path = features.createPath(line, False)`, `twistAngle` set to
  `ValueInput.createByReal(+2*collarHalf/Lambda)` and nothing else set, then
  `sweepFeatures.add`; it leaves one solid body of ten faces whose far end is the same section
  turned by the twist, which the build checks (`[SCREW-F-SWEEP-CHECK]`). The sign is positive:
  the path runs along `+dir_g` and the section's angle `s/Lambda + Phi` grows with `s`, and a
  positive `twistAngle` turns the profile that way, measured on 2026-09-28 with the far end
  landing 0.0047 mm from the spec's section and 4.9 mm from it under the opposite sign. The
  collar is the exact helicoid §4 describes, a rounded rectangle turning rigidly, not a fit
  through samples of it.

  **The rectangle scheme.** Each of the eight section sketches of this section — the four
  collars' and the four bores' — is a rectangle of half-thickness `hv` spanning `u` from `uB` to
  `uF`, turned by `theta = s/Lambda + Phi_g` about the axis point `O`, which lies on the
  rectangle's long centre line but at neither its centre nor a corner. Four lines, two
  dimensions, one coincidence and one angle cannot fix `O` inside such a rectangle; a
  construction spine through `O` can, and this is the scheme:

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
  - **The rectangle.** Four lines sharing their corners: `L1` from `(uB, -hv)` to `(uF, -hv)`,
    `L2` on to `(uF, hv)`, `L3` on to `(uB, hv)`, `L4` back to the start, every seed the solved
    point (`[PB-SHARE-XOR-COINCIDENT]`: shared, no coincident on a corner). Then `L1` parallel to
    `K` with an offset dimension of `hv`, and `L3` likewise on the other side; `E` coincident on
    `L2`, and `L2` perpendicular to `K`; `L4` parallel to `L2` with an offset dimension of
    `uF - uB` (`[PB-OFFSET-DIM]`, `[PB-NO-OVERCONSTRAIN]`).

  Ten degrees of freedom — `E` and the four corners — against ten rows: five dimensions (the
  length, the angle, three offsets) and five constraints (three parallels, a perpendicular, a
  coincidence), so the sketch closes fully constrained. In every one of the eight sketches
  `uB = -(W/2 + clearance)`, `uF = W/2 + clearance` and `hv = T/2 + clearance`, the bore's
  rectangle; in a bore sketch its four lines are solid and are the profile, the one loop of four
  lines, `find_profile_by_curve_counts(sketch, lines=4)` (`[PB-PROFILE-MATCH]`); in a collar
  sketch they are construction and the profile is the rounded rectangle drawn round them
  (`[SCREW-F-TWISTED-SLOT]`). The construction lines bound nothing. The scheme is drawn with
  sketch computing deferred (Sketch Discipline).

Each rod runs through the wall of its own collar, and that is the whole of what joins the two;
nothing else is added. The build checks that join with two numbers per rod, both from the
search's `sp` and `wp` on the rod's own gear, and refuses `ringRadius`, naming the collar, when
either fails:

- **The rod runs inside the collar's wall.** The rod is parallel to the section's `u` direction
  and crosses the ribbon at the one station `sp`, so its axis runs a line across that one
  section, `wp` from the axis, and the nearest it comes to the bore's rectangle is
  `depth = |wp| - hb`, with `hb = (W/2 + clearance)*|sin(theta_g(sp))| + (T/2 +
  clearance)*|cos(theta_g(sp))|` the rectangle's shadow there, as `h(s)` is the crest
  rectangle's. The rod runs inside the wall when `0 < depth <= collarWall`. The search leaves
  the axis `rodDiameter/2 + clearance` outside the crest rectangle's shadow and the bore's
  shadow is at most `clearance*sqrt(2)` wider, so the lower bound fails only for a rod thinner
  than `0.42*clearance`, and the build checks it all the same; the upper bound is what
  `collarWall` must reach, and it is why `collarWall` may not go under `rodDiameter`.
- **The whole rod runs within the collar's length.** The collar's ends are square to the ribbon
  at `sc ± collarHalf`, so the rod's station has to sit its own radius inside them:
  `|sp - sc| + rodDiameter/2 <= collarHalf`.

At the defaults the rod's axis stands 1.00 mm outside the bore for a collar at `-cageRadius`
and 0.92 mm for one at `+cageRadius`, inside the 2 mm wall, and 0.74 mm and 0.72 mm along the
ribbon from the collar's middle, inside the 4 mm length; `TestRodsStandBesideTheirCollars`
checks the same two numbers and walks the rod through the 3-D model beside them.
`TestFrameIsOnePiece` walks ring → rods → loop and rod → collar and fails on any piece the ring
does not reach.

After all four rods are placed the build refuses `ringRadius`, naming the two rods, when any
two feet stand nearer than `rodDiameter + clearance`. That check is reachable, not ornamental:
at a crossing angle of 8°, an engagement of 0.01 mm and a 1 mm rod, with everything else at
its default, every check before it passes and two rods land on one spot, because at a small
crossing angle the rod for one gear's collar has to turn past the other gear's ribbon as well
and comes to rest where the other gear's rod already stands. `TestCoincidentRodsAreRefused`
holds that input.

**Bore each collar with a twisted sweep cut** (`[SCREW-F-TWISTED-SLOT]`, `[PB-SWEEP-TWIST]`):
the crest rectangle plus the clearance, `(W + 2*clearance)` by `(T + 2*clearance)`, 10.6 by
3.1 mm at the defaults, drawn by the rectangle scheme above on the bore's own plane,
`{gearLabel} Bore {-R|+R} Plane`, `setByDistanceOnPath` at fraction `0` of the bore's `bore-` or
`bore+` line of §1, in the sketch `{gearLabel} Bore {-R|+R}`, at that plane's station
`sc - collarHalf - 1 mm`; then swept along that line with `twistAngle` set to
`+2*(collarHalf + 1 mm)/Lambda`, 65.4° at the defaults, as
`sweepFeatures.createInput(profile, path, CutFeatureOperation)` with `participantBodies` set to
a list holding the cage body alone, and nothing else set. The ribbons pass through the channel
and are not on that list, so the cut leaves them whole: measured on 2026-09-28, a body wholly
inside the channel and off the list kept its volume to the last digit while the cage lost the
channel's. The bore is the exact helicoid the model describes — the rectangle turning rigidly at
`1/Lambda` about the axis, the channel `TestRibbonsStayInsideTheirBoresOverTheTravel` and
`TestBoresAdmitOnlyTheScrewMotion` measure against — with no facets and no fit between samples.

**One sweep per bore, spanning the collar's length plus a millimetre at each end**, so the cut
runs clean through the collar's flat ends and through whatever of its rod stands inside the
wall. At the defaults that is 6 mm of axis and 65° of turn. The two bores of one gear stand
2\*`cageRadius` apart and are two sweeps on two lines, as §1 draws them.

**What the proof's stand-in costs.** The proof's solid engine has no twisted sweep, so the
compiled step proof stands in a ruled loft through rotated rectangles for each collar and bore
("What the proof checks"), and the count it lofts through is derived from the turn and the
clearance: no two neighbouring sections more than 5° of twist apart, nor more than the angle at
which the facets between them take 4% of the clearance, `2*acos(1 - 0.04*clearance/R)` with
`R = hypot(W/2 + clearance, T/2 + clearance)` the bore's corner radius, and the count is
`ceil(turn / step) + 1` for the smaller step: 15 at the defaults, where the facet bound is 7.6°
and 5° governs, and 22 at a clearance of 0.05 mm, where the facet bound is 3.2° and governs.
A ruled loft's wall is flat between sections and every facet stands inside the true channel, so
what the stand-in costs is clearance: 0.004 mm of the 0.3 mm at the derived count, 0.054 mm at
the five sections an earlier version of this spec fixed. `TestBoreSubstituteKeepsItsClearance`
builds that ruled channel and holds it at 95% of the clearance, at clearances from 0.05 mm to
0.6 mm, which is what says a measurement made on the stand-in holds for the swept channel to
within that much. The build derives no section count: its channel has none.

**The bore has to twist; a straight hole binds.** Over a collar of length `2*collarHalf` the ribbon
turns by `2*collarHalf/Lambda`, so its corner sweeps `(W/2)*(2*collarHalf/Lambda)` across the
opening. At the defaults that is 3.8 mm against 0.3 mm of clearance.

**The bore is cut to the crest rectangle, never to a tooth.** The crests are the ribbon's outer
edge, so that rectangle holds the whole ribbon, teeth included, and its toothed side bears on the
crests ("Why the cage needs no tooth-shaped cut"); nothing in the frame is shaped like a tooth.

**The sweep's sense is checked, not assumed** (`[SCREW-F-SWEEP-CHECK]`, `[PB-SELF-DIAGNOSING]`).
After each collar sweep the build reads the body's `vertices` and requires that each of the
section's eight outline points at the far station `sc + collarHalf` — the ends of the four
sides, at `(uB, -hv - w)`, `(uF, -hv - w)`, `(uF + w, -hv)`, `(uF + w, hv)`, `(uF, hv + w)`,
`(uB, hv + w)`, `(uB - w, hv)` and `(uB - w, -hv)` with `w = collarWall`, turned by the far
station's own angle — lies within 0.05 mm of some vertex, and the same for the eight at the
near station; it raises naming the collar and the worst distance otherwise. Measured on
2026-09-28: 0.0047 mm at the far end and 0.0000 mm at the near end with the right sign, 4.9 mm
at the far end with the wrong one. After each bore cut the build requires the sweep feature to
leave exactly one body and probes the cage with `pointContainment` at the two points
`origin_g + s*dir_g ± (W/2 + clearance/2)*û(s)`, `û(s)` the section's turned `u` direction, at
`s = sc + collarHalf/2`, the far half of the collar: both must be `PointOutsidePointContainment`,
the channel being open there, and the build raises naming the bore otherwise. Those probes also
tell the two senses apart whenever
`(W/2 + clearance/2) * |sin(2*(1.5*collarHalf + 1 mm)/Lambda)| > T/2 + clearance`, which is
5.14 mm against 1.55 mm at the defaults: under the wrong sense the channel at that station is
turned 87° from the probes, which then sit in the collar's wall.

**Order.** The ring is the first body. The rods are joined into it in one combine, as they are
extruded in one feature; the loop's four bars and four balls are made as new bodies and joined
in one combine of eight tools; the four collars are made as new bodies and joined in one combine
of four tools; then the four bores are swept as cuts, each with the cage as its only
participant, so each cut runs through the collar and through whatever of its rod stands inside
the wall in one operation (`[SCREW-F-ROUND-FRAME]`). Every join and cut has to leave exactly
one body, counted as the combine or sweep feature's `bodies.count`, and that body is the cage
from then on; the build raises with the piece's name otherwise. The one body that remains after
the last cut is the cage.

### 5: Relocate the bodies

Name the cage body `Cage`, then move the two gear bodies and the cage body into their
sub-components with `body.moveToComponent`, which preserves world position and needs no
activation. Then `solids.hide_construction_geometry(self.designOcc.component)`: the `Design`
sub-component is where every sketch and construction plane of this build was made, and the helper
walks it and anything under it.

## What the proof checks

`proof/screwgear` holds two proofs side by side, and they have different jobs.

The **hand-written mechanism proof** is the `Test` functions listed below, in `geometry_test.go`
(the model, the part and the section count), `pair_test.go` (the mesh), `cage_test.go` (the frame)
and `render_test.go` (the pictures). It is written by hand, is not compiled from this spec, and
is not the compile stage's job to reproduce: it proves the mechanism — that these two parts
drive each other 1:1 and move freely in this frame — and the compile stage reads its numbers
rather than re-deriving them. The whole risk in this gear is meshing, and no other proof in this
repository simulates motion, so these files model the ribbon implicitly — a point is inside when
its cross-section coordinates satisfy four inequalities — rather than as a solid. That is exact
where a boolean between two lofted solids would be a tangency `decad`'s exact predicates refuse to
classify, and it is cheap enough to run the search a few million times. These four files import
neither engine.

The **compiled step proof** is what `/compile-gear` writes beside them from the step list, in
the shape the compile contract fixes: one function per build step, no `Test` functions of its
own, registrations generated into `zz_registrations_test.go`, and the `sketch` and `decad`
engines for what it builds. It covers the build steps of "Instructions" — that every sketch
scheme closes fully constrained and unambiguous, and that every solid step yields the body the
next step consumes — and nothing in the list below. A recompile regenerates it and leaves the
four hand-written files alone. Four steps of this build are ones the engines cannot build as
Fusion does, and each takes a stand-in, to be named in the proof beside what it stands in for:

- The Cell Sections sketch (§2) holds points off its plane, and the sketch engine is planar. The
  stand-in is one planar sketch per section, on that station's own plane, of the four corners
  as fixed points and four lines, which pins each section's numbers; Fusion's verdict on the one
  3D sketch — fully constrained, one profile per section — was measured on 2026-09-28 and is
  the sketch step's `[PROSE]` part.
- The cell loft (§2) is smooth in Fusion. `decad` lofts ruled between sections, so the stand-in
  is the ruled loft through the same `c*n + 1` sections, whose departure from the helicoid is
  what `TestLoftSectionCountHoldsTheHelicoid` bounds and whose volume Fusion's smooth loft came
  within 0.23% of.
- A collar or bore sweep (§4) has no `decad` counterpart. The stand-in is a ruled loft through
  the sections "What the proof's stand-in costs" derives, 15 at the defaults, built as the
  sweep's own section turned by `s/Lambda + Phi` at each; `TestBoreSubstituteKeepsItsClearance`
  bounds what that costs against the exact channel, 0.004 mm of the 0.3 mm clearance at the
  defaults. The twist's sense and its linearity are Fusion's (`[PB-SWEEP-TWIST]`), measured on
  2026-09-28 and checked at build time (`[SCREW-F-SWEEP-CHECK]`).
- A bore's cut with a participant list (§4) is a `decad` cut of the stand-in from the cage
  alone; that the ribbons are left whole is Fusion's, measured on 2026-09-28.

The mechanism proof's cases:

- `TestPairDrivesOneToOne` is the one everything rests on. It tracks the free window in B's tooth
  phase through a full pitch of A and asserts three things: the window is never empty (no jam), it
  is narrower than a pitch (the teeth box B in, so something is driven), and its centre advances by
  exactly one pitch (the 1:1 ratio). The third is what separates a gear from two parts that merely
  touch, and an earlier arrangement passed the first two and failed it.
- `TestSymmetricMountJams` holds the reason the Mounting Angle exists, by failing if the `Phi = 0`
  arrangement ever stops jamming.
- `TestFullRibbonsClearOutsideTheEngagement` walks both parts end to end, so the contact search's
  window is not taken on trust.
- `TestRibbonIsInvariantUnderItsScrewStep` is what licenses building the ribbon as one cell
  repeated. It carries points of the twisted blank and the four corners of every cross-section,
  teeth included, through one screw step and requires them to land on the next cell's own, so
  the body genuinely is invariant under `Step`, not only the twist.
- `TestBoresAdmitOnlyTheScrewMotion` is the frame's own proof. It turns a gear out of step with its
  advance and finds where its crests and back corners jam in its collars, which is **3.55°** at
  the defaults. A frame of round holes would report no jam at any angle, and that is the case
  this rules out.
- `TestRibbonsClearTheFrameOverTheTravel` is the other half of that: the gear moves freely through
  its travel. It walks everything both ribbons reach at any phase — the crest rectangle, over
  every station the travel carries it through — against the ring, the loop, the rods and the
  collars, and logs the least distance to each: 2.66 mm to the ring, 3.73 mm to the loop and
  0.30 mm to the rods at the defaults, the last being the clearance itself, since each rod stands
  at the least angle that clears. It is what sizes `cageRise`, and it holds the build's own bound
  on `cageRise` ("Variables") beside the walk.
- `TestRibbonsStayInsideTheirBoresOverTheTravel` holds every point of the ribbon, teeth included,
  inside every bore by the clearance at every phase of the travel, and holds the bore no looser
  than that: the crests, the back edge and the faces each come to the clearance, 0.300 mm at the
  defaults, so the bore is cut to the ribbon rather than merely round it.
- `TestRodsStandBesideTheirCollars` derives where each rod stands by the search the build runs,
  step for step (`rodClears`, `rodShift`), logs the angles, and fails when no point of the
  ring's circle clears the ribbons, when the rod that does runs nowhere inside its collar's wall
  by the closed form the build checks (`rodWallDepth`, held against a walk of the rod through
  the 3-D model), when part of its diameter misses the collar's length, or when two rods touch.
  `TestCoincidentRodsAreRefused` holds an input — crossing angle 8°, engagement 0.01 mm, a 1 mm
  rod — at which every earlier check passes and two rods land on one spot, so the last refusal
  is one the build can reach.
  `TestFrameIsOnePiece` then walks ring → rods → loop and rod → collar and fails on any piece
  the ring does not reach; it also logs the loop's sides.
- `TestBoresSitOnOppositeSidesOfTheMiddle` holds one gear's collars low against the other's high.
- `TestRibbonsClearEachOtherOutsideTheEngagement` walks everything each ribbon reaches at any
  phase, over every station the travel carries it through, against the other ribbon outside the
  engaged zone, and holds the collars outside that zone. The mesh proof walks the bare ribbons at
  the assembly phases; this is the same clearance at every phase of the travel.
- `TestBoreSubstituteKeepsItsClearance` bounds the compiled proof's stand-in for the bore
  sweep: it builds the ruled loft through the sections "What the proof's stand-in costs"
  derives and requires 95% of the clearance to survive the facets, at clearances from 0.05 mm to
  0.6 mm, so that a measurement made on the stand-in holds for the exact channel Fusion sweeps
  to within 0.004 mm at the defaults.
- `TestTheMiddleStaysOpen` keeps the frame out of the space the gears mesh in.
- `TestTravelIsTheRibbonBetweenItsCollars` walks each gear out of the assembly position both ways
  until an end leaves a collar, and until an end leaves the engaged zone, and holds the travel to
  the collars: 115.1 mm for the pair, 65.8 teeth, with the mesh limit further out at 65 mm.
- `TestCrossedHelicalRuleMakesTheCrestHelicesParallel` pins `Sigma = 2*Beta` and the station it
  holds at.
- `TestLoftSectionCountHoldsTheHelicoid` is the arithmetic the section count is bought with: the
  twist chord at the crest and the cosine chord on the toothed edge, over leads from 20 mm to
  400 mm, so the floor of eight steps is held where the twist is slow; it logs the cell's
  section count at `cellTeeth` of 4 and 1, 41 and 11 at the defaults.
- `TestDoublingScheduleCoversTheRibbon` runs the doubling schedule of §3 at every tooth count from
  4 to 512 with cells of one, three and four teeth, and holds that the pieces tile the ribbon
  with no overlap, in `floor(log2 q) + popcount(q) - 1` rounds plus one remainder join when
  `N mod c` is not zero: 5 rounds at the defaults, 7 at a one-tooth cell.
- `TestProportionsFollowTheVideo` holds the defaults inside the ranges read off the video, each
  widened by the ±20% the reading carries, for the ratios this spec follows: teeth per turn,
  thickness and length against the width, the ring's diameter against the width, the frame's
  height against the ring, and one hand for both gears. It logs the ratios the spec departs
  from, so a run shows both.

`TestRenderPair`, `TestRenderPart`, `TestRenderMesh` in `render_test.go` draw the pictures from
the same section function the mesh proof samples. They are skipped unless `-render.out` names a
directory.

### What the proof cannot reach

It cannot say whether the sine tooth is the *right* tooth. Conjugate flanks for two screw motions
follow from the equation of meshing `n·v_rel = 0` against the relative screw, and nothing here
derives them; the proof measures what this tooth does, not what the best tooth would do.

It proves the ideal ribbon, an exact cosine on an exact helicoid, and not the lofted body Fusion
builds. `TestLoftSectionCountHoldsTheHelicoid` bounds the gap between the helicoid and a *ruled*
loft through the build's sections — 0.7 µm at the crest and 0.029 mm on the toothed edge, where
a ruled loft draws a chord of the cosine — against a 0.50 mm backlash. Fusion's loft through
more than two sections is smooth between them, not ruled (§2, "What the loft is"), so that is a
bound on a body Fusion does not build: the built surface passes through the same sections, and
how far it departs from the helicoid between them is measured by nothing here. A Fusion
measurement on 2026-09-28 put it within 0.04 mm at the defaults (`[SCREW-F-CELL-LOFT]`), and
only a Fusion load sees it at other inputs. The collar and bore are exact helicoids in Fusion
and have no such gap; `TestBoreSubstituteKeepsItsClearance` bounds the proof's own stand-in
for them, not the built part.

It cannot see the sweep. The solid engine has no twisted sweep, so which way Fusion turns a
profile for a positive `twistAngle`, and that it turns it linearly along the path, are Fusion's
facts, measured on 2026-09-28 (`[PB-SWEEP-TWIST]`) and re-checked on every build by the
end-face and probe checks of §4 (`[SCREW-F-SWEEP-CHECK]`). Nor can it see a sketch whose
points lie off its plane: the sketch engine is planar, so the Cell Sections sketch's own
constraint verdict is Fusion's, measured on 2026-09-28 at 11, 41 and 81 sections.

It cannot see where Fusion puts a plane. The sketch engine draws each section on an exact plane
at `z = 0`, so a point Fusion leaves a few nanometres off its plane — which made the first Fusion
load fail at `Gear A Collar -R Section 0` (`[PB-SKETCH-ZERO-Z]`) — does not exist in the proof.
What the proof can hold is the rule's side of it: the compiled step list must set `z = 0` on
every mapped point that is meant to lie on its plane, and a step that maps one without it is a
compile defect.

It also cannot see print tolerance or friction. A window of 0.44–0.57 mm is comfortable for fused
filament and tight for resin, and only a printed part settles it.

It cannot see the video's model either. The ratios in "What the video shows" were read off
1280×720 frames by hand, and `TestProportionsFollowTheVideo` holds the defaults inside those
readings widened by the ±20% they carry; a finer reading needs the model, not the video.
