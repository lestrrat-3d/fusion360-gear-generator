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
- **The Cage**, a cylinder with most of its wall gone: a ring at the top, a ring at the bottom, and
  four posts standing between them. Each post sits where one ribbon crosses the cylinder and carries
  a **bore** that ribbon passes through. The two posts of one gear are bored low and the two of the
  other high, so the gears meet in the middle of the cylinder.

  The bore is what makes each gear's motion a screw motion rather than a free slide. It is cut to
  the ribbon's own cross-section and twisted at the ribbon's own lead, so a gear that turns without
  advancing jams in it. A round hole would not: it would leave the mechanism three degrees of
  freedom instead of one.

The gears' toothed edges meet in the middle of the cage, between the two bored heights. Pushing one gear along its axis drives the other, at
a **1:1 ratio** — one tooth pitch of advance each. The mechanism has one degree of freedom.

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

### Why the bores stand where they do

A bore is a hole through a post, and the posts stand along the cage axis, which is the axis a print
stands on. A bore is therefore a horizontal hole whose ceiling has to be bridged, and a bore whose
opening is **tall and narrow** bridges a short span where a wide flat one leaves a ceiling as wide
as the ribbon.

The opening's angle is the ribbon's cross-section angle where it crosses the cage, and there are
four of them: each gear crosses twice, at `+CageRadius` and `-CageRadius`, and those two are turned
in opposite directions. Upright at all four needs **the two mounting angles equal** and
**`CageRadius` a whole number of half turns of the ribbon**, and even then the four sit at plus and
minus the mounting angle. So the best any cage radius can do is the mounting angle itself.

At the defaults that is **15°**, which `TestBoresStandNearlyUpright` both measures and compares
against the best the mounting angles allow.

### The boss, and why the cage needs no tooth-shaped cut

The ribbon carries a **smooth boss** at each of the two places it passes through the cage: a
swelling that stands `BossGrow` proud of the plain section and fades the teeth out of it, running
back into them over `BossTaper` at each end. The bore is cut to the boss, so **no tooth ever enters
a bore** and nothing in the frame has to be cut to the shape of a tooth. Nothing bears on a crest
either.

The boss travels with its gear, so **its length is the stroke**: the mechanism runs while the boss
still fills the bores, which is 4.40 mm, or 2.5 teeth, at the defaults. `TestStrokeIsTheBossLength`
and `TestTeethNeverReachABore` hold both halves of that.

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
| Tooth Count `N` | 40 |
| Twist Lead | 30 mm per turn |
| Crossing Angle `Sigma` | 80° |
| Engagement | 0.36 mm |
| Mounting Angle, both gears | 15° |
| Cage Radius | 15 mm |
| Clearance | 0.3 mm |

Derived: `Beta` = 46.3°, `A` = 9.64 mm, ribbon length = 70 mm, twist per tooth = `P/Lambda` = 21.0°.

**These defaults are set by the frame, not only by the mesh.** A bore has to stand near upright or
it cannot be printed (see "Why the bores stand where they do"), and that fixes the cage radius at
`Lambda*pi` — half the twist lead. A compact cage therefore needs a fast twist, and a slow twist
needs a large cage. 30 mm per turn against a 15 mm cage radius is the compromise: at 20 mm the
ribbon is twisted past the point of looking like a rack, and at 40 mm the only arrangements with
equal mounting angles turned out to jam under finer sampling.

At those values `proof/screwgear` measures a free window in B's tooth phase that is **0.411–0.481 mm
wide** and that **advances by exactly one tooth pitch for each pitch A advances**, departing from
the 1:1 line by 0.064 mm, which is 3.7% of the pitch. That window width is the backlash, and the
winding is what makes this a 1:1 gear rather than two parts that merely touch. Away from the teeth
the two ribbons clear each other by 0.168 mm at their closest.

**Both Mounting Angles are 15°, and the frame is why.** Unequal angles drive too, and some drive
better, but the four bores can only stand near upright when the two angles are equal, and 15° is
the smallest equal angle that drives at this twist. The two gears are the same part and the frame
holds them alike.

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
| Tooth Count | `toothCount` | — | 40 |
| Twist Lead | `twistLead` | mm | 30 |
| Crossing Angle | `crossAngle` | deg | 80 |
| Engagement | `engagement` | mm | 0.36 |
| Mounting Angle A | `mountAngleA` | deg | 15 |
| Mounting Angle B | `mountAngleB` | deg | 15 |
| Boss Half Length | `bossHalf` | mm | 4 |
| Boss Taper | `bossTaper` | mm | 0.9 |
| Boss Height | `bossGrow` | mm | 0.6 |
| Cage Radius | `cageRadius` | mm | 15 |
| Cage Rise | `cageRise` | mm | 12.5 |
| Ring Bar | `ringBar` | mm | 1.2 |
| Post Bar | `postBar` | mm | 2 |
| Block Depth | `blockDepth` | mm | 1.8 |
| Block Wall | `blockWall` | mm | 1.5 |
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
- `cageRadius` is where each gear's bores sit on its own axis, so it must clear the engaged zone,
  which the proof measures at ±5.27 mm. It must also leave the boss room: the boss is centred on the
  bore, so `cageRadius - bossHalf` must stay outside the engaged zone too.
- `cageRise` must put both rings clear of both ribbons. The ribbons reach further from the middle at
  the cage radius than their own width suggests, because a ribbon crosses that radius at more than
  one station. `TestRibbonsClearTheCage` walks the whole of both ribbons against the whole frame
  rather than arguing it.
- `blockDepth` trades grip against stroke. A deeper bore holds the gear closer to its screw motion
  and shortens the travel, because the stroke is `2*(bossHalf - bossTaper - blockDepth/2)`.

**No range is enforced on either Mounting Angle, and none is asserted here.** 15° on both is the
smallest equal pair that drives at this twist, and equal is what the bores need; what happens
elsewhere is the proof's to map, and this spec does not clamp what has not been measured.

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
  → buildCage()                              # tube, then one twisted slot cut per gear
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

**Loft the nine sections in order** (`[PB-LOFT]`). The result is one pitch of the twisted toothed
ribbon.

**The section count is derived, not pinned, and it is not user-configurable.** It is the smallest
count that keeps the twist between neighbouring sections under 2°, which is 12 at the defaults. The loft's surface between two
sections is ruled, so it cuts the corner of the true helicoid by about `(W/2)*(1 - cos(dtheta/2))`
where `dtheta` is the twist between neighbouring sections. At the defaults that is 1.91° per step
and 0.7 µm of departure, which is nearly three orders below the 0.45 mm backlash. A faster twist
needs more sections for the same departure, which is why the count is derived from the twist per
tooth rather than fixed.

### 3: Repeat the cell by doubling

The finished ribbon is the cell repeated `N` times under the **screw step**

```
Step(k) = translate(k*P along the axis) ∘ rotate(k*P/Lambda about the axis)
```

Build it by doubling rather than by `N` separate placements: copy the current body, move the copy by
`Step(m)` where `m` is the number of teeth built so far, join, and repeat; finish with one partial
round for the remainder. That is `ceil(log2(N))` copy-move-join rounds — 6 at the default 40 teeth,
against 39 — and every placement is exact, because the body genuinely is invariant under `Step`.

Copy with `copyPasteBodies`, move with `moveFeatures` / `defineAsFreeMove(matrix)`
(`[PB-MOVE-ROTATE]`, `[SCREW-F-SCREW-STEP]`), and join with a `combineFeatures` join. Adjacent cells
meet at a shared planar cross-section, so the join leaves no sliver.

**A zero-angle matrix is a no-op that Fusion rejects** (`[PB-MOVE-ROTATE]`). A screw step is never
zero for `k >= 1`, so no guard is needed here, but do not "optimize" a `k = 0` case into the loop.

### 4: The cage

Build the two rings, then the four posts, then bore each post.

**The rings** are circles of `cageRadius` swept with a round bar of `ringBar`, on planes through `C`
normal to `n̂` at `±cageRise`. Sweep is not available here (`[SCREW-F-TWISTED-SLOT]` says why), so
revolve a `ringBar` circle about `n̂` at that radius.

**The posts** stand at the four azimuths where the ribbons cross the cylinder: each gear's axis
direction and its opposite. A post is a round bar of `postBar` running the full height from the
bottom ring to the top, so every post meets both rings and the frame is one body
(`TestPostsReachBothRings`).

**Each post carries a block** around its bore: a slab of `blockDepth` standing across that gear's
axis at station `±cageRadius`, reaching `blockWall` past the bore on every side.

**Bore each post with a twisted clearance ribbon** (`[SCREW-F-TWISTED-SLOT]`): loft five rectangles
of `(W + 2*bossGrow + 2*clearance)` by `(T + 2*bossGrow + 2*clearance)` on planes along that gear's
axis, each rotated by `s/Lambda + Phi` exactly as the tooth cell's sections are, spanning the whole
post rather than only the block. **The whole post, not just the block**: a post is solid bar above
and below its block, and the ribbon has to get past that too.

**The bore has to twist; a straight hole binds.** Over a block of depth `tau` the ribbon turns by
`tau/Lambda`, so its corner sweeps `(W/2)*(tau/Lambda)` across the opening. At the defaults that is
1.41 mm against 0.3 mm of clearance.

**The bore is cut to the boss, never to a tooth.** That is the whole reason the ribbon carries a
boss: the frame is plain round bar and plain rectangular openings, and nothing in it is shaped like
a tooth.

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
  advance and finds where it jams in its bores, which is **3.21°** at the defaults. A frame of round
  holes would report no jam at any angle, and that is the case this rules out.
- `TestRibbonsClearTheCage` walks the whole of both ribbons against the whole frame, which is what
  sizes `cageRise`. One static pass settles every position the gears take, because a bore is cut to
  the ribbon and the ribbon is invariant under its own screw motion.
- `TestPostsReachBothRings` and `TestBoresSitOnOppositeSidesOfTheMiddle` hold the frame's shape: one
  body, and one gear's bores low against the other's high.
- `TestStrokeIsTheBossLength` and `TestTeethNeverReachABore` hold the boss: the travel is 2.5 teeth,
  and no tooth is ever inside a bore over that travel.
- `TestCrossedHelicalRuleMakesTheCrestHelicesParallel` pins `Sigma = 2*Beta` and the station it
  holds at.
- `TestLoftSectionCountHoldsTheHelicoid` is the arithmetic the nine sections are bought with.

`TestRenderPair`, `TestRenderPart`, `TestRenderMesh` draw the pictures from the same section
function the mesh proof samples. They are skipped unless `-render.out` names a directory.

### What the proof cannot reach

It cannot say whether the sine tooth is the *right* tooth. Conjugate flanks for two screw motions
follow from the equation of meshing `n·v_rel = 0` against the relative screw, and nothing here
derives them; the proof measures what this tooth does, not what the best tooth would do.

It proves the ideal ribbon, an exact cosine on an exact helicoid, and not the lofted body Fusion
builds. `TestLoftSectionCountHoldsTheHelicoid` bounds the gap between the two at 0.7 µm against a
0.28 mm backlash, but that is arithmetic about the loft rather than a measurement of one.

It also cannot see print tolerance or friction. A window of 0.28 mm is comfortable for fused
filament and tight for resin, and only a printed part settles it.
