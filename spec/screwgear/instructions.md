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
- **The Cage**, a short tube whose wall carries one twisted slot per gear. A twisted plate driven
  through a fixed slot must rotate as it advances, exactly as a twisted-bar screwdriver does, so the
  slot is what makes each gear's motion a screw motion rather than a free slide.

The gears' toothed edges meet inside the cage. Pushing one gear along its axis drives the other, at
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
Phi    = Mounting Angle          the cross-section's angle at the crossing station
```

**The screw motion is a shift of `Z` and nothing else.** Advancing the gear by `dz` translates it by
`dz` and rotates it by `dz/Lambda`; the twisted blank is invariant under exactly that motion, so the
only thing that changes in the body's own frame is the tooth phase. Every step below relies on this:
it is why the whole ribbon is one tooth cell repeated by a screw step, and why the proof can pose
meshing as a search over two numbers.

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

### Why the mounting angle is not zero

The obvious arrangement points both toothed edges straight at each other at the crossing station
(`Phi = 0`). **That arrangement jams.** A meshing sweep over crossing angle, mounting angle, hand
and engagement found that at `Phi = 0` no phase of gear B clears gear A at every phase of gear A,
at any engagement past roughly a quarter of the tooth height, because the engaged zone spans two to
three tooth pairs whose ridges cross at an angle and cannot all interdigitate at once. Turning both
ribbons by `Phi` about their own axes moves the crossing to a station where they do clear.

The same sweep is what the defaults below come from; `mesh-search.md` records its model, its
criterion and what it covered. **The proof owns these numbers** and must re-derive them (see "What
the proof must check").

### Defaults, and what they were measured to do

| Quantity | Default |
|---|---|
| Ribbon Width `W` | 10 mm |
| Ribbon Thickness `T` | 2.5 mm |
| Tooth Pitch `P` | 3.5 mm |
| Tooth Height `H` | 1.2 mm |
| Tooth Count `N` | 24 |
| Twist Lead | 90 mm per turn |
| Engagement | 0.72 mm (60% of Tooth Height) |
| Mounting Angle `Phi` | 15° |
| Clearance | 0.3 mm |

Derived: `Beta` = 19.24°, `Sigma` = 38.48°, `A` = 9.28 mm, ribbon length = 84 mm, twist per tooth =
`P/Lambda` = 14.0°.

At those values `proof/screwgear` measures a free window in B's tooth phase that is **0.21–0.26 mm
wide** and that **advances by exactly one tooth pitch for each pitch A advances**, departing from
the 1:1 line by 0.058 mm, which is 1.7% of the pitch. That window width is the backlash, and the
winding is what makes this a 1:1 gear rather than two parts that merely touch. Away from the teeth
the two ribbons clear each other by 0.081 mm at their closest.

**Assembly phase.** With both gears at `Phi = 15°` and gear A at tooth phase 0, gear B meshes at a
tooth phase near `P/2` — crest against root. Build B half a pitch out of step with A. The exact
value is the centre of the free window and belongs to the proof.

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

User inputs in dialog order. All linear inputs are mm; Mounting Angle is degrees.

| Dialog label | input id | unit | default |
|---|---|---|---|
| Ribbon Width | `ribbonWidth` | mm | 10 |
| Ribbon Thickness | `ribbonThickness` | mm | 2.5 |
| Tooth Pitch | `toothPitch` | mm | 3.5 |
| Tooth Height | `toothHeight` | mm | 1.2 |
| Tooth Count | `toothCount` | — | 24 |
| Twist Lead | `twistLead` | mm | 90 |
| Engagement | `engagement` | mm | 0.72 |
| Mounting Angle | `mountAngle` | deg | 15 |
| Cage Diameter | `cageDiameter` | mm | 24 |
| Cage Wall | `cageWall` | mm | 1.5 |
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
- `cageDiameter` has **two floors, and both are measured rather than derived.**
  `TestCageDiameterFloorsAreMeasured` walks the wall itself and reports them. Below **7 mm** at the
  default proportions the slots cut the tube into pieces and it stops being a frame at all. Below
  **19.25 mm** the tube is still one piece, but each gear's two piercings reach around and join into
  a single opening, leaving the wall standing on two arms rather than four. Require at least 4 mm
  over the second floor; the 24 mm default clears it by 4.75 mm.

  An earlier draft settled this with an arc — the two gears pierce the wall `Sigma` apart, so the
  wall survives while `(cageDiameter/2)*Sigma` exceeds a slot's width — and that rule is wrong twice
  over. A slot's width around the tube is not the ribbon's thickness, because the ribbon crosses the
  wall at whatever station puts it at the tube's radius and the cross-section angle there decides
  how much of the opening lies across the tube. And it watches the wrong pair of openings: what
  merges first is one gear's own two piercings, not one gear's against the other's.

**No range is enforced on Mounting Angle, and none is asserted here.** `Phi = 15°` is the measured
working value at the default proportions; what happens elsewhere is the proof's to map, and this
spec does not clamp what has not been measured.

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

Create **9 construction planes** perpendicular to the gear's axis at stations `s_k = k*P/8`,
`k = 0…8`, with `setByDistanceOnPath` against the axis line (`[PB-CONSTRUCTION-PLANES]`; pass the
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

**Nine sections, not fewer, and the count is not user-configurable.** The loft's surface between two
sections is ruled, so it cuts the corner of the true helicoid by about `(W/2)*(1 - cos(dtheta/2))`
where `dtheta` is the twist between neighbouring sections. At the defaults that is 1.75° per step
and 0.6 µm of departure, which is three orders below the 0.23 mm backlash. Halving the count
quadruples that error and it is still small, but nine sections also keeps each rectangle's
correspondence unambiguous for the loft, which is the failure this count is really buying off.

### 3: Repeat the cell by doubling

The finished ribbon is the cell repeated `N` times under the **screw step**

```
Step(k) = translate(k*P along the axis) ∘ rotate(k*P/Lambda about the axis)
```

Build it by doubling rather than by `N` separate placements: copy the current body, move the copy by
`Step(m)` where `m` is the number of teeth built so far, join, and repeat; finish with one partial
round for the remainder. That is `ceil(log2(N))` copy-move-join rounds — 5 at the default 24 teeth,
against 23 — and every placement is exact, because the body genuinely is invariant under `Step`.

Copy with `copyPasteBodies`, move with `moveFeatures` / `defineAsFreeMove(matrix)`
(`[PB-MOVE-ROTATE]`, `[SCREW-F-SCREW-STEP]`), and join with a `combineFeatures` join. Adjacent cells
meet at a shared planar cross-section, so the join leaves no sliver.

**A zero-angle matrix is a no-op that Fusion rejects** (`[PB-MOVE-ROTATE]`). A screw step is never
zero for `k >= 1`, so no guard is needed here, but do not "optimize" a `k = 0` case into the loop.

### 4: The cage

Build the tube first: a circle of `cageDiameter` on a plane through `C` normal to `n̂`, extruded
symmetrically to a half-height of

```
A/2 + (W/2)*|cos(cageInner/Lambda + Phi)| + 2*cageWall
```

then a concentric circle inset by `cageWall` cut through it. What the tube has to cover is the four
openings rather than the ribbons at their widest: a ribbon crosses the wall about `cageInner` along
its own axis from the closest approach, and its cross-section angle **there** is what decides how
much of its width falls along `n̂`. Two walls of margin keep an opening off the rim, which
`TestCageSlotsStayOffTheRim` checks.

Cut one slot per gear with a **twisted clearance ribbon** (`[SCREW-F-TWISTED-SLOT]`): loft five
rectangles of `(W + 2*clearance)` by `(T + 2*clearance)` on planes along that gear's axis, spanning
the tube's full width plus a margin at each end, each rotated by `s/Lambda + Phi` exactly as the
tooth cell's sections are, and cut the lofted body from the tube. The slot is then a twisted channel
of the same lead as the ribbon.

**The slot has to twist; a straight slot binds.** Over a wall of thickness `tau` the ribbon turns by
`tau/Lambda`, so its corner sweeps `(W/2)*(tau/Lambda)` across the slot — past the clearance at
`tau = 0.86 mm` on the defaults, which is thinner than any wall worth printing.

**Do not build this cutter with a swept feature.** `SweepFeatureInput` has a `twistAngle` that would
produce the exact helicoid from one section in one feature, and it is the obvious tool here, but a
sweep needs an `adsk.fusion.Path`, and `Path.create` on a sketch curve raises
`InternalValidationError` whenever the owning sketch is not trivially resolvable in the current
multi-component context (`[PB-CONSTRUCTION-PLANES]`). This build is multi-component throughout. The
loft costs four extra sketches and needs no path.

### 5: Relocate the bodies

Move the two gear bodies and the cage body into their sub-components with `body.moveToComponent`,
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
- `TestCageSlotsLeaveOneTubeAndFourHoles`, `TestCageSlotsStayOffTheRim` and
  `TestCageDiameterFloorsAreMeasured` walk the tube's own wall. The frame is only a frame while it
  is one body, and the four openings are what can take that away.
- `TestRibbonsPassThroughTheirSlots` walks both ribbons end to end and asserts that neither ever
  meets the wall. One static pass settles every position the gears take, because a slot is cut to
  the blank and the blank is invariant under the gear's own screw motion.
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
builds. `TestLoftSectionCountHoldsTheHelicoid` bounds the gap between the two at 0.6 µm against a
0.25 mm backlash, but that is arithmetic about the loft rather than a measurement of one.

It also cannot see print tolerance or friction. A window of 0.25 mm is comfortable for fused
filament and tight for resin, and only a printed part settles it.
