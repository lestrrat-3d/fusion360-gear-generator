# Bevel gear design record

The decad-gallery model `models/bevelgear` (https://github.com/lestrrat-3d/decad-gallery) owns the bevel pair's
contract: the inputs, their ranges, the derived geometry and the geometry proof. Read its `instructions.md` for the
rules. This file keeps two things the gallery does not hold:

- the reasons behind several of those rules, checked against the gallery code at the revision pinned in
  `port/go.mod`;
- what Fusion did with the previous, hand-written bevel add-in, before the port replaced it.

The previous add-in drew the pair as a constrained sketch lattice in Fusion and built it with its own feature code. That
construction is gone; the current build is described in `bevelgear.md`. The spiral tooth's reasons are in
`bevelgear-spiral-tooth-trace.md`.

Notation: `r` is a member's pitch radius, `γ` its pitch cone angle, `R = r / sin γ` the pitch cone distance (the
gallery's `cone_distance`), and `d = 1.25 · Module` the dedendum.

## Reasons behind the input rules

### Maximum shaft angle

The gallery rejects a shaft angle at or above `acos(−min(Teeth, MateTeeth) / max(Teeth, MateTeeth))` and caps it at
150°. At that limit one pitch cone angle reaches 90°: `R · cos γ`, the distance from the apex to the pitch circle along
the shaft, passes through zero, and the back-cone virtual radius `r / cos γ` becomes unbounded. A 31/17 pair is limited
to 123.26°; equal tooth counts give 180°, so only the 150° cap applies. The 150° cap is a practical ceiling, not a
measured one.

### Base height bounds and the minimum tooth count

The heel face lies `R · cos γ + base height` from the apex along the shaft, and its outer radius is
`r − base height / tan γ`. That radius reaches zero, and the revolved profile crosses its own axis, at a base height
of `r · tan γ`.

- The maximum `0.95 · (r − d · cos γ) · tan γ` starts from the dedendum point rather than the pitch point. It sits
  `d · sin γ` below the true crossing and refuses a band of heights that would still build. The exact bound would be
  `0.95 · r · tan γ`. The conservative form was kept because it was what made the low tooth counts build when the
  bounds were written; the exact form has not been tried on them.
- The minimum `1.05 · d · sin γ` keeps the heel face beyond the dedendum point. Below it the heel root edge runs back
  toward the apex instead of outward.
- The default `Module · Teeth / 8` clears the raw minimum only while `Teeth > 10 · sin γ`, which is 7.07 at Shaft
  Angle 90°. In the previous add-in an equal 8-tooth pair was the smallest that built and a 7-tooth pair failed. Raising
  the default to the minimum fixed that, instead of raising the tooth floor.
- The two bounds meet when `2 · d · cos γ = 0.95 · r`, that is at `Teeth = 5.2632 · cos γ`. The gallery
  rounds the constant up to 5.27 so the floor stays at or above the exact crossing. At Shaft Angle 90° the floor is
  3.73, so 4 teeth is the smallest pair.

Worked case: Module 1, 31/31, Shaft Angle 30°, so `γ = 15°`. The maximum is 3.638 mm, the true crossing 4.153 mm, and
the default 3.875 mm. The default is capped, but it would not have folded the profile uncapped. An earlier text claimed
it did; that claim measured the base height from the wrong point.

### Maximum bore diameter

A through bore removes everything inside its radius. Past the heel term it takes the whole flat back face; past the toe
term it bites into the root cone at the toe. The gallery's `maximumBoreDiameter` is `1.9 · min(r_heel, r_toe)`, that is
`2 · 0.95` times the smaller of the back-face outer radius (section point 2) and the toe root corner radius (point 4).
A larger Toe Extension moves the toe root corner toward the apex and tightens the bound.

The flat front face is not protected. Its radius is the toe radius, which lies inside the root cone, so a bore wider
than it leaves through the toe dish and the body stays whole.

Worked case: Module 1, 31/31, Shaft Angle 35°, so `γ = 17.5°`. The default base height 3.875 mm stands (the maximum is
4.286 mm), so `r_heel = 15.5 − 3.875 / tan 17.5° = 3.210 mm`. The automatic bore radius `Module · Teeth / 8` is
3.875 mm, so without the bound the automatic bore alone would delete both back faces. The default pair at 90° is not
affected; the low shaft angles are.

### Face width

Two lengths are easy to confuse. The default face width is `pitch_span / 6`, where `pitch_span` is
`√((2r)² + (2r_mate)²)`, the diagonal of the two pitch diameters. The conventional limit is `R / 3`. The two agree only
at Shaft Angle 90°, where `pitch_span = 2R`. On a 31/31 pair the diagonal is 43.84 mm at every shaft angle, while `R`
is 59.89 mm at 30° and 16.49 mm at 140°.

The cap `0.95 · R · min(sin² γ, sin² γ_mate)` keeps the toe end of both members off their shaft axes. The previous
add-in found that a profile crossing its axis makes Fusion's revolve fail with `ASM_WIRE_X_AXIS` ([PB-REVOLVE]). At
90° the cap is `0.95 · min(PD)² / (2 · pitch_span)`, using the smaller pitch diameter, whichever member has it. On a
Module 1, 17/31 pair it is 3.883 mm, while the uncapped default is 5.893 mm. The uncapped default exceeds the cap
whenever the larger tooth count is more than about 1.36 times the smaller.

### Toe Extension and Toe Radius

Toe Extension moves the toe end toward the apex by lengthening the root edge, from its length at 0% toward the point
on the root element at the member's toe radius. The pair shares one root length, so the smaller of the two members'
windows sets 100%. 100% stops at 0.99 of the window: at the window's end the toe face has zero length. The previous
add-in then had no toe cone face for its toe trim to find, and the build failed. The last percent is worth well under
a tenth of a millimetre of root length.

With a defaulted toe radius a member can have no window at all: on a large pitch cone angle the inner toe corner sits at
a larger radius than the outer one. The previous spec recorded this as a band over gear ratio and shaft angle that
crosses 90° for ratios from about 2.75 up, independent of Module. The gallery rejects a positive Toe Extension there
("toe extension has no positive root-length window"). A smaller toe radius is not substituted, because that would
change the toe of a gear whose inputs asked for no change.

### Tooth Spacing

The tooth profile's centre moves `ToothSpacing` along the back-cone line past the shaft axis, to the station
`R / cos γ + ToothSpacing · sin γ` and the radius `ToothSpacing · cos γ` on the far side of the axis. Both parts are
needed. Taking the station alone and leaving the centre on the axis loses the whole clearance the input asks for: on a
Module 4, 43/31 pair at Shaft Angle 75° with 0.5 mm spacing that is 0.431 mm on the pinion and 0.356 mm on the gear.

### Virtual tooth count and root sink

The back-cone (Tredgold) virtual tooth count `Teeth / cos γ` is a real number and is not rounded. Sources: NPTEL
Machine Design II ch. 13, eq. 13.1 and 13.2; Osakue et al., FME Transactions 49(3), 2021, section 2.2; the KHK gear
technical reference, eq. 11.6. Rounding rebuilds every circle from the rounded count and shrinks the tooth. On the
default pair (`γ = 45°`) the exact virtual pitch radius is 21.920 mm; a floored count of 43 draws 21.5 mm and leaves an
addendum of 0.580 mm instead of 1 module. The spur flank sampler uses the count only in the half-thickness angle
`π / (2 · count)`, which with the exact count gives the standard tooth thickness `π · Module / 2` at the pitch circle.

The root circle sits `0.1125 · Module` (`0.05 · 2.25 · Module`) inside the dedendum. The tooth is drawn on the
back-cone plane, so only its centreline meets the root cone at its own radius; the root arc's corners stand outside the
cone, by 0.002 module on the default pair and 0.027 module on a 4/4 pair. The sink puts the whole root arc inside the
blank, so the join meets the blank across the root rather than along one line.

## Fusion records from the previous add-in

These loads ran the hand-written add-in, not the translated one. The construction they exercised is gone, so they say
nothing about the current file. They are kept as what Fusion has been seen to do.

- 2026-09-02, default 31/31 pair. The tooth-top arc came out at 0.5743 mm on the pinion and 17.0204 mm on the gear
  instead of the 22.5 mm tip radius, because the arc's centre was copied rather than tied to the origin. One coincident
  constraint in the shared spur tooth drawer fixed it, and both then read 22.5 mm.
- 2026-09-16, branch `fix-bevel-exact-virtual-radius`. The default pair and a 16/12 pair at Module 4 built. It was the
  first load of the non-embedded tooth at the sunk root and of the join at that root.
- 2026-09-16, branch `fix-bevel-spec-defects` (the spiral at `7d253d8`). The default straight pair built one solid per
  gear, and its check of the solved sketch against its seeds raised nothing. The default spiral built on slices
  parallel to the tooth plane. A bore above the maximum was refused; the message text was not read back.
- 2026-09-17, `c6ccb3a` (PR #154). The default spiral pair built.
- 2026-09-17, `2ad1e32` (PR #159). Six configurations built: the default pair straight and spiral, a 16/12 pair at
  Module 4, a bore below the maximum, a Toe Extension with a toe radius, and one shaft angle other than 90°. The shaft
  angle, the Toe Extension and the radii used were not recorded.

Nothing was measured on any of these loads beyond the build finishing (and, where stated, the body count). No solved
point, volume, tip radius or face position was read back. Record the configuration and a measurement on the next load,
not only the verdict.

Fusion errors the previous add-in met, each with the construction that caused it:

- `ASM_WIRE_X_AXIS` from the revolve when the profile crossed its axis. One cause was a toe corner seeded on the far
  side of the shaft axis: at Toe Extension 50% on the default pair the seed sat at −0.27 mm from the axis against a
  solved +5.17 mm.
- `VCS_SKETCH_SOLVING_FAILED` when a sketch point was both shared by a new line and tied to it with a coincident
  ([PB-SHARE-XOR-COINCIDENT]).
- `VCS_SKETCH_OVER_CONSTRAINTS` when a second line was drawn over an existing segment with its own constraints, and
  when a collinear constraint named a line farther up a chain instead of the adjacent one.
- `ASM_RBI_INTERNAL` / `LOFT_NO_TOOLBODY` when the tooth loft took the wrong closed region of the tooth sketch.
- `ASM_API_FAILED` from the conical split when the inner toe corner was pinned onto the shaft axis, on unequal tooth
  counts only.
- `face dist = inf` from the toe trim when the pinion's dedendum point solved onto the gear's, which turns the gear
  inside out.
- The previous add-in activated its Design occurrence around the spiral crown's Scale features, and recorded that Fusion
  needs an active edit target for them. The current runtime activates nothing; see `bevelgear.md`.
- On ratio pairs such as Module 1, 31/43 the heel cone often missed the trimmed tooth entirely, and the heel trim was
  skipped.
