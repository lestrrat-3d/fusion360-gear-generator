# Screw gear — how the meshing numbers were found

`instructions.md` states a crossing angle, a mounting angle, an engagement and a backlash without
deriving them. This file records the model and the search that produced them, so the proof can
re-derive them rather than inherit them on trust.

## The model

Each gear is described implicitly rather than as a solid. A world point belongs to gear A when,
after mapping into A's frame as `(u, v, s)` and undoing the twist by `-(s/Lambda + Phi)`,

```
-W/2 <= u <= W/2 - H/2 + (H/2)*cos(2*pi*(s - Z)/P)      and      |v| <= T/2
```

The signed slack to the nearest of those four bounds is positive inside the body and stands in for a
distance. Sampling one body's boundary and evaluating the other body's slack at each sample gives
the deepest penetration of the pair, negative when they are clear. That is cheap enough to evaluate
a few hundred thousand times, which is what the search needs.

**The screw motion enters only as `Z`.** The twisted blank is invariant under its own screw motion,
so advancing a gear shifts its tooth phase and changes nothing else in its own frame. A pair is
therefore fully described by two numbers, `Za` and `Zb`, whatever the mounting.

## The criterion

For each tooth phase of A, the phases of B that clear it form a **free window**. Three things have
to hold across a full pitch of A for the pair to be a gear:

- The window is never empty. An empty window is a jam.
- The window is narrower than a pitch. A window that spans the pitch means the teeth never box B in,
  so nothing is driven.
- **The window advances by exactly one pitch as A advances one pitch.** This is the 1:1 ratio, and
  it is the test that separates a gear from two parts that merely touch. A pair can satisfy the
  first two and still have a window that oscillates and returns — B rattles and follows nothing.

The window's width is the backlash. Its departure from a straight line is the transmission error.

## What the search covered

Crossing angle over 38.5°–100°, mounting angle of each gear over 0°–90° in 15° steps, both hands,
and engagement over a fifth to nine-tenths of the tooth height — 2352 arrangements at each of three
tooth pitches, sampling the axial window at 0.02 mm.

Results:

- **The symmetric arrangement jams.** At `Phi = 0`, where both toothed edges point straight at each
  other where the axes cross, no arrangement drove at any crossing angle, at any engagement past
  about a quarter of the tooth height. Two to three tooth pairs sit in the engaged zone at once and
  their ridges cross at an angle, so they cannot all interdigitate. This is the single most
  surprising result, and it is why `Phi` is an input at all.
- **Roughly 3% of arrangements drive 1:1** (60 of 2352 at a 1.75 mm pitch, 71 at 3.5 mm, 62 at 5 mm).
  The survivors cluster at mounting angles of 15°–45°.
- **The transmission error tracks the pitch** at about 2% of it: 0.015 mm at a 1.75 mm pitch,
  0.044 mm at 3.5 mm, 0.104 mm at 5 mm. A finer tooth runs smoother, at the cost of more tooth cells
  to build.
- **`Sigma = 2*Beta` is among the winners at every pitch tested**, which is the crossed-helical rule
  arrived at independently. The spec states the rule rather than quoting the search, but the rule
  does not explain where this pair touches: the crest helices run parallel only at the station whose
  cross-section angle is zero, and `proof/screwgear` measures contact several millimetres away from
  it. The contact is a point, as a crossed-helical pair's is.

The chosen default — `W` 10, `T` 2.5, `P` 3.5, `H` 1.2, lead 90, `Sigma` 38.5°, `Phi` 15° on both
gears, same hand, engagement 0.72 — was then re-run at a 0.01 mm sampling step over 14 phases of A.
The free window is 0.22–0.25 mm wide throughout and its centre advances 0.25 mm for each 0.25 mm of
A, which is 1:1 to within the sampling step.

**`proof/screwgear` re-derives all of this** and is the authority. It reports the same winding, a
0.21–0.26 mm window, a 0.058 mm departure from the 1:1 line, and 0.081 mm of clearance between the
two ribbons away from the teeth. This file records only how the arrangement was found.

## What this does not establish

The model is an exact sinusoid on an exact helicoid. The built part is a loft through nine
rectangles per tooth, which departs from the helicoid by about 0.6 µm at the crest — three orders
below the backlash, but measured against the model rather than against Fusion's own surface.

The search says these teeth drive. It says nothing about whether they are the right teeth. Conjugate
flanks for two screw motions follow from the equation of meshing against the relative screw, which
is constant here because both twists are constant, and nothing in this repository derives them.
