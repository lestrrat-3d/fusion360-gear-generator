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
  about a quarter of the tooth height. About four tooth pairs sit in the engaged zone at once and
  their ridges cross at an angle, so they cannot all interdigitate. This is the single most
  surprising result, and it is why `Phi` is an input at all.
- **Roughly 3% of arrangements drive 1:1** (60 of 2352 at a 1.75 mm pitch, 71 at 3.5 mm, 62 at 5 mm).
  The survivors cluster at mounting angles of 15°–45°.
- **The transmission error tracks the pitch** at about 2% of it: 0.015 mm at a 1.75 mm pitch,
  0.044 mm at 3.5 mm, 0.104 mm at 5 mm. A finer tooth runs smoother, and it costs almost nothing to
  build, because the ribbon is assembled by doubling and doubling is logarithmic in the tooth count.
- **`Sigma = 2*Beta` is among the winners at every pitch tested**, which is the crossed-helical rule
  arrived at independently. The spec states the rule rather than quoting the search, but the rule
  does not explain where this pair touches: the crest helices run parallel only at the station whose
  cross-section angle is zero, and `proof/screwgear` measures contact several millimetres away from
  it. The contact is a point, as a crossed-helical pair's is.

The arrangement the spec carried until 2026-09-28 — `W` 10, `T` 2.5, `P` 1.75, `H` 1.2, `Sigma`
80°, `Phi` 15° on both gears, same hand, engagement 0.36 — was re-run at a 0.01 mm sampling step,
which matters: several arrangements that drove at 0.015 mm jam at 0.01 mm, so every number below
is from the finer step. The arrangement the spec carries now came from the second search, "The
search at the 1.5× size" below.

**The twist lead was eased from 30 mm to 33 mm after the search**, and the pair still drives 1:1 at
the finer step. At the time the frame tied the cage radius to half the lead, so that its bores
would stand near upright, and the cage grew from 15 mm to 16.5 mm with it. The frame in
`instructions.md` has since been replaced by the video's, which has no bore through a wall, and
the cage radius is now read from the video rather than from the lead.

**The twist lead came from the video, not from the search.** An earlier draft turned once every 90 mm, which drives well but makes a part that barely looks
twisted and a pair whose axes cross at 38°. Segerman's model turns about once every four
centimetres and crosses near a right angle. `Sigma = 2*Beta` ties those two together, so reading the
lead off the video sets the crossing angle as well, and the search was re-run at the faster twist to
find the mounting angles that go with it.

A later reading of the 0:09 overhead frame, with the ribbon's own width as the unit, puts the
lead at 2.8–3.5 widths and the crossing angle at 85–100°; `instructions.md` "What the video
shows" records how that was measured. The 33 mm lead sat inside that range at a 10 mm width, and
the 49.5 mm lead sits at the same 3.3 widths at 15 mm. The 80° crossing angle sits below it, and
stays: with `CrossAngle` at 90° `TestPairDrivesOneToOne` reports a 0.195 mm departure from the
1:1 line, 7.4% of the pitch, against its bound of 6%.

## The search at the 1.5× size

On 2026-09-28 the user printed the pair at the defaults above and found the teeth far too small
(`instructions.md`, "What the print showed"). Every length was scaled 1.5× — `W` 15, `T` 3.75,
`P` 2.625, lead 49.5 — and the tooth deepened from 0.69 of the pitch to one pitch, `H` 2.625.
A uniform scale leaves the model's ratios alone, but the tooth depth is a new ratio, so the
search was run again at the new size, with the same model and the same criterion:

- **Coarse stage.** Crossing angle over 60°–100° in 5° steps, mounting angle of each gear over
  0°–90° in 15° steps, engagement over a fifth to nine-tenths of the tooth height in tenths,
  right hand on both — 3528 arrangements — at a 0.04 mm station step, three points across an
  edge and four across a face, six phases of A per pitch and 50 steps of B's phase per pitch.
  149 drove 1:1. `Phi = 0` on both jammed at every angle and engagement, as before; the drivers
  again cluster at one angle 15°–45° with the other 0°–30°.
- **Fine stage.** Every arrangement at 70°–90° with equal angles of 10°, 15°, 20° and 25°, and
  with the unequal pairs (0°, 30°), (5°, 25°), (10°, 20°) either way round, at engagements from
  0.6 to 1.1 mm, re-run at the proof's own sampling (0.01 mm, five and seven points, twelve
  phases, 200 steps): 252 arrangements, 190 driving.

What the fine stage found, at 80° unless stated:

| Mounting angles | Engagement | Window | Departure |
|---|---|---|---|
| 10° / 10° | any | jams | — |
| 15° / 15° | 0.60 mm | 0.92–1.06 mm | 0.061 mm, 2.3% |
| 15° / 15° | 0.70 mm | 0.81–0.93 mm | 0.057 mm, 2.2% |
| **15° / 15°** | **0.75 mm** | **0.75–0.89 mm** | **0.063 mm, 2.4%** |
| 15° / 15° | 0.80 mm | 0.70–0.81 mm | 0.057 mm, 2.2% |
| 15° / 15° | 1.00 mm | 0.47–0.55 mm | 0.063 mm, 2.4% |
| 15° / 15° at 75° | 0.75 mm | 0.72–0.76 mm | 0.030 mm, 1.1% |
| 15° / 15° at 90° | 0.75 mm | 0.67–0.88 mm | 0.195 mm, 7.4% |
| 20° / 20°, 25° / 25° | any | B never boxed in | — |
| 0° / 30° | 0.75 mm | 0.91–1.13 mm | 0.020 mm, 0.8% |
| 30° / 0° | 0.75 mm | 0.95–1.02 mm | 0.057 mm, 2.2% |
| 5° / 25° | 0.60 mm | 0.98–1.22 mm | 0.022 mm, 0.8% |

The engagement moves the window almost linearly, about 0.5 mm of window per millimetre of
engagement, and hardly moves the departure. **The spec keeps 80° and 15° on both gears and
takes 0.75 mm of engagement**, 0.29 of the tooth height, the same fraction the earlier proof
run at `ToothHeight` 1.75 and `Engagement` 0.5 drove well at. The 0°/30° arrangement drives
with a wider window and a third of the departure; equal angles are kept because the two gears
are the same part held alike and the frame's two rods per gear then turn by the same angle
(`instructions.md`, "Both Mounting Angles are 15°"). At the chosen values the lead was also
moved: 45 mm gives 0.72–0.76 mm and 0.037 mm, 60 mm gives 0.66–0.76 mm and 0.116 mm, 66 mm
gives 0.55–0.67 mm and 0.074 mm, so 49.5 mm, the video's 3.3 widths, stays.

**`proof/screwgear/pair_test.go` re-derives all of this** and is the authority. At the defaults,
and at the sampling `instructions.md` "Defaults" states beside the numbers, it reports the same
winding, a 0.761–0.892 mm window, a 0.063 mm departure from the 1:1 line, and 0.484 mm of slack
between the two ribbons at the assembly phases. Its bounds are fractions of the pitch, because
the transmission error tracks the pitch (above) and a uniform scale of the model scales the
window with it. This file records only how the arrangement was found.

## What this does not establish

The model is an exact sinusoid on an exact helicoid. The built part is one loft through ten
rectangles per tooth, 41 for the four-tooth cell the build repeats. A ruled loft through those
sections would depart from the helicoid by about 1.0 µm at the crest and draw the toothed edge as
a chord of the cosine between sections, 0.064 mm short of it at the deepest point — 7.8% of the
backlash; Fusion's loft through more than two sections is smooth between them, and how far that
surface departs is measured by nothing in this repository. A Fusion measurement on 2026-09-28
put it within 0.04 mm at the 10 mm ribbon (`instructions.md` §2, "What the loft is").

The search says these teeth drive. It says nothing about whether they are the right teeth. Conjugate
flanks for two screw motions follow from the equation of meshing against the relative screw, which
is constant here because both twists are constant, and nothing in this repository derives them.
