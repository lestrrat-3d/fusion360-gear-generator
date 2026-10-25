# Spiral bevel tooth trace

This file derives the geometry of the spiral bevel tooth: the cutter-circle trace, the twist it gives the tooth
about the shaft, and the lengthwise crown. The decad-gallery model `models/bevelgear` implements it in
`spiral_trace.go` (`buildSpiralTrace`, `cutterIntersectionNear`), `spiral_slabs.go` (`sliceSpiralPointTooth`),
`spiral_twist.go`, `spiral_crown.go` and `spiral_sections.go`. The port translates those functions into
`lib/geargen/bevelgear.py` (`_build_spiral_trace`, `_twist_spiral_slabs`, `_crown_spiral_slabs` and so on). The text
below was checked against the gallery revision pinned in `port/go.mod` (`v0.0.0-20261025104755-80bc6b8ad519`). If the
two disagree, the code is what builds the gear; fix this file.

The reasons recorded here come from the previous, hand-written add-in, which built the same construction with Fusion
features. Where a reason rests on something Fusion did, the text says so.

## 0. What the trace is

A face-mill cutter of radius `r_c` cuts a spiral bevel tooth. In the plane of the generating crown gear the cutter
sweeps a circular arc of radius `r_c`, and that arc is the tooth trace: the curve the tooth follows from toe to heel.
The construction needs only two things: where the circle's centre sits and which part of it is kept.

The build never lays the arc onto the cone. It reads the two ends of the arc and turns them into a rotation of the
straight tooth about the shaft (section 8).

## 1. The drawing plane

The construction is done in a flat plane tangent to the **root** cone along one root element, with the pitch apex as
the origin:

- The x-axis is the root element, pointing from the apex toward the heel. A point's x-coordinate is its cone distance.
- The y-axis is perpendicular to it in that plane, in the circumferential direction.

In the gallery the element is `coneVec`, the unit vector from the apex to the blank's heel root corner (point 3 of
`blankSection`, the dedendum point). Only the two coordinates are used; no plane or sketch is made.

The textbook reference plane is the plane of the generating crown gear, which is tangent to the **pitch** cone. This
build uses the root cone instead. The two planes differ by the dedendum angle `δ_f = atan(1.25 · Module / R)`, which
is 3.26° on the default 31/31 pair at Shaft Angle 90°. ψ is therefore measured against the root element.

The twist in section 8 is not taken on the root cone. It divides by `sin γ`, the pitch cone angle, because the crown
gear the generation law rolls against is tangent to the pitch cone. Dividing by `sin(γ − δ_f)` instead would twist the
default pair about 6% further (`sin 45° / sin 41.74° = 1.062`). The previous add-in once took the root angle from
`acos(coneVec · axisDir)`. On a 17-tooth pinion meshing a 31-tooth gear (pitch angle 28.7°, root angle 24.7°) that
inflated the twist about 1.15 times, and ratio pairs stopped meshing. So the pitch angle stays.

Whether the frame itself should move to the pitch cone is not settled. Nothing measures it: the gallery test
`TestSourceDefaultSpiralToothHeelFaceTwist` checks the twist against the same formula the code computes. Moving the
frame would change `R_toe`, `R_heel` and the element ψ is measured against, so it is a separate derivation.

## 2. The toe, heel and mean marks

The build reads three cone distances along the x-axis:

- `R_toe` is the projection onto the root element of the midpoint of the blank's toe edge. That edge runs from the
  toe root corner (section point 4) to the outer corner of the toe face (point 5).
- `R_heel` is the projection of the midpoint of the heel edge, from the heel root corner (point 3) to the outer corner
  of the back face (point 2).
- `R_mean = ½(R_toe + R_heel)`, and `span = R_heel − R_toe`.

`M = (R_mean, 0)` is the reference point at mid-face. If the heel midpoint comes out nearer the apex than the toe
midpoint, the code swaps them; a span that is still not positive is an error. Because the marks are edge midpoints and
not points on the element, `R_toe` and `R_heel` sit slightly off the root element's own toe and heel ends.

## 3. The cone-distance circles

Every point at cone distance `R` lies on the circle of radius `R` centred at the apex. The trace starts on the toe
circle and ends on the heel circle. These circles are centred on the apex and nowhere else.

## 4. Spiral angle and hand

The mean spiral angle ψ is the angle between the trace and the cone element at `M`. AGMA and ISO usage measure it
against the element; sources that measure it against the cone axis or against the perpendicular to the element are 90°
off. The trace makes exactly ψ only at `M`.

The hand sets the sign `s`: right gives `s = +1` and left gives `s = −1`. A meshing pair has opposite hands, so the
code negates `s` for the pinion. `buildSpiralTrace` refuses ψ outside `(0, 60°)` and a hand other than right or left.

## 5. The cutter-circle centre

The arc has radius `r_c`, passes through `M` and is tangent there to a line at angle ψ to the x-axis. A circle's radius
is perpendicular to its tangent, so the centre `C` lies on the normal to that line at `M`, at distance `r_c`. The
direction from `M` to `C` is `(−sin ψ, s · cos ψ)`:

```
C = (R_mean − r_c · sin ψ,  s · r_c · cos ψ)
```

`r_c` is the Cutter Radius input, or `R_mean` when the input is 0.

The hand sign belongs on the y term. Opposite hands are mirror images across the element (`y = 0`), which flips `Cy`.
Putting `s` on the x term mirrors across `x = R_mean` instead, which is a different curve and gives the two gears
unequal twist. At ψ = 0 the centre is `(R_mean, r_c)`, straight off `M`, and the arc is tangent to the element at `M`.

## 6. Toe and heel ends

The trace ends where the cutter circle meets two cone-distance circles. Each end is taken 6% of the span past the
face, so the kept arc reaches past the end trims:

- the toe end `T` on the circle of radius `R_toe − 0.06 · span`;
- the heel end `H` on the circle of radius `R_heel + 0.06 · span`.

Each pair of circles meets in two points. `cutterIntersectionNear` keeps the one nearer to `M`, which is the branch `M`
lies on. With `d = |C|` the distance from the apex to the centre and `R` the cone-distance radius, the intersection
lies `a = (R² − r_c² + d²) / (2d)` along the line from the apex to `C` and `h = √(R² − a²)` to either side of it. If
the circles do not meet, the build stops with "cutter circle misses cone-distance circle".

## 7. The trace arc

The trace is the arc of the cutter circle from `T` through `M` to `H`. The build keeps only `T` and `H`; it draws no
arc. (The previous add-in drew it in a `{gear} 2D Tooth Trace` sketch for inspection, and no feature used that sketch.)

The spiral angle drifts along the arc: it is larger at the toe and smaller at the heel, because one fixed circle meets
ever larger apex circles at changing angles. That drift is what a circular-cut spiral bevel has.

## 8. From the trace to the tooth: the twist

The angle the two ends subtend at the apex is

```
phi_crown = atan2(H_y, H_x) − atan2(T_y, T_x)
total     = |phi_crown| / sin γ
```

with γ this gear's **pitch** cone angle (`dimensions.coneAngle`). On the developed cone an angle `φ` in the plane
corresponds to a rotation `φ / sin γ` about the shaft, because a full turn about the shaft develops to an arc of only
`2π · sin γ`. The two members of a pair therefore get different twists for the same ψ and cutter: on a 17/31 pair the
factor `1 / sin γ` is 2.08 for the pinion and 1.14 for the gear. A trace that looks right can still give a pair that
does not mesh if this factor is wrong.

The previous add-in once projected the flat arc onto the root cone and measured the azimuth there. For unequal pairs
Fusion's `projectToSurface` returned the arc as several disjoint pieces, the measured sweep collapsed to a fraction of
the true one, and the pinion came out under-twisted. The analytic law above replaced it.

The straight point tooth is cut into slabs before the twist. `sliceSpiralPointTooth` splits it with eight planes
parallel to the back-cone tooth plane, the plane the tooth profile is drawn in. Plane `k` (1 to 8) lies `k · span / 6`
from the tooth plane toward the apex; if no plane cuts the tooth, the sign is flipped once. The piece nearest the apex
is removed, which leaves eight slabs. The planes are parallel to the tooth plane, not perpendicular to the cone element:
the tooth is lofted from the apex to that plane, so the heel face of the heel-most slab is the tooth plane itself. The
two directions differ by the dedendum angle. The previous add-in recorded that the tilted family builds without any
error and moves a face corner by `1.125 · Module · tan δ_f` (0.26 mm on the default pair at Module 4).

Each slab turns about the shaft (the x-axis of the build frame, through the apex) by

```
angle = −s · total · (R_mean − d) / span
```

where `d` is the cone distance of the centroid of the slab's heel face. The heel face is the face whose centroid lies
farthest along the root element, chosen from all of the slab's faces. A slab whose heel face sits at `R_mean` does not
turn, so the mid-face section meshes like the straight tooth. The angle is keyed to the heel face because the loft in
section 10 passes through the heel faces; keying it to the slab's centroid would leave the loft's mid-face section
turned by half a slab. The previous add-in's notes also say that filtering the faces by surface type can pick the
wrong face, and the loft then fails with `ASM_NOT_ALL_SECTIONS_MEET`.

## 9. The lengthwise crown

`crownSpiralSlabs` scales every twisted slab except the heel-most one by

```
u      = (R_heel − d) / span
factor = 1 − 0.5 · (total / 2) · u
```

where `d` is again the cone distance of the slab's heel-face centroid, read after the twist. `u` is about 0 at the
heel and grows toward the toe. It exceeds 1 on the toe-most slab, whose heel face lies past the toe. The relief
therefore grows steadily from the full heel to the toe. A factor at or below zero is an error. The scale is about
the midpoint of the heel face's two vertices nearest the shaft axis, which are its root corners.

These choices each come from a defect seen in the previous add-in:

- Keying the relief on the twist angle's magnitude makes it largest at both ends. With the heel slab held full, the
  slab next to it became the most relieved and dipped below both neighbours. The recorded factors were 0.932 next to
  the heel against 0.972 for the next slab inward.
- Scaling about the heel-face centroid pulls the root edge up by `(1 − factor)` times half the tooth height. The tooth
  then floats above the root cone and the join leaves a gap. It was first seen on a Module 2, 19/13 pair.
- The heel-most slab stays full size so that the heel trim leaves the tooth flush with the gear's back.

The factor 0.5 (`spiralCrownPerRad`) is not derived. It was entered by hand on 2026-06-07, in commit `c3829c0` of the
hand-coded `SpiralBevelGearGenerator`, replacing 0 (crown off). The comment beside it said to raise the value until a
high-ratio pair runs clean. No measurement, source or Fusion load stands behind it. Treat it as an unverified tuning
value.

## 10. The loft

`spiralLoftFaces` sorts the crowned slabs again by their heel-face cone distance, after the twist and the crown. The
loft then passes through the toe face of the toe-most slab, followed by the heel face of every slab in that order: nine
sections for eight slabs. The sort is repeated because a large twist can swap neighbouring slabs along the element. The
previous add-in lofted in the order from before the twist, and a 31/17 pair came out distorted while 31/31 looked
right.

## 11. Checks a correct trace passes

1. The cone-distance circles are centred on the apex, and `R` is measured along the root element.
2. The arc is one circle of radius `r_c`.
3. `C` is `r_c` from `M`, so the arc passes through `M`.
4. The arc meets the element at angle ψ at `M`.
5. Swapping the hand mirrors the construction across the element and changes nothing else. The gallery test checks
   that the driving and pinion slab centres of the default pair are mirror images.
6. `T` and `H` lie on their circles, on the branch through `M`.
7. At ψ = 0 the arc is tangent to the element at `M` but still curves away toward toe and heel. The build never meets
   that case: `prepareBuild` builds a straight member when ψ = 0, `buildSpiralTrace` refuses ψ ≤ 0, and the add-in's
   Straight tooth type passes ψ = 0. The trace straightens onto the element only when `r_c → ∞` with ψ = 0.
