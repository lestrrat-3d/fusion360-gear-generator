# Cycloidal drive: Fusion notes

`gearport` generates `lib/geargen/cycloidal.py` from the gallery model `models/cycloidal`. The translated build
draws the rotor discs, cam, housing, output plate and pins, and chamfers them. It runs on `decadfusion.py` in
millimetres (`Runtime(..., MILLIMETRE)`, scale 0.1), and the runtime converts each length to centimetres where it
enters Fusion. The fragment `port/cycloidal/fragments/cycloidal.py.frag` holds the Fusion wiring: the dialog,
parameter registration, the `Tools` sketch, the sub-components and the final hiding. Rules that hold for every gear
are in `playbook.md` (`[PB-…]`). The runtime rules in `spurgear.md` apply here too.

## [CYCLOIDAL-F-ANCHOR-CHAIN] Anchor projection

`CycloidalDriveGenerator._anchor` makes a `Tools` sketch on the Target Plane, projects the Anchor Point into it with
`sketch.project(anchor).item(0)`, and raises unless the sketch is fully constrained. The runtime gets that projected
point, and every translated sketch projects it and constrains its own local origin to it
(`[SPUR-F-LOCAL-ORIGIN]`, `[SPUR-F-ANCHOR-CHAIN]`). A Target Plane that is not a construction plane is first
replaced by a coplanar one, `Drive Base Plane`.

## Sketch cost

The disc and casing sketches draw one lobe, or one pin pitch, of fitted splines and copy it with a sketch circular
pattern ([PB-SPLINE-RING]). The add-in once dimensioned every lobe, and those two sketches then took 359 s and 99 s
([PB-DIMENSION-COST]).

## [CYCLOIDAL-F-CHAMFERS] Chamfers

`Chamfer Size` chamfers the outer loop of both end faces of each disc, the output plate and each output pin. On the
housing it chamfers only the two outer circles of its end faces: the circular edges longer than
π (outer + inner diameter) / 2. The housing has a ledge face at the top of its base whose outer loop is the scalloped
casing contour. In the hand-written add-in, chamfering that loop failed with
`RuntimeError ... ASM_BL_CAP_COMPLEX (面取りを要求されたサイズで作成できませんでした)`.

## [CYCLOIDAL-F-SUBCOMPONENTS] Sub-components

The gallery has no component tree. After the build, `generate` names each body (`Cycloidal Disk 1` and `2`,
`Housing`, `Eccentric Cam`, `Output Plate`, `Output Pin 1` to `M`). `_buildSubComponents` then moves the bodies into
four sub-components by name: `Rotor Discs`, `Housing`, `Eccentric Cam` and `Output`. It runs last because
`moveToComponent` invalidates the reference to the moved body. It reads all bodies first and moves them after,
because each move changes `bRepBodies`. Each occurrence gets the identity transform, so a moved body keeps its
position. Planes, axes and sketches stay in the drive's component, and `solids.hide_construction_geometry` turns
them off at the end.

## Records from the hand-written add-in

The add-in built the drive with its own Fusion code until it was translated. These Fusion refusals came from that
code and still hold for anyone writing Fusion code here.

- **[CYCLOIDAL-F-DISK-AXIS]** A construction axis made with `setByLine` from an `InfiniteLine3D` raised
  `RuntimeError: 3 : Environment is not supported` in a parametric design. `occurrence.activate()` did not help.
  Axes anchored to a body face, `setByPerpendicularAtPoint` and `setByCircularFace`, worked on a component that was
  not active. `decadfusion._axis_through` uses `setByCircularFace` for this reason.
- **[CYCLOIDAL-F-OUTPUT-HOLES]** A circular pattern of a single cut feature failed with
  `RuntimeError: 3 … NO_TARGET_BODY … PATTERN_FEATURES_NO_PASTE_INT_EDGES` ("no pattern instances could intersect
  the original body"). Setting `patternComputeOption = AdjustPatternCompute` made it succeed. The translated drive
  patterns no features.

## Geometry

The gallery owns the geometry. `models/cycloidal/instructions.md` states it, and `profile.go` (`discPoint`, lobe
sampling, the split at inflections), `resolve.go` (derived sizes, `Problems`, the undercut guard) and `contour.go`
(`housingContour`) compute it. `cycloidal.py` holds the same functions, translated. The reasons below are not
written in the gallery.

- **Valley radius.** At `t = 0`, `psi = 0` and the profile point is `(R - Rr_eff - E, 0)`. Both ends of a lobe
  (`t = 0` and `t = 2π/L`) therefore lie on the root circle `Rv = R - Rr_eff - E` about the disc centre.
- **Automatic pin sizes.** The automatic ring-pin radius `(E + R sin(π/N)) / 2` is the middle of its bounds
  `E < Rr < R sin(π/N)`. The lower bound keeps the profile from crossing itself, and the upper bound keeps the `N`
  pins from overlapping on the pin circle. The automatic output pin `Rop sin(π/M) - E` is the pin that fits the
  middle hole size `E + Rop sin(π/M)`. That hole's bounds are `2E`, so the pin clears its orbit, and
  `2 Rop sin(π/M)`, so the `M` holes do not overlap.
- **Undercut.** `E < R/N` is only the limit where the base curve forms a cusp. The profile is that curve offset
  inward by `Rr_eff`, and the offset crosses itself at a lower `E`: when `Rr_eff` reaches the smallest radius of
  curvature among the points whose centre of curvature lies toward the drive axis. For the default drive the guard
  allows `E` up to 2.50 mm, against `R/N` = 2.81 mm (recomputed on 2026-10-25 from the gallery's formula, with the
  automatic pin radius following `E`).
- **Two discs.** Disc 1 is disc 0 turned half a turn about the drive axis:
  `discPoint(t, -E, 0, π) = -discPoint(t, E, 0, 0)`. The turn maps the `N` pins onto themselves only when `N` is
  even, and the `M` output holes onto the output pins only when `M` is even. Two discs therefore need both even.
- **Sampling.** A fitted spline through evenly spaced points overshoots in loops where the lobe turns sharply. At 60
  even points the turn per step is about 6 degrees for the default drive and about 25 degrees near the undercut
  limit. Even spacing would need about 480 points to stay smooth there, and keeping a point at every 5 degrees of
  turn needs about 30 to 55. The translation keeps only the 5-degree sampling. The gallery's coarser retries exist
  for decad (`docs/gallery-translation-goal.md`, "Retry loops").
- The earlier record checked the profile against a 3D-printed drive (HowToMechatronics: `N = 16`, `R = 45`,
  `Rr = 6.5`, `E = 1.5`) and found a closed curve that does not cross itself.

The hand-written add-in built the same profile with other constructions, which the older notes under `docs/`
still describe. It drew the lobe as one open spline and closed the disc with a root circle, two radial lines and a
feature pattern. It found the casing contour by sorting a 240 by 240 sweep into 80 angle bins. It built the housing
from a base ring and `N` joined sectors, and it worked in centimetres. The gallery splits each lobe into three
splines at its inflections and traces the contour along 81 directions (720 cam angles, 2000 profile steps). It
builds the housing as one cylinder minus the contour cavity and the base hole.

## Where the build time goes

Measured in Fusion on the default drive (gallery 19c0f5f), the build takes 14.1 s: 13.3 s for the translated gallery
build, 0.65 s to move the bodies into sub-components, and 0.02 s for the display settle. Inside the build:

- The sketch solves take about 7.3 s: the disc 2.3 s (37 points), the casing 2.5 s (28 points), the output plate 1.0 s
  (7 points), the output pins 0.9 s (6 points) and four one-circle sketches 0.15 s each. Fusion charges about 0.05 s per
  dimension, so the gallery draws one lobe and patterns it ([PB-DIMENSION-COST], [PB-SPLINE-RING]).
- The disc chamfer takes 1.7 s. The other features (extrudes, cuts, the housing, plate and pin chamfers) take about 1.5 s
  together.
- About 1.6 s is Python arithmetic, measured offline: `_housing_contour` evaluates 720 cam angles by 2,000 profile steps.
  The log does not time it, because it runs before the casing sketch is drawn.
