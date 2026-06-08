# TODO: §2/§3 toe/heel orientation is non-deterministic across regens

**Status (2026-06-09): RESOLVED — spec-fixed and Fusion-verified.** A clean reference-free regen
from the spec (standard prompt, no hand-holding) built correctly in Fusion (user-confirmed). The
root cause below — §3 never told the *caller* (`_createGearBody`) which world points are toe vs
heel — is now **pinned in the spec**: `bevelgear.md` §3a gained an explicit **"Caller hand-off"** table (toe edge = M→N pinion /
O→P driving, heel edge = C→H / D→J; `toeMid`/`heelMid` = the two edges' MIDpoints; `toeConeWorld` =
M/O, `heelConeWorld` = C/D — never H/J), and the Method-contract hook note repeats it. A fresh
reference-free regen now reproduces the correct mapping (verified by diff against `b69beba`:
pinion passes `(M,N,C,H)`, driving `(O,P,D,J)`; `span > 0` falls out). **Still unproven in Fusion** —
confirm 31/17, 31/31, ψ=0 build correctly before closing this. The §3a-A swap guard remains as
belt-and-suspenders. History below is retained for context.

**Status:** ~~open~~. Branch `refactor-bevel-merge-spiral`. Working reference = commit **`b69beba`**
(`lib/geargen/bevelgear.py` there meshes straight + 31/31 + 31/17). This doc is a cold-start handoff.

## One-line problem

A *fresh* reference-free regen of `bevelgear.py` from `bevelgear.md` sometimes builds the bevel tooth
**inverted along the cone** — the "toe" (inner) and "heel" (outer) ends get swapped. For the **spiral**
build this is catastrophic: it silently flips the entire spiral frame and the gears come out
"completely wrong". Equal-teeth and straight gears can survive it; **unequal-ratio spiral pairs do not.**

## How it shows up (the evidence)

In `_buildSpiralTooth`, the frame is:
```
coneVec = normalize(heelConeWorld - apex)      # should point OUTWARD (apex -> heel)
distAlong(p) = (p - apex) . coneVec            # cone distance from apex
R_toe  = distAlong(toeMid)                      # inner end, should be SMALL
R_heel = distAlong(heelMid)                     # outer end, should be LARGE
span   = R_heel - R_toe                         # face width, should be > 0
```
On a bad regen (round 11), instrumenting the slice produced:
```
[DIAG-SLICE] sign=-1 dotv=-1.76777 span=-0.3977 -> 1 pieces     # NEGATIVE span
[DIAG-SLICE] RETRY sign=+1 -> 9 pieces
```
`span < 0` means `R_toe > R_heel` — the "toe" midpoint is **farther** from the apex than the "heel".
Negative span flips: (a) the cutter-arc frame, (b) the slice cut direction (first cut misses → 1
piece → empty `segments` → crown crashes on `max([])`), (c) the per-segment twist distribution. Result:
inverted, garbage spiral.

The good reference (`b69beba`) builds the SAME case with `span > 0` (correct), so §2/§3 *can* orient
correctly — the spec just doesn't pin it tightly enough, so the LLM regen sometimes inverts it.

## What's already been done (and why it's not enough)

`bevelgear.md` §3a step **A** now has a guard:
```
if apex.distanceTo(heelMid) < apex.distanceTo(toeMid):
    swap toeMid<->heelMid and toeConeWorld<->heelConeWorld
```
This forces `span > 0` and `coneVec` outward **at the spiral frame**. It fixes the slice (9 pieces, no
crash) — BUT the gears were **still completely wrong** after it. So the inversion is **deeper than the
midpoint labels**: the underlying `toothBody` (and/or the dedendum cone faces) is itself built inverted
by §2/§3, and swapping two midpoints in the spiral step cannot un-invert the base tooth that was lofted
from the apex toward the wrong end.

So the real fix is **upstream in §2/§3**, not in the spiral step.

## The actual task

Make §2/§3 label and build **toe = inner (smaller cone distance) / heel = outer (larger cone distance)
deterministically**, so that for any regen:
- the tooth-profile plane (`parentToothPlane`, the virtual-spur `{label} Plane`) sits at the **heel**
  (back-cone / outer) end and the tooth lofts apex→heel correctly,
- `toeMid`/`heelMid` and `toeConeWorld`/`heelConeWorld` are passed to `_transformToothBody` /
  `_buildSpiralTooth` with toe=inner, heel=outer,
- so `span = R_heel - R_toe > 0` falls out naturally and the §3a-A guard never needs to fire.

## Where to look

In `bevelgear.md`:
- **§2** builds the lattice. The toe/heel edges are: pinion **M→N (toe, inner)** vs **C→H (heel,
  outer)**; driving **O→P (toe)** vs **D→J (heel)**. Check how the spec defines which is inner vs outer
  — the inner edge is the one nearer the apex / smaller cone distance. The dedendum corners
  (toeConeWorld/heelConeWorld) and the tooth plane derive from these.
- **§3 (Create the Pinion/Driving Gear)** computes `toeMid`/`heelMid`/`toeConeWorld`/`heelConeWorld` and
  passes them into the tooth-body hook. Confirm the spec pins WHICH world points are toe vs heel
  unambiguously (e.g. "toe = midpoint of the M→N edge, which is the inner/smaller-cone-distance edge").
- The handoff into `_transformToothBody(... toeMid, heelMid, toeConeWorld, heelConeWorld ...)`.

Read commit `b69beba`'s `lib/geargen/bevelgear.py` to see the CORRECT orientation (round-4 got it
right). NOTE: the `generate-gear` skill is reference-free — the *regen subagent* must not read that
.py, but the *human authoring the spec* can, to extract the rule to pin.

## How to reproduce + diagnose

1. Regenerate per `.claude/skills/generate-gear/SKILL.md` (analytic spiral spec is in place).
2. `cp` to repo root, reload the add-in (Shift+S → Stop → Run), run **31/17**.
3. Add a one-line DIAG at the top of the spiral frame to see the sign:
   `futil.log('[DIAG] {} span={:.4f} R_toe={:.3f} R_heel={:.3f}'.format(gearLabel, span, R_toe, R_heel), force_console=True)`
   If `span < 0` on a fresh regen, §2/§3 inverted the toe/heel — that's this bug.

## Possible approaches

- **Cleanest:** pin §2/§3 so the orientation is unambiguous (define toe/heel by cone distance, not by
  edge name), so the regen can't flip it. Then the §3a-A guard is just belt-and-suspenders.
- **Belt-and-suspenders already in place:** the §3a-A swap guard fixes the spiral *frame*; if §2/§3 are
  also made to build the toothBody apex→heel correctly, the whole thing is robust.
- Verify the fix on **31/17 (ratio), 31/31 (equal), and ψ=0 (straight)** — all three.

## Related commits / artifacts

- `b69beba` — reference point: round-4 regen + analytic twist (the proven working impl).
- `62ea099` — the round-4 regen (pre-analytic-twist).
- `f670be6` — original hand-merged spiral (also correct orientation; behavioral reference).
- `lib/geargen/bevelgear.md` §3a — analytic crown-gear twist law (the unrelated, already-solved part).
- Memory: `project_spiralbevel_dev.md` (full history).
