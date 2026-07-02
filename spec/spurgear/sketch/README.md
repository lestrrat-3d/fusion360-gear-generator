# Spur Gear Profile — sketch-first constraint proof

This is the **sketch-first gate** ([PB-SKETCH-FIRST]) for the spur gear: a
runnable reproduction of the "Gear Profile" sketch in the
[lestrrat-3d/sketch](https://github.com/lestrrat-3d/sketch) constraint engine that
**proves the constraint scheme fully constrains the geometry before any Fusion
add-in code is generated.**

The idea: constraint sketching is easy to get subtly wrong (an under- or
over-constrained profile, a branch that flips). Rather than discover that inside
Fusion after committing to a build, we reproduce the sketch here first and check
it programmatically. Only once the scheme is proven sound do we generate the
Fusion Python.

## Run it

```sh
./run.sh
```

`run.sh` resolves the sketch engine from a local checkout (it is source-available,
not go-gettable from the public proxy):

1. `$SKETCH_DIR` if set;
2. otherwise a sibling checkout of the main gear repo — `<repo>/../sketch` —
   located via `git --git-common-dir` so it also works from a worktree.

The replace is injected through a throwaway `GOWORK`, so the committed `go.mod`
stays portable. Expected tail:

```
ALL PASS — the spur Gear Profile constraint scheme fully constrains across sizes.
Cleared to generate Fusion add-in code.
```

## What it models

`main.go` rebuilds the spur **Gear Profile** sketch exactly as
`spec/spurgear/instructions.md` + `spec/spurgear/fusion.md` (`[SPUR-F-*]`)
prescribe, mapping each Fusion constraint to its sketch-engine equivalent:

| Fusion (`spec/spurgear`) | sketch engine |
|---|---|
| local origin `SketchPoint` at (0,0), anchored by coincidence (step 5) | `CreatePoint(0,0)` + `MoveTo` + `Fix` |
| 4 circles sharing the origin center + driving diameter dims (step 3) | `CreateCircle(origin, r)` + `NewDiameter` |
| involute flanks as fitted splines (step 4) | `CreateSpline(pts...)` |
| tooth-top / root arcs `addByThreePoints` + diameter (`[SPUR-F-TOOTHTOP-ARC]`) | `CreateArc(freeCenter, a, b)` + `NewDiameter` |
| spine horizontal (`[SPUR-F-SPINE]`, angle 0) | `CreateLine` + `NewHorizontal` |
| ribs: length, midpoint-on-spine, midpoint, ⊥, chain dims (`[SPUR-F-RIBS]`) | `NewDistance`, `NewPointOnLine`, `NewMidpoint`, `NewPerpendicular` |
| flank-to-root lines: root-end-on-circle + origin-on-line (`[SPUR-F-FLANK-ROOT]`) | `NewPointOnCircle` + `NewPointOnLine` |

The involute sampling (`calculateInvolutePoint`, the mirror/rotate) is the spec's
exact math. The check runs across several `Module` / `Tooth Number` sizes to prove
the **parametric** scheme holds, not just one instance.

## The gate

**Primary gate — full constraint (this is what must pass):**
`report.Status == FullyConstrained` **and** `Conditioning >= gate`. This is the
faithful analog of Fusion's `sketch.isFullyConstrained` plus "not
over-constrained": a solvable, well-conditioned, `DOF == 0` sketch with no
redundant or conflicting constraints. `Status == FullyConstrained` already implies
all of solvable + DOF 0 + no redundant + no conflict.

**Advisory signals — reported, interpreted, not part of the gate:**

- `ProfilesValid` — **true**: the tooth forms one clean, extrudable 6-curve loop
  (2 splines + 2 arcs + 2 lines), the exact curve count the spec's extrude step
  expects. This required a **fix in the sketch engine**: a line meeting an arc at
  a shared loop corner (here the flank-to-root line meeting the root arc) was
  false-flagged as a *degenerate arrangement* by the profile consistency gate,
  which counted the endpoint corner-join against the sampled interior-crossing
  count. Fixed in lestrrat-3d/sketch `main` (PR #12 — adds `cornerJoin` handling
  in `geom/arrange.go` + `TestRegionsLineArcCornerJoinNotDegenerate`). Against an
  older engine *without* that fix this reads false — a tool bug, not a defect in
  the gear scheme.
- `probeAmbiguous` — **true and expected**: a draw-then-constrain CAD tooth is
  seeded at its target pose (`MoveTo`) and then constrained; the pure-constraint
  system still admits mirror/branch flips that the seed resolves, exactly as
  Fusion relies on initial geometry placement. `DOF == 0` means each discrete
  solution is itself rigid, so this is not an under-constraint.

## A fragility the gate catches

Near the embedded transition (`base ≈ root`, about `N = 42` at 20° pressure
angle) the flank-to-root stubs shrink toward zero length and the "origin-on-line"
constraint through two near-coincident points turns ill-conditioned: e.g.
`M=1.5, N=40` gives `Conditioning ≈ 1.6e-8` and the solve fails to converge. That
is a real fragility surfaced here rather than in Fusion. The healthy sizes this
proof runs are comfortably clear of it.

## Scope

Models the non-embedded Gear Profile sketch (the `base > root` case, the default
spur). The Tools sketch carries no geometry (just an anchor projection — nothing
to constrain). The embedded 4-curve variant (`base < root`) is not modelled here;
its bottom-arc constraint recipe is not separately pinned in the spec.
