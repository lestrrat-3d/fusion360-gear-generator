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

**Exit codes.** `run.sh` reports the verdict as its exit status: **0** = every case passes
the primary gate (final `ALL PASS` line), **1** = some case fails it (final `FAIL` line),
**2** = the sketch engine checkout was not found. This is the contract every gear's bench
implements ([PB-SKETCH-FIRST]). The canonical way to run the gate is the repo-side wrapper,
which turns that status into the house 0/1/2 verdict convention:

```sh
python3 .claude/skills/generate-gear/run_sketch_bench.py spurgear
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
| tooth-top arc `addByCenterStartEnd` without a diameter, centre coincident to the local origin (`[SPUR-F-TOOTHTOP-ARC]`) | `CreateArc(freeCentre, a, b)` + `NewCoincident(freeCentre, origin)` |
| root-circle split into the tooth's bottom boundary (non-embedded only) | `CreateArc(freeCenter, a, b)` + `NewDiameter` |
| +X reference and angular pin (`[SPUR-F-SPINE]`, every angle) | `NewHorizontalDistance` + `NewVerticalDistance` + `NewAngle` |
| ribs: signed across-spine dimension, midpoint-on-spine, midpoint, ⊥ except on the final rib, signed along-spine chain dims (`[SPUR-F-RIBS]`) | `NewHorizontalDistance` or `NewVerticalDistance` by axis, `NewPointOnLine`, `NewMidpoint`, `NewPerpendicular` except on the final rib |
| flank-to-root lines: root endpoint pinned by signed Δx and Δy from the local origin (`[SPUR-F-FLANK-ROOT]`) | `NewHorizontalDistance(origin, rootEnd, dx)` + `NewVerticalDistance(origin, rootEnd, dy)` |

The involute sampling (`calculateInvolutePoint`, the mirror/rotate) is the spec's
exact math. The check runs across several `Module` / `Tooth Number` sizes and zero,
positive, negative, and axis-swapping tooth angles to prove the **parametric** scheme
holds, not just one instance.

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

## The negative control

The tooth-top arc's centre is the one place this bench earns its keep, so it also proves
it can still *detect* the defect. `addByCenterStartEnd` shares its start and end points
but **copies** the centre, so the arc's centre is a fresh free point and the spec's
explicit `addCoincident` is what pins it (`[SPUR-F-TOOTHTOP-ARC]`, `[PB-SHARE-XOR-COINCIDENT]`).
After the normal sweep the bench re-runs one embedded case with that coincidence dropped
and **requires it to fail**:

```
--- negative control: tooth-top arc centre left free (must FAIL) ---
Solve: converged=true DOF=2 redundant=0 | Verify: status=underconstrained conditioning=+Inf
negative control failed as required: a free arc centre does not fully constrain.
```

If that control ever passes, the bench has stopped seeing a free arc centre and the run
fails on that alone. This is the defect that shipped a crowned bevel pinion in Fusion on
2026-09-02: a stranded centre 22.9 mm behind its origin gave a 0.5743 mm tooth-top arc
where 22.5 mm was intended, with no error anywhere.

## A fragility the gate catches

Near the embedded transition (`base ≈ root`, about `N = 42` at 20° pressure
angle) the flank-to-root stubs shrink toward zero length, which leaves the root
endpoint signed offsets close to the flank start. The healthy sizes this proof
runs are comfortably clear of it.

## Scope

Models both variants of the Gear Profile sketch. The **non-embedded** 6-curve case
(`base > root`, the default spur) draws the two flank-to-root stubs and an explicit
root arc. The **embedded** 4-curve case (`base < root`, above about `N = 42` at 20°
pressure angle) draws neither: its flanks already start inside the root circle, so
the bottom boundary is the solid root circle itself, which is what Fusion splits.

The embedded case is not a curiosity. The bevel gear borrows this drawer through
`VirtualSpurProxy`, and its virtual tooth count is embedded for every ordinary bevel,
so the bevel tooth has only ever been drawn on that branch. `M=1 N=43 angle=180°` is
exactly what a default 31/31 bevel pair draws.

The Tools sketch carries no geometry (just an anchor projection — nothing to constrain).
