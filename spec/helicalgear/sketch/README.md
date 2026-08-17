# Helical twisted Gear Profile — sketch-first constraint proof

The **sketch-first gate** ([PB-SKETCH-FIRST]) for the helical gear. Helical reuses the whole spur
build pipeline but draws a **second, twisted** "Twisted Gear Profile" sketch — the spur tooth
generator run at `angle=helixAngle` — and lofts the bottom (untwisted) profile to this top one. This
bench reproduces that twisted profile in the [lestrrat-3d/sketch](https://github.com/lestrrat-3d/sketch)
engine and proves its constraint scheme **fully constrains** before any Fusion code is generated.

## Run it

```sh
./run.sh
```

Resolves the engine from `$SKETCH_DIR` or a sibling `../sketch` (via `git --git-common-dir`), same as
the spur bench. Expected tail:

```
ALL PASS — the helical twisted Gear Profile fully constrains across sizes and helix angles.
```

## What this proves that the spur bench did not

The tooth geometry and its constraint scheme are spur's (see `spec/spurgear/sketch/README.md` for the
circles / involute flanks / ribs / flank-to-root construction). The **one new thing** is the spine's
rotation lock. The spur bench runs the seed tooth at `angle == 0` with the same signed +X reference
recipe. Helical draws the tooth pre-rotated by `helixAngle` and exercises that shared recipe across
non-zero angles:

- a **+X reference** construction line from the origin, whose **far end is pinned with signed
  horizontal and vertical distances** (`NewHorizontalDistance` and `NewVerticalDistance`). Without
  these signed dimensions the reference line's length or direction can float, and the whole sketch
  is under-constrained even though the geometry looks right — the exact failure `[SPUR-F-SPINE]`
  warns about;
- an **angular dimension** from the reference to the spine (`NewAngle`, in degrees) fixing the twist.

This bench runs the twisted profile across **sizes** (M/N) at the default 14.5° **and** a **helix-angle
sweep** (0°, 10°, 25°, 35°) at N=17, confirming the angle ≠ 0 path reaches `DOF == 0`, no
redundant/conflicting constraints, well-conditioned, for every combination. (angle 0° is included as
the spur-equivalent baseline.)

## The gate

Identical to the spur bench: **primary gate** = `Status == FullyConstrained` and healthy
`Conditioning` (⇒ `DOF == 0`, no redundant/conflicting constraints). `ProfilesValid` is **true** (needs
the engine's #12 corner-join fix, now in sketch `main`); probe ambiguity is expected for a seeded
draw-then-constrain sketch. See `spec/spurgear/sketch/README.md` for the full rationale.

## Scope

Models the **non-embedded** twisted profile, which is all helical supports: `helicalgear.py`'s
`loftTooth` finds both loft sections with a fixed `nurbs=2, arcs=2, lines=2` (6-curve) count and never
reads `ctx.toothProfileIsEmbedded`, so an embedded low-tooth-count helical gear is unsupported by the
implementation. The bench matches that and skips embedded sizes. Herringbone reuses this exact twisted
profile (same tooth at the helix angle); its extra work — mirror + combine — is a solid-body
operation, not a sketch, so it needs no separate proof.
