# Gallery translation goal

Goal: every gear add-in in `lib/geargen/` is a translation of its decad-gallery model, with no Fusion construction
kept as stored text. `feat-gallery-port` merges into `main` only when this goal is reached.

Owners: `docs/gallery-port-pipeline.md` describes the current pipeline and its history. `docs/fusion/` holds what
Fusion taught us. This file owns the goal, the rules for reaching it, and the definition of done.

## Division of work

| Layer | Lives in | Contains |
|-------|----------|----------|
| Modelling | decad-gallery | Instructions to 3D model: sketches, features, parameters, the geometry proof |
| Translation | `port/` in this repo | Rules that turn gallery operations into Fusion API calls |
| Fusion wiring | stored fragments in `port/<gear>/fragments/` | Dialog, command handlers, parameter registration, anything the gallery cannot express |

- The gallery owns geometry. A geometry change goes to the gallery. NEVER fix geometry in a fragment or a rule.
- A gear whose fragment contains Fusion construction (sketch, feature, pattern, fillet, bore, chamfer, loft) is not done.

## Translation rules

- ALWAYS one shared rule set. A rule maps one gallery operation to Fusion calls and serves every gear.
- NEVER write translation code for one gear only. A gear that needs something new gets a new shared rule.
- A new rule MUST leave the generated output of every finished gear byte-identical. A test checks it.
- Fusion knowledge goes into a rule with a test, or into `docs/fusion/`. NEVER into `lib/geargen/<gear>.py`, which is
  build output.
- The translator reads the gallery's Go source in the order it creates entities and features. That order is the
  contract. A gallery change to the order stops the port until the rules are reviewed.
- Anything the translator does not recognise is an error. NEVER skip it silently.
- Fusion-specific adjustments live in the Fusion runtime layer of the fragment, not in the gallery. Examples already
  recorded in `docs/fusion/spurgear.md`: `[SPUR-F-ARC-CENTRE]`, `[SPUR-F-ALIGN-DUPLICATE]`, `[SPUR-F-ROOT-ARC]`.

## Structure

- ALWAYS translate 1:1. One gallery sketch → one Fusion sketch. One gallery operation → one Fusion feature.
- NEVER keep the old add-in's structure where it differs from the gallery's (shared profile sketch, profile finder by
  curve counts, to-entity cut). The gallery's order and grouping win.
- Fusion-only helpers that the gallery does not model (reference circles, text labels) go in their own sketch. They
  NEVER share a sketch with a gallery sketch.
- Every translated sketch follows the Fusion rules in `docs/fusion/spurgear.md` (local origin, anchor projection).
- Decided with the user on 2026-10-24. The spur is re-verified in Fusion after the change.

## Retry loops

- A loop that retries a build with other numbers because decad refused the first try runs ONCE, with the first entry of
  each list. The spiral bevel build has two: `spiralOvershoots` (five overshoot values, in modules) and the join orders
  tried for each. The translation uses overshoot 0.5 and the first join order.
- A failure of that first try is an error, reported as it is. NEVER add a retry to the translation until Fusion refuses
  a member the gallery builds. Record the case here when it happens.
- Why: decad's booleans work on float meshes and can refuse a result whose rounding folds two facets through each other
  or leaves two facets too close to tell. The refusal depends on the numbers, not on the gear, so the gallery builds
  again with the next numbers. Every try that decad accepts gives the same gear, because the overshoot is trimmed flush
  afterwards. Fusion's kernel is exact and has no such refusal. A failed Fusion feature stays in the timeline as an
  error, so retrying there would need a rollback that does not exist.
- The retry loops are listed in `port/bevel/mapping.go` (`FirstEntryOnly`), so the rule is visible in the code that
  applies it. Decided with the user on 2026-10-24.

## Order of work

1. Spur: the embedded tooth, then the feature sequence (root disc, tooth extrude, pattern, join, fillets, bore,
   chamfers).
2. Helical and herringbone.
3. Bevel.
4. Cycloidal.

- Finish one gear before starting the next.
- Starting a gear: list the gallery operations it uses, then list which are already covered by a rule.
- The screw gear has no gallery model and is outside this goal. It stays on `feat-screwgear` branches.

## Waiting on other sessions

- Bevel: the gear is two members in one pair. The Fusion add-in keeps them in components and describes both blank
  profiles in one shared sketch. Requested on 2026-10-24: decad gets a generic grouping container (`decad-7e`: nestable,
  grouping only, no instancing, no transform), then the gallery's `BuildPair` draws one shared sketch for both blanks and
  puts each member in a container (`decad-gallery-93`). The translation maps the containers to Fusion components and
  keeps the shared sketch. Until both land, the straight bevel path is translated against the current model and reworked.

## Definition of done for a gear

- The fragment holds no Fusion construction.
- `gearport check <gear>` passes and `go test -C port ./...` passes.
- `python3 -m unittest discover -s tests -p 'test_*.py'` passes.
- Every finished gear's generated output is unchanged by the rules added for this one, or the change is explained.
- The user loaded the generated add-in in Fusion and reported it works. NEVER mark a gear done on tests alone.

## Definition of done for the goal

- Every gear in the order above is done.
- A change to a gallery model's operations or order stops `gearport gen` for that gear.
- `docs/gallery-port-pipeline.md` describes the final state.
- The user decides when `feat-gallery-port` merges into `main`. NEVER merge it earlier.

## Working rules

- Work in a worktree off `feat-gallery-port`, named `.worktrees/<branch>`.
- Open PRs against `feat-gallery-port`. NEVER push, open a PR or merge unless the user says so in that turn.
- A Fusion verdict follows `CLAUDE.md`, "When Fusion gives a verdict": write the finding down, then regenerate.
- A gallery or decad change needed goes to the owning session. NEVER edit those repos' main branches directly.
