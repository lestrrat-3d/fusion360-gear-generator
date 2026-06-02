---
name: generate-gear
description: Generate (or regenerate) a gear generator implementation `lib/geargen/<gear>.py` from its natural-language spec `lib/geargen/<gear>.md`, the way `bevelgear.md` produces `bevelgear.py`. Use when asked to (re)generate gear code from a spec, verify a gear spec is complete enough to reproduce its implementation, or check a gear `.py` against its `.md`. Produces logically-equivalent code (not byte-identical) and verifies it by code-review against the existing implementation. Args: optional `<gear>` name (default `spurgear`).
---

# Generate gear code from a spec

This repo generates gear implementations from natural-language design docs:
`lib/geargen/<gear>.md` (the geometry spec) → `lib/geargen/<gear>.py` (the implementation),
exactly as `bevelgear.md` → `bevelgear.py`. This skill runs that generation as a repeatable,
agent-driven workflow and proves the output is **logically equivalent** to the reference.

Two inputs drive every generation:
- the per-gear **spec** `lib/geargen/<gear>.md` — the *what* (geometry, parameters, steps);
- the shared **playbook** `PLAYBOOK.md` (next to this file) — the *how* (framework scaffolding,
  class architecture, Fusion-API conventions, the subclass contract). Read it in full first.

The spec is intentionally ~pure geometry; the playbook supplies everything else, so the spec +
playbook together must be sufficient. If they are not, the fix is to improve the **spec or
playbook** — never to copy from the reference implementation.

## Inputs

- `gear` (default `spurgear`): names the spec `lib/geargen/<gear>.md`, the reference
  `lib/geargen/<gear>.py`, and the scratch output `.tmp/<gear>.generated.py`.
- Optionally a different reference path, if comparing against a specific variant.

## Procedure

1. **Setup.** Work in a worktree (per the repo's CLAUDE.md — never the root checkout). Ensure
   `.tmp/` exists. Read this skill, `PLAYBOOK.md`, and the spec end-to-end. Skim the shared
   framework the output builds on: `lib/geargen/base.py`, `misc.py`, `utilities.py`,
   `lib/fusion360utils/`, and `commands/<gear>/entry.py`.

2. **Identify the contract.** Read `lib/geargen/helicalgear.py` and `herringbonegear.py` and
   list every class, method, and `ctx` field they subclass / override / read by name. This is
   the hard equivalence constraint (the playbook's "Subclass-extension contract"). The generated
   code must preserve all of it.

3. **Generate blind.** Spawn a subagent that writes `.tmp/<gear>.generated.py` from **the spec +
   playbook + the shared framework files only**. It must **not** read the reference
   `lib/geargen/<gear>.py` — that is what makes this an honest test of the spec. Give it the
   contract list from step 2 as a hard requirement.

4. **Verify.** Spawn a reviewer subagent that diffs `.tmp/<gear>.generated.py` against the
   reference using the rubric below. It returns a structured report: an equivalence verdict, a
   list of **contract violations**, a list of **behavioral divergences** (with severity), and a
   list of **spec gaps** (anything the generator had to guess because the spec/playbook didn't
   say). The reviewer is the only step allowed to read both files.

5. **Iterate.** For each contract violation or behavioral divergence, fix the **spec or
   playbook** (not the reference, not the generated file by hand) and regenerate from step 3.
   Repeat up to ~3 rounds. The loop converges when the only differences left are cosmetic.

6. **Validate mechanically.** Syntax-check the generated file:
   `python3 -c "import ast; ast.parse(open('.tmp/<gear>.generated.py').read())"`. (Fusion's
   `adsk` modules can't be imported in this environment — the repo has no runnable tests; a
   parse check is the available mechanical gate.) Then confirm every name from the step-2
   contract list is present in the generated file.

7. **Report.** State the equivalence verdict, the spec/playbook edits made, and any residual
   gaps. Do not commit the scratch generated file or the report (they live in `.tmp/`). Do
   commit spec/playbook improvements, with no push without explicit approval.

## Logical-equivalence rubric

**Must match exactly — the contract surface** (rename/drop = breakage):
- Class names: `<Gear>CommandInputsConfigurator`, `<Gear>GenerationContext`,
  `<Gear>InvoluteToothDesignGenerator`, `<Gear>Generator`.
- Overridable generator methods consumed by subclasses: `newContext`, `prefixBase`,
  `generateName`, `addExtraPrimaryParameters`, `filletHelixFactorExpression`,
  `chamferWantEdges`, `buildSketches`, `buildTooth` (plus the tooth generator's
  `draw`/`drawTooth`/`drawCircles`/`drawBore`/`calculateInvolutePoint`).
- Tooth generator constructor `(sketch, parent)` and `.draw(anchorPoint, angle=…)` signature.
- `ctx.*` field names from the spec's "Generation Context" section.
- Fusion user-parameter names and dialog input ids/labels/units/defaults.
- For spur specifically, the reviewer must assert: **`helicalgear.py` and `herringbonegear.py`
  would still import and run unchanged** against the generated file.

**Must be behaviorally equivalent:**
- Same parameter formulas and which ones are live Fusion expressions vs Python-precomputed.
- Same geometry construction (circles, involute sampling, mirroring/rotation, ribs, root
  connectors, tooth-top arc) in the same order.
- Same features and order (extrude tooth → chamfer → extrude body → pattern → combine → fillet →
  bore), and the same edge cases: embedded vs non-embedded tooth, sketch-only short-circuit,
  optional bore/chamfer/fillet, fillet axial-edge selection.

**May differ freely:** local variable names, comments, log text, and how steps are split into
private helper methods.

## Notes

- This skill is gear-agnostic. It is first proven on `spurgear`; the same procedure regenerates
  `helicalgear`/`herringbonegear` once those specs exist (their specs would describe only the
  delta over spur, and the reference is their own `.py`).
- The reference `spurgear.py` is the committed production implementation — the authoritative
  behavior and the one the subclasses actually bind to. If a refactored variant is the intended
  target instead, pass it as the reference path explicitly.
