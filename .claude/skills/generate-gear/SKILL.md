---
name: generate-gear
description: Generate (or regenerate) a gear generator implementation `lib/geargen/<gear>.py` from its natural-language spec `lib/geargen/<gear>.md` plus the shared `PLAYBOOK.md`, using the spec as the SOLE source of truth — no reference implementation is consulted during generation. Use when asked to (re)generate gear code from a spec, or to check that a spec is complete enough to drive generation on its own. Args: optional `<gear>` name (default `spurgear`).
---

# Generate gear code from a spec

This repo generates gear implementations from natural-language design docs:
`lib/geargen/<gear>.md` (the spec) → `lib/geargen/<gear>.py` (the implementation). This skill
runs that generation as a repeatable, agent-driven workflow.

**The spec is the sole source of truth.** Generation reads only the spec, the shared playbook,
and the framework files. It does **not** read any existing `lib/geargen/<gear>.py` — if a spec
cannot reproduce its gear from those inputs alone, the spec is incomplete, and the fix is to
improve the **spec or playbook**, never to consult or copy an existing implementation. A gear with
no implementation yet is generated the same way as one being regenerated; the workflow does not
depend on a prior `.py` existing.

Two inputs drive every generation:
- the per-gear **spec** `lib/geargen/<gear>.md` — the *what* (geometry, parameters, the contract
  surface, generation order);
- the shared **playbook** `PLAYBOOK.md` (next to this file) — the *how* (framework scaffolding,
  Fusion-API conventions, optional architectural patterns). Read it in full first.

The spec + playbook together MUST be sufficient. If they are not, fix the spec or playbook.

## Inputs

- `gear` (default `spurgear`): names the spec `lib/geargen/<gear>.md` and the output
  `lib/geargen/<gear>.py` (generated first to the scratch path `.tmp/<gear>.generated.py`).

## Procedure

1. **Setup.** Work in a worktree (per the repo's CLAUDE.md — never the root checkout). Ensure
   `.tmp/` exists. Read this skill, `PLAYBOOK.md`, and the spec end-to-end. Skim the shared
   framework the output builds on: `lib/geargen/base.py`, `misc.py`, `utilities.py`,
   `lib/fusion360utils/`, and `commands/<gear>/entry.py`.

2. **Extract the contract from the spec.** Read the spec's **Contract** sections (the classes,
   hook methods, generation-context fields, generation order, and exact input ids / parameter-name
   strings it declares — see "Required spec sections" below). This list is the hard requirement
   the generated code must satisfy. If the spec declares **dependent gears** (e.g. a subclass
   family, or another gear it borrows a class from), read those files too and treat the surface
   they bind to as part of the required contract.

3. **Generate.** Spawn a subagent that writes `.tmp/<gear>.generated.py` from **the spec +
   playbook + the framework files + any declared dependency files only**. It MUST NOT read an
   existing `lib/geargen/<gear>.py` if one is present.
   - **Use the verbatim "Standard generation prompt" in the appendix below — do NOT improvise it,
     and do NOT add per-gear hints, "high-risk" checklists, reminders of specific gotchas, or any
     gear-specific guidance to the prompt.** Substitute only `<gear>`. All gear-specific knowledge
     (geometry, failure modes, the exact ⚠️ rules) MUST live in the **spec/playbook**, never in the
     generation prompt. Reason: a hand-tuned prompt (a) adds orchestrator variance round-to-round on
     top of the model's, and (b) **masks spec gaps** — if the prompt spoon-feeds a rule, a green run
     no longer proves the *spec* states it well enough. Identical prompt every run ⇒ a pass/fail is
     attributable to the spec, which is the whole point. When a regen reveals a gap, fix the
     **spec/playbook** and re-run the **same** standard prompt — never patch the prompt.

4. **Validate (reference-free).** The output is checked against the **spec**, not against any
   implementation:
   - **Parse:** `python3 -c "import ast; ast.parse(open('.tmp/<gear>.generated.py').read())"`.
     (Fusion's `adsk` modules can't be imported here; the repo has no runnable tests — parse is
     the available mechanical gate.)
   - **Undefined-name check (catches `NameError`s that parse can't).** A typo'd or wrong-scope
     local (e.g. using a `generate()`-scope variable inside a build helper) is valid syntax but a
     runtime `NameError`. Catch it statically with pyflakes, filtering out the star-import names it
     can't resolve: `python3 -m pyflakes .tmp/<gear>.generated.py | grep "may be undefined" |
     grep -vE "'(to_cm|to_mm|get_design|get_ui|Generator|GenerationContext|get_value|get_boolean|get_selection|ParamNamePrefix|ComponentCleaner|get_normal|cast)'"`.
     Anything left is a **real undefined name** — fix it (it would otherwise only surface at Fusion
     runtime). (The grep filter drops the legit `from .misc/.base/.utilities import *` exports, which
     pyflakes flags as "may be undefined" because star imports defeat its resolution; the framework's
     star-export set is small and fixed, so subtracting it leaves only genuine bugs.) Install pyflakes
     once with `python3 -m pip install --break-system-packages pyflakes` if absent.
   - **Contract self-check:** every class name, hook method, tooth/profile-generator entry point,
     `ctx` field, Fusion user-parameter name, and dialog input id the spec's Contract sections
     declare is present in the generated file. If the spec declares dependent gears, confirm the
     surface those dependents bind to (by name) exists unchanged.
   - **Dependency resolution:** every name the generated file imports from another gear or
     framework module actually exists in that module.

5. **Iterate.** A parse error, a missing contract item, or an unresolved dependency means the
   **spec or playbook** is incomplete or wrong — fix it there (never hand-edit the generated file,
   never copy from an existing implementation) and regenerate from step 3. Repeat up to ~3 rounds.
   Converges when the output parses and satisfies the full declared contract.

6. **Install (on approval).** The generated file is the product. With the user's approval, copy
   `.tmp/<gear>.generated.py` to `lib/geargen/<gear>.py`. Without approval, leave it in `.tmp/`.

7. **Report.** State whether the spec drove a complete, contract-satisfying generation, the
   spec/playbook edits made, and any **asserted-but-unproven** gaps (geometry the spec describes
   that no mechanical check can confirm — see the honesty note). Commit spec/playbook improvements
   (and the installed `.py` if approved). No push without explicit approval.

## Required spec sections (the contract surface)

For generation to succeed from the spec alone, the spec MUST declare its own contract. A spec is
"complete enough" when it pins, in its own text:

- **Architecture** — the classes the gear defines, each with its base class (if any) and whether
  it uses `base.Generator` / a `GenerationContext` at all. Gears do not all share one class shape;
  the spec states the shape it uses.
- **Method contract / call graph** — the methods that are distinct, overridable steps (and, if a
  subclass family exists, the override boundaries `super()` is called at), plus what each returns
  for this gear.
- **Generation Context fields** — the canonical `ctx` field names passed between steps, or an
  explicit "none" if the gear carries no context object.
- **Generation Order** — the build steps in order, and which method owns each.
- **Exact input ids & parameter-name strings** — every dialog input id, label, unit, default, and
  every Fusion user-parameter name, plus which parameters are live expressions vs. Python-
  precomputed.
- **Dependencies** — any other gear/module this gear imports a class or helper from.
- **Sketch-discipline deltas** — any per-gear deviation from the playbook's shared rules.

`spurgear.md` is the worked example of a spec carrying all of these. The contract names are
**gear-specific identifiers**: reproduce them exactly, but do not assume spur's class names or
methods apply to another gear — read them from that gear's spec.

## What the generated code must satisfy vs. what may vary

**Must match the spec's declared contract exactly** (rename/drop = breakage):
- Class names and base classes as the spec's Architecture section declares.
- Overridable methods / tooth-generator entry points / `draw(...)` signatures the spec's Method
  contract declares.
- `ctx` field names from the spec's Generation Context section.
- Fusion user-parameter names and dialog input ids/labels/units/defaults from the spec.
- The surface of any dependent gear the spec declares (so those dependents still import and run
  unchanged).

**Must follow the spec's prescribed behavior:**
- The parameter formulas and which are live Fusion expressions vs. Python-precomputed.
- The geometry construction and feature order exactly as the spec's Generation Order lists,
  including the edge cases the spec calls out.

**May differ freely:** local variable names, comments, log text, and how steps are split into
private helper methods beyond the method boundaries the spec pins.

## Honesty note — what this workflow can and cannot prove

Dropping the reference means the **spec is the only oracle**. The mechanical checks (parse,
contract self-check, dependency resolution) prove the output *honors the spec's declared contract*
and is structurally sound. They CANNOT prove the spec itself captured the geometry correctly —
that correctness is asserted by whoever authored the spec. State any such asserted-but-unproven
geometry in the report so it is visible, not silently assumed.

## Notes

- This skill is gear-agnostic. The same procedure generates `spurgear`, `bevelgear`,
  `helicalgear`, `herringbonegear`, etc., from each gear's own spec; the spec declares that gear's
  contract surface, which may differ in class shape from spur.
- A gear that borrows from another (e.g. one reusing another's tooth generator) must declare that
  in its Dependencies section so generation reads the dependency and the contract self-check
  covers the borrowed surface.

## Standard generation prompt (use verbatim — substitute only `<gear>`)

This is the fixed prompt for the step-3 generation subagent. Use it **exactly**, changing only the
`<gear>` token. Do **not** add gear-specific hints, gotcha reminders, or "high-risk" lists — those
belong in the spec/playbook. Identical prompt every run is what makes the regen an honest test of
the spec.

> Generate a Fusion 360 gear-generator implementation **purely from its spec** — reference-free.
> Work in the repo's worktree. Write the result to `.tmp/<gear>.generated.py` (parse-only; the
> `adsk` modules are not importable here — do not try to run it).
>
> **HARD RULE — reference-free:** Do NOT open, read, grep, or otherwise consult an existing
> `lib/geargen/<gear>.py`. If anything is unclear, record it as a spec-gap note in your report —
> never resolve it by peeking at an existing implementation.
>
> **Read, in full, ONLY these:**
> - `lib/geargen/<gear>.md` — THE SPEC, the sole source of truth. Read it end-to-end. **Every
>   rule, formula, and ⚠️ note in it is binding** — the spec already encodes the known failure
>   modes; obey them precisely. Do not rely on any guidance outside the spec/playbook.
> - `.claude/skills/generate-gear/PLAYBOOK.md` — shared framework conventions and Fusion-API rules.
> - Every document the spec references by name (e.g. a geometry-derivation `.md`).
> - The framework files the output builds on: `lib/geargen/base.py`, `lib/geargen/misc.py`,
>   `lib/geargen/utilities.py`, and `lib/fusion360utils/`.
> - Every gear/module the spec's **Dependencies** section declares (read the exact surface bevel/…
>   borrows: class names, constructor/`draw` signatures, and any attribute it reads or writes).
>
> **Contract = the spec's own declarations.** The spec's Architecture, Method-contract / call-graph,
> Generation-Context, Generation-Order, Exact-input-ids, Dependencies, and Sketch-Discipline
> sections ARE the hard requirement. Extract them yourself and satisfy them exactly: reproduce every
> class name, method/hook name and signature, constant, dialog input id/label/unit/default, and
> declared helper. Follow the Instructions sections' geometry and feature order verbatim, including
> every edge case and ⚠️ the spec calls out. Local variable names, comments, and private-helper
> decomposition beyond the pinned boundaries may vary.
>
> **Self-check before finishing** (fix and repeat until all pass):
> 1. `python3 -c "import ast; ast.parse(open('.tmp/<gear>.generated.py').read())"` succeeds.
> 2. `python3 -m pyflakes .tmp/<gear>.generated.py | grep "may be undefined" | grep -vE
>    "'(to_cm|to_mm|get_design|get_ui|Generator|GenerationContext|get_value|get_boolean|get_selection|ParamNamePrefix|ComponentCleaner|get_normal|cast)'"`
>    prints nothing.
> 3. Every class / method / constant / input id / declared helper the spec's contract sections name
>    is present, spelled exactly.
> 4. Every name imported from another module exists in that module.
>
> **Report:** whether the self-checks pass (with line numbers for the contract items); the final
> line count; and — most important — a precise list of any place the spec was **ambiguous or
> insufficient** and you had to infer, since those are the spec gaps to fix. Do not smooth over
> inferences; name them.

When a run fails validation or (later) misbehaves in Fusion, the fix is always to the **spec or
playbook**, after which you re-run **this same prompt** — you never edit the prompt to compensate.
