---
name: generate-gear
description: Generate (or regenerate) a gear generator implementation `lib/geargen/<gear>.py` from its natural-language spec `spec/<gear>/instructions.md` plus the shared `PLAYBOOK.md`, using the spec as the SOLE source of truth — no reference implementation is consulted during generation. Use when asked to (re)generate gear code from a spec, or to check that a spec is complete enough to drive generation on its own. For a gear that already has a compiled step list `spec/<gear>/steps.md`, use `/compile-gear` + `/emit-gear` instead; this skill is for gears without one, or when named explicitly. Args: optional `<gear>` name (default `spurgear`).
---

# Generate gear code from a spec

This repo generates gear implementations from natural-language design docs:
`spec/<gear>/instructions.md` (the spec) → `lib/geargen/<gear>.py` (the implementation). This skill
runs that generation as a repeatable, agent-driven workflow.

**Check for a compiled step list first.** If `spec/<gear>/steps.md` exists, stop and use the
compile+emit pipeline instead: `/emit-gear <gear>` when
`python3 .claude/skills/generate-gear/check_compile.py <gear>` passes, `/compile-gear <gear>`
first when it does not. That pipeline transcribes from the checked step list rather than
re-interpreting the prose spec and playbook every round. Proceed with this skill only when no
step list exists, or when the user asked for this skill by name.

**The spec is the sole source of truth.** Generation reads only the spec, the shared playbook,
and the framework files. It does **not** read any existing `lib/geargen/<gear>.py` — if a spec
cannot reproduce its gear from those inputs alone, the spec is incomplete, and the fix is to
improve the **spec or playbook**, never to consult or copy an existing implementation. A gear with
no implementation yet is generated the same way as one being regenerated; the workflow does not
depend on a prior `.py` existing.

Each gear's spec lives in its own directory `spec/<gear>/`: the entry point is
`spec/<gear>/instructions.md`, optionally alongside auxiliary docs it references (e.g. a geometry
derivation like `spec/bevelgear/spiral-tooth-trace.md`). The generated code goes to
`lib/geargen/<gear>.py` (the `.py` modules stay in `lib/geargen/`; only the specs live under `spec/`).

Two inputs drive every generation:
- the per-gear **spec** `spec/<gear>/instructions.md` — the *what* (geometry, parameters, the contract
  surface, generation order) — plus any auxiliary docs it references in `spec/<gear>/`;
- the shared **playbook** `PLAYBOOK.md` (next to this file) — the *how* (framework scaffolding,
  Fusion-API conventions, optional architectural patterns). Read it in full first.

The spec + playbook together MUST be sufficient. If they are not, fix the spec or playbook.

## Inputs

- `gear` (default `spurgear`): names the spec `spec/<gear>/instructions.md` and the output
  `lib/geargen/<gear>.py` (generated first to the scratch path `.tmp/<gear>.generated.py`).

## Procedure

1. **Setup.** Work in a worktree (per the repo's CLAUDE.md — never the root checkout). Ensure
   `.tmp/` exists. Run `python3 .claude/skills/generate-gear/preflight.py <gear> --stage generate`
   and fix every `[FAIL]` before drafting; it verifies the engines, the go toolchain and the API
   database so a broken environment fails here instead of mid-run. Read this skill end-to-end.
   Do **not** read `PLAYBOOK.md`, the spec body, or the framework files up front: the generation
   subagent reads all of them, and each orchestrator decision that needs a section names that
   section in the step that makes the decision (steps 2, 3, 5, 6 and 8 below). Note the gear's
   sketch-first proof at `spec/<gear>/sketch/` if present (run in step 3).

2. **Extract the contract from the spec.** Read the spec's **Contract** sections (the classes,
   hook methods, generation-context fields, generation order, and exact input ids / parameter-name
   strings it declares — see "Required spec sections" below). This list is the hard requirement
   the generated code must satisfy. If the spec declares **dependent gears** (e.g. a subclass
   family, or another gear it borrows a class from), read those files too and treat the surface
   they bind to as part of the required contract.

3. **Prove the sketch fully constrains (sketch-first gate — `[PB-SKETCH-FIRST]`).** If the gear's
   profile is a non-trivial constrained sketch, run
   `python3 .claude/skills/generate-gear/run_sketch_bench.py <gear>` and act on the exit code:
   **0** the primary gate passed (the bench proved `Status == FullyConstrained` with healthy
   conditioning) — proceed; **1** the constraint scheme does not fully constrain — a spec/playbook
   defect to fix here, never inside Fusion; **2** a setup problem (no bench yet, missing
   sketch-engine checkout, a bench that does not build, or a bench that printed no verdict) — fix
   the environment or build the bench first. This proves the constraint scheme is sound *before*
   any Fusion code is emitted; the advisory signals (`ProfilesValid`, `Probe.Ambiguous()`) remain
   in the bench output and are reported and interpreted, not hard-blocking
   (see `[PB-SKETCH-FIRST]`).
   If the proof does not yet exist for this gear, build it from the spec's sketch recipes (the spur
   `spec/spurgear/sketch/` is the worked example) — a scheme that cannot reach `DOF == 0` on the
   bench is a spec/playbook defect to fix here, not to discover inside Fusion. Read the playbook's
   `[PB-SKETCH-FIRST]` section here when interpreting the advisory signals, and the spec's
   sketch-recipe sections only if the proof has to be built at this step. Requires a local
   checkout of the `sketch` engine (`$SKETCH_DIR` or a sibling `../sketch`).

4. **Generate.** Spawn a subagent that writes `.tmp/<gear>.generated.py` from **the spec +
   playbook + the framework files + any declared dependency files only**. It MUST NOT read an
   existing `lib/geargen/<gear>.py` if one is present.
   - **Use the standard generation prompt: run
     `python3 .claude/skills/generate-gear/render_prompt.py generate-gear <gear>` and pass its
     printed output to the subagent unchanged — do NOT improvise the prompt, and do NOT add
     per-gear hints, "high-risk" checklists, reminders of specific gotchas, or any gear-specific
     guidance to it.** All gear-specific knowledge
     (geometry, failure modes, the exact ⚠️ rules) MUST live in the **spec/playbook**, never in the
     generation prompt. Reason: a hand-tuned prompt (a) adds orchestrator variance round-to-round on
     top of the model's, and (b) **masks spec gaps** — if the prompt spoon-feeds a rule, a green run
     no longer proves the *spec* states it well enough. Identical prompt every run ⇒ a pass/fail is
     attributable to the spec, which is the whole point. When a regen reveals a gap, fix the
     **spec/playbook** and re-run the **same** standard prompt — never patch the prompt.
   - **Launch in the background and let the watcher do the waiting.** Clear the previous run's
     telemetry first — run
     `python3 .claude/skills/generate-gear/watch_progress.py .tmp/<gear>.progress.log --clear`,
     which removes the log and its `.watch` sidecar and refuses (exit 5) if the log was written in
     the last minute, since that suggests a run still in flight — then run
     the subagent as a background task (the standard prompt makes it append one milestone line per
     phase). Do **not** poll the log by hand. Wait with one foreground call, Bash timeout 600000:

     ```
     python3 .claude/skills/generate-gear/watch_progress.py .tmp/<gear>.progress.log
     ```

     It blocks until the run reaches a verdict or its 9-minute call budget expires, then prints the
     status, the elapsed time, the milestone counts and the tail of the log. Report from that
     output; do not re-read the log. Act on the exit code: **0** the subagent is done — go to step
     5; **4** still healthy — run the identical command again (a healthy 10–15 minute round takes
     one or two of these); **1** no heartbeat within the launch window, so the launch never began
     (e.g. an unanswered approval gate) — stop and surface it; **2** silent past the stall window —
     stop the run, relaunch once, and only then involve the user; **3** past the 45-minute ceiling
     with the run still alive — stop it and surface the elapsed time; **5** the progress log is not
     in the state the watch assumed — inspect before relaunching, and clear a confirmed-stale log
     with the same script's `--clear` before the relaunch. The script carries SKILL.md's
     windows as its defaults (`--launch-window 300`, `--stall-window 600`, `--max-runtime 2700`);
     override one only with a stated reason.

5. **Validate (reference-free).** The output is checked against the **spec**, not against any
   implementation. Run the mechanical battery with one command from the repo root:

   ```
   python3 .claude/skills/generate-gear/run_gates.py <gear> --skip-missing-steps
   ```

   It runs parse, input-read pairing, the contract manifest (auto-skipped with a note when the
   gear has no `spec/<gear>/contract.json`), step-calls (auto-skipped when the gear has no
   compiled `spec/<gear>/steps.md`, which is what `--skip-missing-steps` permits), anchors,
   API-call existence, and pyright with the Fusion stubs, then prints one verdict plus the
   advisory novel-type report. Exit 0 means every gate that ran passed; exit 1 means a gate
   failed on content — fix the **spec/playbook** and regenerate (step 4), never the generated
   file; exit 2 means a setup problem (missing candidate, unreachable stubs or API database)
   that no new draft can fix. Ignore the runner's emit-vs-compile fault labels at this stage:
   in the one-shot workflow every content failure routes the same way, to a spec or playbook
   fix followed by a regeneration. Two reading notes carry over from the per-check era:

   - **pyright:** only **BLOCKING** findings gate. **REVIEW** findings are advisory stub
     pessimism (idiomatic downcasts, Optional-typed API returns); correct code like
     `spurgear.py` emits ~27 with zero real bugs, so never gate or thrash the spec on them.
     Stubs come from `$FUSION_API_STUBS`; on first run the check auto-clones a sparse copy of
     `FusionAPIReference` into `~/.cache/fusion360-gear-generator/` and auto-installs pyright
     there. If the stubs are unavailable (the gate reports a setup error), fall back to a
     pyflakes undefined-name grep and to `fusion:query-api` `show <Name>`
     ([PB-ADSK-MODULES]) for submodule questions.
   - **Novel types:** read each advisory finding and decide; record stub noise with
     `check_novel_types.py --accept N --why "<reason>"`. It is the only check that has caught
     a method called on a class that does not define it.

   The runner cannot see prose, so after it passes, prose-check what no gate carries:

   - **Contract (no manifest):** for a gear without `spec/<gear>/contract.json`, check every
     class name, hook method, tooth/profile-generator entry point, `ctx` field, Fusion
     user-parameter name, and dialog input id the spec's Contract sections declare, and the
     surface any declared dependent gears bind to. (`spec/spurgear/contract.json` is the
     worked example of moving this into the manifest.)
   - **Helper shadowing:** the manifest gate catches a literal re-definition of a framework
     helper (PLAYBOOK "Shared geargen helper library"), but a private re-implementation under
     a different name still needs a prose check. A shadow means the spec/playbook failed to
     direct the generator to the helper; fix there and regenerate. Perform the prose half by
     reading the playbook's "Shared geargen helper library" section and the `def` names in
     `lib/geargen/solids.py` at this point; no earlier read supports it.
   - **Dependency resolution:** `check_contract.py` verifies `lib/geargen/` imports
     mechanically; confirm any other imported surface the spec's Dependencies section names.

6. **Iterate.** A failed gate, a missing contract item, or an unresolved dependency means the
   **spec or playbook** is incomplete or wrong — fix it there (never hand-edit the generated file,
   never copy from an existing implementation) and regenerate from the Generate step (step 4) with
   a fresh subagent, because the edit made the context the previous drafter read stale. Repeat up
   to ~3 rounds. Converges when the output parses and satisfies the full declared contract. When
   diagnosing, read only the spec or playbook sections the failure implicates; the diagnosis is
   when they earn their read.

   One failure class skips the fresh spawn: a mechanical slip no spec wording caused (a syntax
   error, a typo in a name the spec spells correctly). No input file changed, so send the check
   output to the same drafting subagent with `SendMessage` and let it fix
   `.tmp/<gear>.generated.py` in place. Skip step 4's telemetry reset and the watcher for such a
   round — the fix runs without re-reading inputs, and completion arrives as the subagent's
   reply. If no reply comes within about ten minutes, or the same check fails again, treat the
   failure as a spec/playbook gap and take the fresh-spawn path above.

7. **Install (on approval).** The generated file is the product. With the user's approval, copy
   `.tmp/<gear>.generated.py` to `lib/geargen/<gear>.py`. Without approval, leave it in `.tmp/`.

8. **Report.** State whether the spec drove a complete, contract-satisfying generation, the
   spec/playbook edits made, and any **asserted-but-unproven** gaps (geometry the spec describes
   that no mechanical check can confirm — see the honesty note). Commit spec/playbook improvements
   (and the installed `.py` if approved). No push without explicit approval. Source the
   asserted-but-unproven list from the subagent's own report plus a skim of the spec's geometry
   sections at this step, not from an up-front spec read.

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

`spec/spurgear/instructions.md` is the worked example of a spec carrying all of these. The contract names are
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

## Standard generation prompt

The prompt text lives in `.claude/skills/generate-gear/prompt.md`; `{{gear}}` is its only
placeholder. This is the fixed prompt for the Generate-step (step 4) subagent. Render it —
never retype or paraphrase it — with:

    python3 .claude/skills/generate-gear/render_prompt.py generate-gear <gear>

Hand the printed output to the subagent unchanged. Do **not** add gear-specific hints, gotcha
reminders, or "high-risk" lists — those belong in the spec/playbook. An identical prompt every
run is what makes the regen an honest test of the spec, and the renderer refuses to print
anything for an unknown skill name, a missing template, or a template carrying a placeholder it
was not given.

When a run fails validation or (later) misbehaves in Fusion, the fix is always to the **spec or
playbook**, after which you re-run **this same prompt** — you never edit the prompt to compensate.
