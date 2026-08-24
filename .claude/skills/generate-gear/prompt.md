Generate a Fusion 360 gear-generator implementation **purely from its spec** — reference-free.
Work in the repo's worktree. Write the result to `.tmp/{{gear}}.generated.py` (parse-only; the
`adsk` modules are not importable here — do not try to run it).

**HARD RULE — reference-free:** Do NOT open, read, grep, or otherwise consult an existing
`lib/geargen/{{gear}}.py`. If anything is unclear, record it as a spec-gap note in your report —
never resolve it by peeking at an existing implementation.

**Progress heartbeat (liveness telemetry only — it does not alter what you generate):** append
one line `<unix-epoch-seconds> <milestone>` to `.tmp/{{gear}}.progress.log` at each milestone
(e.g. `date +%s` for the timestamp): `start` as your first action; `read:<path>` after finishing
each required input file; `draft:written` immediately after writing `.tmp/{{gear}}.generated.py`;
`check:<n>:pass` / `check:<n>:fail` after each self-check round; `done` as your last action
before reporting. Never go more than a few minutes without a heartbeat line.

**Read, in full, ONLY these:**
- `spec/{{gear}}/instructions.md` — THE SPEC, the sole source of truth. Read it end-to-end. **Every
  rule, formula, and ⚠️ note in it is binding** — the spec already encodes the known failure
  modes; obey them precisely. Do not rely on any guidance outside the spec/playbook.
- `.claude/skills/generate-gear/PLAYBOOK.md` — shared framework conventions and Fusion-API rules.
- Every document the spec references by name (e.g. a geometry-derivation `.md`).
- The framework files the output builds on: `lib/geargen/base.py`, `lib/geargen/misc.py`,
  `lib/geargen/utilities.py`, `lib/geargen/solids.py`, `lib/geargen/spurproxy.py`, and
  `lib/fusion360utils/`.
- Every gear/module the spec's **Dependencies** section declares (read the exact surface bevel/…
  borrows: class names, constructor/`draw` signatures, and any attribute it reads or writes).

**Contract = the spec's own declarations.** The spec's Architecture, Method-contract / call-graph,
Generation-Context, Generation-Order, Exact-input-ids, Dependencies, and Sketch-Discipline
sections ARE the hard requirement. Extract them yourself and satisfy them exactly: reproduce every
class name, method/hook name and signature, constant, dialog input id/label/unit/default, and
declared helper. Follow the Instructions sections' geometry and feature order verbatim, including
every edge case and ⚠️ the spec calls out. Local variable names, comments, and private-helper
decomposition beyond the pinned boundaries may vary.

**Self-check before finishing** (fix and repeat until all pass):
1. `python3 -c "import ast; ast.parse(open('.tmp/{{gear}}.generated.py').read())"` succeeds.
2. `python3 .claude/skills/generate-gear/pyright_check.py .tmp/{{gear}}.generated.py` reports
   **0 BLOCKING** (undefined names / typos and wrong-adsk-submodule refs). REVIEW findings are
   advisory stub noise — do not act on them or change the code to silence them.
3. `python3 .claude/skills/generate-gear/check_input_read.py .tmp/{{gear}}.generated.py` exits 0:
   every dialog input is read with the helper matching its declared type (`get_value` for
   value/string inputs, `get_boolean` for `addBoolValueInput`, `get_selection` for selections —
   `[PB-INPUT-READ]`). A mismatch is a real runtime crash pyright can't see.
4. Every class / method / constant / input id / declared helper the spec's contract sections name
   is present, spelled exactly.
5. Every name imported from another module exists in that module.

**Report:** whether the self-checks pass (with line numbers for the contract items); the final
line count; and — most important — a precise list of any place the spec was **ambiguous or
insufficient** and you had to infer, since those are the spec gaps to fix. Do not smooth over
inferences; name them.
