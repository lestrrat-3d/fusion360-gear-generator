Write the Fusion 360 add-in implementation for `{{gear}}` by following its compiled step list. Work
in the repo worktree. Write `.tmp/{{gear}}.generated.py`. Do not try to run it; the `adsk` modules
only exist inside Fusion, so parsing is as far as it goes.

**Read, in this order:** `spec/{{gear}}/steps.md`, which is your instruction set and which you work
through in order; `proof/{{gear}}/`, the checked geometry, which steps tagged `[GO]` tell you to
transliterate literally rather than re-derive; `.tmp/{{gear}}.playbook-extract.md`, the generated
extract of the playbook rules the steps cite by anchor plus the shared core sections (it replaces
reading `PLAYBOOK.md`, which you must not open — an anchor the extract lacks and the step list
still needs is a defect to report, not a reason to go find the full playbook); and the framework
you build on and must not reimplement, which is
`lib/geargen/base.py`, `misc.py`, `utilities.py`, `spurproxy.py` and `lib/fusion360utils/`.

**Do not read** `lib/geargen/{{gear}}.py`, `spec/{{gear}}/instructions.md`, `spec/{{gear}}/fusion.md`,
or any previous draft. The step list is deliberately the only description of the gear you get. If
a step is unclear, record it as a defect in your report and make your best attempt.

**The step list's call spans are pre-verified.** Every Fusion call written in a code span in
`spec/{{gear}}/steps.md` was checked against the API database when the step list was compiled, and
the spans carry the argument shapes the signatures ask for. Write those calls as the steps give
them; do not re-query them. Ask the `fusion:query-api` skill only about a call you introduce that
the step list does not carry, a span whose arguments the step leaves unstated, or a call a gate
flags. One question carries most of the work: `show <Class>.<member>` confirms in a few lines
that the class you are calling on really has the member — it resolves members declared on any
base and names the class that declares each — and gives its signature and documentation. Pass
what the signature asks for. Where it says `ValueInput`, a bare number raises. Where it says
`ObjectCollection`, a Python list raises. Where it says `Point3D`, a `SketchPoint` raises. When
`show` reports no match or returns a candidate list, the name as written does not exist; only
then ask `members <Class>`, which lists everything the class offers, inherited members included,
to find what the step list meant. If the step list names a call the API does not have, report it
and do not quietly correct it.

**Do every step.** A step you cannot finish is a defect to report, never a comment left in the
file and never a silent omission.

**Self-check before finishing**, by running
`python3 .claude/skills/generate-gear/run_gates.py {{gear}} --no-advisory` and fixing until it
exits 0. It runs every gate and reports all failures at once, so fix the whole list before
re-running. Exit 2 is a setup problem, not a draft problem: report it and stop rather than
editing around it. Run it once more without `--no-advisory` before you finish, and report what
its advisory row says. Never silence a finding by deleting a comment, renaming a variable, or
removing the call it objects to.

**Report:** the gate results, the final line count, and every step that was unclear, incomplete,
or that you could not carry out as written, named by its step ID.
