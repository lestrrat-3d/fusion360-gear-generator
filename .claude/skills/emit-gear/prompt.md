Write the Fusion 360 add-in implementation for `{{gear}}` by following its compiled step list. Work
in the repo worktree. Write `.tmp/{{gear}}.generated.py`. Do not try to run it; the `adsk` modules
only exist inside Fusion, so parsing is as far as it goes.

**Read, in this order:** `spec/{{gear}}/steps.md`, which is your instruction set and which you work
through in order; `proof/{{gear}}/`, the checked geometry, which steps tagged `[GO]` tell you to
transliterate literally rather than re-derive; `.claude/skills/generate-gear/PLAYBOOK.md` for the
rules the steps cite by anchor; and the framework you build on and must not reimplement, which is
`lib/geargen/base.py`, `misc.py`, `utilities.py`, `spurproxy.py` and `lib/fusion360utils/`.

**Do not read** `lib/geargen/{{gear}}.py`, `spec/{{gear}}/instructions.md`, `spec/{{gear}}/fusion.md`,
or any previous draft. The step list is deliberately the only description of the gear you get. If
a step is unclear, record it as a defect in your report and make your best attempt.

**Before writing any `adsk.*` call**, ask the `fusion:query-api` skill about it. Two questions
carry most of the work: `members <Class>` lists everything a class offers, inherited members
included, each with the class that declares it, which is how you find out whether the class you
are calling on really has the member; and `show <Class>.<member>` gives one member's signature
and documentation. Pass what the signature asks for. Where it says `ValueInput`, a bare number
raises. Where it says `ObjectCollection`, a Python list raises. Where it says `Point3D`, a
`SketchPoint` raises. If the step list names a call the API does not have, report it and do not
quietly correct it.

**Do every step.** A step you cannot finish is a defect to report, never a comment left in the
file and never a silent omission.

**Self-check before finishing**, fixing until all seven gates pass: parse, `pyright_check.py` with
0 BLOCKING, `check_input_read.py`, `check_contract.py`, `check_anchors.py`, `check_step_calls.py`
and `check_api_calls.py`. Never silence a finding by deleting a comment, renaming a variable, or
removing the call it objects to.

**Report:** the gate results, the final line count, and every step that was unclear, incomplete,
or that you could not carry out as written, named by its step ID.
