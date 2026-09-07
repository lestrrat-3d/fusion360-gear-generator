Write the Fusion 360 add-in implementation for `{{gear}}` by following its compiled step list. Work
in the repo worktree. Write `.tmp/{{gear}}.generated.py`, then stop drafting and report what you
produced. Do not run `run_gates.py` or its individual check scripts, and do not start an internal
correction loop before submitting the artifact. You may run this cheap syntax check once:

    python3 -c "import ast; ast.parse(open('.tmp/{{gear}}.generated.py').read())"

It does not validate Fusion behavior and never replaces the orchestrator's complete gate battery.
Do not execute the generated module; the `adsk` modules exist only inside Fusion, so runtime
behavior cannot be checked during drafting.

**Read, in this order:** `spec/{{gear}}/steps.md`, including its complete generated
`## Compilation contract` section when present, which carries every manifest rule the output must
satisfy, then work through the timeline steps in order; `proof/{{gear}}/`, the checked geometry,
which steps tagged `[GO]` tell you to
transliterate literally rather than re-derive; `.tmp/{{gear}}.playbook-extract.md`, the generated
extract of the playbook rules the steps cite by anchor plus the shared core sections (it replaces
reading `PLAYBOOK.md`, which you must not open — an anchor the extract lacks and the step list
still needs is a defect to report, not a reason to go find the full playbook); and the framework
you build on and must not reimplement, which is
`lib/geargen/base.py`, `misc.py`, `utilities.py`, `spurproxy.py` and `lib/fusion360utils/`.
Read `docs/prose-pipeline-handoffs/formats.md` for step metadata.

**Do not read** `lib/geargen/{{gear}}.py`, `spec/{{gear}}/instructions.md`, `spec/{{gear}}/fusion.md`,
or any previous draft. The step list is deliberately the only description of the gear you get. If
a step is unclear, record it as a defect in your report and make your best attempt.

**Use version-2 `required` call declarations as the execution checklist.** Preserve each declaration's
stated condition and write a reachable executable call; comments and dead branches do not satisfy it.
Conditional requirements still need execution; the gate does not prove branch coverage.
Read `docs/prose-pipeline-handoffs/formats.md#version-2` for role rules;
`.claude/skills/generate-gear/step_metadata.py` owns accepted syntax.
Other roles create no positive execution requirement. Existing source guards still enforce prohibitions.
Legacy and version-1 step lists retain their existing call checks.
A pre-emission compile report may remain failed solely because the current module lacks these required
calls. Implement them here; only the orchestrator's post-placement full compile rerun can accept the pipeline.

**The step list's required API calls have shared API-status decisions.** Compilation allows documented
calls and preserves visible advisory evidence for exact unverified receiver/member pairs; it
blocks refuted pairs and missing declarations. Write those calls as the steps give them. For a
call you introduce, a span whose arguments are unstated, or a call a gate flags, run
`python3 .claude/skills/generate-gear/query_api_status.py --owner <qualified-class> --member
<member>` for the same support decision the gates use. For signature and argument detail, ask the
`fusion:query-api` skill `show <Class>.<member>`; status does not provide a full signature. Pass
what the signature asks for. Where it says `ValueInput`, a bare number raises. Where it says
`ObjectCollection`, a Python list raises. Where it says `Point3D`, a `SketchPoint` raises. Use
`members <Class>` to discover the documented alternative when status blocks. If the step list
names a blocked call, report it and do not quietly correct it.

**Every literal the step list states is exact — copy it, never regularise it.** Input ids, label
strings, tooltips, constant names and their values, unit strings and default expressions are
contract surface: the step list carries them because nothing you can read recovers them. An id
written `boreEnable` is `boreEnable`, not `enableBore`; `spiralAngle` is not `meanSpiralAngle`.
The failure here is not carelessness but tidying, and it has already shipped a broken dialog once.
If a value you need is genuinely absent from the step list, that is a defect to report, never a gap
to fill with a plausible invention.

**Where a step names the entity a call is made against, use that entity and no other.** A step that
says a line is collinear with `A->E` means `A->E`, not the axis further up the same chain, even
though both describe the same infinite line. Substituting a geometrically equivalent operand there
has already drawn a Fusion `VCS_SKETCH_OVER_CONSTRAINTS` refusal.

**Ask the database before calling a type complaint a stub artifact.** Where a complaint is about
whether a member exists at all, `show <Class>.<member>` settles it in one call. Reasoning from what
the type ought to be has already passed a real bug through: three assignments to `SketchLine.name`,
a property that class does not declare, which Fusion refuses at runtime. Every required gate passed
on that build, and only the advisory novel-type row caught it.

**Do every step.** A step you cannot finish is a defect to report, never a comment left in the
file and never a silent omission.

Never silence a gate finding by deleting a comment, renaming a variable, or removing the call it
objects to. Preserve the step's intent while correcting the reported defect.

**Report:** the artifact path, its final line count if useful, and every step that was unclear,
incomplete, contradictory, or that you could not carry out as written, named by its step ID.
The orchestrator owns gate results, advisory triage, and retry decisions; report facts you can
know from the artifact and the step list only.
