Write the Fusion 360 add-in implementation for `{{gear}}` by following its compiled step list. Work
in the repo worktree. Write `.tmp/{{gear}}.generated.py`, then stop drafting and report what you
produced. Do not run `run_gates.py` or its individual check scripts, and do not start an internal
correction loop before submitting the artifact. You may run this cheap syntax check once:

    python3 -c "import ast; ast.parse(open('.tmp/{{gear}}.generated.py').read())"

It does not validate Fusion behavior and never replaces the orchestrator's complete gate battery.
Do not execute the generated module; the `adsk` modules exist only inside Fusion, so runtime
behavior cannot be checked during drafting.

**Read, in this order:** `.tmp/{{gear}}.steps-view.md`, the checked view of
`spec/{{gear}}/steps.md` and your instruction set, which you work through in order;
`.tmp/{{gear}}.proof-bundle.md`, the verified construction view of the checked
Go proof, which steps tagged `[GO]` tell you to transliterate literally rather than re-derive;
`.tmp/{{gear}}.playbook-extract.md`, the generated
extract of the playbook rules the steps cite by anchor plus the shared core sections (it replaces
reading `PLAYBOOK.md`, which you must not open — an anchor the extract lacks and the step list
still needs is a defect to report, not a reason to go find the full playbook); and the framework
you build on and must not reimplement, which is
`lib/geargen/base.py`, `misc.py`, `utilities.py`, `spurproxy.py` and `lib/fusion360utils/`.
Read `.tmp/{{gear}}.dependency-view.md` for signatures, decorators, and constants of gear modules
named by the checked steps. It is generated from the current source and verified before drafting.
Read `docs/spec-to-code-performance/06-step-metadata-format.md` for version-2 call roles.

**Do not read** `lib/geargen/{{gear}}.py`, `spec/{{gear}}/steps.md`,
`spec/{{gear}}/instructions.md`, `spec/{{gear}}/fusion.md`, or any previous draft.
The checked step view is deliberately the only description of the gear you get. If
a step is unclear, record it as a defect in your report and make your best attempt.

The checked step list includes `## Compilation contract` when the gear has a manifest. Read its
v1 JSON as requirements for the module, its classes, methods, constants, and source guards.
Apply each guard inside its named function. Do not read `contract.json`; the complete manifest
is already in the checked view. The canonical gates check its constants against the omitted
`## Exact values` section.

**Use version-2 `required` declarations as the execution checklist.** Preserve each
stated condition. Other roles add no positive call requirement; existing source guards still
enforce forbidden behavior. Legacy step lists retain their existing call checks.

**The step list's required call spans are pre-verified.** Every required Fusion call written in a code span in
the canonical `spec/{{gear}}/steps.md` was checked against the API database when the step list was compiled, and
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

**Every literal the step list states is exact — copy it, never regularise it.** Input ids, label
strings, tooltips, constant names and their values, unit strings and default expressions are
contract surface: the step list carries them because nothing you can read recovers them. An id
written `boreEnable` is `boreEnable`, not `enableBore`; `spiralAngle` is not `meanSpiralAngle`.
The failure here is not carelessness but tidying, and it has already shipped a broken dialog once.
If a value you need is genuinely absent from the step list, that is a defect to report, never a gap
to fill with a plausible invention.

If the checked view contains `## Deterministic setup`, the deterministic renderer supplies
exported constants and the dialog and parameter setup after your draft.
Leave the module-level exact constants absent. Define `configure` with `pass`, and define
the setup method with `pass` when the gear adds its own primary parameters through an override.
For a generator with `processInputs`, keep its selection handling and final
`self.registerDerivedParameters()` call; define `registerDerivedParameters` with `pass` when it
exists. The renderer inserts the checked setup and calls a base configurator for subclasses.
Keep other methods and geometry complete.

**Where a step names the entity a call is made against, use that entity and no other.** A step that
says a line is collinear with `A->E` means `A->E`, not the axis further up the same chain, even
though both describe the same infinite line. Substituting a geometrically equivalent operand there
has already drawn a Fusion `VCS_SKETCH_OVER_CONSTRAINTS` refusal.

**Ask the database before calling a type complaint a stub artifact.** Where a complaint is about
whether a member exists at all, `show <Class>.<member>` settles it in one call. Reasoning from what
the type ought to be has already passed a real bug through: three assignments to `SketchLine.name`,
a property that class does not declare, which Fusion refuses at runtime. Every required gate passed
on that build, and only the advisory novel-type row caught it.

**Write every Fusion call on a receiver whose type the checker can follow.** The API-call gate
refuses a call whose receiver's type it cannot work out, and it follows types only through what
the code states. Measured: the first draft of one gear failed this gate on 40 to 50 calls, on
every emit of that gear, and each needed two or three retry rounds that changed nothing but types.
Write it right the first time:

- Annotate parameters that carry Fusion objects, such as a configurator's `command:
  adsk.core.Command` and a generator's `inputs: adsk.core.CommandInputs`, and give return
  annotations to helpers that return Fusion objects.
- Cast a lookup to the type you use, such as `adsk.core.ValueCommandInput.cast(inputs.itemById(i))`.
- A nested function does not see the enclosing function's annotations. Pass the objects it uses as
  typed parameters, or make it a method.
- The checker does not see through a list subscript. Bind the element to a typed local first, as
  `body: adsk.fusion.BRepBody = adsk.fusion.BRepBody.cast(self.bodies[0])`, then call on `body`.
- Declare instance fields with a typed placeholder, as `self.body = adsk.fusion.BRepBody.cast(None)`
  or `[adsk.fusion.BRepBody.cast(None)] * 2`, never `None` or `[None, None]`.
- Give the result of a call that returns a fresh object a typed local, as `v: adsk.core.Vector3D =
  u.copy()`.
- A helper that can find nothing raises a clear error instead of returning `None` into arithmetic
  or a call.
- Standard-library calls the checker does not know, such as `math.fmod`, `bisect.bisect_left` and
  `int.bit_length`, become plain arithmetic.

None of this changes which calls the module makes. Never satisfy the gate by deleting a call the
step list names, by replacing it with another, or by hiding an assignment behind `setattr`.

**Write a long module in pieces.** A module of more than about a thousand lines does not fit in
one response, and a draft that tries to write it in one tool call dies with nothing written. Create
the file with its first part, then add the rest in edits or appends of a few hundred lines each.

**Do every step.** A step you cannot finish is a defect to report, never a comment left in the
file and never a silent omission.

Never silence a gate finding by deleting a comment, renaming a variable, or removing the call it
objects to. Preserve the step's intent while correcting the reported defect.

**Report:** the artifact path, its final line count if useful, and every step that was unclear,
incomplete, contradictory, or that you could not carry out as written, named by its step ID.
The orchestrator owns gate results, advisory triage, and retry decisions; report facts you can
know from the artifact and the step list only.
